﻿using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace VelcroPhysics.Tools.Triangulation.Seidel
{
    internal class MonotoneMountain
    {
        // Almost Pi!
        private const float PiSlop = 3.1f;

        private HashSet<Point> _convexPoints;
        private Point _head;

        // Monotone mountain points
        private List<Point> _monoPoly;

        // Used to track which side of the line we are on
        private bool _positive;

        private int _size;
        private Point _tail;

        // Triangles that constitute the mountain
        public List<List<Point>> Triangles;

        public MonotoneMountain()
        {
            _size = 0;
            _tail = null;
            _head = null;
            _positive = false;
            _convexPoints = [];
            _monoPoly = [];
            Triangles = [];
        }

        // Append a point to the list
        public void Add(Point point)
        {
            if (_size == 0)
            {
                _head = point;
                _size = 1;
            }
            else if (_size == 1)
            {
                // Keep repeat points out of the list
                _tail = point;
                _tail.Prev = _head;
                _head.Next = _tail;
                _size = 2;
            }
            else
            {
                // Keep repeat points out of the list
                _tail.Next = point;
                point.Prev = _tail;
                _tail = point;
                _size += 1;
            }
        }

        // Remove a point from the list
        public void Remove(Point point)
        {
            var next = point.Next;
            var prev = point.Prev;
            point.Prev.Next = next;
            point.Next.Prev = prev;
            _size -= 1;
        }

        // Partition a x-monotone mountain into triangles O(n)
        // See "Computational Geometry in C", 2nd edition, by Joseph O'Rourke, page 52
        public void Process()
        {
            // Establish the proper sign
            _positive = AngleSign();

            // create monotone polygon - for dubug purposes
            GenMonoPoly();

            // Initialize internal angles at each nonbase vertex
            // Link strictly convex vertices into a list, ignore reflex vertices
            var p = _head.Next;
            while (p.Neq(_tail))
            {
                var a = Angle(p);

                // If the point is almost colinear with it's neighbor, remove it!
                if (a >= PiSlop || a <= -PiSlop || a == 0.0f)
                    Remove(p);
                else if (IsConvex(p))
                    _convexPoints.Add(p);
                p = p.Next;
            }

            Triangulate();
        }

        private void Triangulate()
        {
            while (_convexPoints.Count != 0)
            {
                using (IEnumerator<Point> e = _convexPoints.GetEnumerator())
                {
                    e.MoveNext();
                    var ear = e.Current;

                    _convexPoints.Remove(ear);
                    var a = ear.Prev;
                    var c = ear.Next;
                    var triangle = new List<Point>(3);
                    triangle.Add(a);
                    triangle.Add(ear);
                    triangle.Add(c);

                    Triangles.Add(triangle);

                    // Remove ear, update angles and convex list
                    Remove(ear);
                    if (Valid(a))
                        _convexPoints.Add(a);
                    if (Valid(c))
                        _convexPoints.Add(c);
                }
            }

            Debug.Assert(_size <= 3, "Triangulation bug, please report");
        }

        private bool Valid(Point p)
        {
            return p.Neq(_head) && p.Neq(_tail) && IsConvex(p);
        }

        // Create the monotone polygon
        private void GenMonoPoly()
        {
            var p = _head;
            while (p != null)
            {
                _monoPoly.Add(p);
                p = p.Next;
            }
        }

        private float Angle(Point p)
        {
            var a = p.Next - p;
            var b = p.Prev - p;
            return (float)Math.Atan2(a.Cross(b), a.Dot(b));
        }

        private bool AngleSign()
        {
            var a = _head.Next - _head;
            var b = _tail - _head;
            return Math.Atan2(a.Cross(b), a.Dot(b)) >= 0;
        }

        // Determines if the inslide angle is convex or reflex
        private bool IsConvex(Point p)
        {
            if (_positive != Angle(p) >= 0)
                return false;
            return true;
        }
    }
}