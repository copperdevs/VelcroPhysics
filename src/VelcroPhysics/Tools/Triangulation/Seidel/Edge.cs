﻿using System.Collections.Generic;

namespace VelcroPhysics.Tools.Triangulation.Seidel
{
    internal class Edge
    {
        // Pointers used for building trapezoidal map
        public Trapezoid Above;

        public float B;
        public Trapezoid Below;

        // Montone mountain points
        public HashSet<Point> MPoints;

        public Point P;
        public Point Q;

        // Slope of the line (m)
        public float Slope;

        public Edge(Point p, Point q)
        {
            P = p;
            Q = q;

            if (q.X - p.X != 0)
                Slope = (q.Y - p.Y) / (q.X - p.X);
            else
                Slope = 0;

            B = p.Y - p.X * Slope;
            Above = null;
            Below = null;
            MPoints =
            [
                p,
                q
            ];
        }

        public bool IsAbove(Point point)
        {
            return P.Orient2D(Q, point) < 0;
        }

        public bool IsBelow(Point point)
        {
            return P.Orient2D(Q, point) > 0;
        }

        public void AddMpoint(Point point)
        {
            foreach (var mp in MPoints)
            {
                if (!mp.Neq(point))
                    return;
            }

            MPoints.Add(point);
        }
    }
}