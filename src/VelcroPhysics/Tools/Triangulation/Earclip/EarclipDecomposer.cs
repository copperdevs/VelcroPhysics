/*
 * C# Version Ported by Matt Bettcher and Ian Qvist 2009-2010
 *
 * Original C++ Version Copyright (c) 2007 Eric Jordan
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Tools.Triangulation.Earclip
{
    /// <summary>
    /// Convex decomposition algorithm using ear clipping
    /// Properties:
    /// - Only works on simple polygons.
    /// - Does not support holes.
    /// - Running time is O(n^2), n = number of vertices.
    /// Source: http://www.ewjordan.com/earClip/
    /// </summary>
    internal static class EarclipDecomposer
    {
        //box2D rev 32 - for details, see http://www.box2d.org/forum/viewtopic.php?f=4&t=83&start=50 

        /// <summary>
        /// Decompose the polygon into several smaller non-concave polygon. Each resulting polygon will have no more than
        /// Settings.MaxPolygonVertices vertices.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="tolerance">The tolerance.</param>
        public static List<Vertices> ConvexPartition(Vertices vertices, float tolerance = 0.001f)
        {
            Debug.Assert(vertices.Count > 3);
            Debug.Assert(!vertices.IsCounterClockWise());

            return TriangulatePolygon(vertices, tolerance);
        }

        /// <summary>
        /// Triangulates a polygon using simple ear-clipping algorithm. Returns size of Triangle array unless the polygon
        /// can't be triangulated. This should only happen if the polygon self-intersects, though it will not _always_ return null
        /// for a bad polygon - it is the caller's responsibility to check for self-intersection, and if it doesn't, it should at
        /// least check that the return value is non-null before using. You're warned! Triangles may be degenerate, especially if
        /// you have identical points in the input to the algorithm.  Check this before you use them. This is totally unoptimized,
        /// so for large polygons it should not be part of the simulation loop.
        /// </summary>
        /// <remarks>Only works on simple polygons.</remarks>
        private static List<Vertices> TriangulatePolygon(Vertices vertices, float tolerance)
        {
            //Velcro note: Check is needed as invalid triangles can be returned in recursive calls.
            if (vertices.Count < 3)
                return [];

            var results = new List<Vertices>();

            //Recurse and split on pinch points
            var pin = new Vertices(vertices);
            if (ResolvePinchPoint(pin, out var pA, out var pB, tolerance))
            {
                var mergeA = TriangulatePolygon(pA, tolerance);
                var mergeB = TriangulatePolygon(pB, tolerance);

                if (mergeA.Count == -1 || mergeB.Count == -1)
                    throw new Exception("Can't triangulate your polygon.");

                results.AddRange(mergeA.Select(t => new Vertices(t)));

                results.AddRange(mergeB.Select(t => new Vertices(t)));

                return results;
            }

            var buffer = new Vertices[vertices.Count - 2];
            var bufferSize = 0;
            var xrem = new float[vertices.Count];
            var yrem = new float[vertices.Count];
            for (var i = 0; i < vertices.Count; ++i)
            {
                xrem[i] = vertices[i].X;
                yrem[i] = vertices[i].Y;
            }

            var vNum = vertices.Count;

            while (vNum > 3)
            {
                // Find an ear
                var earIndex = -1;
                var earMaxMinCross = -10.0f;
                for (var i = 0; i < vNum; ++i)
                {
                    if (!IsEar(i, xrem, yrem, vNum))
                        continue;

                    var lower = Remainder(i - 1, vNum);
                    var upper = Remainder(i + 1, vNum);
                    var d1 = Vector2.Normalize(new Vector2(xrem[upper] - xrem[i], yrem[upper] - yrem[i]));
                    var d2 = Vector2.Normalize(new Vector2(xrem[i] - xrem[lower], yrem[i] - yrem[lower]));
                    var d3 = Vector2.Normalize(new Vector2(xrem[lower] - xrem[upper], yrem[lower] - yrem[upper]));

                    MathUtils.Cross(ref d1, ref d2, out var cross12);
                    cross12 = Math.Abs(cross12);

                    MathUtils.Cross(ref d2, ref d3, out var cross23);
                    cross23 = Math.Abs(cross23);

                    MathUtils.Cross(ref d3, ref d1, out var cross31);
                    cross31 = Math.Abs(cross31);

                    //Find the maximum minimum angle
                    var minCross = Math.Min(cross12, Math.Min(cross23, cross31));

                    if (!(minCross > earMaxMinCross))
                        continue;

                    earIndex = i;
                    earMaxMinCross = minCross;
                }

                // If we still haven't found an ear, we're screwed.
                // Note: sometimes this is happening because the
                // remaining points are collinear.  Really these
                // should just be thrown out without halting triangulation.
                if (earIndex == -1)
                {
                    for (var i = 0; i < bufferSize; i++)
                    {
                        results.Add(buffer[i]);
                    }

                    return results;
                }

                // Clip off the ear:
                // - remove the ear tip from the list

                --vNum;
                var newx = new float[vNum];
                var newy = new float[vNum];
                var currDest = 0;
                for (var i = 0; i < vNum; ++i)
                {
                    if (currDest == earIndex)
                        ++currDest;
                    newx[i] = xrem[currDest];
                    newy[i] = yrem[currDest];
                    ++currDest;
                }

                // - add the clipped triangle to the triangle list
                var under = earIndex == 0 ? vNum : earIndex - 1;
                var over = earIndex == vNum ? 0 : earIndex + 1;
                var toAdd = new Triangle(xrem[earIndex], yrem[earIndex], xrem[over], yrem[over], xrem[under],
                    yrem[under]);
                buffer[bufferSize] = toAdd;
                ++bufferSize;

                // - replace the old list with the new one
                xrem = newx;
                yrem = newy;
            }

            var tooAdd = new Triangle(xrem[1], yrem[1], xrem[2], yrem[2], xrem[0], yrem[0]);
            buffer[bufferSize] = tooAdd;
            ++bufferSize;

            for (var i = 0; i < bufferSize; i++)
            {
                results.Add(new Vertices(buffer[i]));
            }

            return results;
        }

        /// <summary>
        /// Finds and fixes "pinch points," points where two polygon vertices are at the same point. If a pinch point is
        /// found, pin is broken up into poutA and poutB and true is returned; otherwise, returns false. Mostly for internal use.
        /// O(N^2) time, which sucks...
        /// </summary>
        /// <param name="pin">The pin.</param>
        /// <param name="poutA">The pout A.</param>
        /// <param name="poutB">The pout B.</param>
        /// <param name="tolerance"></param>
        private static bool ResolvePinchPoint(Vertices pin, out Vertices poutA, out Vertices poutB, float tolerance)
        {
            poutA = [];
            poutB = [];

            if (pin.Count < 3)
                return false;

            var hasPinchPoint = false;
            var pinchIndexA = -1;
            var pinchIndexB = -1;
            for (var i = 0; i < pin.Count; ++i)
            {
                for (var j = i + 1; j < pin.Count; ++j)
                {
                    //Don't worry about pinch points where the points
                    //are actually just dupe neighbors
                    if (Math.Abs(pin[i].X - pin[j].X) < tolerance && Math.Abs(pin[i].Y - pin[j].Y) < tolerance && j != i + 1)
                    {
                        pinchIndexA = i;
                        pinchIndexB = j;
                        hasPinchPoint = true;
                        break;
                    }
                }

                if (hasPinchPoint)
                    break;
            }

            if (hasPinchPoint)
            {
                var sizeA = pinchIndexB - pinchIndexA;
                if (sizeA == pin.Count)
                    return false; //has dupe points at wraparound, not a problem here
                for (var i = 0; i < sizeA; ++i)
                {
                    var ind = Remainder(pinchIndexA + i, pin.Count); // is this right
                    poutA.Add(pin[ind]);
                }

                var sizeB = pin.Count - sizeA;
                for (var i = 0; i < sizeB; ++i)
                {
                    var ind = Remainder(pinchIndexB + i, pin.Count); // is this right    
                    poutB.Add(pin[ind]);
                }
            }

            return hasPinchPoint;
        }

        /// <summary>Fix for obnoxious behavior for the % operator for negative numbers...</summary>
        /// <param name="x">The x.</param>
        /// <param name="modulus">The modulus.</param>
        /// <returns></returns>
        private static int Remainder(int x, int modulus)
        {
            var rem = x % modulus;
            while (rem < 0)
            {
                rem += modulus;
            }

            return rem;
        }

        /// <summary>Checks if vertex i is the tip of an ear in polygon defined by xv[] and  yv[].</summary>
        /// <param name="i">The i.</param>
        /// <param name="xv">The xv.</param>
        /// <param name="yv">The yv.</param>
        /// <param name="xvLength">Length of the xv.</param>
        /// <remarks>Assumes clockwise orientation of polygon.</remarks>
        /// <returns><c>true</c> if the specified i is ear; otherwise, <c>false</c>.</returns>
        private static bool IsEar(int i, float[] xv, float[] yv, int xvLength)
        {
            float dx0, dy0, dx1, dy1;
            if (i >= xvLength || i < 0 || xvLength < 3)
                return false;
            var upper = i + 1;
            var lower = i - 1;
            if (i == 0)
            {
                dx0 = xv[0] - xv[xvLength - 1];
                dy0 = yv[0] - yv[xvLength - 1];
                dx1 = xv[1] - xv[0];
                dy1 = yv[1] - yv[0];
                lower = xvLength - 1;
            }
            else if (i == xvLength - 1)
            {
                dx0 = xv[i] - xv[i - 1];
                dy0 = yv[i] - yv[i - 1];
                dx1 = xv[0] - xv[i];
                dy1 = yv[0] - yv[i];
                upper = 0;
            }
            else
            {
                dx0 = xv[i] - xv[i - 1];
                dy0 = yv[i] - yv[i - 1];
                dx1 = xv[i + 1] - xv[i];
                dy1 = yv[i + 1] - yv[i];
            }

            var cross = dx0 * dy1 - dx1 * dy0;

            if (cross > 0)
                return false;

            var myTri = new Triangle(xv[i], yv[i], xv[upper], yv[upper], xv[lower], yv[lower]);

            for (var j = 0; j < xvLength; ++j)
            {
                if (j == i || j == lower || j == upper)
                    continue;
                if (myTri.IsInside(xv[j], yv[j]))
                    return false;
            }

            return true;
        }
    }
}