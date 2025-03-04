﻿/*
* Velcro Physics
* Copyright (c) 2021 Ian Qvist
* 
* Original source Box2D:
* Copyright (c) 2019 Erin Catto
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

using System;
using System.Diagnostics;
using System.Numerics;
using VelcroPhysics.Collision.Narrowphase;
using VelcroPhysics.Shared.Optimization;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision.Distance
{
    /// <summary>
    /// The Gilbert–Johnson–Keerthi distance algorithm that provides the distance between shapes. Using Voronoi
    /// regions (Christer Ericson) and Barycentric coordinates.
    /// </summary>
    public static class DistanceGJK
    {
        /// <summary>
        /// The number of calls made to the ComputeDistance() function. Note: This is only activated when
        /// Settings.EnableDiagnostics = true
        /// </summary>
        [ThreadStatic]
        public static int GJKCalls;

        /// <summary>
        /// The number of iterations that was made on the last call to ComputeDistance(). Note: This is only activated
        /// when Settings.EnableDiagnostics = true
        /// </summary>
        [ThreadStatic]
        public static int GJKIters;

        /// <summary>
        /// The maximum number of iterations calls to the CompteDistance() function. Note: This is only activated when
        /// Settings.EnableDiagnostics = true
        /// </summary>
        [ThreadStatic]
        public static int GJKMaxIters;

        public static void ComputeDistance(ref DistanceInput input, out DistanceOutput output, out SimplexCache cache)
        {
            cache = new SimplexCache();

            ++GJKCalls;

            // Initialize the simplex.
            var simplex = new Simplex();
            simplex.ReadCache(ref cache, ref input.ProxyA, ref input.TransformA, ref input.ProxyB, ref input.TransformB);

            // These store the vertices of the last simplex so that we
            // can check for duplicates and prevent cycling.
            var saveA = new FixedArray3<int>();
            var saveB = new FixedArray3<int>();

            // Main iteration loop.
            var iter = 0;

            //Velcro: Moved the max iterations to settings
            while (iter < Settings.MaxGJKIterations)
            {
                // Copy simplex so we can identify duplicates.
                var saveCount = simplex.Count;
                for (var i = 0; i < saveCount; ++i)
                {
                    saveA[i] = simplex.V[i].IndexA;
                    saveB[i] = simplex.V[i].IndexB;
                }

                switch (simplex.Count)
                {
                    case 1:
                        break;
                    case 2:
                        simplex.Solve2();
                        break;
                    case 3:
                        simplex.Solve3();
                        break;
                    default:
                        Debug.Assert(false);
                        break;
                }

                // If we have 3 points, then the origin is in the corresponding triangle.
                if (simplex.Count == 3)
                    break;

                // Get search direction.
                var d = simplex.GetSearchDirection();

                // Ensure the search direction is numerically fit.
                if (d.LengthSquared() < MathConstants.Epsilon * MathConstants.Epsilon)
                {
                    // The origin is probably contained by a line segment
                    // or triangle. Thus the shapes are overlapped.

                    // We can't return zero here even though there may be overlap.
                    // In case the simplex is a point, segment, or triangle it is difficult
                    // to determine if the origin is contained in the CSO or very close to it.
                    break;
                }

                // Compute a tentative new simplex vertex using support points.
                var vertex = simplex.V[simplex.Count];
                vertex.IndexA = input.ProxyA.GetSupport(MathUtils.MulT(input.TransformA.q, -d));
                vertex.WA = MathUtils.Mul(ref input.TransformA, input.ProxyA._vertices[vertex.IndexA]);

                vertex.IndexB = input.ProxyB.GetSupport(MathUtils.MulT(input.TransformB.q, d));
                vertex.WB = MathUtils.Mul(ref input.TransformB, input.ProxyB._vertices[vertex.IndexB]);
                vertex.W = vertex.WB - vertex.WA;
                simplex.V[simplex.Count] = vertex;

                // Iteration count is equated to the number of support point calls.
                ++iter;

                ++GJKIters;

                // Check for duplicate support points. This is the main termination criteria.
                var duplicate = false;
                for (var i = 0; i < saveCount; ++i)
                {
                    if (vertex.IndexA == saveA[i] && vertex.IndexB == saveB[i])
                    {
                        duplicate = true;
                        break;
                    }
                }

                // If we found a duplicate support point we must exit to avoid cycling.
                if (duplicate)
                    break;

                // New vertex is OK and needed.
                ++simplex.Count;
            }

            GJKMaxIters = Math.Max(GJKMaxIters, iter);

            // Prepare output.
            simplex.GetWitnessPoints(out output.PointA, out output.PointB);
            output.Distance = (output.PointA - output.PointB).Length();
            output.Iterations = iter;

            // Cache the simplex.
            simplex.WriteCache(ref cache);

            // Apply radii if requested.
            if (input.UseRadii)
            {
                var rA = input.ProxyA._radius;
                var rB = input.ProxyB._radius;

                if (output.Distance > rA + rB && output.Distance > MathConstants.Epsilon)
                {
                    // Shapes are still no overlapped.
                    // Move the witness points to the outer surface.
                    output.Distance -= rA + rB;
                    var normal = Vector2.Normalize(output.PointB - output.PointA);
                    output.PointA += rA * normal;
                    output.PointB -= rB * normal;
                }
                else
                {
                    // Shapes are overlapped when radii are considered.
                    // Move the witness points to the middle.
                    var p = 0.5f * (output.PointA + output.PointB);
                    output.PointA = p;
                    output.PointB = p;
                    output.Distance = 0.0f;
                }
            }
        }

        // GJK-raycast
        // Algorithm by Gino van den Bergen.
        // "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010

        /// <summary>
        /// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and
        /// translation fraction.
        /// </summary>
        /// <returns>true if hit, false if there is no hit or an initial overlap</returns>
        public static bool ShapeCast(ref ShapeCastInput input, out ShapeCastOutput output)
        {
            output = new ShapeCastOutput
            {
                Iterations = 0,
                Lambda = 1.0f,
                Normal = Vector2.Zero,
                Point = Vector2.Zero
            };

            var proxyA = input.ProxyA;
            var proxyB = input.ProxyB;

            var radiusA = MathUtils.Max(proxyA._radius, Settings.PolygonRadius);
            var radiusB = MathUtils.Max(proxyB._radius, Settings.PolygonRadius);
            var radius = radiusA + radiusB;

            var xfA = input.TransformA;
            var xfB = input.TransformB;

            var r = input.TranslationB;
            var n = new Vector2(0.0f, 0.0f);
            var lambda = 0.0f;

            // Initial simplex
            var simplex = new Simplex
            {
                Count = 0
            };

            // Get simplex vertices as an array.
            //SimplexVertex vertices = simplex.V.Value0; //Velcro: we don't need this as we have an indexer instead

            // Get support point in -r direction
            var indexA = proxyA.GetSupport(MathUtils.MulT(xfA.q, -r));
            var wA = MathUtils.Mul(ref xfA, proxyA.GetVertex(indexA));
            var indexB = proxyB.GetSupport(MathUtils.MulT(xfB.q, r));
            var wB = MathUtils.Mul(ref xfB, proxyB.GetVertex(indexB));
            var v = wA - wB;

            // Sigma is the target distance between polygons
            var sigma = MathUtils.Max(Settings.PolygonRadius, radius - Settings.PolygonRadius);
            var tolerance = 0.5f * Settings.LinearSlop;

            // Main iteration loop.
            var iter = 0;

            //Velcro: We have moved the max iterations into settings
            while (iter < Settings.MaxGJKIterations && v.Length() - sigma > tolerance)
            {
                Debug.Assert(simplex.Count < 3);

                output.Iterations += 1;

                // Support in direction -v (A - B)
                indexA = proxyA.GetSupport(MathUtils.MulT(xfA.q, -v));
                wA = MathUtils.Mul(ref xfA, proxyA.GetVertex(indexA));
                indexB = proxyB.GetSupport(MathUtils.MulT(xfB.q, v));
                wB = MathUtils.Mul(ref xfB, proxyB.GetVertex(indexB));
                var p = wA - wB;

                // -v is a normal at p
                v = Vector2.Normalize(v);

                // Intersect ray with plane
                var vp = MathUtils.Dot(ref v, ref p);
                var vr = MathUtils.Dot(ref v, ref r);
                if (vp - sigma > lambda * vr)
                {
                    if (vr <= 0.0f)
                        return false;

                    lambda = (vp - sigma) / vr;
                    if (lambda > 1.0f)
                        return false;

                    n = -v;
                    simplex.Count = 0;
                }

                // Reverse simplex since it works with B - A.
                // Shift by lambda * r because we want the closest point to the current clip point.
                // Note that the support point p is not shifted because we want the plane equation
                // to be formed in unshifted space.
                var vertex = simplex.V[simplex.Count];
                vertex.IndexA = indexB;
                vertex.WA = wB + lambda * r;
                vertex.IndexB = indexA;
                vertex.WB = wA;
                vertex.W = vertex.WB - vertex.WA;
                vertex.A = 1.0f;
                simplex.V[simplex.Count] = vertex; //Velcro: we have to copy the value back
                simplex.Count += 1;

                switch (simplex.Count)
                {
                    case 1:
                        break;

                    case 2:
                        simplex.Solve2();
                        break;

                    case 3:
                        simplex.Solve3();
                        break;

                    default:
                        Debug.Assert(false);
                        break;
                }

                // If we have 3 points, then the origin is in the corresponding triangle.
                if (simplex.Count == 3)
                {
                    // Overlap
                    return false;
                }

                // Get search direction.
                v = simplex.GetClosestPoint();

                // Iteration count is equated to the number of support point calls.
                ++iter;
            }

            if (iter == 0)
            {
                // Initial overlap
                return false;
            }

            // Prepare output.
            simplex.GetWitnessPoints(out _, out var pointB);

            if (v.LengthSquared() > 0.0f)
            {
                n = -v;
                n = Vector2.Normalize(n);
            }

            output.Point = pointB + radiusA * n;
            output.Normal = n;
            output.Lambda = lambda;
            output.Iterations = iter;
            return true;
        }
    }
}