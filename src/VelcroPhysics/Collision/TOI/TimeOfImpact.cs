/*
* Velcro Physics:
* Copyright (c) 2017 Ian Qvist
* 
* Original source Box2D:
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org 
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
using System.Diagnostics;
using VelcroPhysics.Collision.Distance;

namespace VelcroPhysics.Collision.TOI
{
    public static class TimeOfImpact
    {
        // CCD via the local separating axis method. This seeks progression
        // by computing the largest time at which separation is maintained.

        [ThreadStatic]
        public static int TOICalls,
                          TOIIters,
                          TOIMaxIters;

        [ThreadStatic]
        public static int TOIRootIters,
                          TOIMaxRootIters;

        /// <summary>
        /// Compute the upper bound on time before two shapes penetrate. Time is represented as a fraction between
        /// [0,tMax]. This uses a swept separating axis and may miss some intermediate, non-tunneling collision. If you change the
        /// time interval, you should call this function again. Note: use Distance() to compute the contact point and normal at the
        /// time of impact.
        /// </summary>
        /// <param name="input">The input.</param>
        /// <param name="output">The output.</param>
        public static void CalculateTimeOfImpact(ref TOIInput input, out TOIOutput output)
        {
            ++TOICalls;

            output = new TOIOutput
            {
                State = TOIOutputState.Unknown,
                T = input.TMax
            };

            var sweepA = input.SweepA;
            var sweepB = input.SweepB;

            // Large rotations can make the root finder fail, so we normalize the
            // sweep angles.
            sweepA.Normalize();
            sweepB.Normalize();

            var tMax = input.TMax;

            var totalRadius = input.ProxyA._radius + input.ProxyB._radius;
            var target = Math.Max(Settings.LinearSlop, totalRadius - 3.0f * Settings.LinearSlop);
            var tolerance = 0.25f * Settings.LinearSlop;
            Debug.Assert(target > tolerance);

            var t1 = 0.0f;
            const int k_maxIterations = 20;
            var iter = 0;

            // Prepare input for distance query.
            var distanceInput = new DistanceInput
            {
                ProxyA = input.ProxyA,
                ProxyB = input.ProxyB,
                UseRadii = false
            };

            // The outer loop progressively attempts to compute new separating axes.
            // This loop terminates when an axis is repeated (no progress is made).
            for (;;)
            {
                sweepA.GetTransform(out var xfA, t1);
                sweepB.GetTransform(out var xfB, t1);

                // Get the distance between shapes. We can also use the results
                // to get a separating axis.
                distanceInput.TransformA = xfA;
                distanceInput.TransformB = xfB;
                DistanceGJK.ComputeDistance(ref distanceInput, out var distanceOutput, out var cache);

                // If the shapes are overlapped, we give up on continuous collision.
                if (distanceOutput.Distance <= 0.0f)
                {
                    // Failure!
                    output.State = TOIOutputState.Overlapped;
                    output.T = 0.0f;
                    break;
                }

                if (distanceOutput.Distance < target + tolerance)
                {
                    // Victory!
                    output.State = TOIOutputState.Touching;
                    output.T = t1;
                    break;
                }

                SeparationFunction.Initialize(ref cache, input.ProxyA, ref sweepA, input.ProxyB, ref sweepB, t1, out var axis, out var localPoint, out var type);

                // Compute the TOI on the separating axis. We do this by successively
                // resolving the deepest point. This loop is bounded by the number of vertices.
                var done = false;
                var t2 = tMax;
                var pushBackIter = 0;
                for (;;)
                {
                    // Find the deepest point at t2. Store the witness point indices.
                    var s2 = SeparationFunction.FindMinSeparation(out var indexA, out var indexB, t2, input.ProxyA, ref sweepA, input.ProxyB, ref sweepB, ref axis, ref localPoint, type);

                    // Is the final configuration separated?
                    if (s2 > target + tolerance)
                    {
                        // Victory!
                        output.State = TOIOutputState.Seperated;
                        output.T = tMax;
                        done = true;
                        break;
                    }

                    // Has the separation reached tolerance?
                    if (s2 > target - tolerance)
                    {
                        // Advance the sweeps
                        t1 = t2;
                        break;
                    }

                    // Compute the initial separation of the witness points.
                    var s1 = SeparationFunction.Evaluate(indexA, indexB, t1, input.ProxyA, ref sweepA, input.ProxyB, ref sweepB, ref axis, ref localPoint, type);

                    // Check for initial overlap. This might happen if the root finder
                    // runs out of iterations.
                    if (s1 < target - tolerance)
                    {
                        output.State = TOIOutputState.Failed;
                        output.T = t1;
                        done = true;
                        break;
                    }

                    // Check for touching
                    if (s1 <= target + tolerance)
                    {
                        // Victory! t1 should hold the TOI (could be 0.0).
                        output.State = TOIOutputState.Touching;
                        output.T = t1;
                        done = true;
                        break;
                    }

                    // Compute 1D root of: f(x) - target = 0
                    var rootIterCount = 0;
                    float a1 = t1, a2 = t2;
                    for (;;)
                    {
                        // Use a mix of the secant rule and bisection.
                        float t;
                        if ((rootIterCount & 1) != 0)
                        {
                            // Secant rule to improve convergence.
                            t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                        }
                        else
                        {
                            // Bisection to guarantee progress.
                            t = 0.5f * (a1 + a2);
                        }

                        ++rootIterCount;
                            ++TOIRootIters;

                        var s = SeparationFunction.Evaluate(indexA, indexB, t, input.ProxyA, ref sweepA, input.ProxyB, ref sweepB, ref axis, ref localPoint, type);

                        if (Math.Abs(s - target) < tolerance)
                        {
                            // t2 holds a tentative value for t1
                            t2 = t;
                            break;
                        }

                        // Ensure we continue to bracket the root.
                        if (s > target)
                        {
                            a1 = t;
                            s1 = s;
                        }
                        else
                        {
                            a2 = t;
                            s2 = s;
                        }

                        if (rootIterCount == 50)
                            break;
                    }

                    TOIMaxRootIters = Math.Max(TOIMaxRootIters, rootIterCount);

                    ++pushBackIter;

                    if (pushBackIter == Settings.MaxPolygonVertices)
                        break;
                }

                ++iter;
                ++TOIIters;

                if (done)
                    break;

                if (iter == k_maxIterations)
                {
                    // Root finder got stuck. Semi-victory.
                    output.State = TOIOutputState.Failed;
                    output.T = t1;
                    break;
                }
            }

            TOIMaxIters = Math.Max(TOIMaxIters, iter);
        }
    }
}