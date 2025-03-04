using System;
using System.Diagnostics;
using System.Numerics;
using VelcroPhysics.Collision.Distance;
using VelcroPhysics.Shared;
using VelcroPhysics.Shared.Optimization;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision.Narrowphase
{
    internal struct Simplex
    {
        internal int Count;
        internal FixedArray3<SimplexVertex> V;

        internal void ReadCache(ref SimplexCache cache, ref DistanceProxy proxyA, ref Transform transformA, ref DistanceProxy proxyB, ref Transform transformB)
        {
            Debug.Assert(cache.Count <= 3);

            // Copy data from cache.
            Count = cache.Count;
            for (var i = 0; i < Count; ++i)
            {
                var v = V[i];
                v.IndexA = cache.IndexA[i];
                v.IndexB = cache.IndexB[i];
                var wALocal = proxyA._vertices[v.IndexA];
                var wBLocal = proxyB._vertices[v.IndexB];
                v.WA = MathUtils.Mul(ref transformA, wALocal);
                v.WB = MathUtils.Mul(ref transformB, wBLocal);
                v.W = v.WB - v.WA;
                v.A = 0.0f;
                V[i] = v;
            }

            // Compute the new simplex metric, if it is substantially different than
            // old metric then flush the simplex.
            if (Count > 1)
            {
                var metric1 = cache.Metric;
                var metric2 = GetMetric();
                if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < MathConstants.Epsilon)
                {
                    // Reset the simplex.
                    Count = 0;
                }
            }

            // If the cache is empty or invalid ...
            if (Count == 0)
            {
                var v = V[0];
                v.IndexA = 0;
                v.IndexB = 0;
                var wALocal = proxyA._vertices[0];
                var wBLocal = proxyB._vertices[0];
                v.WA = MathUtils.Mul(ref transformA, wALocal);
                v.WB = MathUtils.Mul(ref transformB, wBLocal);
                v.W = v.WB - v.WA;
                v.A = 1.0f;
                V[0] = v;
                Count = 1;
            }
        }

        internal void WriteCache(ref SimplexCache cache)
        {
            cache.Metric = GetMetric();
            cache.Count = (ushort)Count;
            for (var i = 0; i < Count; ++i)
            {
                cache.IndexA[i] = (byte)V[i].IndexA;
                cache.IndexB[i] = (byte)V[i].IndexB;
            }
        }

        internal Vector2 GetSearchDirection()
        {
            switch (Count)
            {
                case 1:
                    return -V[0].W;

                case 2:
                    {
                        var e12 = V[1].W - V[0].W;
                        var sgn = MathUtils.Cross(e12, -V[0].W);
                        if (sgn > 0.0f)
                        {
                            // Origin is left of e12.
                            return MathUtils.Cross(1.0f, e12);
                        }

                        // Origin is right of e12.
                        return MathUtils.Cross(e12, 1.0f);
                    }

                default:
                    Debug.Assert(false);
                    return Vector2.Zero;
            }
        }

        internal Vector2 GetClosestPoint()
        {
            switch (Count)
            {
                case 0:
                    Debug.Assert(false);
                    return Vector2.Zero;

                case 1:
                    return V[0].W;

                case 2:
                    return V[0].A * V[0].W + V[1].A * V[1].W;

                case 3:
                    return Vector2.Zero;

                default:
                    Debug.Assert(false);
                    return Vector2.Zero;
            }
        }

        internal void GetWitnessPoints(out Vector2 pA, out Vector2 pB)
        {
            switch (Count)
            {
                case 0:
                    pA = Vector2.Zero;
                    pB = Vector2.Zero;
                    Debug.Assert(false);
                    break;

                case 1:
                    pA = V[0].WA;
                    pB = V[0].WB;
                    break;

                case 2:
                    pA = V[0].A * V[0].WA + V[1].A * V[1].WA;
                    pB = V[0].A * V[0].WB + V[1].A * V[1].WB;
                    break;

                case 3:
                    pA = V[0].A * V[0].WA + V[1].A * V[1].WA + V[2].A * V[2].WA;
                    pB = pA;
                    break;

                default:
                    throw new Exception();
            }
        }

        internal float GetMetric()
        {
            switch (Count)
            {
                case 0:
                    Debug.Assert(false);
                    return 0.0f;
                case 1:
                    return 0.0f;

                case 2:
                    return MathUtils.Distance(V[0].W, V[1].W);

                case 3:
                    return MathUtils.Cross(V[1].W - V[0].W, V[2].W - V[0].W);

                default:
                    Debug.Assert(false);
                    return 0.0f;
            }
        }

        // Solve a line segment using barycentric coordinates.
        //
        // p = a1 * w1 + a2 * w2
        // a1 + a2 = 1
        //
        // The vector from the origin to the closest point on the line is
        // perpendicular to the line.
        // e12 = w2 - w1
        // dot(p, e) = 0
        // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
        //
        // 2-by-2 linear system
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        //
        // Define
        // d12_1 =  dot(w2, e12)
        // d12_2 = -dot(w1, e12)
        // d12 = d12_1 + d12_2
        //
        // Solution
        // a1 = d12_1 / d12
        // a2 = d12_2 / d12

        internal void Solve2()
        {
            var w1 = V[0].W;
            var w2 = V[1].W;
            var e12 = w2 - w1;

            // w1 region
            var d12_2 = -Vector2.Dot(w1, e12);
            if (d12_2 <= 0.0f)
            {
                // a2 <= 0, so we clamp it to 0
                V.Value0.A = 1.0f;
                Count = 1;
                return;
            }

            // w2 region
            var d12_1 = Vector2.Dot(w2, e12);
            if (d12_1 <= 0.0f)
            {
                // a1 <= 0, so we clamp it to 0
                V.Value1.A = 1.0f;
                Count = 1;
                V.Value0 = V.Value1;
                return;
            }

            // Must be in e12 region.
            var inv_d12 = 1.0f / (d12_1 + d12_2);
            V.Value0.A = d12_1 * inv_d12;
            V.Value1.A = d12_2 * inv_d12;
            Count = 2;
        }

        // Possible regions:
        // - points[2]
        // - edge points[0]-points[2]
        // - edge points[1]-points[2]
        // - inside the triangle
        internal void Solve3()
        {
            var w1 = V[0].W;
            var w2 = V[1].W;
            var w3 = V[2].W;

            // Edge12
            // [1      1     ][a1] = [1]
            // [w1.e12 w2.e12][a2] = [0]
            // a3 = 0
            var e12 = w2 - w1;
            var w1e12 = Vector2.Dot(w1, e12);
            var w2e12 = Vector2.Dot(w2, e12);
            var d12_2 = -w1e12;

            // Edge13
            // [1      1     ][a1] = [1]
            // [w1.e13 w3.e13][a3] = [0]
            // a2 = 0
            var e13 = w3 - w1;
            var w1e13 = Vector2.Dot(w1, e13);
            var w3e13 = Vector2.Dot(w3, e13);
            var d13_2 = -w1e13;

            // Edge23
            // [1      1     ][a2] = [1]
            // [w2.e23 w3.e23][a3] = [0]
            // a1 = 0
            var e23 = w3 - w2;
            var w2e23 = Vector2.Dot(w2, e23);
            var w3e23 = Vector2.Dot(w3, e23);
            var d23_2 = -w2e23;

            // Triangle123
            var n123 = MathUtils.Cross(e12, e13);

            var d123_1 = n123 * MathUtils.Cross(w2, w3);
            var d123_2 = n123 * MathUtils.Cross(w3, w1);
            var d123_3 = n123 * MathUtils.Cross(w1, w2);

            // w1 region
            if (d12_2 <= 0.0f && d13_2 <= 0.0f)
            {
                V.Value0.A = 1.0f;
                Count = 1;
                return;
            }

            // e12
            if (w2e12 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
            {
                var inv_d12 = 1.0f / (w2e12 + d12_2);
                V.Value0.A = w2e12 * inv_d12;
                V.Value1.A = d12_2 * inv_d12;
                Count = 2;
                return;
            }

            // e13
            if (w3e13 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
            {
                var inv_d13 = 1.0f / (w3e13 + d13_2);
                V.Value0.A = w3e13 * inv_d13;
                V.Value2.A = d13_2 * inv_d13;
                Count = 2;
                V.Value1 = V.Value2;
                return;
            }

            // w2 region
            if (w2e12 <= 0.0f && d23_2 <= 0.0f)
            {
                V.Value1.A = 1.0f;
                Count = 1;
                V.Value0 = V.Value1;
                return;
            }

            // w3 region
            if (w3e13 <= 0.0f && w3e23 <= 0.0f)
            {
                V.Value2.A = 1.0f;
                Count = 1;
                V.Value0 = V.Value2;
                return;
            }

            // e23
            if (w3e23 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
            {
                var inv_d23 = 1.0f / (w3e23 + d23_2);
                V.Value1.A = w3e23 * inv_d23;
                V.Value2.A = d23_2 * inv_d23;
                Count = 2;
                V.Value0 = V.Value2;
                return;
            }

            // Must be in triangle123
            var inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
            V.Value0.A = d123_1 * inv_d123;
            V.Value1.A = d123_2 * inv_d123;
            V.Value2.A = d123_3 * inv_d123;
            Count = 3;
        }
    }
}