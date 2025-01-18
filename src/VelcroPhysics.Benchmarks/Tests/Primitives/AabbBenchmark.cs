using System.Numerics;
using BenchmarkDotNet.Attributes;
using VelcroPhysics.Benchmarks.Code;
using VelcroPhysics.Collision.RayCast;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Benchmarks.Tests.Primitives
{
    public class AabbBenchmark : UnmeasuredBenchmark
    {
        private AABB _a;
        private AABB _b;
        private AABB _c;

        public AabbBenchmark()
        {
            _a = new AABB(new Vector2(100, 100), 10, 10);
            _b = new AABB(new Vector2(95, 95), 10, 10); //A and B overlap
            _c = new AABB(Vector2.Zero, 10, 10);
        }

        [Benchmark]
        public bool TestOverlap()
        {
            return AABB.TestOverlap(ref _a, ref _b);
        }

        [Benchmark]
        public bool TestNoOverlap()
        {
            return AABB.TestOverlap(ref _a, ref _c);
        }

        [Benchmark]
        public bool RayCast()
        {
            var input = new RayCastInput
            {
                Point1 = Vector2.Zero,
                Point2 = _a.Center
            };

            return _a.RayCast(ref input, out _);
        }


        [Benchmark]
        public AABB ComputeCircleAABBBenchmarkNormal()
        {
            var pos = new Vector2(1, 1);
            var radius = 4f;
            var tf = new Transform();

            AABB aabb;
            var p = tf.p + MathUtils.Mul(ref tf.q, pos);
            aabb.LowerBound = new Vector2(p.X - radius, p.Y - radius);
            aabb.UpperBound = new Vector2(p.X + radius, p.Y + radius);
            return aabb;
        }

        [Benchmark]
        public AABB ComputeCircleAABBBenchmarkOptimized()
        {
            var pos = new Vector2(1, 1);
            var radius = 4f;
            var tf = new Transform();

            AABB aabb;
            var posX = tf.p.X + (tf.q.c * pos.X - tf.q.s * pos.Y);
            var posY = tf.p.Y + (tf.q.s * pos.X + tf.q.c * pos.Y);

            aabb.LowerBound.X = posX - radius;
            aabb.LowerBound.Y = posY - radius;
            aabb.UpperBound.X = posX + radius;
            aabb.UpperBound.Y = posY + radius;

            return aabb;
        }
    }
}