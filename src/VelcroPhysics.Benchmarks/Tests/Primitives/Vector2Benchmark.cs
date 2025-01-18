using BenchmarkDotNet.Attributes;
using VelcroPhysics.Benchmarks.Code;
using Vector2 = System.Numerics.Vector2;

namespace VelcroPhysics.Benchmarks.Tests.Primitives
{
    public class Vector2Benchmark : UnmeasuredBenchmark
    {
        [Benchmark]
        public Vector2 CreateWithCtor()
        {
            return new Vector2(1, 2);
        }

        [Benchmark]
        public Vector2 CreateWithNoInit()
        {
            Vector2 v;
            v.X = 1;
            v.Y = 2;
            return v;
        }

        [Benchmark]
        public Vector2 CreateWithZero()
        {
            var v = Vector2.Zero;
            v.X = 1;
            v.Y = 2;
            return v;
        }
    }
}