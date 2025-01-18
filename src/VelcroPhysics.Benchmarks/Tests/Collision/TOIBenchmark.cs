using System.Numerics;
using BenchmarkDotNet.Attributes;
using VelcroPhysics.Benchmarks.Code;
using VelcroPhysics.Collision.Distance;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Collision.TOI;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Benchmarks.Tests.Collision
{
    public class ToiBenchmark : MeasuredBenchmark
    {
        private PolygonShape _shapeA;
        private PolygonShape _shapeB;
        private Sweep _sweepA;
        private Sweep _sweepB;

        [GlobalSetup]
        public void Setup()
        {
            _shapeA = new PolygonShape(PolygonUtils.CreateRectangle(25.0f, 5.0f), 0);
            _shapeB = new PolygonShape(PolygonUtils.CreateRectangle(2.5f, 2.5f), 0);

            _sweepA = new Sweep
            {
                C0 = new Vector2(24.0f, -60.0f),
                A0 = 2.95f
            };
            _sweepA.C = _sweepA.C0;
            _sweepA.A = _sweepA.A0;
            _sweepA.LocalCenter = Vector2.Zero;

            _sweepB = new Sweep
            {
                C0 = new Vector2(53.474274f, -50.252514f),
                A0 = 513.36676f,
                C = new Vector2(54.595478f, -51.083473f),
                A = 513.62781f,
                LocalCenter = Vector2.Zero
            };
        }

        [Benchmark]
        public void Distance()
        {
            var input = new TOIInput
            {
                ProxyA = new DistanceProxy(_shapeA, 0),
                ProxyB = new DistanceProxy(_shapeB, 0),
                SweepA = _sweepA,
                SweepB = _sweepB,
                TMax = 1.0f
            };

            TimeOfImpact.CalculateTimeOfImpact(ref input, out _);
        }
    }
}