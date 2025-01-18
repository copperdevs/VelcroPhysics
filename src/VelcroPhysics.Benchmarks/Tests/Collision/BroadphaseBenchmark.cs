using System;
using System.Numerics;
using BenchmarkDotNet.Attributes;
using VelcroPhysics.Benchmarks.Code;
using VelcroPhysics.Benchmarks.Utilities;
using VelcroPhysics.Collision.Broadphase;
using VelcroPhysics.Collision.RayCast;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Benchmarks.Tests.Collision
{
    public class BroadphaseBenchmark : MeasuredBenchmark
    {
        private Actor[] _actors;
        private float _proxyExtent;
        private AABB _queryAABB;
        private Random _random;
        private RayCastInput _rayCastInput;

        private DynamicTree<Actor> _tree;
        private float _worldExtent;

        [GlobalSetup]
        public void Setup()
        {
            _worldExtent = 15.0f;
            _proxyExtent = 0.5f;

            _random = new Random(888);

            var h = _worldExtent;
            _queryAABB.LowerBound = new Vector2(-3.0f, -4.0f + h);
            _queryAABB.UpperBound = new Vector2(5.0f, 6.0f + h);

            _rayCastInput.Point1 = new Vector2(-5.0f, 5.0f + h);
            _rayCastInput.Point2 = new Vector2(7.0f, -4.0f + h);
            _rayCastInput.MaxFraction = 1.0f;

            _tree = new DynamicTree<Actor>();
            _actors = new Actor[100];

            for (var i = 0; i < _actors.Length; i++)
            {
                var a = new Actor();
                GetRandomAABB(out var randAabb);
                a.AABB = randAabb;
                _actors[i] = a;
            }
        }

        [Benchmark]
        public void CreateMoveDelete()
        {
            for (var i = 0; i < _actors.Length; i++)
            {
                _actors[i].ProxyId = _tree.CreateProxy(ref _actors[i].AABB, null);
            }

            foreach (var a in _actors)
            {
                if (a.ProxyId == -1)
                    continue;

                var aabb0 = a.AABB;
                RandomMoveAABB(ref a.AABB);
                var displacement = a.AABB.Center - aabb0.Center;
                _tree.MoveProxy(a.ProxyId, ref a.AABB, displacement);
            }

            foreach (var a in _actors)
            {
                if (a.ProxyId == -1)
                    continue;

                _tree.DestroyProxy(a.ProxyId);
            }
        }

        [Benchmark]
        public void Query()
        {
            _tree.Query(QueryCallback, ref _queryAABB);
        }

        [Benchmark]
        public void RayCast()
        {
            var input = _rayCastInput;
            _tree.RayCast(RayCastCallback, ref input);
        }

        private bool QueryCallback(int proxyId)
        {
            var actor = _tree.GetUserData(proxyId);
            AABB.TestOverlap(ref _queryAABB, ref actor.AABB);
            return true;
        }

        private float RayCastCallback(RayCastInput input, int proxyId)
        {
            var actor = _tree.GetUserData(proxyId);

            RayCastOutput output;
            var hit = actor.AABB.RayCast(ref input, out output);
            return hit ? output.Fraction : input.MaxFraction;
        }

        private void GetRandomAABB(out AABB aabb)
        {
            aabb = new AABB();

            var w = new Vector2(2.0f * _proxyExtent, 2.0f * _proxyExtent);
            aabb.LowerBound.X = _random.RandomFloat(-_worldExtent, _worldExtent);
            aabb.LowerBound.Y = _random.RandomFloat(0.0f, 2.0f * _worldExtent);
            aabb.UpperBound = aabb.LowerBound + w;
        }

        private void RandomMoveAABB(ref AABB aabb)
        {
            var d = Vector2.Zero;
            d.X = _random.RandomFloat(-0.5f, 0.5f);
            d.Y = _random.RandomFloat(-0.5f, 0.5f);

            aabb.LowerBound += d;
            aabb.UpperBound += d;

            var c0 = 0.5f * (aabb.LowerBound + aabb.UpperBound);
            var min = new Vector2(-_worldExtent, 0.0f);
            var max = new Vector2(_worldExtent, 2.0f * _worldExtent);
            var c = Vector2.Clamp(c0, min, max);

            aabb.LowerBound += c - c0;
            aabb.UpperBound += c - c0;
        }

        private sealed class Actor
        {
            internal AABB AABB;
            internal int ProxyId;
        }
    }
}