using BenchmarkDotNet.Attributes;
using VelcroPhysics.Benchmarks.Code;
using VelcroPhysics.Benchmarks.Code.TestClasses;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Benchmarks.Tests.Shared
{
    public class PoolBenchmarks : UnmeasuredBenchmark
    {
        private readonly Pool<PoolObject> _pool;

        public PoolBenchmarks()
        {
            _pool = new Pool<PoolObject>(() => new PoolObject(), o => o.Reset(), 1000);
        }

        [Benchmark]
        public void NewObject()
        {
            for (var i = 0; i < 1000; i++)
            {
                var obj = new PoolObject
                {
                    TestInteger = 5,
                    TestString = "test"
                };
            }
        }

        [Benchmark]
        public void NewPooledObject()
        {
            for (var i = 0; i < 1000; i++)
            {
                var obj = _pool.GetFromPool();
                obj.TestInteger = 5;
                obj.TestString = "test";

                _pool.ReturnToPool(obj);
            }
        }

        [Benchmark]
        public void NewPooledObjectMany()
        {
            var many = _pool.GetManyFromPool(1000);
            foreach (var obj in many)
            {
                obj.TestInteger = 5;
                obj.TestString = "test";
                _pool.ReturnToPool(obj);
            }
        }
    }
}