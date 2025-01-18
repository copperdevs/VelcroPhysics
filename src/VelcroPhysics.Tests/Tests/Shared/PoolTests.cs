using System.Linq;
using VelcroPhysics.Shared;
using VelcroPhysics.Tests.Code;
using Xunit;

namespace VelcroPhysics.Tests.Tests.Shared
{
    public class PoolTests
    {
        [Fact]
        public void GetWhileAdding()
        {
            var pool = new Pool<PoolObject>(() => new PoolObject(), x => x.Reset(), 1);

            var first = true;

            //We use the fact that it is lazy loaded to re-add the same item
            var many = pool.GetManyFromPool(10);
            foreach (var obj in many)
            {
                if (first)
                {
                    Assert.True(obj.IsNew);
                    first = false;
                }
                else
                    Assert.False(obj.IsNew);

                pool.ReturnToPool(obj);
            }
        }

        [Fact]
        public void GetManyAcrossBoundary()
        {
            var pool = new Pool<PoolObject>(() => new PoolObject(), null, 6);

            //We get twice as many as in pool
            var many = pool.GetManyFromPool(12).ToList();
            foreach (var obj in many)
            {
                Assert.True(obj.IsNew);
            }

            Assert.Equal(12, many.Count);
        }

        [Fact]
        public void GetManyNewAndPooled()
        {
            var pool = new Pool<PoolObject>(() => new PoolObject(), x => x.Reset(), 10);

            //Empty whole pool
            var many = pool.GetManyFromPool(10).ToList();

            foreach (var obj in many)
            {
                Assert.True(obj.IsNew);
            }

            Assert.Equal(10, many.Count);

            many = pool.GetManyFromPool(10).ToList();
            foreach (var obj in many)
            {
                Assert.True(obj.IsNew);
                pool.ReturnToPool(obj);
            }

            Assert.Equal(10, many.Count);

            many = pool.GetManyFromPool(10).ToList();
            foreach (var obj in many)
            {
                Assert.False(obj.IsNew);
            }

            Assert.Equal(10, many.Count);
        }

        [Fact]
        public void GetOnePooled()
        {
            var pool = new Pool<PoolObject>(() => new PoolObject(), x => x.Reset(), 1);
            var obj = pool.GetFromPool();

            Assert.True(obj.IsNew);

            pool.ReturnToPool(obj);
            obj = pool.GetFromPool();

            Assert.False(obj.IsNew);
        }

        [Fact]
        public void GetOneNew()
        {
            var pool = new Pool<PoolObject>(() => new PoolObject(), x => x.Reset(), 0);
            var obj = pool.GetFromPool();

            Assert.True(obj.IsNew);

            obj = pool.GetFromPool();
            Assert.True(obj.IsNew);
        }
    }
}