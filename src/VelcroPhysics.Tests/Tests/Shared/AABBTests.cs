using System.Numerics;
using VelcroPhysics.Shared;
using Xunit;

namespace VelcroPhysics.Tests.Tests.Shared
{
    public class AABBTests
    {
        [Fact]
        public void TestOverlap()
        {
            {
                var bb1 = new AABB(new Vector2(-2, -3), new Vector2(-1, 0));
                Assert.True(AABB.TestOverlap(ref bb1, ref bb1));
            }
            {
                var vec = new Vector2(-2, -3);
                var bb1 = new AABB(vec, vec);
                Assert.True(AABB.TestOverlap(ref bb1, ref bb1));
            }
            {
                var bb1 = new AABB(new Vector2(-2, -3), new Vector2(-1, 0));
                var bb2 = new AABB(new Vector2(-1, -1), new Vector2(1, 2));
                Assert.True(AABB.TestOverlap(ref bb1, ref bb2));
            }
            {
                var bb1 = new AABB(new Vector2(-99, -3), new Vector2(-1, 0));
                var bb2 = new AABB(new Vector2(76, -1), new Vector2(-2, 2));
                Assert.True(AABB.TestOverlap(ref bb1, ref bb2));
            }
            {
                var bb1 = new AABB(new Vector2(-20, -3), new Vector2(-18, 0));
                var bb2 = new AABB(new Vector2(-1, -1), new Vector2(1, 2));
                Assert.False(AABB.TestOverlap(ref bb1, ref bb2));
            }
            {
                var bb1 = new AABB(new Vector2(-2, -3), new Vector2(-1, 0));
                var bb2 = new AABB(new Vector2(-1, +1), new Vector2(1, 2));
                Assert.False(AABB.TestOverlap(ref bb1, ref bb2));
            }
            {
                var bb1 = new AABB(new Vector2(-2, +3), new Vector2(-1, 0));
                var bb2 = new AABB(new Vector2(-1, -1), new Vector2(0, -2));
                Assert.False(AABB.TestOverlap(ref bb1, ref bb2));
            }
        }
    }
}