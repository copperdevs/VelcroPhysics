using System;
using System.Numerics;
using VelcroPhysics.Collision.TOI;
using Xunit;

namespace VelcroPhysics.Tests.Tests
{
    public class MathTest
    {
        [Fact]
        public void SweepGetTransform()
        {
            // From issue https://github.com/erincatto/box2d/issues/447
            var sweep = new Sweep
            {
                LocalCenter = Vector2.Zero,
                C0 = new Vector2(-2.0f, 4.0f),
                C = new Vector2(3.0f, 8.0f),
                A0 = 0.5f,
                A = 5.0f,
                Alpha0 = 0.0f
            };

            sweep.GetTransform(out var transform, 0.0f);
            Assert.Equal(transform.p.X, sweep.C0.X);
            Assert.Equal(transform.p.Y, sweep.C0.Y);
            Assert.Equal(transform.q.c, (float)Math.Cos(sweep.A0));
            Assert.Equal(transform.q.s, (float)Math.Sin(sweep.A0));

            sweep.GetTransform(out transform, 1.0f);
            Assert.Equal(transform.p.X, sweep.C.X);
            Assert.Equal(transform.p.Y, sweep.C.Y);
            Assert.Equal(transform.q.c, (float)Math.Cos(sweep.A));
            Assert.Equal(transform.q.s, (float)Math.Sin(sweep.A));
        }
    }
}