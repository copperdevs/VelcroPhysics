using System.Diagnostics;
using System.Numerics;
using VelcroPhysics.Collision.Narrowphase;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Dynamics.Solver
{
    public static class PositionSolverManifold
    {
        public static void Initialize(ContactPositionConstraint pc, ref Transform xfA, ref Transform xfB, int index, out Vector2 normal, out Vector2 point, out float separation)
        {
            Debug.Assert(pc.PointCount > 0);

            switch (pc.Type)
            {
                case ManifoldType.Circles:
                {
                    var pointA = MathUtils.Mul(ref xfA, pc.LocalPoint);
                    var pointB = MathUtils.Mul(ref xfB, pc.LocalPoints[0]);
                    normal = pointB - pointA;

                    //Velcro: Fix to handle zero normalization
                    if (normal != Vector2.Zero)
                        normal = Vector2.Normalize(normal);


                    point = 0.5f * (pointA + pointB);
                    separation = Vector2.Dot(pointB - pointA, normal) - pc.RadiusA - pc.RadiusB;
                }
                    break;

                case ManifoldType.FaceA:
                {
                    normal = MathUtils.Mul(xfA.q, pc.LocalNormal);
                    var planePoint = MathUtils.Mul(ref xfA, pc.LocalPoint);

                    var clipPoint = MathUtils.Mul(ref xfB, pc.LocalPoints[index]);
                    separation = Vector2.Dot(clipPoint - planePoint, normal) - pc.RadiusA - pc.RadiusB;
                    point = clipPoint;
                }
                    break;

                case ManifoldType.FaceB:
                {
                    normal = MathUtils.Mul(xfB.q, pc.LocalNormal);
                    var planePoint = MathUtils.Mul(ref xfB, pc.LocalPoint);

                    var clipPoint = MathUtils.Mul(ref xfA, pc.LocalPoints[index]);
                    separation = Vector2.Dot(clipPoint - planePoint, normal) - pc.RadiusA - pc.RadiusB;
                    point = clipPoint;

                    // Ensure normal points from A to B
                    normal = -normal;
                }
                    break;
                default:
                    normal = Vector2.Zero;
                    point = Vector2.Zero;
                    separation = 0;
                    break;
            }
        }
    }
}