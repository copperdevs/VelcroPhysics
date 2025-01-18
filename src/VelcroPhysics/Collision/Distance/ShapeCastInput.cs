using System.Numerics;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Collision.Distance
{
    /// <summary>Input parameters for b2ShapeCast</summary>
    public struct ShapeCastInput
    {
        public DistanceProxy ProxyA;
        public DistanceProxy ProxyB;
        public Transform TransformA;
        public Transform TransformB;
        public Vector2 TranslationB;
    }
}