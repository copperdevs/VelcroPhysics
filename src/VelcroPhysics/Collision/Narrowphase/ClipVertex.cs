using System.Numerics;
using VelcroPhysics.Collision.ContactSystem;

namespace VelcroPhysics.Collision.Narrowphase
{
    /// <summary>Used for computing contact manifolds.</summary>
    internal struct ClipVertex
    {
        public ContactId Id;
        public Vector2 V;
    }
}