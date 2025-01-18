using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Dynamics.Joints;
using VelcroPhysics.Primitives;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Interfaces
{
    public interface IDebugView
    {
        void DrawJoint(Joint joint);
        void DrawShape(Shape shape, ref Transform transform, Color color);
    }
}