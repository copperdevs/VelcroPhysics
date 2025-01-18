using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Definitions.Shapes
{
    public sealed class PolygonShapeDef : ShapeDef
    {
        public PolygonShapeDef() : base(ShapeType.Polygon)
        {
            SetDefaults();
        }

        public Vertices Vertices { get; set; }

        public override void SetDefaults()
        {
            Vertices = null;
            base.SetDefaults();
        }
    }
}