using System.Collections.Generic;

namespace VelcroPhysics.Extensions.PhysicsLogics.Explosion
{
    /// <summary>This is a comparer used for detecting angle difference between rays</summary>
    internal class RayDataComparer : IComparer<float>
    {
        int IComparer<float>.Compare(float a, float b)
        {
            var diff = a - b;
            if (diff > 0)
                return 1;
            if (diff < 0)
                return -1;
            return 0;
        }
    }
}