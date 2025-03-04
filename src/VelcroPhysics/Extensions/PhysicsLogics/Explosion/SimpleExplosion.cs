﻿using System;
using System.Collections.Generic;
using System.Numerics;
using Microsoft.Xna.Framework;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Extensions.PhysicsLogics.PhysicsLogicBase;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Extensions.PhysicsLogics.Explosion
{
    /// <summary>Creates a simple explosion that ignores other bodies hiding behind static bodies.</summary>
    public sealed class SimpleExplosion(World world) : PhysicsLogic(world, PhysicsLogicType.Explosion)
    {
        /// <summary>
        /// This is the power used in the power function. A value of 1 means the force applied to bodies in the explosion
        /// is linear. A value of 2 means it is exponential.
        /// </summary>
        public float Power { get; set; } = 1; //linear

        /// <summary>Activate the explosion at the specified position.</summary>
        /// <param name="pos">The position (center) of the explosion.</param>
        /// <param name="radius">The radius of the explosion.</param>
        /// <param name="force">The force applied</param>
        /// <param name="maxForce">A maximum amount of force. When force gets over this value, it will be equal to maxForce</param>
        /// <returns>A list of bodies and the amount of force that was applied to them.</returns>
        public Dictionary<Body, Vector2> Activate(Vector2 pos, float radius, float force, float maxForce = float.MaxValue)
        {
            var affectedBodies = new HashSet<Body>();

            AABB aabb;
            aabb.LowerBound = pos - new Vector2(radius);
            aabb.UpperBound = pos + new Vector2(radius);

            // Query the world for bodies within the radius.
            World.QueryAABB(fixture =>
            {
                if (Vector2.Distance(fixture.Body.Position, pos) <= radius)
                    affectedBodies.Add(fixture.Body);

                return true;
            }, ref aabb);

            return ApplyImpulse(pos, radius, force, maxForce, affectedBodies);
        }

        private Dictionary<Body, Vector2> ApplyImpulse(Vector2 pos, float radius, float force, float maxForce, HashSet<Body> overlappingBodies)
        {
            var forces = new Dictionary<Body, Vector2>(overlappingBodies.Count);

            foreach (var overlappingBody in overlappingBodies)
            {
                if (!IsActiveOn(overlappingBody))
                    continue;

                var distance = Vector2.Distance(pos, overlappingBody.Position);
                var forcePercent = GetPercent(distance, radius);

                var forceVector = pos - overlappingBody.Position;
                forceVector *= 1f / (float)Math.Sqrt(forceVector.X * forceVector.X + forceVector.Y * forceVector.Y);
                forceVector *= MathHelper.Min(force * forcePercent, maxForce);
                forceVector *= -1;

                overlappingBody.ApplyLinearImpulse(forceVector);
                forces.Add(overlappingBody, forceVector);
            }

            return forces;
        }

        private float GetPercent(float distance, float radius)
        {
            //(1-(distance/radius))^power-1
            var percent = (float)Math.Pow(1 - (distance - radius) / radius, Power) - 1;

            if (float.IsNaN(percent))
                return 0f;

            return MathHelper.Clamp(percent, 0f, 1f);
        }
    }
}