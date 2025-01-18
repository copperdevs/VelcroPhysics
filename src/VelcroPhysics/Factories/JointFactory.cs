﻿using System.Numerics;
using VelcroPhysics.Definitions.Joints;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Dynamics.Joints;

namespace VelcroPhysics.Factories
{
    /// <summary>An easy to use factory for using joints.</summary>
    public static class JointFactory
    {
        public static MotorJoint CreateMotorJoint(World world, Body bodyA, Body bodyB, bool useWorldCoordinates = false)
        {
            var joint = new MotorJoint(bodyA, bodyB, useWorldCoordinates);
            world.AddJoint(joint);
            return joint;
        }

        public static WeldJoint CreateWeldJoint(World world, Body bodyA, Body bodyB, Vector2 anchorA, Vector2 anchorB, bool useWorldCoordinates = false)
        {
            var weldJoint = new WeldJoint(bodyA, bodyB, anchorA, anchorB, useWorldCoordinates);
            world.AddJoint(weldJoint);
            return weldJoint;
        }

        public static PrismaticJoint CreatePrismaticJoint(World world, Body bodyA, Body bodyB, Vector2 anchor, Vector2 axis, bool useWorldCoordinates = false)
        {
            var joint = new PrismaticJoint(bodyA, bodyB, anchor, axis, useWorldCoordinates);
            world.AddJoint(joint);
            return joint;
        }

        public static AngleJoint CreateAngleJoint(World world, Body bodyA, Body bodyB)
        {
            var angleJoint = new AngleJoint(bodyA, bodyB);
            world.AddJoint(angleJoint);
            return angleJoint;
        }

        public static GearJoint CreateGearJoint(World world, Body bodyA, Body bodyB, Joint jointA, Joint jointB, float ratio)
        {
            var gearJoint = new GearJoint(bodyA, bodyB, jointA, jointB, ratio);
            world.AddJoint(gearJoint);
            return gearJoint;
        }

        public static PulleyJoint CreatePulleyJoint(World world, Body bodyA, Body bodyB, Vector2 anchorA, Vector2 anchorB, Vector2 worldAnchorA, Vector2 worldAnchorB, float ratio, bool useWorldCoordinates = false)
        {
            var pulleyJoint = new PulleyJoint(bodyA, bodyB, anchorA, anchorB, worldAnchorA, worldAnchorB, ratio, useWorldCoordinates);
            world.AddJoint(pulleyJoint);
            return pulleyJoint;
        }

        public static FixedMouseJoint CreateFixedMouseJoint(World world, Body body, Vector2 worldAnchor)
        {
            var joint = new FixedMouseJoint(body, worldAnchor);
            world.AddJoint(joint);
            return joint;
        }

        public static RevoluteJoint CreateRevoluteJoint(World world, Body bodyA, Body bodyB, Vector2 anchorA, Vector2 anchorB, bool useWorldCoordinates = false)
        {
            var joint = new RevoluteJoint(bodyA, bodyB, anchorA, anchorB, useWorldCoordinates);
            world.AddJoint(joint);
            return joint;
        }

        public static RevoluteJoint CreateRevoluteJoint(World world, Body bodyA, Body bodyB, Vector2 anchor)
        {
            var localanchorA = bodyA.GetLocalPoint(bodyB.GetWorldPoint(anchor));
            var joint = new RevoluteJoint(bodyA, bodyB, localanchorA, anchor);
            world.AddJoint(joint);
            return joint;
        }

        public static WheelJoint CreateWheelJoint(World world, Body bodyA, Body bodyB, Vector2 anchor, Vector2 axis, bool useWorldCoordinates = false)
        {
            var joint = new WheelJoint(bodyA, bodyB, anchor, axis, useWorldCoordinates);
            world.AddJoint(joint);
            return joint;
        }

        public static WheelJoint CreateWheelJoint(World world, Body bodyA, Body bodyB, Vector2 axis)
        {
            return CreateWheelJoint(world, bodyA, bodyB, Vector2.Zero, axis);
        }

        public static DistanceJoint CreateDistanceJoint(World world, Body bodyA, Body bodyB, Vector2 anchorA, Vector2 anchorB, bool useWorldCoordinates = false)
        {
            var distanceJoint = new DistanceJoint(bodyA, bodyB, anchorA, anchorB, useWorldCoordinates);
            world.AddJoint(distanceJoint);
            return distanceJoint;
        }

        public static DistanceJoint CreateDistanceJoint(World world, Body bodyA, Body bodyB)
        {
            return CreateDistanceJoint(world, bodyA, bodyB, Vector2.Zero, Vector2.Zero);
        }

        public static FrictionJoint CreateFrictionJoint(World world, Body bodyA, Body bodyB, Vector2 anchor, bool useWorldCoordinates = false)
        {
            var frictionJoint = new FrictionJoint(bodyA, bodyB, anchor, useWorldCoordinates);
            world.AddJoint(frictionJoint);
            return frictionJoint;
        }

        public static FrictionJoint CreateFrictionJoint(World world, Body bodyA, Body bodyB)
        {
            return CreateFrictionJoint(world, bodyA, bodyB, Vector2.Zero);
        }

        public static Joint CreateFromDef(World world, JointDef def)
        {
            var joint = Joint.Create(def);
            world.AddJoint(joint);
            return joint;
        }
    }
}