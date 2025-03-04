﻿using System.Diagnostics;
using System.Numerics;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Factories;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Tools.PathGenerator
{
    public static class LinkFactory
    {
        /// <summary>Creates a chain.</summary>
        /// <param name="world">The world.</param>
        /// <param name="start">The start.</param>
        /// <param name="end">The end.</param>
        /// <param name="linkWidth">The width.</param>
        /// <param name="linkHeight">The height.</param>
        /// <param name="numberOfLinks">The number of links.</param>
        /// <param name="linkDensity">The link density.</param>
        /// <param name="attachRopeJoint">
        /// Creates a rope joint between start and end. This enforces the length of the rope. Said in
        /// another way: it makes the rope less bouncy.
        /// </param>
        public static Path CreateChain(World world, Vector2 start, Vector2 end, float linkWidth, float linkHeight, int numberOfLinks, float linkDensity, bool attachRopeJoint)
        {
            Debug.Assert(numberOfLinks >= 2);

            //Chain start / end
            var path = new Path();
            path.Add(start);
            path.Add(end);

            //A single chainlink
            var shape = new PolygonShape(PolygonUtils.CreateRectangle(linkWidth, linkHeight), linkDensity);

            //Use PathManager to create all the chainlinks based on the chainlink created before.
            var chainLinks = PathManager.EvenlyDistributeShapesAlongPath(world, path, shape, BodyType.Dynamic, numberOfLinks);

            //TODO
            //if (fixStart)
            //{
            //    //Fix the first chainlink to the world
            //    JointFactory.CreateFixedRevoluteJoint(world, chainLinks[0], new Vector2(0, -(linkHeight / 2)),
            //                                          chainLinks[0].Position);
            //}

            //if (fixEnd)
            //{
            //    //Fix the last chainlink to the world
            //    JointFactory.CreateFixedRevoluteJoint(world, chainLinks[chainLinks.Count - 1],
            //                                          new Vector2(0, (linkHeight / 2)),
            //                                          chainLinks[chainLinks.Count - 1].Position);
            //}

            //Attach all the chainlinks together with a revolute joint
            PathManager.AttachBodiesWithRevoluteJoint(world, chainLinks, new Vector2(0, -linkHeight), new Vector2(0, linkHeight), false, false);

            if (attachRopeJoint)
                JointFactory.CreateDistanceJoint(world, chainLinks[0], chainLinks[^1], Vector2.Zero, Vector2.Zero);

            return path;
        }
    }
}