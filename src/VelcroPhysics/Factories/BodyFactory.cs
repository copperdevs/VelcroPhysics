﻿using System;
using System.Collections.Generic;
using System.Numerics;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Definitions;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Shared;
using VelcroPhysics.Tools.Triangulation.TriangulationBase;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Factories
{
    public static class BodyFactory
    {
        public static Body CreateBody(World world, Vector2 position = new(), float rotation = 0, BodyType bodyType = BodyType.Static, object userData = null)
        {
            var def = new BodyDef
            {
                Position = position,
                Angle = rotation,
                Type = bodyType,
                UserData = userData
            };

            return CreateFromDef(world, def);
        }

        public static Body CreateEdge(World world, Vector2 start, Vector2 end, object userData = null)
        {
            var body = CreateBody(world);
            body.UserData = userData;

            FixtureFactory.AttachEdge(start, end, body);
            return body;
        }

        public static Body CreateChainShape(World world, Vertices vertices, Vector2 position = new(), object userData = null)
        {
            var body = CreateBody(world, position);
            body.UserData = userData;

            FixtureFactory.AttachChainShape(vertices, body);
            return body;
        }

        public static Body CreateLoopShape(World world, Vertices vertices, Vector2 position = new(), object userData = null)
        {
            var body = CreateBody(world, position);
            body.UserData = userData;

            FixtureFactory.AttachLoopShape(vertices, body);
            return body;
        }

        public static Body CreateRectangle(World world, float width, float height, float density, Vector2 position = new(), float rotation = 0, BodyType bodyType = BodyType.Static, object userData = null)
        {
            if (width <= 0)
                throw new ArgumentOutOfRangeException(nameof(width), "Width must be more than 0 meters");

            if (height <= 0)
                throw new ArgumentOutOfRangeException(nameof(height), "Height must be more than 0 meters");

            var body = CreateBody(world, position, rotation, bodyType, userData);

            var rectangleVertices = PolygonUtils.CreateRectangle(width / 2, height / 2);
            FixtureFactory.AttachPolygon(rectangleVertices, density, body);

            return body;
        }

        public static Body CreateCircle(World world, float radius, float density, Vector2 position = new(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            var body = CreateBody(world, position, 0, bodyType, userData);
            FixtureFactory.AttachCircle(radius, density, body);
            return body;
        }

        public static Body CreateEllipse(World world, float xRadius, float yRadius, int edges, float density, Vector2 position = new(), float rotation = 0, BodyType bodyType = BodyType.Static, object userData = null)
        {
            var body = CreateBody(world, position, rotation, bodyType, userData);
            FixtureFactory.AttachEllipse(xRadius, yRadius, edges, density, body);
            return body;
        }

        public static Body CreatePolygon(World world, Vertices vertices, float density, Vector2 position = new(), float rotation = 0, BodyType bodyType = BodyType.Static, object userData = null)
        {
            var body = CreateBody(world, position, rotation, bodyType, userData);
            FixtureFactory.AttachPolygon(vertices, density, body);
            return body;
        }

        public static Body CreateCompoundPolygon(World world, List<Vertices> list, float density, Vector2 position = new(), float rotation = 0, BodyType bodyType = BodyType.Static, object userData = null)
        {
            //We create a single body
            var body = CreateBody(world, position, rotation, bodyType, userData);
            FixtureFactory.AttachCompoundPolygon(list, density, body);
            return body;
        }

        public static Body CreateGear(World world, float radius, int numberOfTeeth, float tipPercentage, float toothHeight, float density, Vector2 position = new(), float rotation = 0, BodyType bodyType = BodyType.Static, object userData = null)
        {
            var gearPolygon = PolygonUtils.CreateGear(radius, numberOfTeeth, tipPercentage, toothHeight);

            //Gears can in some cases be convex
            if (!gearPolygon.IsConvex())
            {
                //Decompose the gear:
                var list = Triangulate.ConvexPartition(gearPolygon, TriangulationAlgorithm.Earclip);

                return CreateCompoundPolygon(world, list, density, position, rotation, bodyType, userData);
            }

            return CreatePolygon(world, gearPolygon, density, position, rotation, bodyType, userData);
        }

        public static Body CreateCapsule(World world, float height, float topRadius, int topEdges, float bottomRadius, int bottomEdges, float density, Vector2 position = new(), float rotation = 0, BodyType bodyType = BodyType.Static, object userData = null)
        {
            var verts = PolygonUtils.CreateCapsule(height, topRadius, topEdges, bottomRadius, bottomEdges);

            //There are too many vertices in the capsule. We decompose it.
            if (verts.Count >= Settings.MaxPolygonVertices)
            {
                var vertList = Triangulate.ConvexPartition(verts, TriangulationAlgorithm.Earclip);
                return CreateCompoundPolygon(world, vertList, density, position, rotation, bodyType, userData);
            }

            return CreatePolygon(world, verts, density, position, rotation, bodyType, userData);
        }

        public static Body CreateCapsule(World world, float height, float endRadius, float density, Vector2 position = new(), float rotation = 0, BodyType bodyType = BodyType.Static, object userData = null)
        {
            //Create the middle rectangle
            var rectangle = PolygonUtils.CreateRectangle(endRadius, height / 2);

            var list = new List<Vertices>();
            list.Add(rectangle);

            var body = CreateCompoundPolygon(world, list, density, position, rotation, bodyType, userData);
            FixtureFactory.AttachCircle(endRadius, density, body, new Vector2(0, height / 2));
            FixtureFactory.AttachCircle(endRadius, density, body, new Vector2(0, -(height / 2)));

            //Create the two circles
            //CircleShape topCircle = new CircleShape(endRadius, density);
            //topCircle.Position = new Vector2(0, height / 2);
            //body.CreateFixture(topCircle);

            //CircleShape bottomCircle = new CircleShape(endRadius, density);
            //bottomCircle.Position = new Vector2(0, -(height / 2));
            //body.CreateFixture(bottomCircle);
            return body;
        }

        public static Body CreateRoundedRectangle(World world, float width, float height, float xRadius, float yRadius, int segments, float density, Vector2 position = new(), float rotation = 0, BodyType bodyType = BodyType.Static, object userData = null)
        {
            var verts = PolygonUtils.CreateRoundedRectangle(width, height, xRadius, yRadius, segments);

            //There are too many vertices in the capsule. We decompose it.
            if (verts.Count >= Settings.MaxPolygonVertices)
            {
                var vertList = Triangulate.ConvexPartition(verts, TriangulationAlgorithm.Earclip);
                return CreateCompoundPolygon(world, vertList, density, position, rotation, bodyType, userData);
            }

            return CreatePolygon(world, verts, density, position, rotation, bodyType, userData);
        }

        public static Body CreateLineArc(World world, float radians, int sides, float radius, bool closed = false, Vector2 position = new(), float rotation = 0, BodyType bodyType = BodyType.Static, object userData = null)
        {
            var body = CreateBody(world, position, rotation, bodyType, userData);
            FixtureFactory.AttachLineArc(radians, sides, radius, closed, body);
            return body;
        }

        public static Body CreateSolidArc(World world, float density, float radians, int sides, float radius, Vector2 position = new(), float rotation = 0, BodyType bodyType = BodyType.Static, object userData = null)
        {
            var body = CreateBody(world, position, rotation, bodyType, userData);
            FixtureFactory.AttachSolidArc(density, radians, sides, radius, body);

            return body;
        }

        public static BreakableBody CreateBreakableBody(World world, Vertices vertices, float density, Vector2 position = new(), float rotation = 0)
        {
            //TODO: Implement a Voronoi diagram algorithm to split up the vertices
            var triangles = Triangulate.ConvexPartition(vertices, TriangulationAlgorithm.Earclip);

            var breakableBody = new BreakableBody(world, triangles, density, position, rotation)
            {
                MainBody =
                {
                    Position = position
                }
            };
            world.AddBreakableBody(breakableBody);
            return breakableBody;
        }

        public static BreakableBody CreateBreakableBody(World world, IEnumerable<Shape> shapes, Vector2 position = new(), float rotation = 0)
        {
            var breakableBody = new BreakableBody(world, shapes, position, rotation)
            {
                MainBody =
                {
                    Position = position
                }
            };
            world.AddBreakableBody(breakableBody);
            return breakableBody;
        }

        public static Body CreateFromDef(World world, BodyDef def)
        {
            var body = new Body(def);
            world.AddBody(body);
            return body;
        }
    }
}