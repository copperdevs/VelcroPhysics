﻿using System.Collections.Generic;
using FarseerPhysics.Collision.Shapes;
using FarseerPhysics.Common;
using FarseerPhysics.Common.Decomposition;
using FarseerPhysics.Common.PolygonManipulation;
using FarseerPhysics.Dynamics;
using FarseerPhysics.Factories;
using FarseerPhysics.TestBed.Framework;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace FarseerPhysics.TestBed.Tests
{
    public class TextureVerticesTest : Test
    {
        private Body _polygonBody;
        private Texture2D _polygonTexture;
        private List<Vertices> list;

        private TextureVerticesTest()
        {
            Body ground = BodyFactory.CreateBody(World);

            Vertices edge = PolygonTools.CreateEdge(new Vector2(-40.0f, 0.0f), new Vector2(40.0f, 0.0f));
            PolygonShape shape = new PolygonShape(edge, 0);
            ground.CreateFixture(shape);
        }

        public override void Initialize()
        {
            //load texture that will represent the physics body
            _polygonTexture = GameInstance.Content.Load<Texture2D>("Texture");

            //Create an array to hold the data from the texture
            uint[] data = new uint[_polygonTexture.Width * _polygonTexture.Height];

            //Transfer the texture data to the array
            _polygonTexture.GetData(data);

            //Find the vertices that makes up the outline of the shape in the texture
            Vertices verts = PolygonTools.CreatePolygon(data, _polygonTexture.Width, _polygonTexture.Height, true);

            //For now we need to scale the vertices (result is in pixels, we use meters)
            Vector2 scale = new Vector2(0.07f, 0.07f);
            verts.Scale(ref scale);

            //Simplify the vertices (less is better)
            verts = BooleanTools.Simplify(verts);

            //Since it is a concave polygon, we need to partition it into several smaller convex polygons
            list = BayazitDecomposer.ConvexPartition(verts);

            //We create a single body
            _polygonBody = BodyFactory.CreateBody(World);
            _polygonBody.BodyType = BodyType.Dynamic;

            //Then we create several fixtures using the body
            foreach (Vertices vert in list)
            {
                PolygonShape shape = new PolygonShape(vert, 1);
                _polygonBody.CreateFixture(shape);
            }

            base.Initialize();
        }

        public static Test Create()
        {
            return new TextureVerticesTest();
        }
    }
}