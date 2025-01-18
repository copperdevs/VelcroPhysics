using VelcroPhysics.Shared;
using VelcroPhysics.Tests.Code;
using Xunit;

namespace VelcroPhysics.Tests.Tests.Shared
{
    public class GraphTests
    {
        [Fact]
        public void TestEmptyConstruction()
        {
            var graph = new Graph<int>();
            Assert.Equal(0, graph.Count);
            Assert.Null(graph.First);

            var graph2 = new Graph<Dummy>();
            Assert.Equal(0, graph2.Count);
            Assert.Null(graph2.First);
        }

        [Fact]
        public void TestAddValueType()
        {
            var graph = new Graph<int>();
            var node = graph.Add(10);

            Assert.Equal(10, node.Item);
            Assert.Equal(node, node.Prev);
            Assert.Equal(node, node.Next);
        }

        [Fact]
        public void TestAddReferenceType()
        {
            var graph = new Graph<Dummy>();

            var instance = new Dummy();

            var node = graph.Add(instance);

            Assert.Equal(instance, node.Item);
            Assert.Equal(node, node.Prev);
            Assert.Equal(node, node.Next);
        }

        [Fact]
        public void TestRemoveValueType()
        {
            var graph = new Graph<int>();
            var node = graph.Add(10);

            Assert.Equal(1, graph.Count);

            graph.Remove(node);

            Assert.Equal(0, graph.Count);

            //Check that the node was cleared;
            Assert.Null(node.Prev);
            Assert.Null(node.Next);
        }

        [Fact]
        public void TestRemoveReferenceType()
        {
            var graph = new Graph<Dummy>();
            var node = graph.Add(new Dummy());

            Assert.Equal(1, graph.Count);

            graph.Remove(node);

            Assert.Equal(0, graph.Count);

            //Check that the node was cleared;
            Assert.Null(node.Prev);
            Assert.Null(node.Next);
        }

        [Fact]
        public void TestIteration()
        {
            var graph = new Graph<int>();

            for (var i = 0; i < 10; i++)
            {
                graph.Add(i);
            }

            Assert.Equal(10, graph.Count);

            var count = 0;

            foreach (var i in graph)
            {
                Assert.Equal(count++, i);
            }

            Assert.Equal(10, count);
        }
    }
}