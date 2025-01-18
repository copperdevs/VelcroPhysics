using System.Collections.Generic;
using BenchmarkDotNet.Attributes;
using VelcroPhysics.Benchmarks.Code;
using VelcroPhysics.Benchmarks.Code.TestClasses;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Benchmarks.Tests.Shared
{
    public class GraphBenchmark : UnmeasuredBenchmark
    {
        private Graph<Dummy> _graph;
        private List<Dummy> _list;

        [GlobalSetup]
        public void Setup()
        {
            _list = ConstructList();
            _graph = ConstructGraph();
        }

        [Benchmark]
        public List<Dummy> ConstructList()
        {
            var list = new List<Dummy>(1000);

            for (var i = 0; i < 1000; i++)
            {
                list.Add(new Dummy());
            }

            return list;
        }

        [Benchmark]
        public Graph<Dummy> ConstructGraph()
        {
            var graph = new Graph<Dummy>();

            for (var i = 0; i < 1000; i++)
            {
                graph.Add(new Dummy());
            }

            return graph;
        }

        [Benchmark]
        public int IterateList()
        {
            var i = 0;
            foreach (var d in _list)
            {
                i++;
            }

            return i;
        }

        [Benchmark]
        public int IterateGraph()
        {
            var i = 0;
            foreach (var d in _graph)
            {
                i++;
            }

            return i;
        }

        [Benchmark]
        public void RemoveFromList()
        {
            var d = new Dummy();
            _list.Add(d);
            _list.Remove(d);
        }

        [Benchmark]
        public void RemoveFromGraph()
        {
            var d = new Dummy();
            _graph.Add(d);
            _graph.Remove(d);
        }

        [Benchmark]
        public void RemoveFromGraphFast()
        {
            var d = new Dummy();
            var node = _graph.Add(d);
            _graph.Remove(node);
        }
    }
}