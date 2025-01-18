using BenchmarkDotNet.Attributes;
using VelcroPhysics.Benchmarks.Code;
using VelcroPhysics.Benchmarks.Code.TestClasses;

namespace VelcroPhysics.Benchmarks.Tests.CLR
{
    public class FieldPropertyBenchmarks : UnmeasuredBenchmark
    {
        private readonly Dummy _dummy = new() { ValueField = new Struct32() };

        [Benchmark]
        public Struct32 PropertyGetTest()
        {
            return _dummy.ValueProperty;
        }

        [Benchmark]
        public Struct32 FieldGetTest()
        {
            return _dummy.ValueField;
        }

        [Benchmark]
        public Struct32 MethodGetTest()
        {
            return _dummy.ValueMethod();
        }
    }
}
