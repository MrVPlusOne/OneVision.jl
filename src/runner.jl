"""
A convinent entry point for running OneVision
"""
module Runner

include("OneVision.jl")


using OneVision.Car1DExample: run_example
# using BenchmarkTools: @benchmark
# @benchmark run_example(1:10 * 20, 1.0 / 20)

@time foreach(display, run_example(1:20 * 20, 1.0 / 20))

end