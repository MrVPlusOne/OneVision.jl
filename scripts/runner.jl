using DrWatson
@quickactivate "OneVision"

if isdefined(@__MODULE__, :LanguageServer)  # hack to make vscode linter work properly
    include("../src/OneVision.jl")
    # using .OneVision
    using .OneVision.Car2DExamples: run_example, plot_cars
else
    # using OneVision
    using OneVision.Car2DExamples: run_example, plot_cars
end

using BenchmarkTools: @benchmark
import AbstractPlotting
AbstractPlotting.__init__()
AbstractPlotting.inline!(false)

# using Plots, Measures
# plotlyjs()

run_example(time_end=20)

# @profview run_example(1:20 * t_end, freq; noise=0.0, plot_result=false)
# run_example(1:20 * t_end, freq; noise=0.0, plot_result=false) modules=[OneVision] maxdepth=3

# @benchmark run_example(time_end=20, plot_result=false)