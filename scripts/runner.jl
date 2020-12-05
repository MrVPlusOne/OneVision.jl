using DrWatson
@quickactivate "OneVision"

if isdefined(@__MODULE__, :LanguageServer)  # hack to make vscode linter work properly
    include("../src/OneVision.jl")
    # using .OneVision
    using .OneVision.Car2DExamples
else
    # using OneVision
    using OneVision.Car2DExamples
end

using BenchmarkTools: @benchmark
import AbstractPlotting
AbstractPlotting.__init__()
AbstractPlotting.inline!(false)

# Car2DExamples.tracking_example(time_end=20, noise_level = 0.003, plot_result=true)
Car2DExamples.formation_example(time_end=20, noise_level = 0.003, plot_result=true)


import OneVision.Car1DExample
Car1DExample.run_example(1:20 * 20, 20.0; noise=0.0, plot_result=true)
@benchmark Car2DExamples.formation_example(time_end=20, noise_level = 0.003, plot_result=false)
@benchmark Car2DExamples(time_end=20, plot_result=false)

# @profview run_example(1:20 * t_end, freq; noise=0.0, plot_result=false)
# run_example(1:20 * t_end, freq; noise=0.0, plot_result=false) modules=[OneVision] maxdepth=3

# @benchmark run_example(time_end=20, plot_result=false)
