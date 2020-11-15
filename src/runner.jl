if isdefined(@__MODULE__, :LanguageServer)  # hack to make vscode linter work properly
    include("OneVision.jl")
    # using .OneVision
    using .OneVision.Car1DExample: run_example
else
    # using OneVision
    using OneVision.Car1DExample: run_example
end

using BenchmarkTools: @benchmark
using Plots, Measures
plotlyjs()

t_end = 30
freq = 20.0

let
    @time plts = run_example(1:freq * t_end, freq; noise=0.0);

    width = 500
    height = 300
    n = length(plts)
    plot(plts...;layout=(n, 1), size=(width, height * n), bottom_margin = 1cm) |> display
end


# using StatProfilerHTML
# @profilehtml for _ in 1:1000; run_example(1:20 * t_end, freq) end

@benchmark run_example(1:20 * t_end, freq; noise=0.0, plot_result=false)

