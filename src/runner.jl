if isdefined(@__MODULE__, :LanguageServer)  # hack to make vscode linter work properly
    include("OneVision.jl")
    using .OneVision
else
    using OneVision
    using OneVision.Car1DExample: run_example
end

using BenchmarkTools: @benchmark
using Plots, Measures

const t_end = 40
plotlyjs()
function run_and_plot()
    @time plts = run_example(1:20 * t_end, 1.0 / 20)

    width = 500
    height = 300
    n = length(plts)
    plot(plts...;layout=(n, 1), size=(width, height * n), bottom_margin = 1cm) |> display
end

run_and_plot()

# using StatProfilerHTML
# @profilehtml for _ in 1:1000; run_example(1:20 * t_end, 1.0 / 20) end

@benchmark run_example(1:20 * t_end, 1.0 / 20; plot_result=false)

