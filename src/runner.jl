if false  # hack to make vscode linter work properly 
    include("OneVision.jl")
    using .OneVision
    using .OneVision.Car1DExample: run_example
end

using OneVision
using OneVision.Car1DExample: run_example
using BenchmarkTools: @benchmark 
using Plots

function run_and_plot()
    @time plts = run_example(1:20 * 20, 1.0 / 20)
    
    width = 500
    height = 300
    n = length(plts)
    plot(plts...;layout=(n, 1), size=(width, height * n)) |> display
end

run_and_plot()

# using StatProfilerHTML
@benchmark run_example(1:20 * 20, 1.0 / 20; plot_result=false)

# @profilehtml for _ in 1:1000; run_example(1:20 * 20, 1.0 / 20) end