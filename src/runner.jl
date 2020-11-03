using Plots
using OneVision
using OneVision.Car1DExample: run_example
using BenchmarkTools: @benchmark
# @benchmark run_example(1:10 * 20, 1.0 / 20)

function run_and_plot()
    @time plts = run_example(1:20 * 20, 1.0 / 20)
    
    width = 500
    height = 300
    n = length(plts)
    plot(plts...;layout=(n, 1), size=(width, height * n)) |> display
end

# run_and_plot()
@benchmark run_example(1:20 * 20, 1.0 / 20)
