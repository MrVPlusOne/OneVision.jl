using OneVision.Car1DExample: run_example
using BenchmarkTools: @benchmark

function main()
    # @benchmark run_example(1:10 * 20, 1.0 / 20)
    foreach(display, run_example(1:10 * 20, 1.0 / 20))
end


