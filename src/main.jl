using OneVision.Car1DExample: run_example
using BenchmarkTools: @benchmark

function main()
    foreach(display, run_example(1:10 * 20, 1.0 / 20))
end


