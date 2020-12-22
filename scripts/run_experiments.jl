using DrWatson
@quickactivate "OneVision"

using OneVision.Benchmarks
import AbstractPlotting
AbstractPlotting.__init__()
AbstractPlotting.inline!(false)


run_benchmarks()

show_benchmark("1D Leader Linear", "Larger Delays", :Naive)
show_benchmark("1D Leader Linear", "Larger Delays", :OneVision)