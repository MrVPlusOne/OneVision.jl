using DrWatson
@quickactivate "OneVision"

using OneVision.Benchmarks
import AbstractPlotting
AbstractPlotting.__init__()
AbstractPlotting.inline!(false)

run_benchmarks()

show_benchmark("1D Leader Linear", "Larger Delays", :Local)
show_benchmark("1D Leader Linear", "Larger Delays", :OneVision)
show_benchmark("2D Formation Driving", "Larger Delays", :OneVision)