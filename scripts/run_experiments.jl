using DrWatson
@quickactivate "OneVision"

using OneVision.Benchmarks
import AbstractPlotting
AbstractPlotting.__init__()
AbstractPlotting.inline!(false)

gen_raw_data()
# run_benchmarks()

show_benchmark("1D Leader Linear", "Larger Delays", :Local)
show_benchmark("1D Leader With Obstacle", "Larger Delays", :ConstU)
show_benchmark("2D Formation Driving", "No Delays", :OneVision)
show_benchmark("2D Formation Switching", "No Delays", :ConstU)