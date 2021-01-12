using DrWatson
@quickactivate "OneVision"

using OneVision.Benchmarks
import AbstractPlotting
AbstractPlotting.__init__()
AbstractPlotting.inline!(false)

# generate and save results
tables = run_performance_exps()
wsave(datadir("results", "performance_exps.bson"), @dict tables)

# load and display results
tables = wload(datadir("results", "performance_exps.bson"))[:tables]
show_performance_exps(tables)

gen_raw_data()

visualize_benchmark("1D Leader Linear", "Larger Delays", :Local)
visualize_benchmark("1D Leader With Obstacle", "Larger Delays", :ConstU)
visualize_benchmark("2D Formation Driving", "No Delays", :OneVision)
visualize_benchmark("2D Formation Switching", "No Delays", :ConstU)