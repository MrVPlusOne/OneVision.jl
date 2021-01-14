using DrWatson
@quickactivate "OneVision"

using OneVision.Benchmarks

using DataFrames
import AbstractPlotting
AbstractPlotting.__init__()
AbstractPlotting.inline!(false)

perf_path = datadir("results", "performance_exps.bson")
var_path = datadir("results", "variation_exps.bson")


# generate and save results
perf_tables = run_performance_exps()
wsave(perf_path, @dict perf_tables)

# load and display results
perf_tables = wload(perf_path)[:perf_tables]
show_performance_exps(perf_tables)

# generate and save results
var_tables = run_variation_exps()
wsave(var_path, @dict var_tables)

# load and display results
var_tables = wload(var_path)[:var_tables]
show_variation_exps(var_tables)



visualize_benchmark("1D Leader Linear", "Larger Delays", :Local)
visualize_benchmark("1D Leader With Obstacle", "Larger Delays", :ConstU)
visualize_benchmark("2D Formation Driving", "16X Disturance", :OneVision)
visualize_benchmark("2D Formation Switching", "No Delays", :ConstU)