using DrWatson
@quickactivate "OneVision"

using OneVision.Benchmarks

using DataFrames
import Plots
import AbstractPlotting
AbstractPlotting.__init__()
AbstractPlotting.inline!(false)
Plots.gr()

perf_path = datadir("results", "performance_exps.bson")
var_path = datadir("results", "variation_exps.bson")


# generate and save results
perf_tables = run_performance_exps(num_of_runs = 100)
wsave(perf_path, @dict perf_tables)

# load and display results
perf_tables = wload(perf_path)[:perf_tables]
show_performance_exps(perf_tables, datadir("processed", "performance_exps"))

# generate and save results
var_tables = run_variation_exps(num_of_runs = 10)
wsave(var_path, @dict var_tables)

# load and display results
var_tables = wload(var_path)[:var_tables]
show_variation_exps(var_tables, out_path = datadir("processed", "variation_exps"))

visualize_benchmark("2D Formation Switching", default_setting(), "OneVision")