using DrWatson
@quickactivate "OneVision"

using OneVision.Examples.Car2DExamples
using BenchmarkTools: @benchmark
import AbstractPlotting
AbstractPlotting.__init__()
AbstractPlotting.inline!(false)

Car2DExamples.live_demo()

# Car2DExamples.tracking_example(time_end=20, noise_level = 0.003, plot_result=true)
# Car2DExamples.formation_example(time_end=20, 
#     noise = 0.001, sensor_noise = 0.005, plot_result=true)

# import OneVision.Car1DExample
# Car1DExample.run_example(time_end = 20.0, freq = 20.0, noise=0.0, plot_result=true)
# @benchmark Car2DExamples.formation_example(time_end=20, 
#     noise = 0.001, sensor_noise = 0.003, plot_result=false)

# @profview run_example(1:20 * t_end, freq; noise=0.0, plot_result=false)
# run_example(1:20 * t_end, freq; noise=0.0, plot_result=false) modules=[OneVision] maxdepth=3

# @benchmark run_example(time_end=20, plot_result=false)
