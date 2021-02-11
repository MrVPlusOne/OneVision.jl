module ROS_SIM_RUNNER

export run_benchmarks, run_open_loop

using OneVision
using OneVision.Examples

using ProgressMeter
using DataFrames

function with_args(f; kws...)
    (args...; kws2...) -> f(args...; kws2..., kws...)
end

benchmarks = [
    "1D Leader Linear" => with_args(Car1DExample.run_example, 
        use_bang_bang = false, has_obstacle = false, plot_result = false) 
    "1D Leader With Obstacle" => with_args(Car1DExample.run_example,
        use_bang_bang = true, has_obstacle = true, plot_result = false)
    "2D Formation Driving" => with_args(Car2DExamples.formation_example,
        switch_formation = false, plot_result = false)
    "2D Formation Switching" => with_args(Car2DExamples.formation_example,
        switch_formation = true, plot_result = false)
]

CFs = [
    :Naive => naive_cf, 
    :Local => local_cf, 
    :ConstU => const_u_cf,
    :OneVision => onevision_cf,
]

defaults = Dict([
    :freq => 100.0
    :noise => 0.1 / sqrt(100)
    :delays => DelayModel(obs = 3, act = 4, com = 5, ΔT = 5)
])

setting_args = [
    "Default" => defaults
    "Noiseless" => @set defaults[:noise] = 0.0
    "Larger Delays" =>  @set defaults[:delays].com *= 4
    "No Delays" =>  @set defaults[:delays] = DelayModel(;obs = 0, act = 0, com = 1, ΔT = 5)
    "Lower Freq" => 
        let s1 = copy(defaults)
            s1[:freq] /= 5
            s1[:noise] *= sqrt(5)
            s1[:delays] = DelayModel(obs = 1, act = 1, com = 1, ΔT = 1)
            s1
        end
]

function run_benchmarks()
    #Car2DExamples.formation_ros_example(switch_formation = false, plot_result = false)
    #Car1DExample.run_ros_example(use_bang_bang = false, has_obstacle = false, plot_result = false)
end

function run_open_loop(car_id::Integer, fleet_size::Integer, freq::Int32, preheat::Bool)
    @assert freq > 0
    #@assert fleet_size > 0
    #Car1DExample.run_open_example(car_id, fleet_size)
    @assert fleet_size >= 1 
    Car2DExamples.run_open_example(car_id, fleet_size, freq, preheat)
end
end # module ROS_SIM_RUNNER