module Benchmarks

export run_benchmarks, show_benchmark

using OneVision
using OneVision.Examples

using ProgressMeter
using DataFrames

function with_args(f; kws...)
    (args...; kws2...) -> f(args...; kws2..., kws...)
end

scenarios = [
    "1D Leader Linear" => with_args(Car1DExample.run_example, 
        use_bang_bang = false, has_obstacle = false) 
    "1D Leader With Obstacle" => with_args(Car1DExample.run_example,
        use_bang_bang = true, has_obstacle = true)
    "2D Formation Driving" => with_args(Car2DExamples.formation_example,
        switch_formation = false)
    "2D Formation Switching" => with_args(Car2DExamples.formation_example,
        switch_formation = true)
]

CFs = [
    :Naive => naive_cf, 
    :Local => local_cf, 
    :ConstU => const_u_cf,
    :OneVision => onevision_cf,
]

defaults = ExampleSetting(
    freq = 100.0,
    noise = 0.1 / sqrt(100),
    sensor_noise = 0.1 / sqrt(100),
    delays = DelayModel(obs = 3, act = 4, com = 5, ΔT = 5),
    H = 20,
    model_error = 0.0,
)

disturbance_settings = [
    "0X Disturance" => @set defaults.noise = 0.0
    "4X Disturance" => @set defaults.noise *= 4.0
    "16X Disturance" => @set defaults.noise *= 16.0
]

noise_settings = [
    "0X Noise" => @set defaults.sensor_noise = 0.0
    "4X Noise" => @set defaults.sensor_noise *= 4.0
    "16X Noise" => @set defaults.sensor_noise *= 16.0
]

latency_settings = [
    "Delay = 1" => @set defaults.delays.com = 1
    "4X Delay" => @set defaults.delays.com *= 4
    "16X Delay" => @set defaults.delays.com *= 16
]

horizon_settings = [
    "H = 1" => @set defaults.H = 1
    "H = 5" => @set defaults.H = 5
    "H = 40" => @set defaults.H = 40
]

basic_settings = [
    "Default" => defaults
    "Larger Delays" =>  @set defaults.delays.com *= 4
    "No Delays" =>  @set defaults.delays = DelayModel(;obs = 0, act = 0, com = 1, ΔT = 5)
    "Lower Freq" => 
        let s1 = deepcopy(defaults)
            s1.freq /= 5
            s1.noise *= sqrt(5)
            s1.sensor_noise *= sqrt(5)
            s1.delays = DelayModel(obs = 1, act = 1, com = 1, ΔT = 1)
            s1
        end
]

# benchmarks = Dict([
#     "Performance vs Sensor Noise" => noise_settings
#     "Performance vs Disturbance" => disturbance_settings
#     "Performance vs Latency" => latency_settings
#     "Performance vs Prediction Horizon" => horizon_settings
# ])

function run_benchmarks()
    num_tasks = length(basic_settings) * length(scenarios)
    prog = Progress(num_tasks+1+length(basic_settings), 0.1, "Running benchmarks...")

    iolock = ReentrantLock()

    function display_table(rows, name)
        sep = " "^displaysize(stdout)[2]
        println("\r$sep")
        println("Setting: $name")
        DataFrame(rows) |> display
    end

    tables = Dict{String, Any}()
    Threads.@threads for (setname, setting) in basic_settings
        rows = []
        for (name, ex) in scenarios
            results = [
                cf_str => sum(ex(;plot_result = false, CF, setting)) / setting.freq
                for (cf_str, CF) in CFs]
            best = sortperm(results, by = x -> x[2])[1]
            push!(rows, (Task = name, results..., Best = results[best][1]))
            next!(prog)
        end
        lock(iolock) do
            display_table(rows, setname)
            flush(stdout)
            next!(prog)
            tables[setname] = rows
        end
    end
    finish!(prog)
    for (setname, _) in basic_settings
        display_table(tables[setname], setname)
    end
end

function show_benchmark(bench_name::String, setting_name::String, cf_name::Symbol)
    bench_name = strip(bench_name)
    setting_name = strip(setting_name)

    ex = Dict(scenarios)[bench_name]
    setting = Dict(basic_settings)[setting_name]
    CF = Dict(CFs)[cf_name]
    loss = ex(;plot_result = true, CF, setting)
    nothing
end

end # module Benchmarks