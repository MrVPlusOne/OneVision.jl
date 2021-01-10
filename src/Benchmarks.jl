module Benchmarks

export run_benchmarks, show_benchmark, gen_raw_data

using OneVision
using OneVision.Examples

using ProgressMeter
using DataFrames
using ThreadsX
using DrWatson
using CSV

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
    time_end = 20.0,
    freq = 100.0,
    noise = 0.1 / sqrt(100),
    sensor_noise = 0.1 / sqrt(100),
    delays = DelayModel(obs = 3, act = 4, com = 5, ΔT = 5),
    H = 20,
    model_error = 0.0,
)

disturbance_settings() = [
    "0X Disturance" => @set defaults.noise = 0.0
    "4X Disturance" => @set defaults.noise *= 4.0
    "16X Disturance" => @set defaults.noise *= 16.0
]

noise_settings() = [
    "0X Noise" => @set defaults.sensor_noise = 0.0
    "4X Noise" => @set defaults.sensor_noise *= 4.0
    "16X Noise" => @set defaults.sensor_noise *= 16.0
]

latency_settings() = [
    "Delay = 1" => @set defaults.delays.com = 1
    "4X Delay" => @set defaults.delays.com *= 4
    "16X Delay" => @set defaults.delays.com *= 16
]

horizon_settings() = [
    "H = 1" => @set defaults.H = 1
    "H = 5" => @set defaults.H = 5
    "H = 40" => @set defaults.H = 40
]

basic_settings() = [
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

function run_setting(setting)
    rows = []
    for (name, ex) in scenarios
        results = [
            cf_str => sum(ex(;plot_result = false, CF, setting)) / setting.freq
            for (cf_str, CF) in CFs]
        best = sortperm(results, by = x -> x[2])[1]
        push!(rows, (Task = name, results..., Best = results[best][1]))
    end
    rows
end

function run_benchmarks()
    function display_table(rows, name)
        sep = " "^displaysize(stdout)[2]
        println("\r$sep")
        println("Setting: $name")
        DataFrame(rows) |> display
    end
    settings = basic_settings()
    num_tasks = length(settings)
    prog = Progress(num_tasks, 0.1, "Running benchmarks...")
    iolock = ReentrantLock()

    tables = Dict{String, Any}()
    ThreadsX.foreach(settings) do (setname, setting)
        rows = run_setting(setting)
        lock(iolock) do
            display_table(rows, setname)
            flush(stdout)
            next!(prog)
            tables[setname] = rows
        end
    end
    finish!(prog)
    for (setname, _) in settings
        display_table(tables[setname], setname)
    end
end

function show_benchmark(bench_name::String, setting_name::String, cf_name::Symbol)
    bench_name = strip(bench_name)
    setting_name = strip(setting_name)

    ex = Dict(scenarios)[bench_name]
    setting = Dict(basic_settings())[setting_name]
    CF = Dict(CFs)[cf_name]
    loss = ex(;plot_result = true, CF, setting)
    nothing
end

function ensure_empty(dirpath)
    if isdir(dirpath)
        println("** $dirpath already exists, delete to proceed? (y/n):")
        answer = readline()
        if lowercase(strip(answer)) == "y"
            rm(dirpath, force=true, recursive=true)
        else
            error("Aborted.")
        end
    end
    mkdir(dirpath)
    dirpath
end

"""
Run and save raw data for the variation experiments.
"""
function gen_raw_data()
    OneVision.TrajPlanning.check_optimizer_converge[] = false
    allsettings = vcat(
        ["Default" => defaults], disturbance_settings(), 
        noise_settings(), latency_settings(), horizon_settings())
    results_dir = datadir("results_raw") |> ensure_empty

    @info "Pre-run the default setting for 1 sec..."
    @time run_setting(@set defaults.time_end = 1.0)

    @info "Running benchmarks, results will be saved under $results_dir..."
    prog = Progress(length(allsettings), 0.1, "Running benchmarks...")

    time_used = []
    iolock = ReentrantLock()
    @time ThreadsX.foreach(allsettings; basesize=1) do (name, setting)
        stats = @timed run_setting(setting) 
        result = stats.value |> DataFrame
        filename = joinpath(results_dir, "$name.csv")
        lock(iolock) do
            CSV.write(filename, result)
            push!(time_used, (name = name, time = stats.time))
            next!(prog)
        end
    end
    
    sort(time_used, by = x -> x.time, rev = true) |> DataFrame |> display
end

end # module Benchmarks