module Benchmarks

export run_performance_exps, show_performance_exps, visualize_benchmark
export run_variation_exps, show_variation_exps

using OneVision
using OneVision.Examples

using ProgressMeter
using DataFrames
using ThreadsX
using DrWatson
using CSV
using Plots

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
    # "Lower Freq" => 
    #     let s1 = deepcopy(defaults)
    #         s1.freq /= 5
    #         s1.noise *= sqrt(5)
    #         s1.sensor_noise *= sqrt(5)
    #         s1.delays = DelayModel(obs = 1, act = 1, com = 1, ΔT = 1)
    #         s1
    #     end
]

# benchmarks = Dict([
#     "Performance vs Sensor Noise" => noise_settings
#     "Performance vs Disturbance" => disturbance_settings
#     "Performance vs Latency" => latency_settings
#     "Performance vs Prediction Horizon" => horizon_settings
# ])

function run_setting(setting)
    metric_tables = Dict{String, Any}()
    for (name, ex) in scenarios
        foreach(CFs) do (cf_str, CF)
            loss, metrics = ex(;plot_result = false, CF, setting)
            metrics["loss"] = sum(loss) / setting.freq
            for (metric, v) in metrics
                rows = get!(metric_tables, metric, [])
                push!(rows, (Task = name, CF = cf_str, value = v))
            end
        end
    end
    Dict(k => rearrange_result_table(DataFrame(v)) for (k, v) in metric_tables)
end

"""
Rearrange the result DataFrame for easy comparison of the performance of 
different CFs
"""
function rearrange_result_table(table::DataFrame)
    rows = []
    for df in groupby(table, :Task)
        task = df[1, :Task]
        cols = Symbol.(df[:, :CF]) .=> df[:, :value]
        best = sortperm(cols, by = x -> x[2])[1]
        
        push!(rows, (Task = task, cols..., Best = df[best, :CF]))
    end
    DataFrame(rows)
end

function display_table(rows, name)
    sep = " "^displaysize(stdout)[2]
    println("\r$sep")
    println("==== $name ====")
    display(rows)
end

function run_performance_exps()
    settings = basic_settings()
    num_tasks = length(settings)
    prog = Progress(num_tasks, 0.1, "Running benchmarks...")

    tables = Dict(ThreadsX.map(settings) do (setname, setting)
        dfs = run_setting(setting)
        next!(prog)
        setname => dfs
    end)
    finish!(prog)
    show_performance_exps(tables)
    return tables
end

function show_performance_exps(tables)
    for (setname, _) in basic_settings(), (metric, df) in tables[setname]
        display_table(df, "$setname: $metric")
    end
end

function visualize_benchmark(bench_name::String, setting_name::String, cf_name::Symbol)
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

function run_variation_exps()
    OneVision.TrajPlanning.check_optimizer_converge[] = false
    allsettings = vcat(
        ["Default" => defaults], disturbance_settings(), 
        noise_settings(), latency_settings(), horizon_settings())

    @info "Pre-run the default setting for 1 sec..."
    @time run_setting(@set defaults.time_end = 1.0)

    @info "Running benchmarks..."
    prog = Progress(length(allsettings), 0.1, "Running benchmarks...")

    time_used = []
    iolock = ReentrantLock()
    @time tables = Dict(ThreadsX.map(allsettings; basesize=1) do (name, setting)
        stats = @timed run_setting(setting) 
        lock(iolock) do
            push!(time_used, (name = name, time = stats.time))
            next!(prog)
        end
        name => stats.value
    end)
    
    sort(time_used, by = x -> x.time, rev = true) |> DataFrame |> display
    return tables
end

function show_variation_exps(tables)
    scenario_names = first.(scenarios)
    # map graph names to `(latency_settings, Default setting position)`
    graph_names = [
        "Communication Delay" => (latency_settings(), 2)
        "External Disturbance" => (disturbance_settings(), 2)
        "Sensor Noise" => (noise_settings(), 2)
        "Prediction Horizon" => (horizon_settings(), 3)
    ]

    cfnames = first.(CFs)

    for (gname, (settings, def_pos)) in graph_names
        setnames = first.(settings)
        insert!(setnames, def_pos, "Default")
        xs = setnames
        subplots = []
        for (scenario_id, scenario_name) in enumerate(scenario_names)
            ys = [tables[setting]["loss"][scenario_id, cf] 
                    for setting in setnames, cf in cfnames]
            p = plot(
                xs, ys; 
                title = scenario_name, 
                yaxis = :log,
            )
            push!(subplots, p)
        end
        plot(subplots...; 
            labels = hcat(string.(cfnames)...), 
            legend = :outertopright,
            size = (800, 500),
        ) |> display
    end
end

end # module Benchmarks