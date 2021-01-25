module Benchmarks

export run_performance_exps, show_performance_exps, visualize_benchmark
export run_variation_exps, show_variation_exps
export default_setting, variation_settings

using OneVision
using OneVision.Examples

using ProgressMeter
using DataFrames
using ThreadsX
using DrWatson
using CSV
using Plots
using Random
using Statistics
using Measurements

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

default_setting() = defaults

defaults = ExampleSetting(
    time_end = 20.0,
    freq = 100.0,
    noise = 0.05 / sqrt(100),
    sensor_noise = 0.05 / sqrt(100),
    delays = DelayModel(obs = 3, act = 4, com = 5, ΔT = 5),
    H = 20,
    model_error = 0.0,
)

variation_settings() = [
    "disturbance" => 
        [x => @set defaults.noise *= x 
        for x in [0.0, 1.0, 3.0, 6.0, 10.0]]
    "noise" => 
        [x => @set defaults.sensor_noise *= x 
        for x in [0.0, 1.0, 3.0, 6.0, 10.0]]
    "com delay" => 
        [x => @set defaults.delays.com = x 
        for x in [1, 5, 15, 30, 50]]
    "horizon" =>
        [x => @set defaults.H = x
        for x in [1, 5, 10, 20, 30]]
    "model error" =>
        [x => @set defaults.model_error = x
        for x in [0.0, 0.1, 0.3, 0.6, 1.0]]
]

basic_settings() = [
    "Default" => defaults
    "Larger Delays" =>  @set defaults.delays.com *= 4
    "No Delays" =>  @set defaults.delays = DelayModel(;obs = 0, act = 0, com = 1, ΔT = 5)
]


function run_setting_old(setting)
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

function run_setting(setting, seed)
    rows = []
    for (name, ex) in scenarios
        foreach(CFs) do (cf_str, CF)
            loss, metrics = ex(;plot_result = false, CF, setting, seed)
            metrics["loss"] = sum(loss) / setting.freq
            metrics["loss-state"] = sum(loss[:, :, :x]) / setting.freq
            for (metric, v) in metrics
                row = (task = name, cf = cf_str, metric = metric, value = v)
                push!(rows, row)
            end
        end
    end
    rows
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

"""
Generates `num` random seeds that are unlikely to be correlated.
"""
function independent_seeds(num; seed = 1)
    rng = MersenneTwister(seed)
    (1:num) .+ rand(rng, 1:num*100, num)
end

function run_performance_exps(; num_of_runs)
    OneVision.TrajPlanning.check_optimizer_converge[] = false
    seeds = independent_seeds(num_of_runs)

    @info "Running performance benchmarks ($num_of_runs tasks)..."
    prog = Progress(num_of_runs, 0.1, "Running benchmarks...")
    
    iolock = ReentrantLock()
    all_rows = []
    @time Threads.@threads for seed in seeds
        results = run_setting(defaults, seed)
        lock(iolock) do
            for result in results
                @unpack task, cf, metric, value = result
                push!(all_rows, @ntuple(task, cf, metric, seed, value))
            end
        end
        next!(prog)
    end

    return DataFrame(all_rows)
end

"""
Convert "loss" into "loss sqrt" and drop "loss-state".
"""
function prepare_metrics(df::AbstractDataFrame)
    df = filter(r -> r.metric != "loss-state", df)
    rows = df.metric .== "loss"
    df[rows, :value] = log10.(df[rows, :value])
    df[rows, :metric] .= Ref("log loss")
    df
end

function show_performance_exps(df, out_path = nothing)
    comp_stats(vs) = mean(vs) ± std(vs)

    (out_path === nothing) || mkpath(out_path)
    df = prepare_metrics(df)
    df = groupby(df, Not([:seed, :value]))
    df = combine(df, :value => comp_stats => :value)
    cf_symbols = first.(CFs)
    for subdf in groupby(df, :metric)
        metric = subdf.metric[1]
        if metric == "loss-state" continue end
        println("=== Metric: $(metric) ===")
        table = @_ groupby(subdf, :task) |> 
            combine(__, [:cf, :value] => ((cfs, vs) -> (; zip(cfs, vs)...)) => AsTable)
        display(table)
        (out_path === nothing) || CSV.write(joinpath(out_path, "$(metric).csv"), table)
    end
end

function visualize_benchmark(scenario::String, setting::ExampleSetting, cf::String; seed = 1)
    ex = Dict(scenarios)[scenario]
    CF = Dict(CFs)[Symbol(cf)]
    ex(;plot_result = true, CF, setting, seed)
    nothing
end


function run_variation_exps(; num_of_runs)
    OneVision.TrajPlanning.check_optimizer_converge[] = false
    seeds = independent_seeds(num_of_runs)
    all_tasks = [
        @ntuple variation variable seed setting
        for seed in seeds  # this prevents different seeds from running in parallel and helps precompilation
        for (variation, tasks) in variation_settings() 
        for (variable, setting) in tasks
    ]
    shuffle!(all_tasks)

    @info "Running benchmarks ($(length(all_tasks)) tasks)..."
    prog = Progress(length(all_tasks), 0.1, "Running benchmarks...")
    
    time_used = []
    iolock = ReentrantLock()
    all_rows = []
    @time Threads.@threads for task in all_tasks
        @unpack variation, variable, seed, setting = task
        stats = @timed run_setting(setting, seed)
        lock(iolock) do
            time = stats.time
            push!(time_used, @ntuple variation variable seed time)
            for result in stats.value
                @unpack task, cf, metric, value = result
                r = @ntuple variation variable task cf metric seed value
                push!(all_rows, r)
            end
        end
        next!(prog)
    end

    @_ DataFrame(time_used) |>
        groupby(__, Not(:seed)) |>
        combine(__, :time => mean => :avg_time) |>
        sort!(__, [:avg_time], rev = true) |>
        display

    sort!(all_rows)
    return DataFrame(all_rows)
end

function show_variation_exps(df::DataFrame, out_path = nothing)
    function comp_stats(vs)
        # quart1, quart2, quart3 = quantile(vs, [0.25, 0.5, 0.75])
        (mid = mean(vs), lb = minimum(vs), ub = maximum(vs))
    end

    data = prepare_metrics(df)
    data = groupby(data, Not([:seed, :value]))
    data = combine(data, :value => comp_stats => [:mid, :lb, :ub])
    (out_path === nothing) || mkpath(out_path)
    @showprogress map(d -> draw_variation_metric(d, out_path), groupby(data, [:variation, :metric], sort = true))
    nothing
end

function draw_variation_metric(subdata, out_path)
    var_name = subdata[1, :variation]
    metric_name = subdata[1, :metric]
    # yscale = startswith(metric_name, "loss") ? :log10 : :identity
    cf_names = hcat(first.(CFs)...)
    plots = map(groupby(subdata, :task)) do taskdata
        task_name = taskdata.task[1]
        p = plot(;xlabel = var_name, ylabel = metric_name)
        for cfname in cf_names
            cfdata = filter(x -> x.cf == cfname, taskdata)
            plot!(cfdata.variable, cfdata.mid; 
                title = task_name, #yscale = yscale,
                marker = true, markersize = 3,
                ribbon = (cfdata.mid .- cfdata.lb, cfdata.ub .- cfdata.mid))
        end
        p
    end
    N = length(plots)
    plt = plot(plots...; 
        labels=string.(cf_names), legend = :outertopright, fillalpha = 0.3,
        fontsize = 10, legendfontsize = 6, titlefontsize = 10,
        dpi=300,
        layout = (N, 1), size = (400, 250 * N)) 
    display(plt)
    if out_path !== nothing
        path = joinpath(out_path, "$(var_name)-$(metric_name).png")
        savefig(plt, path)
    end
end


end # module Benchmarks