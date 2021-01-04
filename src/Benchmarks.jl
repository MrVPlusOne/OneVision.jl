module Benchmarks

export run_benchmarks, show_benchmark

using OneVision
using OneVision.Examples

using ProgressMeter
using DataFrames

function with_args(f; kws...)
    (args...; kws2...) -> f(args...; kws2..., kws...)
end

benchmarks = [
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
    num_tasks = length(setting_args) * length(benchmarks)
    prog = Progress(num_tasks+1+length(setting_args), 0.1, "Running benchmarks...")

    iolock = ReentrantLock()

    function display_table(rows, name)
        sep = " "^displaysize(stdout)[2]
        println("\r$sep")
        println("Setting: $name")
        DataFrame(rows) |> display
    end

    tables = Dict{String, Any}()
    Threads.@threads for (setname, setting) in setting_args
        rows = []
        for (name, ex) in benchmarks
            results = [
                cf_str => sum(ex(;plot_result = false, CF, setting...)) / setting[:freq]
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
    for (setname, _) in setting_args
        display_table(tables[setname], setname)
    end
end

function show_benchmark(bench_name::String, setting_name::String, cf_name::Symbol)
    bench_name = strip(bench_name)
    setting_name = strip(setting_name)

    ex = Dict(benchmarks)[bench_name]
    setting = Dict(setting_args)[setting_name]
    CF = Dict(CFs)[cf_name]
    loss = ex(;plot_result = true, CF, setting...)
    nothing
end

end # module Benchmarks