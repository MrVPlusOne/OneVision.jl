using ProgressMeter
using DataFrames

benchmarks = [
    "1D Leader Brake" => Car1DExample.run_example,
    "2D Formation Driving" => Car2DExamples.formation_example
]

CFs = [
    :Naive => NaiveCF, 
    :Local => LocalCF, 
    :OneVision => OvCF,
]

function run_benchmarks()

    freq = 100.0
    defaults = Dict([
        :freq => 100.0
        :noise => 0.1 / sqrt(freq)
        :delays => DelayModel(obs = 3, act = 4, com = 5, ΔT = 5)
    ])

    setting_args = [
        "Default" => defaults
        "Noiseless" => @set defaults[:noise] = 0.0
        "Larger Delays" =>  @set defaults[:delays].com *= 4
        "Lower Freq" => 
            let m1 = @set defaults[:freq] = 20.0
                @set m1[:delays] = DelayModel(obs = 1, act = 1, com = 1, ΔT = 1)
            end
    ]
    
    num_tasks = length(setting_args) * length(benchmarks)
    prog = Progress(num_tasks+1, 0.1, "Running benchmarks...")

    for (setname, setting) in setting_args
        rows = []
        for (name, ex) in benchmarks
            results = [
                cf_name => sum(ex(;plot_result = false, CF, setting...)) 
                for (cf_name, CF) in CFs]
            push!(rows, (task = name, results...))
            next!(prog)
        end
        sep = " "^displaysize(stdout)[2]
        println("\r$sep")
        println("Setting: $setname")
        DataFrame(rows) |> display
    end
    finish!(prog)
end