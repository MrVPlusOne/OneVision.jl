using ProgressMeter
using DataFrames

benchmarks = [
    "1D Leader Brake" => Car1DExample.run_example,
    "2D Formation Driving" => Car2DExamples.formation_example
]

CFs = [
    :Naive => NaiveCF, 
    :Local => LocalCF, 
    :OneVision => OvCF
]

function run_benchmarks()
    
    rows = []
    
    @showprogress "Running Benchmarks" for (name, ex) in benchmarks
        results = [cf_name => sum(ex(;plot_result = false, CF)) for (cf_name, CF) in CFs]
        push!(rows, (task = name, results...))
    end
    DataFrame(rows)
end