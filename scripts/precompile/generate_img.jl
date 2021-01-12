using DrWatson
@quickactivate "OneVision"

using PackageCompiler
include("pkg_list.jl")
println("Creating Sysimage...")
time_taken = @elapsed create_sysimage(pkg_list, 
    sysimage_path="JuliaSysimage.dylib", 
    precompile_execution_file="scripts/precompile/run_tests.jl"
)
println(time_taken)