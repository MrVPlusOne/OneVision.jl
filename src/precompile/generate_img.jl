using PackageCompiler
include("pkg_list.jl")
@elapsed create_sysimage(pkg_list, 
    sysimage_path="JuliaSysimage.dylib", 
    # precompile_execution_file="src/precompile/precompile_examples.jl"
    precompile_execution_file="src/precompile/run_main_and_test.jl"
)