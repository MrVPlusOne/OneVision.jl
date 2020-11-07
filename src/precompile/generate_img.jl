using PackageCompiler
include("pkg_list.jl")
@elapsed create_sysimage(pkg_list, 
    sysimage_path="JuliaSysimage.dylib", 
    precompile_execution_file="src/precompile/precompile_examples.jl")