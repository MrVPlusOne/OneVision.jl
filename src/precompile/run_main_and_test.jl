include("../../test/runtests.jl")
include("../runner.jl")
import Makie
include(joinpath(pkgdir(Makie), "test", "test_for_precompile.jl"))