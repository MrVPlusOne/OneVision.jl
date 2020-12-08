module Car2DExamples

using OneVision
using OneVision: ℝ, 𝕋, ℕ, @kwdef, °
using Random
using StaticArrays
using Parameters

include("common_2D.jl")
include("tracking_example.jl")
include("formation_example.jl")
include("interactive_example.jl")

end  # module Car2DExamples