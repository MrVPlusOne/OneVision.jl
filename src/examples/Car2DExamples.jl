module Car2DExamples

using OneVision
using OneVision: ℝ, 𝕋, ℕ, @kwdef, °, @set, @_
using OneVision.NumericalIntegration
using OneVision.SymbolMaps
using OneVision.Examples
using Random
using StaticArrays
using AxisArrays
using Parameters
using AbstractPlotting
using Makie
using AbstractPlotting.MakieLayout
using Printf: @sprintf
using LinearAlgebra: norm
using ConfParser
import ColorSchemes
import Dates

include("common_2D.jl")
include("tracking_example.jl")
include("formation_example.jl")
include("interactive_example.jl")

end  # module Car2DExamples