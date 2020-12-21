module Examples

using OneVision

default_delays = DelayModel(obs = 2, act = 4, com = 6, ΔT = 5)

include("Car1DExample.jl")
include("Car2DExamples.jl")
include("benchmarks.jl")

end # module Examples