module Examples

export default_delays, mk_cf
export Car1DExample, Car2DExamples
export CFName, naive_cf, local_cf, const_u_cf, onevision_cf

using OneVision

default_delays = DelayModel(obs = 2, act = 1, com = 3, ΔT = 1)

@enum CFName naive_cf local_cf const_u_cf onevision_cf

@inline function mk_cf(
    name, world_model::WorldDynamics{N}, central, delays, 
    loss_model; X, Z, H
) where N
    if name == naive_cf
        NaiveCF(X, Z, N, central, msg_queue_length(delays), delays.ΔT)
    elseif name == local_cf
        LocalCF(central, world_model, delays; X, Z, conpensate_comm = false)
    elseif name == const_u_cf
        LocalCF(central, world_model, delays; X, Z, conpensate_comm = true)
    elseif name == onevision_cf
        OvCF(loss_model, delays; Z, H)
    else
        error("Unexpected CF name: $name")
    end
end

include("Car1DExample.jl")
include("Car2DExamples.jl")

end # module Examples

# ransac, outliler, robost function