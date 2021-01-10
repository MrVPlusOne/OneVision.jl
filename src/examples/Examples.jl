module Examples

export default_delays, mk_cf, ExampleSetting
export Car1DExample, Car2DExamples
export CFName, naive_cf, local_cf, const_u_cf, onevision_cf

using OneVision
using OneVision: ℝ, 𝕋, @kwdef

default_delays = DelayModel(obs = 3, act = 4, com = 6, ΔT = 5)

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

@kwdef mutable struct ExampleSetting
    "physics freqency"
    freq::ℝ
    "Sensor noise strength"
    sensor_noise::ℝ
    "external disturbance strength"
    noise::ℝ
    "Delay model"
    delays::DelayModel
    "Prediction horizon"
    H::𝕋
    "The error ratio between the modeled and true dynamics"
    model_error::ℝ
end

include("Car1DExample.jl")
include("Car2DExamples.jl")

end # module Examples