export ℝ, ℕ, 𝕋, Act, State, Obs, Each
export SysDynamics, SysDynamicsLTI, ObsDynamics, discretize
export forward, state_names, act_names, obs_names
export DelayModel, WorldDynamics, CentralControl, control_one, control_all
export Controller, control!
export ControllerFramework, make_controllers, MsgQueue

const ℝ = Float64  # use 64-bit precision for real numbers
const ℕ = Int64
"The set of discrete times\n"
const 𝕋 = ℕ

"The actuation vector, abbreviation: `u`\n"
const Act = Vector{ℝ}
"The state vector, abbreviation: `x`\n"
const State = Vector{ℝ}
"The observation vector, abbreviation: `z`\n"
const Obs = Vector{ℝ}

""" 
`Each{T}` is a vector containing elements of type `T`, 
one for each agent in the fleet.
"""
Each{T} = Vector{T}

abstract type SysDynamics end

"Simulate one time step forward using the system dynamics.\n"
forward(dy::SysDynamics, x::State, u::Act, t::𝕋)::State = @require_impl

state_names(::SysDynamics)::Vector{String} = @require_impl
act_names(::SysDynamics)::Vector{String} = @require_impl

"""
An LTI Dynamics of the form ``x(t+1) = A x(t) + B u(t) + w(t)``.
"""
struct SysDynamicsLTI <: SysDynamics
    A::Matrix{ℝ}
    B::Matrix{ℝ}
    w::Function
    state_names::Vector{String}
    act_names::Vector{String}
end

forward(dy::SysDynamicsLTI, x::State, u::Act, t::𝕋)::State =
    col_vector(dy.A * x + dy.B * u + dy.w(t))

state_names(dy::SysDynamicsLTI) = dy.state_names
act_names(dy::SysDynamicsLTI) = dy.act_names

"""
Converts a continous LTI system, given in the form of (A, B), into a 
discrete-time LTI system, given in the form (A′, B′).
"""
discretize(A::AbstractMatrix, B::AbstractMatrix, delta_t::ℝ) = begin
    A_int, err = quadgk(t -> exp(A .* t), 0.0, delta_t)
    println("discretization error: $err")
    A′ = exp(A .* delta_t)
    B′ = A_int * B
    A′, B′
end

"""
Converts a continous LTI system into a discrete-time LTI system.
"""
discretize(continuous::SysDynamicsLTI, delta_t::ℝ)::SysDynamicsLTI = begin
    A′, B′ = discretize(continuous.A, continuous.B, delta_t)
    result = @set continuous.A = A′
    @set result.B = B′
end

abstract type ObsDynamics end

forward(dy::ObsDynamics, x::State, z::Obs, t::𝕋)::Obs = @require_impl

obs_names(::ObsDynamics)::Vector{String} = @require_impl

@kwdef struct DelayModel
    obs::ℕ  # observation delay
    act::ℕ  # actuation delay
    com::ℕ  # communication delay
end

"A centralized controller with no delays.\n"
abstract type CentralControl end

control_one(f::CentralControl, xs::Each{State},zs::Each{Obs}, id::ℕ)::Act = @require_impl
control_all(f::CentralControl, xs::Each{State},zs::Each{Obs}, ids::Vector{ℕ})::Each{Act} =
    map(i -> f(xs, zs, i), ids)


"A distributed controller, typically generated from a controller framework.\n"
abstract type Controller{Msg} end

function control!(
    ctrl::Controller{Msg},
    x::State,
    obs::Obs,
    msgs::Each{Msg},
)::Tuple{Act,Each{Msg}} where {Msg} 
    @require_impl 
end

@kwdef struct WorldDynamics
    num_agents::ℕ
    dynamics::Each{<:SysDynamics}
    obs_dynamics::Each{<:ObsDynamics}
end

WorldDynamics(info::Each{<:Tuple{SysDynamics,ObsDynamics}}) = begin
    WorldDynamics(length(info), first.(info), last.(info))
end

MsgQueue{Msg} = Queue{Each{Msg}}

abstract type ControllerFramework{Msg} end

function make_controllers(
    framework::ControllerFramework{Msg},
    init_status::Each{Tuple{State,Obs,Act}},
)::Tuple{Each{Controller{Msg}},Each{MsgQueue{Msg}}} where Msg 
    @require_impl 
end

