export ℝ, ℕ, 𝕋, Each
export SysDynamics, SysDynamicsLTI, ObsDynamics, discretize
export sys_forward, state_names, act_names, obs_names
export DelayModel, WorldDynamics, CentralControl, control_one, control_all
export Controller, control!
export ControllerFramework, make_controllers, MsgQueue

const ℝ = Float64  # use 64-bit precision for real numbers
const ℕ = Int64
"The set of discrete times\n"
const 𝕋 = ℕ

""" 
`Each{T}` is a vector containing elements of type `T`, 
one for each agent in the fleet.
"""
Each{T} = Vector{T}

""" A system dynamics with the state type `X` and action type `U`.
"""
abstract type SysDynamics{X,U} end

"Simulate one time step forward using the system dynamics.\n"
function sys_forward(dy::SysDynamics{X,U}, x::X, u::U, t::𝕋)::X where {X,U} @require_impl end

state_names(::SysDynamics)::Vector{String} = @require_impl
act_names(::SysDynamics)::Vector{String} = @require_impl

struct TestType2{X}
    x::X
end

    """
An LTI Dynamics of the form ``x(t+1) = A x(t) + B u(t) + w(t)``.
"""
struct SysDynamicsLTI{X,U,MA,MB} <: SysDynamics{X,U}
    A::MA
    B::MB
    w::Function
    state_names::Vector{String}
    act_names::Vector{String}
end

function sys_forward(dy::SysDynamicsLTI{X,U,MA,MB}, x::X, u::U, t::𝕋)::X where {X,U,MA,MB}
    (dy.A * x + dy.B * u + (dy.w(t)::X)) |> col_vector |> v -> convert(X, v)
end

state_names(dy::SysDynamicsLTI) = dy.state_names
act_names(dy::SysDynamicsLTI) = dy.act_names

"""
Converts a continous LTI system, given in the form of (A, B), into a 
discrete-time LTI system, given in the form (A′, B′).
"""
function discretize(A::TA, B::TB, delta_t::ℝ)::Tuple{TA,TB} where {TA,TB}
    A_int, err = quadgk(t -> exp(A .* t), 0.0, delta_t)
    println("discretization error: $err")
    A′ = exp(A .* delta_t)
    B′ = A_int * B
    A′, B′
end

abstract type ObsDynamics{X,Z} end

function obs_forward(dy::ObsDynamics{X,Z}, x::X, z::Z, t::𝕋)::X where {X,Z} @require_impl end
    
obs_names(::ObsDynamics)::Vector{String} = @require_impl

@kwdef struct DelayModel
    obs::ℕ  # observation delay
    act::ℕ  # actuation delay
    com::ℕ  # communication delay
end

"A centralized controller with no delays.\n"
abstract type CentralControl{X,Z,U} end

function control_one(
    f::CentralControl{X,Z,U}, xs::Each{X}, zs::Each{Z}, id::ℕ
)::U  where {X,Z,U} 
    @require_impl
end

function control_all(
    f::CentralControl{X,Z,U}, xs::Each{X},zs::Each{Z}, ids::Vector{ℕ}
)::Each{U} where {X,Z,U}
    map(i -> f(xs, zs, i), ids)
end


"A distributed controller, typically generated from a controller framework.\n"
abstract type Controller{X,Z,U,Msg} end

function control!(
    ctrl::Controller{X,Z,U,Msg},
    x::X,
    obs::Z,
    msgs::Each{Msg},
)::Tuple{U,Each{Msg}} where {X,Z,U,Msg} 
    @require_impl 
end

@kwdef struct WorldDynamics{X,Z,U}
    num_agents::ℕ
    dynamics::Each{<:SysDynamics{X,U}}
    obs_dynamics::Each{<:ObsDynamics{X,Z}}
end

WorldDynamics(info::Each{<:Tuple{SysDynamics,ObsDynamics}}) = begin
    WorldDynamics(length(info), first.(info), last.(info))
end

MsgQueue{Msg} = Queue{Each{Msg}}

abstract type ControllerFramework{X,Z,U,Msg} end

function make_controllers(
    framework::ControllerFramework{X,Z,U,Msg},
    init_status::Each{Tuple{X,Z,U}},
)::Tuple{Each{Controller{X,Z,U,Msg}},Each{MsgQueue{Msg}}} where {X,Z,U,Msg}
    @require_impl 
end

