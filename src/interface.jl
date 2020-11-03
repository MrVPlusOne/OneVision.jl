export ℝ, ℕ, 𝕋, Each
export SysDynamics, SysDynamicsLinear, SysDynamicsLTI, ObsDynamics, discretize
export sys_forward, sys_A, sys_B, sys_w
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
abstract type SysDynamics end

"Simulate one time step forward using the system dynamics.\n"
function sys_forward(dy::SysDynamics, x, u, t::𝕋) @require_impl end

"""
A time-variant linear dynamics of the form ``x(t+1) = A(t) x(t) + B(t) u(t) + w(t)``.
Should implement 3 methods: `sys_A`, `sys_B`, and `sys_w`.
"""
abstract type SysDynamicsLinear <: SysDynamics end

function sys_A(sys::SysDynamicsLinear, t) @require_impl end
function sys_B(sys::SysDynamicsLinear, t) @require_impl end
function sys_w(sys::SysDynamicsLinear, t) @require_impl end

"""
Get the corresponding system matrices from a linear system.
"""
sys_A, sys_B, sys_w

"""
An LTI Dynamics of the form ``x(t+1) = A x(t) + B u(t) + w(t)``.
"""
struct SysDynamicsLTI{MA,MB} <: SysDynamicsLinear
    A::MA
    B::MB
    w::Function
end

function sys_forward(dy::SysDynamicsLTI, x::X, u, t::𝕋)::X where {X}
    (dy.A * x + dy.B * u + dy.w(t)) |> colvec2vec |> v -> convert(X, v)
end

"""
Converts a continous LTI system, given in the form of (A, B), into a 
discrete-time LTI system, given in the form (A′, B′).
"""
function discretize(A::TA, B::TB, delta_t::ℝ)::Tuple{TA,TB} where {TA,TB}
    A_int, err = quadgk(t -> exp(A .* t), 0.0, delta_t)
    # println("discretization error: $err")
    A′ = exp(A .* delta_t)
    B′ = A_int * B
    A′, B′
end

abstract type ObsDynamics end

function obs_forward(dy::ObsDynamics, x::X, z, t::𝕋)::X where {X} @require_impl end
    

@kwdef struct DelayModel
    obs::ℕ  # observation delay
    act::ℕ  # actuation delay
    com::ℕ  # communication delay
end

"A centralized controller with no delays.\n"
abstract type CentralControl end

function control_one(
    f::CentralControl, xs::Each{X}, zs::Each{Z}, id::ℕ
)::U  where {X,Z,U} 
    @require_impl
end

function control_all(
    f::CentralControl, xs::Each{X},zs::Each{Z}, ids::Vector{ℕ}
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

@kwdef struct WorldDynamics
    num_agents::ℕ
    dynamics::Each{<:SysDynamics}
    obs_dynamics::Each{<:ObsDynamics}
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

