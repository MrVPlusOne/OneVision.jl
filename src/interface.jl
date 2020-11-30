export ℝ, ℕ, 𝕋, Each
export SysDynamics, SysDynamicsLinear, SysDynamicsLTI, ObsDynamics, discretize
export sys_forward, sys_A, sys_B, sys_w, obs_forward
export DelayModel, WorldDynamics 
export CentralControl, CentralControlStateless, init_state, control_one, control_all
export Controller, control!, write_logs
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
function sys_forward(dy::SysDynamics, x::X, u, t::𝕋)::X where {X} @require_impl end

"""
A time-variant linear dynamics of the form ``x(t+1) = A(t) x(t) + B(t) u(t) + w(t)``.
Should implement 3 methods: `sys_A`, `sys_B`, and `sys_w`.
"""
abstract type SysDynamicsLinear <: SysDynamics end

function sys_A(sys::SysDynamicsLinear, t) @require_impl end
function sys_B(sys::SysDynamicsLinear, t) @require_impl end
function sys_w(sys::SysDynamicsLinear, t) @require_impl end

function sys_forward(dy::SysDynamicsLinear, x::X, u, t::𝕋)::X where {X}
    sys_A(dy, t) * x + sys_B(dy, t) * u + (sys_w(dy, t)::X)
end

"""
Get the corresponding system matrices from a linear system.
"""
sys_A, sys_B, sys_w

"""
An LTI Dynamics of the form ``x(t+1) = A x(t) + B u(t) + w(t)``.
"""
struct SysDynamicsLTI{MA,MB,NF <: Function} <: SysDynamicsLinear
    A::MA
    B::MB
    w::NF
end

sys_A(s::SysDynamicsLTI, t) = s.A
sys_B(s::SysDynamicsLTI, t) = s.B
sys_w(s::SysDynamicsLTI, t) = s.w(t)

function sys_forward(dy::SysDynamicsLTI, x::X, u, t::𝕋)::X where {X}
    dy.A * x + dy.B * u + convert(X, dy.w(t))
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


"""
    obs_forward(dy::ObsDynamics, x, z::Z, t::𝕋)::Z where Z

Forward predict new observation from old observation and current state. Note that
the OneVision will only work well if `z` does not (or weakly) depend on `x`.
"""
function obs_forward(dy::ObsDynamics, x, z::Z, t::𝕋)::Z where {Z} @require_impl end


struct DelayModel
    obs::ℕ  # observation delay: Tx
    act::ℕ  # actuation delay: Tu
    com::ℕ  # communication delay: Tc
    total::ℕ

    DelayModel(;obs, act, com) = begin 
        @assert com ≥ 1 "Communication delay should be at least 1, but got: $com."
        new(obs, act, com, obs + act + com)
    end
end

# function Base.getproperty(dm::DelayModel, s::Symbol)
#     if 
# end

"""
A centralized controller with actuation type `U` and an internal state of type `S`.
`S` could be mutable and the framework will make a deep copy of the state when needed.

Should implement `control_one` and `init_state`.
"""
abstract type CentralControl{U,S} end

function control_one(
    f::CentralControl{U,S}, s::S, xs, zs, t::𝕋, id::ℕ
)::U  where {U,S}
    @require_impl
end

function control_all(
    π::CentralControl{U,S}, s::S, xs,zs, t::𝕋, ids::AbstractVector{ℕ}
)::AbstractVector{U} where {U,S}
    map(i -> control_one(π, s, xs, zs, t, i), ids)
end

function init_state(f::CentralControl{U,S})::S where {U,S}
    @require_impl
end

"""
A stateless central control. Only needs to implement the following function:
```
    control_one(f, xs, zs, t, id)
```
"""
abstract type CentralControlStateless{U} <: CentralControl{U,Nothing} end

function control_one(f::CentralControlStateless, xs, zs, t::𝕋, id::ℕ) @require_impl end

function control_one(
    f::CentralControlStateless{U}, ::Nothing, xs, zs, t::𝕋, id::ℕ
)::U  where {U}
    control_one(f, xs, zs, t, id)
end

function init_state(::CentralControlStateless) nothing end

function init_state(::CentralControl{U,SymbolMaps.SymbolMap}) where U
    SymbolMaps.SymbolMap()
end

"A distributed controller, typically generated from a controller framework.\n"
abstract type Controller{X,Z,U,Msg,Log} end

function control!(
    ctrl::Controller{X,Z,U,Msg},
    x::X,
    obs::Z,
    msgs::Each{Msg},
)::Tuple{U,Each{Msg}} where {X,Z,U,Msg}
    @require_impl
end

"""
This function will be called at the end of the simulation.
"""
function write_logs(
    ctrl::Controller{X,Z,U,Msg,Log} where {X,Z,U,Msg}
)::Dict{𝕋,Log} where Log
    @require_impl
end

struct WorldDynamics{N,XDynamics <: Tuple,UDynamics <: Tuple}
    dynamics::XDynamics
    obs_dynamics::UDynamics

    WorldDynamics(info::Each{<:Tuple{SysDynamics,ObsDynamics}}) = begin
        ds = Tuple(first.(info))
        os = Tuple(last.(info))
        new{length(info),typeof(ds),typeof(os)}(ds, os)
    end
end


MsgQueue{Msg} = FixedQueue{Each{Msg}}

abstract type ControllerFramework{X,Z,U,Msg,Log} end

"""
Returns a tuple of `(controllers, message queues)`
"""
function make_controllers(
    framework::ControllerFramework{X,Z,U,Msg},
    init_status::Each{Tuple{X,Z,U}},
    init_t::𝕋,
) where {X,Z,U,Msg}
    @require_impl
end
