export ℝ, ℕ, 𝕋, Each, round_ceil, round_floor
export SysDynamics, SysDynamicsLinear, SysDynamicsLTI, discretize
export ObsDynamics, StaticObsDynamics
export sys_forward, limit_control, sys_A, sys_B, sys_w, obs_forward
export DelayModel, WorldDynamics, RegretLossModel, msg_queue_length, short_delay_names
export CentralControl, CentralControlStateless, init_state, control_one, control_all
export Controller, control!, write_logs
export ControllerFramework, make_controllers, MsgQueue
export Timed, attime, TimedQueue, pushpop!

const ℝ = Float64  # use 64-bit precision for real numbers
const ℕ = Int64
"The set of discrete times\n"
const 𝕋 = ℕ

round_ceil(x::ℝ)::ℕ = ℕ(ceil(x))
round_floor(x::ℝ)::ℕ = ℕ(floor(x))

"""
`Each{T}` is a vector containing elements of type `T`,
one for each agent in the fleet.
"""
Each{T} = Vector{T}

"""
A state dynamics model.

 - Should implement: `sys_forward`
 - Can optionally implement: `limit_control`
"""
abstract type SysDynamics end

"Simulate one time step forward using the system dynamics.\n"
function sys_forward(dy::SysDynamics, x::X, u, t::𝕋)::X where {X} @require_impl end

"""
    Restrict controls output by π into some valid range. The default implementation 
    simply returns the original control unmodified.
"""
function limit_control(dy::SysDynamics, u::U, x, t)::U where {U}
    u
end

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
Assume the observation remains the same.
"""
struct StaticObsDynamics <: ObsDynamics end

"""
    obs_forward(dy::ObsDynamics, x, z::Z, t::𝕋)::Z where Z

Forward predict new observation from old observation and current state. Note that
the OneVision will only work well if `z` does not (or weakly) depend on `x`.
"""
function obs_forward(dy::ObsDynamics, x, z::Z, t::𝕋)::Z where {Z} @require_impl end

function obs_forward(
    dy::StaticObsDynamics, x, z, t::𝕋
) z end

struct DelayModel
    "observation delay: Tx"
    obs::𝕋
    "actuation delay: Tu" 
    act::𝕋
    "communication delay: Tc"
    com::𝕋
    "total delay"
    total::𝕋
    "control interaval, i.e., the time between adjacent control steps"
    ΔT::𝕋
end

@inline short_delay_names(dm::DelayModel) = 
    (Tx = dm.obs, Tu = dm.act, Tc = dm.com, Ta = dm.total, ΔT = dm.ΔT)

DelayModel(;obs, act, com, ΔT = 1) = begin 
    @assert com ≥ 1 "Communication delay should be at least 1, but got: $com."
    @assert obs ≥ 0 && act ≥ 0 && ΔT ≥ 1
    DelayModel(obs, act, com, obs + act + com, ΔT)
end

msg_queue_length(dm::DelayModel) = dm.com ÷ dm.ΔT + 1


"""
A centralized controller with actuation type `U` and an internal state of type `S`.
`S` could be mutable and the framework will make a deep copy of the state when needed.

Should implement `control_one` and `init_state`.
"""
abstract type CentralControl{U,S} end

function control_one(
    π::CentralControl{U,S}, s::S, xs, zs, t::𝕋, id::ℕ
)::U  where {U,S}
    @require_impl
end

function control_all(
    π::CentralControl{U,S}, s::S, xs,zs, t::𝕋, ids::AbstractVector{ℕ}
)::AbstractVector{U} where {U,S}
    map(i -> control_one(π, s, xs, zs, t, i), ids)
end

function init_state(f::CentralControl{U,S}, t0)::S where {U,S}
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

function init_state(::CentralControlStateless, t0) nothing end

function init_state(::CentralControl{U,SymbolMaps.SymbolMap}, t0) where U
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

struct WorldDynamics{N,XDynamics <: Tuple,ZDynamics <: Tuple}
    dynamics::XDynamics
    obs_dynamics::ZDynamics

    WorldDynamics(info::Each{<:Tuple{SysDynamics,ObsDynamics}}) = begin
        ds = Tuple(first.(info))
        os = Tuple(last.(info))
        new{length(info),typeof(ds),typeof(os)}(ds, os)
    end
end

struct RegretLossModel{N,X,U,S,XDynamics,UDynamics}
    central::CentralControl{U,S}
    world_model::WorldDynamics{N, XDynamics, UDynamics}
    x_weights::SVector{N, X}
    u_weights::SVector{N, U}
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

struct Timed{X}
    time::𝕋
    value::X
end

Base.getindex(timed::Timed, time::𝕋) = begin
    @assert timed.time == time "expect time = $time, got: $timed."
    timed.value
end

Base.convert(::Type{Timed{Y}}, x::Timed{X}) where {X, Y} =
    if X == Y
        x
    else
        Timed{Y}(x.time, convert(Y, x.value))
    end

@inline attime(t::𝕋) = v -> Timed(t, v)

mutable struct TimedQueue{X}
    in_time::Int
    queue::FixedQueue{Timed{X}}
end

"""
Creates a TimedQueue with all initial elements having the same value.
"""
function TimedQueue(value::X, t1::𝕋, t2::𝕋; eltype::Type{E} = X) where {X, E}
    in_time = t2+1
    queue = FixedQueue([Timed{E}(t, value) for t in t1:t2])
    TimedQueue(in_time, queue)
end

Base.length(q::TimedQueue) = length(q.queue)
Base.isempty(q::TimedQueue) = isempty(q.queue)
Base.getindex(q::TimedQueue, t::𝕋) = q.queue[t]

function pushpop!(q::TimedQueue{T}, x::Timed) where T
    @assert x.time == q.in_time "Queue in_time: $(q.in_time), x = $x"
    q.in_time += 1
    pushpop!(q.queue, x)
end