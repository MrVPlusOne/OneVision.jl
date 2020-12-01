using OneVision.NumericalIntegration
using OneVision.SymbolMaps

@kwdef struct CarX{R} <: FieldVector{5,R}
    "x position"
    x::R
    "y position"
    y::R
    "orientation"
    θ::R
    "linear speed"  
    v::R = 0.0
    "steering angle"
    ψ::R = 0.0
end

@kwdef struct CarU{R} <: FieldVector{2,R}
    "desired linear speed" 
    v̂::R = 0.0
    "desired steering angle"
    ψ̂::R = 0.0
end

# TODO: fine-tune these parameters
@kwdef struct CarDynamics <: SysDynamics
    "control time interval in seconds"
    delta_t::ℝ
    "maximal linear speed"
    max_v::ℝ = 5.0
    "maximal steering angle"
    max_ψ::ℝ = 60°
    "wheelbase between the front and rear wheels"
    l::ℝ = 0.1
    "rate of convergence for v to converge to v̂"
    k_v::ℝ = 5.0
    "rate of convergence for ψ to converge to ψ̂"
    k_ψ::ℝ = 5.0
end

ψ_from_v_ω(v, ω, l) = v ≈ 0 ? atan(ω * l, v) :  atan(ω * l / v)
ω_from_v_ψ(v, ψ, l) = tan(ψ) * v / l

function u_from_v_ω(v, ω, dy::CarDynamics)
    v = clamp(v, -dy.max_v, dy.max_v)
    ψ = ψ_from_v_ω(v, ω, dy.l)
    ψ = clamp(ψ, -dy.max_ψ, dy.max_ψ)
    CarU(v, ψ)
end


function limit_control(dy::CarDynamics, u::U)::U where {U}
    v̂, ψ̂ = u
    v̂1 = clamp(v̂, -dy.max_v, dy.max_v)
    ψ̂1 = clamp(ψ̂, -dy.max_ψ, dy.max_ψ)
    if v̂1 == v̂ && ψ̂1 == ψ̂
        u
    else
        U(v̂, ψ̂)
    end
end


@inline function sys_derivates(dy::CarDynamics, x::X, u)::X where X
    x, y, θ, v, ψ = x
    v̂, ψ̂ = u

    ẋ = cos(θ) * v
    ẏ = sin(θ) * v
    θ̇ = ω_from_v_ψ(v, ψ, dy.l)
    v̇ = dy.k_v * (v̂ - v)
    ψ̇ = dy.k_ψ * (ψ̂ - ψ)
    
    X(ẋ, ẏ, θ̇, v̇, ψ̇)
end

function OneVision.sys_forward(dy::CarDynamics, x::X, u, t::𝕋)::X where X
    u = limit_control(dy, u)

    N = 1
    dt = dy.delta_t / N
    @inline f(x) = sys_derivates(dy, x, u)

    integrate_forward_invariant(f, x, dt, RK38, N)
end

"""
Should implement `track_ref`.
"""
abstract type TrackingControl end


@kwdef struct RefPointTrackControl <: TrackingControl
    dy::CarDynamics
    "The distance between the reference point and rear axis, positive means forward"
    ref_pos::ℝ
    delta_t::ℝ
    "Propotional gain"
    kp::ℝ
    "(Discrete) Integral gain"
    ki::ℝ = 0.0
    "(Discrete) Derivative gain"
    kd::ℝ = 0.0
end

function ref_point(K::RefPointTrackControl, s::CarX)
    d = K.ref_pos
    x = s.x + d * cos(s.θ)
    y = s.y + d * sin(s.θ)
    @SVector[x, y]
end

function ref_point_v(K::RefPointTrackControl, ŝ::CarX)
    d = K.ref_pos
    @unpack v, θ, ψ = ŝ
    ω = ω_from_v_ψ(v, ψ, K.dy.l)
    v̂_x = v * cos(θ) - sin(θ) * ω * d
    v̂_y = v * sin(θ) + cos(θ) * ω * d
    @SVector[v̂_x, v̂_y]
end

function track_ref(
    K::RefPointTrackControl, ξ::SymbolMap, ŝ::CarX{R}, s::CarX{R}, t
)::CarU{R} where R
    # compute the desired ref point velocity
    p̂ = ref_point(K, ŝ)
    v_p̂ = ref_point_v(K, ŝ)

    track_refpoint(K, ξ, (p̂, v_p̂), s, t)
end

function track_refpoint(
    K::RefPointTrackControl, ξ::SymbolMap, (p̂, v_p̂), s::CarX{R}, t
)::CarU{R} where R
    ξ = submap(ξ, :track_refpoint)
    p = ref_point(K, s)
    v_p = let
        Δt = K.delta_t
        ∫edt = K.ki == 0 ? zero(p̂) : K.ki * Δt * integral!(ξ, :integral, t, p̂ - p)
        dedt = K.kd == 0 ? zero(p̂) : K.kd / Δt * derivative!(ξ, :derivative, t, p̂ - p)
        K.kp * (p̂ - p) + ∫edt + v_p̂ 
    end
    # convert `v_p` back into the control `CarU`
    d = K.ref_pos
    θ = s.θ
    v, v_y = rotation2D(-θ) * v_p
    ω = v_y / d
    u_from_v_ω(v, ω, K.dy)
end


@kwdef struct ConfigTrackControl <: TrackingControl
    dy::CarDynamics
    k1::ℝ
    k2::ℝ
    k3::ℝ
end


"""
Full configuration tracking control for a 2D Car.
"""
function track_ref(
    K::ConfigTrackControl, ξ, x̂::CarX{R}, x::CarX{R}, t
)::CarU{R} where R 
    θ_d, x_d, y_d, v_d = x̂.θ, x̂.x, x̂.y, x̂.v
    w_d = ω_from_v_ψ(x̂.v, x̂.ψ, K.dy.l)
    θ_e = x.θ - θ_d
    Δx = x.x - x_d
    Δy = x.y - y_d
    x_e =  cos(θ_d) * Δx + sin(θ_d) * Δy
    y_e = -sin(θ_d) * Δx + cos(θ_d) * Δy
    
    co = cos(θ_e) 
    ta = tan(θ_e)
    v̂ = (v_d - K.k1 * abs(v_d) * (x_e + y_e * ta)) / co
    ŵ = w_d - (K.k2 * v_d * y_e + K.k3 * abs(v_d) * ta) * co * co
    u_from_v_ω(v̂, ŵ, K.dy)
end

struct RefTrackCentralControl{TC <: TrackingControl,Tr} <: CentralControl{CarU{ℝ},SymbolMap}
    K::TC
    trajectories::FuncT{Tuple{ℕ,𝕋},CarX{ℝ},Tr}
end

function OneVision.control_one(
    ctrl::RefTrackCentralControl, ξ::SymbolMap, xs, zs, t::𝕋, id::ℕ
)
    x = xs[id]
    x̂ = ctrl.trajectories((id, t))
    ξ = submap(ξ, Symbol(id))
    track_ref(ctrl.K, ξ, x̂, x, t)
end

# === formation driving example ====
const Formation{R} = Vector{CarX{R}}

function rotate_formation(form::Formation{R}, α)::Formation{R} where R
    mat = rotation2D(α)
    function f(s::CarX{R})
        @unpack x, y, θ, v, ψ = s
        x, y = mat * @SVector([x, y])
        θ += α
        CarX(; x, y, θ, v, ψ)
    end
    f.(form)
end

"""
Leader-follower formation control
"""
struct FormationControl <: CentralControl{CarU{ℝ}, SymbolMap}
    formation::Formation{ℝ}
    K::RefPointTrackControl
    dy::CarDynamics
end

function formation_controller(ctrl, ξ, xs, zs, t)
    K = ctrl.K
    s_leader = xs[1]
    rot = rotation2D(s_leader.θ)
    pos_offset = ref_point(K, s_leader)
    v_offset = ref_point_v(K, s_leader)
    ω = ω_from_v_ψ(s_leader.v, s_leader.ψ, ctrl.dy.l)

    function formpoint_to_refpoint(fp)
        pos = rot * fp + pos_offset
        x, y = fp
        v = @SVector[-ω*y, ω*x] + v_offset
        pos, v
    end

    function action(id)
        (id == 1) && return zs[1]

        s = ctrl.formation[id]
        p_ref = formpoint_to_refpoint(@SVector[s.x, s.y])
        ξi = submap(ξ, Symbol(id))
        track_refpoint(K, ξi, p_ref, xs[id], t)
    end

    action
end

function OneVision.control_one(ctrl::FormationControl, ξ::SymbolMap, xs, zs, t::𝕋, id::ℕ)
    formation_controller(ctrl, ξ, xs, zs, t)(id)
end

function OneVision.control_all(
    ctrl::FormationControl, ξ::SymbolMap, xs, zs, t::𝕋, ids:: AbstractVector{ℕ}
)
    f = formation_controller(ctrl, ξ, xs, zs, t)
    f.(ids)
end