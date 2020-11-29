using OneVision.NumericalIntegration

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

struct CarZ{R} <: FieldVector{0,R} end

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

ψ_from_v_ω(v, ω, l) = atan(ω * l, v)
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
    
    X(ẋ,ẏ,θ̇,v̇,ψ̇)
end

struct CarObsDynamics <: ObsDynamics end

function OneVision.sys_forward(dy::CarDynamics, x::X, u, t::𝕋)::X where X
    u = limit_control(dy, u)

    N = 1
    dt = dy.delta_t / N
    @inline f(x) = sys_derivates(dy, x, u)

    integrate_forward_invariant(f, x, dt, RK38, N)
end

function OneVision.obs_forward(
    dy::CarObsDynamics, x::CarX, z::CarZ, t::𝕋
) z end

"""
Should implement `track_ref`.
"""
abstract type TrackingControl end


@kwdef struct RefPointTrackControl <: TrackingControl
    dy::CarDynamics
    "The distance between the reference point and rear axis, positive means forward"
    ref_pos::ℝ
    "The propotional gain"
    k::ℝ
end

function ref_point(K::RefPointTrackControl, s::CarX)
    d = K.ref_pos
    x = s.x + d * cos(s.θ)
    y = s.y + d * sin(s.θ)
    SVector(x, y)
end

function ref_point_v(K::RefPointTrackControl, ŝ::CarX)
    d = K.ref_pos
    @unpack v, θ, ψ = ŝ
    ω = ω_from_v_ψ(v, ψ, K.dy.l)
    v̂_x = v * cos(θ) - sin(θ) * ω * d
    v̂_y = v * sin(θ) + cos(θ) * ω * d
    SVector(v̂_x, v̂_y)
end

function track_ref(
    K::RefPointTrackControl, ŝ::CarX{R}, s::CarX{R}
)::CarU{R} where R
    # compute the desired ref point velocity
    p, p̂ = ref_point(K, s), ref_point(K, ŝ)
    v_p̂ = ref_point_v(K, ŝ)
    v_p = v_p̂ + K.k * (p̂ - p)
    # convert `v_p` back into the control `CarU`
    d = K.ref_pos
    θ = s.θ
    v = cos(θ) * v_p[1] + sin(θ) * v_p[2]
    ω = -sin(θ) / d * v_p[1] + cos(θ) / d * v_p[2]
    u_from_v_ω(v, ω, K.dy)
end


@kwdef struct TrajectoryTrackControl <: TrackingControl
    dy::CarDynamics
    k1::ℝ
    k2::ℝ
    k3::ℝ
end


"""
Full configuration tracking control for a 2D Car.
"""
function track_ref(
    K::TrajectoryTrackControl, x̂::CarX{R}, x::CarX{R}
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

struct RefTrackCentralControl{TC <: TrackingControl, Tr} <: CentralControlStateless{CarU{ℝ}}
    K::TC
    trajectories::FuncT{Tuple{ℕ,𝕋},CarX{ℝ},Tr}
end

function OneVision.control_one(
    ctrl::RefTrackCentralControl, xs,zs, t::𝕋, id::ℕ
)
    x = xs[id]
    x̂ = ctrl.trajectories((id,t))
    track_ref(ctrl.K, x̂, x)
end