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

get_pos(s::CarX) = @SVector[s.x, s.y]

@kwdef struct CarU{R} <: FieldVector{2,R}
    "desired linear speed" 
    v̂::R = 0.0
    "desired steering angle"
    ψ̂::R = 0.0
end

# TODO: fine-tune these parameters
@kwdef struct CarDynamics{NF} <: SysDynamics
    "control time interval in seconds"
    delta_t::ℝ
    "maximal linear speed"
    max_v::ℝ = 5.0
    "maximal steering angle"
    max_ψ::ℝ = 15°
    "wheelbase between the front and rear wheels"
    l::ℝ = 0.324
    "rate of convergence for v to converge to v̂"
    k_v::ℝ = 5.0*5
    "rate of convergence for ψ to converge to ψ̂"
    k_ψ::ℝ = 10.0*1.7
    "add_noise(x, t) -> x′"
    add_noise::NF = (x, t) -> x
    integrator_samples::ℕ = 1
end

ψ_from_v_ω(v, ω, l) = (abs(v) < 1e-4) ? 0.0 : atan(ω * l / v)
ω_from_v_ψ(v, ψ, l) = tan(ψ) * v / l

function u_from_v_ω(v, ω, dy::CarDynamics)
    v = clamp(v, -dy.max_v, dy.max_v)
    ψ = ψ_from_v_ω(v, ω, dy.l)
    ψ = clamp(ψ, -dy.max_ψ, dy.max_ψ)
    CarU(v, ψ)
end

# Simple clamping limit
function OneVision.limit_control(dy::CarDynamics, u::U, x, t)::U where {U}
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
    N = dy.integrator_samples
    dt = dy.delta_t / N
    @inline f(x) = sys_derivates(dy, x, u)

    x′ = integrate_forward_invariant(f, x, dt, RK38, N)
    dy.add_noise(x′,t)
end

"""
Should implement `track_ref`.
"""
abstract type TrackingControl end


@kwdef struct RefPointTrackControl <: TrackingControl
    dy::CarDynamics
    "The distance between the reference point and rear axis, positive means forward"
    ref_pos::ℝ
    ctrl_interval::ℝ
    "Propotional gain"
    kp::ℝ
    "(Discrete) Integral gain"
    ki::ℝ = 0.0
    "(Discrete) Derivative gain"
    kd::ℝ = 0.0
end

function ref_point(ref_pos, s::CarX)
    x = s.x + ref_pos * cos(s.θ)
    y = s.y + ref_pos * sin(s.θ)
    @SVector[x, y]
end

ref_point(K::RefPointTrackControl, s::CarX) = ref_point(K.ref_pos, s)

function ref_point_v(ref_pos, car_l, s::CarX)
    d = ref_pos
    @unpack v, θ, ψ = s
    ω = ω_from_v_ψ(v, ψ, car_l)
    v̂_x = v * cos(θ) - sin(θ) * ω * d
    v̂_y = v * sin(θ) + cos(θ) * ω * d
    @SVector[v̂_x, v̂_y]
end

ref_point_v(K::RefPointTrackControl, s::CarX) = ref_point_v(K.ref_pos, K.dy.l, s)

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
        Δt = K.ctrl_interval
        ∫edt = K.ki == 0 ? zero(p̂) : K.ki * Δt * integral!(ξ, :integral, t, p̂ - p)
        dedt = K.kd == 0 ? zero(p̂) : K.kd / Δt * derivative!(ξ, :derivative, t, p̂ - p)
        K.kp * (p̂ - p) + ∫edt + v_p̂ 
    end
    # convert `v_p` back into the control `CarU`
    d = K.ref_pos
    θ = s.θ
    v, v_y = rotation2D(-θ) * v_p
    ω = v_y / d
    (v < 0) && (ω *= -1)
    u_from_v_ω(v, ω, K.dy)
end


@kwdef struct ConfigTrackControl <: TrackingControl
    dy::CarDynamics
    k1::ℝ
    k2::ℝ
    k3::ℝ
end

ref_point(::ConfigTrackControl, s::CarX) = ref_point(0.0, s)
ref_point_v(K::ConfigTrackControl, s::CarX) = ref_point_v(0.0, K.dy.l, s)

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
    track_ref(ctrl.K, ξ, x̂, x, t)::CarU
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

@kwdef struct CollisionAvoidance
    "Force scale"
    scale::ℝ
    "Force goes to infinity below this distance"
    min_r::ℝ
    "Force drops to zero beyound this distance"
    max_r::ℝ
end

"""
Leader-follower formation control
"""
struct FormationControl{TC <: TrackingControl, F} <: CentralControl{CarU{ℝ}, SymbolMap}
    "formation(xs, t)::Formation{ℝ}"
    formation::F
    "TrackingControl"
    K::TC 
    dy::CarDynamics
    avoidance::CollisionAvoidance
end

const repl_rotation = rotation2D(10°)
function repel_force(avoidance::CollisionAvoidance, center, pos)
    @unpack scale, min_r, max_r = avoidance
    r = pos - center
    d = norm(r)
    (d > max_r) && return zero(center)

    α = max(eps(), (d - min_r) / (max_r - min_r))
    mag = scale * (1/α)
    repl_rotation * (r / d) * mag
end

function to_formation_frame(ctrl::FormationControl, s_leader)
    K = ctrl.K
    rot = rotation2D(s_leader.θ)
    pos_offset = ref_point(K, s_leader)
    v_offset = ref_point_v(K, s_leader)
    ω = ω_from_v_ψ(s_leader.v, s_leader.ψ, ctrl.dy.l)

    function formpoint_to_refpoint(fp)
        pos = rot * fp + pos_offset
        x, y = fp
        v = rot * @SVector[-ω*y, ω*x] + v_offset
        (pos = pos, v = v)
    end

    formpoint_to_refpoint
end

function formation_controller(ctrl::FormationControl{RefPointTrackControl}, ξ, xs, zs, t)
    formpoint_to_refpoint = to_formation_frame(ctrl, xs[1])
    form = ctrl.formation(xs, zs, t)

    N = length(xs)
    function avoid_collision(i)
        ca = ctrl.avoidance
        pos = get_pos(xs[i]) # ref_point(ctrl.K, xs[i])
        sum(repel_force(ca, get_pos(xs[j]), pos) for j in 1:N if j != i)
    end

    function action(id)::CarU
        (id == 1) && return zs[1].c

        s = form[id]
        (p, v_p) = formpoint_to_refpoint(@SVector[s.x, s.y])
        ξi = submap(ξ, Symbol(id))
        v_o = avoid_collision(id)
        track_refpoint(ctrl.K, ξi, (p, v_p + v_o), xs[id], t)
    end

    action
end

"""
Compute the `CarX` that rotates around the given `center` at the specified angular 
velocity `ω` 
"""
function rotate_around(center, ω, l, pos)
    x, y = pos
    dir = pos - center
    θ = atan(dir[2], dir[1]) + sign(ω) * 90°
    r = norm(dir)
    v = r * abs(ω)
    ψ = ψ_from_v_ω(v, ω, l)
    CarX(x, y, θ, v, ψ)
end

function center_of_rotation(s::CarX, l)
    ω = ω_from_v_ψ(s.v, s.ψ, l)
    r = ω ≈ 0 ? sign(ω) * 1e10 : s.v / ω
    θ = s.θ
    center = get_pos(s) + @SVector[-sin(θ), cos(θ)] * r
    center, ω
end


function formation_controller(ctrl::FormationControl{ConfigTrackControl}, ξ, xs, zs, t)
    s_leader = xs[1]
    @unpack l = ctrl.dy
    θ = s_leader.θ
    center, ω = center_of_rotation(s_leader, l)

    form = ctrl.formation(xs, zs, t)
    s′ = form[1]
    rot = rotation2D(s_leader.θ - s′.θ)
    offset = get_pos(s_leader) - get_pos(s′)

    function action(id)::CarU
        (id == 1) && return zs[1].c

        s = form[id]
        pos = rot * get_pos(s) + offset
        ŝ = rotate_around(center, ω, l, pos)
        ξi = submap(ξ, Symbol(id))
        track_ref(ctrl.K, ξi, ŝ, xs[id], t)
    end

    action
end

function OneVision.control_one(ctrl::FormationControl, ξ::SymbolMap, xs, zs, t::𝕋, id::ℕ)
    formation_controller(ctrl, ξ, xs, zs, t)(id)::CarU
end

function OneVision.control_all(
    ctrl::FormationControl, ξ::SymbolMap, xs, zs, t::𝕋, ids:: AbstractVector{ℕ}
)
    f = formation_controller(ctrl, ξ, xs, zs, t)
    f.(ids)
end

function car_triangle(x, y, θ; len = 0.2, width = 0.06)
    base = Point2f0(x, y)
    dir = Point2f0(cos(θ), sin(θ))
    left = Point2f0(sin(θ), -cos(θ))
    p1 = base + dir * len
    p2 = base + left * width
    p3 = base - left * width
    [p1,p2,p3]
end

function with_alpha(color, alpha)
    @unpack r, g, b = color
    RGBAf0(r, g, b, alpha)
end
