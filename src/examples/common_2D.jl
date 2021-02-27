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
    ##CarX(x, y, θ, v=0.0, ψ = 0.0) = new{typeof(x)}(x, y, θ, v, ψ)
    #function CarX{R}(;x::R, y::R, θ::R,v::R = 0.0, ψ::R =0.0) where {R}
    #    new{R}(x, y, restrict(θ), v, restrict(ψ))
    #end
end

#function CarX{R}(;x::R, y::R, θ::R,v::R = 0.0, ψ::R =0.0) where {R}
#    CarX{R}(x, y, restrict(θ), v, restrict(ψ))
#end

#CarX{R}(x::R, y::R, θ::R, v::R = 0.0, ψ::R =0.0) where {R} = CarX{R}(x, y, restrict(θ), v, restrict(ψ))
#CarX{R}(x::R, y::R, θ::R) where {R} = CarX{R}(x, y, restrict(θ), 0.0 , 0.0)

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
    l::ℝ = 0.324 # TODO: change to old default value
    "rate of convergence for v to converge to v̂"
    k_v::ℝ = 1.0
    "rate of convergence for ψ to converge to ψ̂"
    k_ψ::ℝ = 1.0
    "maximum linear acceleration/deceleration per frequency"
    k_max_a::ℝ = 0.25
    "maximum angular acceleration/deceleration per frequency"
    k_max_ω::ℝ = 0.001°

    "add_noise(x, t) -> x′"
    add_noise::NF = (x, t) -> x
    integrator_samples::ℕ = 1
end

ψ_from_v_ω(v, ω, l) = (abs(v) < 1e-4) ? 0.0 : atan(ω * l / v)
ω_from_v_ψ(v, ψ, l) = tan(ψ) * v / l

function u_from_v_ω(v, ω, dy::CarDynamics)
    ψ = ψ_from_v_ω(v, ω, dy.l)
    v = clamp(v, -dy.max_v, dy.max_v)
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
        U(v̂1, ψ̂1)
        #U(v̂, ψ̂)
    end
end


@inline function sys_derivates(dy::CarDynamics, x::X, u)::X where X
    x, y, θ, v, ψ = x
    v̂, ψ̂ = u
    #v = restrict(v)
    ψ̂ = restrict(ψ̂ )
    θ = restrict(θ)
    ψ = restrict(ψ)
    ẋ = cos(θ) * v
    ẏ = sin(θ) * v
    θ̇ = ω_from_v_ψ(v, ψ, dy.l)
    v̇ = dy.k_v * (v̂ - v)
    ψ_dir = dy.k_ψ * restrict(ψ̂ - ψ)
    
    # bounds the variables
    N = dy.integrator_samples
    v̇_min, v̂_max = -dy.k_max_a/N, dy.k_max_a/N#v*(1.0+dy.k_max_v/N), v*(1.0-dy.k_max_v/N)
    v̇ = clamp(v̇, v̇_min, v̂_max)
    if v == 0 && v̇ < 0.15 
        v̇ = 0.0
    end
    ψ_dir_min, ψ_dir_max= -dy.k_max_ω/N, dy.k_max_ω/N#ψ*(1.0+dy.k_max_ψ/N), ψ*(1.0-dy.k_max_ψ/N)
    ψ_dir = clamp(ψ_dir, ψ_dir_min, ψ_dir_max)
    X(ẋ, ẏ, θ̇, v̇, ψ_dir)
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
    "(Discrete) velocity gain"
    kv::ℝ = 0.5
end

@kwdef struct WallObsControl <: TrackingControl
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
    "(Discrete) velocity gain"
    kv::ℝ = 0.5
    "1dtoc values"
    max_a::ℝ = 4.0
    "Stopping distance"
    stop_distance::ℝ =0.0
    "Stopping distance tolerance"
    stop_dist_tol::ℝ = 5e-3
    "(leader car only) target velocity"
    target_v::ℝ =0.0
    "maximum range of sensor"
    sensor_range::ℝ = 10.0
    "point 1"
    p1::Vector{ℝ} = [0.0, 0.0]
    "point 2"
    p2::Vector{ℝ} = [0.0, 1.0]
    "radius "
    r::ℝ = 1.0
    "leader pid config"
    kp_leader::ℝ=0.2
    ki_leader::ℝ=0.05
    kd_leader::ℝ=0.0
    kv_leader::ℝ=1.0
end

function ref_point(ref_pos, s::CarX)
    @info "ref pos is $ref_pos, type is $(typeof(ref_pos))"
    x = s.x + ref_pos * cos(s.θ)
    y = s.y + ref_pos * sin(s.θ)
    @SVector[x, y]
end

ref_point(K::Union{RefPointTrackControl, WallObsControl}, s::CarX) = ref_point(K.ref_pos, s)

function ref_point_v(ref_pos, car_l, s::CarX)
    d = ref_pos
    @unpack v, θ, ψ = s
    ω = ω_from_v_ψ(v, ψ, car_l)
    v̂_x = v * cos(θ) - sin(θ) * ω * d
    v̂_y = v * sin(θ) + cos(θ) * ω * d
    @SVector[v̂_x, v̂_y]
end

ref_point_v(K::Union{RefPointTrackControl, WallObsControl}, s::CarX) = ref_point_v(K.ref_pos, K.dy.l, s)

function track_ref(
    K::Union{RefPointTrackControl, WallObsControl}, ξ::SymbolMap, ŝ::CarX{R}, s::CarX{R}, t
)::CarU{R} where R
    # compute the desired ref point velocity
    p̂ = ref_point(K, ŝ)
    v_p̂ = ref_point_v(K, ŝ)

    track_refpoint(K, ξ, (p̂, v_p̂), s, t)
end

function track_refpoint(
    K::WallObsControl,lξ::SymbolMap, (p̂, v_p̂), s::CarX{R}, t
)::CarU{R} where R
    track_refpoint(K.kp, K.ki, K.kd, K.kv, K, ξ, (p̂, v_p̂), s, t)
end

function track_refpoint(
    kp::ℝ, ki::ℝ, kd::ℝ, kv::ℝ, K::Union{RefPointTrackControl, WallObsControl}, ξ::SymbolMap, (p̂, v_p̂), s::CarX{R}, t
)::CarU{R} where R
    ξ = submap(ξ, :track_refpoint)
    p = ref_point(K, s)
    v_p = let
        Δt = K.ctrl_interval
        ∫edt = ki == 0 ? zero(p̂) : ki * Δt * integral!(ξ, :integral, t, p̂ - p)
        dedt = kd == 0 ? zero(p̂) : kd / Δt * derivative!(ξ, :derivative, t, p̂ - p)
        p_ctrl = kp * (p̂ - p)
        ∫edt = [clamp(∫edt[i], -abs(p_ctrl[i])*0.1, abs(p_ctrl[i])*0.1) for i in 1:length(∫edt)]
        dedt = [clamp(dedt[i], -abs(p_ctrl[i])*0.1, abs(p_ctrl[i])*0.1) for i in 1:length(dedt)]
        p_ctrl + ∫edt + dedt + kv*v_p̂ 
    end
    #print("ref pt is $p, optimal is $p̂")
    # convert `v_p` back into the control `CarU`
    d = K.ref_pos
    θ = s.θ
    v, v_y = rotation2D(-θ) * v_p
    ω = v_y / d
    (v < 0) && (ω *= -1)
    u = u_from_v_ω(v, ω, K.dy)
    # bound steering based on state
    ψ_min, ψ_max = s.ψ-10°, s.ψ+10°
    ψ = clamp(u.ψ̂, ψ_min, ψ_max)
    u = CarU(u.v̂, ψ)
    @info "ideal ref point is $p̂, ref point is $p"
    @info "[ctrl] lin vel is $v, ang vel is $ω action is $u"
    u
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
        #return CarU(0.0, 0.0)
        (id == 1) &&return zs[1].c
        
        #xs[id].θ = restrict(xs[id].θ)
        s = form[id] # initial state
        (p, v_p) = formpoint_to_refpoint(@SVector[s.x, s.y]) # 
        ξi = submap(ξ, Symbol(id))
        v_o = avoid_collision(id)
        #println("s:$s p:$p vp:$v_p ξi:$ξi, v_o:$v_o, K $(ctrl.K)")
        #u = track_refpoint(ctrl.K, ξi, (p, v_p + v_o), xs[id], t)
        u = track_refpoint(ctrl.K, ξi, (p, v_p + v_o), xs[id], t)
        @debug "[$t] states are $xs\n refvel is $v_p obss are:$zs actions are:$u"

        #println("t$t id$id u: $u s:$s xs:$(xs) p:$p vp:$v_p ")
        return u
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





function formation_controller(ctrl::FormationControl{WallObsControl}, ξ, xs, zs, t)
    formpoint_to_refpoint = to_formation_frame(ctrl, xs[1])
    form = ctrl.formation(xs, zs, t)
    
    N = length(xs)
    function avoid_collision(i)
        ca = ctrl.avoidance
        pos = get_pos(xs[i]) # ref_point(ctrl.K, xs[i])
        sum(repel_force(ca, get_pos(xs[j]), pos) for j in 1:N if j != i)
    end

    function oneDTOC(dist_left::Float64, speed::Float64)
        if(isinf(dist_left))
            return ctrl.K.target_v
        end
        # one D Time Optimal Control
        # TODO: add latency compensation
        min_a, max_a = -ctrl.K.max_a, ctrl.K.max_a
        min_v, max_v = -ctrl.K.dy.max_v, ctrl.K.target_v
        dt = ctrl.K.dy.delta_t
        s = speed
        if (square(speed + max_a*dt)/(2*max_a) + 1.0/2.0(speed*2+max_a*dt)*dt < dist_left)
            s = speed + max_a*dt
        elseif (square(speed)/(2*max_a) + dt*speed < dist_left)
            s = speed
        else
            s = speed - max_a*dt
        end
        @info "dist left is $dist_left s is $s, speed is $speed"
        if abs(speed) <  0.01 && (sign(s)==1 && s > speed || sign(s)==-1 && s < speed)
            s = 0.2*sign(s)
        end
        clamp(s, min_v, max_v)
    end

    function action(id)::CarU
        #return CarU(0.0, 0.0)
        #xs[id].θ = restrict(xs[id].θ)
        @info "in action, obs are $zs"
        s = form[id] # initial state
        (p, v_p) = formpoint_to_refpoint(@SVector[s.x, s.y]) # 
        ξi = submap(ξ, Symbol(id))
        fleet_size = length(xs)
        v_o = if fleet_size > 1 avoid_collision(id) else @SVector[0.0, 0.0] end
        wall_pos = if zs[id].c[1] == Inf || zs[id].c[2] == Inf
            @SVector[Inf, Inf]
        else
            @SVector[zs[id].c[1], zs[id].c[2]]
        end
        # control ref point - project ref point to line
        # velocity - mag = 1dTOC, dir=line dir

        if (id == 1)
            state = xs[id]
            # compute for distance left
            # change to static vector
            dist_left = sqrt((wall_pos[1] - state.x)^2 + (wall_pos[2] - state.y)^2)
            dist_left -= ctrl.K.stop_distance
            dist_left = clamp(dist_left, 0.0, ctrl.K.sensor_range)
            # compute target point
            cur_pos = @SVector[state.x, state.y]
            res = intersect_line_circle(ctrl.K.p1, ctrl.K.p2, cur_pos, ctrl.K.r)
            θ = rem2pi(state.θ, RoundNearest)
            i = 0
            while isempty(res)
                res = intersect_line_circle(ctrl.K.p1, ctrl.K.p2, cur_pos, ctrl.K.r+i)
                i += 1
            end
            
            r = res[argmin([abs(rem2pi(atan(r[2] - cur_pos[2], r[1] - cur_pos[1]) - θ, RoundNearest)) for r in res])]
            θ = atan(r[2] - cur_pos[2], r[1] - cur_pos[1])

            #θ = clamp(θ, rem2pi(state.θ, RoundNearest) -15°, rem2pi(state.θ, RoundNearest) + 15°)
            p = rotation2D(θ)*@SVector[clamp(dist_left, 0.0, 0.1), 0] + ref_point(ctrl.K.ref_pos, state)#rotation2D(state.θ)*@SVector[clamp(dist_left, 0.0, 1.0), 0] + @SVector[state.x, state.y]
            v_p = rotation2D(θ)*@SVector[oneDTOC(dist_left, state.v), 0.0] 
            v_o = @SVector[0.0, 0.0]
            @info "theta is $θ, dist_left is $dist_left, vel is $(oneDTOC(dist_left, state.v)) intersection is $res"
        end
        #println("s:$s p:$p vp:$v_p ξi:$ξi, v_o:$v_o, K $(ctrl.K)")
        #u = track_refpoint(ctrl.K, ξi, (p, v_p + v_o), xs[id], t)
        u = if id ==1 
            track_refpoint(ctrl.K.kp_leader, ctrl.K.ki_leader, ctrl.K.kd_leader, ctrl.K.kv_leader, ctrl.K, ξi, (p, v_p + v_o), xs[id], t)
        else
            track_refpoint(ctrl.K, ξi, (p, v_p + v_o), xs[id], t)
        end
        @info "[$t] states are $xs\n refpt is $p refvel is $v_p \n wall pos is $(wall_pos) obss are:$zs \n actions are:$u"
        #println("t$t id$id u: $u s:$s xs:$(xs) p:$p vp:$v_p ")
        return u
    end

    action
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
