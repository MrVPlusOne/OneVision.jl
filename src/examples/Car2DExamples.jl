module Car2DExamples

using OneVision
using OneVision: ℝ, 𝕋, ℕ, @kwdef
using Random
using StaticArrays
# using Plots
using AbstractPlotting
using Makie
using AbstractPlotting.MakieLayout

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

ψ_from_v_ω(v, ω, l) = atan(v, ω * l)
ω_from_v_ψ(v, ψ, l) = tan(ψ) * v / l

struct CarZ{R} <: FieldVector{0,R} end

# TODO: fine-tune these parameters
@kwdef struct CarDynamics <: SysDynamics
    "control time interval in seconds"
    delta_t::ℝ
    "numerical integration number of samples"
    int_samples::ℕ = 10
    "maximal linear speed"
    max_v::ℝ = 5.0
    "maximal steering angle"
    max_ψ::ℝ = 1pi / 3
    "wheelbase between the front and rear wheels"
    l::ℝ = 0.1
    "rate of convergence for v to converge to v̂"
    k_v::ℝ = 5.0
    "rate of convergence for ψ to converge to ψ̂"
    k_ψ::ℝ = 5.0
end

function limit_control(dy::CarDynamics, u::CarU)
    u1 = CarU(
        v̂ = clamp(u.v̂, -dy.max_v, dy.max_v),
        ψ̂ = clamp(u.ψ̂, -dy.max_ψ, dy.max_ψ),
    )
    u1 == u ? u : u1
end

@inline function sys_derivates(dy::CarDynamics, x::CarX, u::CarU)
    CarX(
        x = cos(x.θ) * x.v,
        y = sin(x.θ) * x.v,
        θ = ω_from_v_ψ(x.v, x.ψ, dy.l),
        v = dy.k_v * (u.v̂ - x.v),
        ψ = dy.k_ψ * (u.ψ̂ - x.ψ),
    )
end

struct CarObsDynamics <: ObsDynamics end

function OneVision.sys_forward(dy::CarDynamics, x::CarX, u::CarU, t::𝕋)
    u = limit_control(dy, u)

    N = dy.int_samples 
    dt = dy.delta_t / N
    f(x) = sys_derivates(dy, x, u)
    integrate_Euler(f, x, dt, N)
end

function OneVision.obs_forward(
    dy::CarObsDynamics, x::CarX, z::CarZ, t::𝕋
) z end


@kwdef struct RefPointTrackControl
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

function ref_point_track_control(
    K::RefPointTrackControl, ŝ::CarX{R}, s::CarX{R}
)::CarU{R} where R
    # compute the difference of the reference points, then apply propotional control
    p, p̂ = ref_point.(Ref(K), (s, ŝ))
    ṗ = K.k * (p̂ - p)
    # convert `ṗ` back into the control `CarU`
    d = K.ref_pos
    θ = s.θ
    v = cos(θ) * ṗ[1] + sin(θ) * ṗ[2]
    ω = -sin(θ) / d * ṗ[1] + cos(θ) / d * ṗ[2]
    ψ = ψ_from_v_ω(v, ω, K.dy.l)
    limit_control(K.dy, CarU(v, ψ))
end


@kwdef struct TrajectoryTrackControl
    dy::CarDynamics
    k1::ℝ
    k2::ℝ
    k3::ℝ
end


"""
Full configuration tracking control for a 2D Car.
"""
function config_track_control(
    K::TrajectoryTrackControl, x̂::CarX{R}, x::CarX{R}
)::CarU{R} where R 
    θ_d, x_d, y_d, v_d = x̂.θ, x̂.x, x̂.y, x̂.v
    w_d = ω_from_v_ψ(x̂.v, x̂.ψ, K.dy.l)
    θ_e = x.θ - θ_d
    Δx = x.x - x_d
    Δy = x.y - y_d
    x_e =  cos(θ_d) * Δx + sin(θ_d) * Δy
    y_e = -sin(θ_d) * Δx + cos(θ_d) * Δy
    
    co = cos(θ_e), ta = tan(θ_e)
    ŵ = w_d - (K.k2 * v_d * y_e + K.k3 * abs(v_d) * ta) * co * co
    ψ̂ = ψ_from_v_ω(x.v, ŵ, K.dy.l)
    v̂ = (v_d - K.k1 * abs(v_d) * (x_e + y_e * ta)) / co
    limit_control(K.dy, CarU(v̂, ψ̂))
end

struct CentralRefPointTrackControl <: CentralControl{CarU{ℝ}}
    K::RefPointTrackControl
    trajectories::Matrix{CarX{ℝ}}  # index by (agent, time)
end

function OneVision.control_one(
    ctrl::CentralRefPointTrackControl, xs,zs, t::𝕋, id::ℕ
)
    x = xs[id]
    x̂ = ctrl.trajectories[id,t]
    ref_point_track_control(ctrl.K, x̂, x)
end

function circular_traj(dy::CarDynamics, x0, u0, t_end::𝕋)::Vector{CarX{ℝ}}
    x::CarX{ℝ} = x0
    out = Vector{CarX{ℝ}}(undef, t_end)
    for t in 0:t_end - 1
        x = sys_forward(dy, x, u0, t)
        out[t + 1] = x
    end
    out
end

function plot_cars(data::TrajectoryData, freq::ℝ, ref_traj::Vector{CarX{ℝ}})
    function car_triangle(x,y,θ; len = 0.1, width = 0.02)
        base = Point2f0(x,y)
        dir = Point2f0(cos(θ), sin(θ))
        left = Point2f0(sin(θ), -cos(θ))
        p1 = base + dir * len
        p2 = base + left * width
        p3 = base - left * width
        [p1,p2,p3]
    end

    AbstractPlotting.inline!(false)
    scene, layout = layoutscene(resolution = (1600, 1600))
    scene

    xs, ys = data["x"][:,1], data["y"][:,1]
    θs = data["θ"][:,1]

    ax_traj = layout[1,1] = LAxis(scene, title = "Trajectories", aspect = DataAspect())
    ref_plot = let 
        local xs = (p -> p.x).(ref_traj)
        local ys = (p -> p.y).(ref_traj)
        lines!(ax_traj, xs, ys; color = :green, linewidth = 4)
    end 
    traj_plot = lines!(ax_traj, xs, ys; linewidth = 4)

    times = data.times
    time_ls = labelslider!(
    scene, "time", times; 
    format = x -> "$(round((x - 1) / freq, digits = 2))s")
    layout[2,1] = time_ls.layout

    t = time_ls.slider.value
    car = @lift car_triangle(xs[$t], ys[$t], θs[$t])
    ref_car = @lift let c = ref_traj[$t] 
        car_triangle(c.x, c.y, c.θ)
    end
    poly!(ax_traj, car; color=:red)
    poly!(ax_traj, ref_car; color=:green)

    scene
end

function run_example(;freq = 20.0, time_end = 20.0)
    X, Z, U = CarX{ℝ}, CarZ{ℝ}, CarU{ℝ}

    dy = CarDynamics(delta_t = 1 / freq)
    z_dy = CarObsDynamics()
    t_end = 𝕋(ceil(time_end * freq))
    x_ref0 = X(x = 0, y = -0.5, θ = 0.0)
    u_ref0 = U(v̂ = 1, ψ̂ = 0.1pi)
    circ_traj = circular_traj(dy, x_ref0, u_ref0, t_end + 1)

    RefK = RefPointTrackControl(;dy, ref_pos = dy.l, k = 1.0)
    central = CentralRefPointTrackControl(RefK, reshape(circ_traj, 1, :))

    N = 1
    world = WorldDynamics([(dy, z_dy)])
    delay_model = DelayModel(obs = 0, act = 0, com = 1)
    framework = NaiveCF{X,Z,U}(N, central, delay_model.com)
    init = let x0 = X(x = 0, y = 0, θ = 0), z0 = Z(), u0 = U()
        [(x0, z0, u0)]
    end 
    comps = ["x", "y", "θ", "ψ"]
    function record_f(xs, zs, us)
        [(x -> x.x).(xs) (x -> x.y).(xs) (x -> x.θ).(xs) (x -> x.ψ).(xs)]
    end
    times = 1:t_end

    result, logs = simulate(world, delay_model, framework, init, (comps, record_f), times)
    # visualize(result; delta_t = 1 / freq)
    # result
    plot_cars(result, freq, circ_traj)
end


end  # module Car2DExamples