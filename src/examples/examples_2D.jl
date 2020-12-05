using AbstractPlotting
using Makie
using AbstractPlotting.MakieLayout
import ColorSchemes

function circular_traj(dy::CarDynamics, x0, u0, t_end::𝕋)::Vector{CarX{ℝ}}
    x::CarX{ℝ} = x0
    out = Vector{CarX{ℝ}}(undef, t_end)
    for t in 1:t_end
        out[t] = x
        x = sys_forward(dy, x, u0, t)
    end
    out
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

function plot_tracking(data::TrajectoryData, freq::ℝ, ref_traj::Vector{CarX{ℝ}})
    scene, layout = layoutscene(resolution = (1600, 1600))

    xs, ys = data["x"][:,1], data["y"][:,1]
    θs = data["θ"][:,1]

    ax_traj = layout[1,1] = LAxis(
        scene, title = "Trajectories", aspect = DataAspect(), 
        backgroundcolor = RGBf0(0.98, 0.98, 0.98))
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
    poly!(ax_traj, car; color = :red)
    poly!(ax_traj, ref_car; color = :green)

    scene
end

function plot_formation(data::TrajectoryData, freq::ℝ, ctrl::FormationControl)
    scene, layout = layoutscene(resolution = (1600, 1600))

    ax_traj = layout[1,1] = LAxis(
        scene, title = "Trajectories", aspect = DataAspect(), 
        backgroundcolor = RGBf0(0.98, 0.98, 0.98))

    times = data.times
    time_ls = labelslider!(
        scene, "time", times; 
        format = x -> "$(round((x - 1) / freq, digits = 2))s")
    layout[2,1] = time_ls.layout
    t = time_ls.slider.value

    N = size(data.values, 2)
    colors = [get(ColorSchemes.Spectral_10, i/N) for i in 1:N]
    # draw trajectories
    for id in 1:N
        xs, ys, θs = map(l -> data[l][:,id], ("x", "y", "θ"))
        lines!(ax_traj, xs, ys; color=colors[id], linewidth=3)
    end
    # draw cars
    for id in 1:N
        car = @lift let 
            x, y, θ = map(l -> data[l][$t,id], ("x", "y", "θ"))
            car_triangle(x, y, θ)
        end
        poly!(ax_traj, car; color = colors[id], strokecolor=:black, strokewidth=2)
    end
    # draw ref points
    refpoints = @lift let
        x, y, θ, ψ, v = map(l -> data[l][$t,1], ("x", "y", "θ", "ψ", "v"))
        leader = CarX(;x, y, θ, ψ, v)
        @_ (ctrl.formation 
            |> map(ref_point(ctrl.K, _),__) 
            |> map(to_formation_frame(ctrl, leader), __))
    end
    points = @lift (x -> Point2f0(x[1])).($refpoints)
    directions = @lift (x -> Point2f0(x[2])).($refpoints)
    arrows!(ax_traj, points, directions; 
        linecolor=:red, arrowcolor=:red, lengthscale=0.2, arrowsize=0.1)
    scatter!(ax_traj, points; color=:red)

    scene
end


function tracking_example(;freq = 20.0, time_end = 20.0, plot_result = true)
    X, Z, U = CarX{ℝ}, Nothing, CarU{ℝ}
    t_end = 𝕋(ceil(time_end * freq))
    delta_t = 1 / freq

    # Car dynamics parameters
    dy = CarDynamics(;delta_t, max_ψ = 60°)
    z_dy = StaticObsDynamics()

    delay_model = DelayModel(obs = 3, act = 3, com = 1)
    H = 20

    # reference trajectory to track
    x_ref0 = X(x = 0, y = -0.5, θ = 0.0)
    u_ref0 = U(v̂ = 0.5, ψ̂ = 0.1pi)
    circ_traj = circular_traj(dy, x_ref0, u_ref0, t_end + 1 + H + delay_model.total)
    traj_f = FuncT(Tuple{ℕ,𝕋}, CarX{ℝ}) do (id, t)
        t = max(1, t) 
        circ_traj[t]
    end

    RefK = RefPointTrackControl(;dy, ref_pos = dy.l, delta_t, kp = 1.0, ki = 1.0)
    # RefK = ConfigTrackControl(;dy, k1 = 2, k2 = 1, k3 = 1)
    central = RefTrackCentralControl(RefK, traj_f)

    init = let x0 = X(x = 0, y = 0, θ = 180°), z0 = Z(), u0 = U()
        [(x0, z0, u0)]
    end 

    N = 1
    world = WorldDynamics([(dy, z_dy)])
    # framework = NaiveCF{X,Z,U}(N, central, delay_model.com)
    framework = let 
        world_model = world
        x_weights = fill(X(x = 1, y = 1, θ = 1), N)
        u_weights = fill(U(v̂ = 1, ψ̂ = 1), N)
        OvCF(central, world_model, delay_model, x_weights, u_weights; X, Z, N, H)
    end

    comps = ["x", "y", "θ", "ψ"]
    function record_f(xs, zs, us)
        [(x -> x.x).(xs) (x -> x.y).(xs) (x -> x.θ).(xs) (x -> x.ψ).(xs)]
    end

    result, logs = simulate(
        world, delay_model, framework, init, (comps, record_f), 1:t_end)
    # visualize(result; delta_t = 1 / freq) |> display
    if plot_result
        plot_tracking(result, freq, circ_traj) |> display
    end
    result, logs
end

struct FormationObsDynamics{F} <: ObsDynamics 
    external_u::FuncT{𝕋, CarU{ℝ}, F}
end

function OneVision.obs_forward(dy::FormationObsDynamics, x, z, t::𝕋)
    dy.external_u(t)
end

function formation_example(;freq = 100.0, time_end = 20.0, noise_level = 0.005, plot_result = true)
    X, U = CarX{ℝ}, CarU{ℝ}
    Z = U
    t_end = 𝕋(ceil(time_end * freq))
    delta_t = 1 / freq

    # Car dynamics parameters
    dy_model  = CarDynamics(;delta_t, max_ψ = 45°)
    rng = MersenneTwister(1234)
    function add_noise(x::X, t)::X where X
        x + randn(rng, X) * noise_level
    end
    dy_actual = @set dy_model.add_noise = add_noise

    external_control(t) = begin
        if t ≤ 3 * freq
            U(v̂ = 1.0, ψ̂ = 0.0)
        elseif t ≤ 5 * freq
            U(v̂ = 1.0, ψ̂ = 10°)
        elseif t ≤ 8 * freq
            U(v̂ = 1.0, ψ̂ = 0°)
        elseif t ≤ 15 * freq
            U(v̂ = 2.0, ψ̂ = 2°)
        else
            U(v̂ = 2(1 - (t/freq-15)/5), ψ̂ = 2°)
        end
    end
    leader_z_dy = FormationObsDynamics(FuncT(external_control, 𝕋, U))

    # running at 20Hz
    delays_model  = DelayModel(obs = 1, act = 1, com = 5, ctrl_interval = 5)
    delays_actual = DelayModel(obs = 1, act = 1, com = 5, ctrl_interval = 5)
    H = 20

    N = 4
    formation = begin
        l = 0.5
        Δϕ = 360° / (N-1) 
        circle = [X(x = l * cos(Δϕ * i), y = l * sin(Δϕ * i), θ = 0) for i in 1:N-1]
        [[zero(X)]; circle]
    end

    RefK = RefPointTrackControl(;
        dy = dy_model, ref_pos = dy_model.l, delta_t, kp = 1.0, ki = 0.5, kd = 0.5)
    central = FormationControl(formation, RefK, dy_model)

    formation = rotate_formation(formation, 0°)
    init = map(1:N) do i 
        x = formation[i]
        x, zero(Z), zero(U)
    end

    world_model = WorldDynamics(fill((dy_model, StaticObsDynamics()), N))
    framework = NaiveCF(X, Z, N, central, msg_queue_length(delays_model))
    # framework = let 
    #     x_weights = fill(X(x = 1, y = 1, θ = 1), N)
    #     u_weights = fill(U(v̂ = 1, ψ̂ = 1), N)
    #     OvCF(central, world_model, delays_model, x_weights, u_weights; 
    #         X, Z, N, H)
    # end

    comps = ["x", "y", "θ", "ψ", "v"]
    function record_f(xs, zs, us)
        comps = [:x, :y, :θ, :ψ, :v]
        hcat(((p -> getfield(p,c)).(xs) for c in comps)...)
    end

    world = ([(dy_actual, leader_z_dy); fill((dy_actual, StaticObsDynamics()), N-1)] 
            |> WorldDynamics)
    result, logs = simulate(
        world, delays_actual, framework, init, (comps, record_f), 1:t_end)
    # visualize(result; delta_t = 1 / freq) |> display
    if plot_result
        plot_formation(result, freq, central) |> display
    end
    result, logs
end

run_example = formation_example