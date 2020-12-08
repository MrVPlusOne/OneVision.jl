function circular_traj(dy::CarDynamics, x0, u0, t_end::𝕋)::Vector{CarX{ℝ}}
    x::CarX{ℝ} = x0
    out = Vector{CarX{ℝ}}(undef, t_end)
    for t in 1:t_end
        out[t] = x
        x = sys_forward(dy, x, u0, t)
    end
    out
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

function tracking_example(;freq = 100.0, time_end = 20.0, noise_level=0.0, plot_result = true)
    X, Z, U = CarX{ℝ}, SVector{0, ℝ}, CarU{ℝ}
    t_end = 𝕋(ceil(time_end * freq))
    delta_t = 1 / freq

    # Car dynamics parameters
    dy = CarDynamics(;delta_t, max_ψ = 60°)
    rng = MersenneTwister(1234)
    function add_noise(x::X, t)::X where X
        x + randn(rng, X) * noise_level
    end
    dy_actual = @set dy.add_noise = add_noise
    z_dy = StaticObsDynamics()

    delay_model = DelayModel(obs = 3, act = 3, com = 1, ΔT = 5)
    ΔT = delay_model.ΔT
    H = 20

    # reference trajectory to track
    x_ref0 = X(x = 0, y = -0.5, θ = 0.0)
    u_ref0 = U(v̂ = 0.5, ψ̂ = 0.1pi)
    circ_traj = circular_traj(dy, x_ref0, u_ref0, t_end + 1 + H * ΔT + delay_model.total)
    traj_f = FuncT(Tuple{ℕ,𝕋}, CarX{ℝ}) do (id, t)
        t = max(1, t) 
        circ_traj[t]
    end

    RefK = RefPointTrackControl(;dy, ref_pos = dy.l, ctrl_interval = ΔT * delta_t, kp = 1.0, ki = 1.0)
    # RefK = ConfigTrackControl(;dy, k1 = 2, k2 = 1, k3 = 1)
    central = RefTrackCentralControl(RefK, traj_f)

    init = let x0 = X(x = 0, y = 0, θ = 180°), z0 = Z(), u0 = U()
        [(x0, z0, u0)]
    end 

    N = 1
    world_model = WorldDynamics([(dy, z_dy)])
    world_actual = WorldDynamics([(dy_actual, z_dy)])
    # framework = NaiveCF{X,Z,U}(N, central, delay_model.com)
    framework = let 
        world_model = world_model
        x_weights = fill(X(x = 1, y = 1, θ = 1), N)
        u_weights = fill(U(v̂ = 1, ψ̂ = 1), N)
        OvCF(central, world_model, delay_model, x_weights, u_weights; X, Z, N, H)
    end

    comps = ["x", "y", "θ", "ψ"]
    function record_f(xs, zs, us)
        [(x -> x.x).(xs) (x -> x.y).(xs) (x -> x.θ).(xs) (x -> x.ψ).(xs)]
    end

    result, logs = simulate(
        world_actual, delay_model, framework, init, (comps, record_f), 1:t_end)
    # visualize(result; delta_t = 1 / freq) |> display
    if plot_result
        plot_tracking(result, freq, circ_traj) |> display
    end
    result, logs
end
