function plot_formation(
    data::TrajectoryData, freq::ℝ, ctrl::FormationControl, formation_at_t
)
    scene, layout = layoutscene(resolution = (1600, 1600))

    ax_traj = layout[1,1] = LAxis(
        scene, title = "Trajectories", aspect = DataAspect(), 
        backgroundcolor = RGBf0(1,1,1))

    times = data.times
    time_ls = labelslider!(
        scene, "time", times; 
        format = x -> "$(round((x - 1) / freq, digits = 2))s")
    layout[2,1] = time_ls.layout
    t = time_ls.slider.value

    N = size(data.values, 2)
    colors = [get(ColorSchemes.Spectral_10, i/N) for i in 1:N]

    function plot_snapshot(t)
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
            @_ (formation_at_t($t) 
                |> map(ref_point(ctrl.K, _),__) 
                |> map(to_formation_frame(ctrl, leader), __))
        end
        points = @lift (x -> Point2f0(x[1])).($refpoints)
        directions = @lift (x -> Point2f0(x[2])).($refpoints)
        arrows!(ax_traj, points, directions; 
            linecolor=:red, arrowcolor=:red, lengthscale=0.15, arrowsize=0.10, linewidth=1)
        scatter!(ax_traj, points; color=:red)
    end
    
    # draw trajectories
    for id in 1:N
        xs, ys, θs = map(l -> data[l][:,id], ("x", "y", "θ"))
        lines!(ax_traj, xs, ys; color=colors[id], linewidth=4)
    end
    # plot current snapshot
    plot_snapshot(t)
    # plot additional snapshots (used for paper figures)
    interesting_times = [] # [386, 833, 963, 1254, 1610]
    foreach(plot_snapshot, Observable.(interesting_times))

    scene
end

struct FormationObsDynamics{Fu,Ff} <: ObsDynamics 
    "`external_u(x, t) -> u`"
    external_u::Fu
    "`external_formation(x, t) -> formation_idx`"
    external_formation::Ff
end

function OneVision.obs_forward(dy::FormationObsDynamics, x, z, t::𝕋)
    HVec(dy.external_u(x, t), dy.external_formation(x, t))
end

"""
Compute the time-averaged distance between each follower and their expected 
formation position using a quadratic norm. Mathematically, this metric is defined as 

     √(1/T ∫ ∑_i |x_i - x⋆_i|^2 dt) .
"""
function avg_formation_deviation(
    data::TrajectoryData, ctrl::FormationControl, formation_at_t
)::ℝ
    function deviation_at(t)
        x, y, θ, ψ, v = map(l -> data[l][t, 1], ("x", "y", "θ", "ψ", "v"))
        leader = CarX(;x, y, θ, ψ, v)
        N = size(data.values, 2)
        form = formation_at_t(t)[2:end]
        @asserteq length(form) (N-1)
        ref_pos = 
            @_ (form
            |> map(ref_point(ctrl.K, _),__) 
            |> map(to_formation_frame(ctrl, leader), __)
            |> map(_.pos, __))
        x_data = data["x"][t, :]
        y_data = data["y"][t, :]
        total = 0.0
        for i in 2:N
            pos = @SVector[x_data[i], y_data[i]]
            norm(pos - ref_pos[i-1])
        end
        actual_pos = [@SVector[x_data[i], y_data[i]] for i in 2:N]
        mean(norm.(actual_pos .- ref_pos))
    end

    mean(deviation_at.(eachindex(data.times)))
end

function formation_example(;
        setting::ExampleSetting,
        CF::CFName = onevision_cf,
        switch_formation = true,
        seed = 1234,
        leader_noise = false,
        track_config = false,
        plot_result = true,
    )
    @unpack time_end, freq, noise, sensor_noise, delays, H, model_error = setting
    X, U = CarX{ℝ}, CarU{ℝ}
    Z = HVec{U, ℕ}
    t_end = 𝕋(ceil(time_end * freq))
    delta_t = 1 / freq

    # Car dynamics parameters
    max_ψ = 45°
    dy_model = CarDynamics(l = 0.1 * (1+model_error) ;delta_t)
    rng = MersenneTwister(seed)
    function add_noise(x::X, t)::X where X
        x + CarX(x=0.0, y=0.0, θ=0.0, 
                v = randn(rng, ℝ), ψ = randn(rng, ℝ)) * noise
    end
    dy_actual = CarDynamics(; delta_t, max_ψ, add_noise)
    dy_actual_leader = leader_noise ? dy_actual : CarDynamics(; delta_t, max_ψ)
    xs_observer = 
        if leader_noise
            (xs, t) -> @_ (_ + randn(rng, X) * sensor_noise).(xs)
        else
            (xs, t) -> 
                [i == 1 ? x : x + randn(rng, X) * sensor_noise 
                for (i, x) in enumerate(xs)]
        end

    function external_control(_, t)
        (if t ≤ 3 * freq
            U(v̂ = 1.0, ψ̂ = 0.0)
        elseif t ≤ 5 * freq
            U(v̂ = 1.0, ψ̂ = 4°)
        elseif t ≤ 9 * freq
            U(v̂ = 1.0, ψ̂ = 0°)
        elseif t ≤ 15 * freq
            U(v̂ = 2.0, ψ̂ = 2°)
        else
            U(v̂ = 2(1 - (t/freq-15)/5), ψ̂ = 2°)
        end) + U(v̂ = randn(rng, ℝ), ψ̂ = randn(rng, ℝ)) * noise/4
    end

    function external_formation(_, t)
        if t ≥ 8 * freq && switch_formation
            2
        else
            1
        end
    end

    leader_z_dy = FormationObsDynamics(external_control, external_formation)

    # running at 20Hz
    delays_model  = delays
    delays_actual = delays_model
    ΔT = delays_model.ΔT

    N = 4
    triangle_formation = let
        l = 0.8
        Δϕ = 360° / (N-1) 
        circle = [X(x = l * cos(Δϕ * i), y = l * sin(Δϕ * i), θ = 0) for i in 1:N-1]
        [[zero(X)]; circle]
    end
    vertical_formation = let
        l = 0.6
        leader_idx = round_ceil(N/2)
        line = [X(x = l * (i - leader_idx), y = 0, θ = 0) for i in 1:N]
        [line[mod1(leader_idx + j - 1, N)] for j in 1:N]
    end

    formations = [triangle_formation, vertical_formation]
    form_from_id(i) = formations[i]

    RefK = if track_config
        ConfigTrackControl(dy_model, 1.0, 1.0, 1.0)
    else
        RefPointTrackControl(;
            dy = dy_model, ref_pos = dy_model.l, ctrl_interval = delta_t * ΔT, 
            kp = 1.0, ki = 0.0, kd = 0.0)
    end
    avoidance = CollisionAvoidance(scale=1.0, min_r=dy_model.l, max_r=3*dy_model.l)
    central = FormationControl(
        (_, zs, _) -> form_from_id(zs[1].d),
        RefK, dy_model, avoidance)
    
    init = let 
        formation = rotate_formation(form_from_id(1), 90°)
        map(1:N) do i 
            x = formation[i]
            x, HVec(zero(U), 1), zero(U)
        end
    end

    world_model = WorldDynamics(fill((dy_model, StaticObsDynamics()), N))
    loss_model = let 
        x_weights = SVector{N}(fill(X(x = 100, y = 100, θ = 1), N))
        u_weights = SVector{N}(fill(U(v̂ = 1, ψ̂ = 10), N))
        RegretLossModel(central, world_model, x_weights, u_weights)
    end
    framework = mk_cf(CF, world_model, central, delays, loss_model; X, Z, H)

    comps = ["x", "y", "θ", "ψ", "v"]
    function record_f(xs, zs, us)
        comps = [:x, :y, :θ, :ψ, :v]
        hcat(((p -> getfield(p,c)).(xs) for c in comps)...)
    end

    formation_at_t(t) = form_from_id(external_formation(missing, t))

    world = WorldDynamics([
        (dy_actual_leader, leader_z_dy) 
        fill((dy_actual, StaticObsDynamics()), N-1)])
    result, (logs, loss) = simulate(
        world, delays_actual, framework, init, (comps, record_f), 1:t_end
        ; xs_observer, loss_model)
    if plot_result
        visualize(result; delta_t = 1 / freq, loss) |> display
        plot_formation(result, freq, central, formation_at_t) |> display
    end
    metrics = Dict(
        "avg deviation" => avg_formation_deviation(result, central, formation_at_t))
    loss, metrics
end