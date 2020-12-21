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
        @_ (ctrl.formation(nothing, nothing, $t) 
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

struct FormationObsDynamics{Fu,Ff} <: ObsDynamics 
    "`external_u(x, t) -> u`"
    external_u::Fu
    "`external_formation(x, t) -> formation_idx`"
    external_formation::Ff
end

function OneVision.obs_forward(dy::FormationObsDynamics, x, z, t::𝕋)
    HVec(dy.external_u(x, t), dy.external_formation(x, t))
end


function formation_example(;time_end = 20.0, freq = 100.0, 
        noise = 0.005, sensor_noise = noise, 
        delays = default_delays,
        CF = OvCF,
        plot_result = true,
    )
    X, U = CarX{ℝ}, CarU{ℝ}
    Z = HVec{U, ℕ}
    t_end = 𝕋(ceil(time_end * freq))
    delta_t = 1 / freq

    # Car dynamics parameters
    dy_model = CarDynamics(;delta_t, max_ψ = 45°)
    rng = MersenneTwister(1234)
    function add_noise(x::X, t)::X where X
        x + CarX(x=0.0, y=0.0, θ=0.0, 
                v = randn(rng, ℝ), ψ = randn(rng, ℝ)) * noise
    end
    dy_actual = @set dy_model.add_noise = add_noise
    function xs_observer(xs, t)
        (x -> x + randn(rng, X) * sensor_noise).(xs)
    end

    function external_control(_, t)
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

    function external_formation(_, t)
        1
    end

    leader_z_dy = FormationObsDynamics(external_control, external_formation)

    # running at 20Hz
    delays_model  = delays
    delays_actual = delays_model
    H = 20
    ΔT = delays_model.ΔT

    N = 4
    formation = begin
        l = 0.8
        Δϕ = 360° / (N-1) 
        circle = [X(x = l * cos(Δϕ * i), y = l * sin(Δϕ * i), θ = 0) for i in 1:N-1]
        [[zero(X)]; circle]
    end

    RefK = RefPointTrackControl(;
        dy = dy_model, ref_pos = dy_model.l, ctrl_interval = delta_t * ΔT, kp = 1.0, ki = 0.0, kd = 0.0)
    avoidance = CollisionAvoidance(scale=1.0, min_r=dy_model.l, max_r=3*dy_model.l)
    central = FormationControl((xs, zs, t) -> formation, RefK, dy_model, avoidance)

    formation = rotate_formation(formation, 0°)
    init = map(1:N) do i 
        x = formation[i]
        x, zero(Z), zero(U)
    end

    world_model = WorldDynamics(fill((dy_model, StaticObsDynamics()), N))
    loss_model = let 
        x_weights = SVector{N}(fill(X(x = 1, y = 1, θ = 1), N))
        u_weights = SVector{N}(fill(U(v̂ = 1, ψ̂ = 1), N))
        RegretLossModel(central, world_model, x_weights, u_weights)
    end
    framework = 
        if CF == NaiveCF
            NaiveCF(X, Z, N, central, msg_queue_length(delays_model), ΔT)
        elseif CF == LocalCF
            LocalCF(central, world_model, delays; X, Z)
        elseif CF == OvCF
            OvCF(loss_model, delays_model; Z, H)
        else
            error("Unexpected CF: $CF")
        end

    comps = ["x", "y", "θ", "ψ", "v"]
    function record_f(xs, zs, us)
        comps = [:x, :y, :θ, :ψ, :v]
        hcat(((p -> getfield(p,c)).(xs) for c in comps)...)
    end

    world = ([(dy_actual, leader_z_dy); fill((dy_actual, StaticObsDynamics()), N-1)] 
            |> WorldDynamics)
    result, (logs, loss) = simulate(
        world, delays_actual, framework, init, (comps, record_f), 1:t_end
        ; xs_observer, loss_model)
    # visualize(result; delta_t = 1 / freq) |> display
    if plot_result
        plot_formation(result, freq, central) |> display
    end
    loss
end