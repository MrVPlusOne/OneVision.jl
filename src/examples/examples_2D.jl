using AbstractPlotting
using Makie
using AbstractPlotting.MakieLayout
using Printf: @sprintf
import ColorSchemes
import Dates

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

struct FormationObsDynamics{F} <: ObsDynamics 
    "external_u(x, t) -> u"
    external_u::F
end

function OneVision.obs_forward(dy::FormationObsDynamics, x, z, t::𝕋)
    dy.external_u(x, t)
end

function formation_example(;freq = 100.0, time_end = 20.0, plot_result = true,
        dynamics_noise = 0.005, sensor_noise = dynamics_noise)
    X, U = CarX{ℝ}, CarU{ℝ}
    Z = U
    t_end = 𝕋(ceil(time_end * freq))
    delta_t = 1 / freq

    # Car dynamics parameters
    dy_model  = CarDynamics(;delta_t, max_ψ = 45°)
    rng = MersenneTwister(1234)
    function add_noise(x::X, t)::X where X
        x + CarX(x=0.0, y=0.0, θ=0.0, 
                v = randn(rng, ℝ), ψ = randn(rng, ℝ)) * dynamics_noise
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
    leader_z_dy = FormationObsDynamics(FuncT(external_control, 𝕋, U))

    # running at 20Hz
    delays_model  = DelayModel(obs = 3, act = 6, com = 13, ΔT = 5)
    delays_actual = DelayModel(obs = 3, act = 6, com = 13, ΔT = 5)
    H = 20
    ΔT = delays_model.ΔT

    N = 4
    formation = begin
        l = 0.5
        Δϕ = 360° / (N-1) 
        circle = [X(x = l * cos(Δϕ * i), y = l * sin(Δϕ * i), θ = 0) for i in 1:N-1]
        [[zero(X)]; circle]
    end

    RefK = RefPointTrackControl(;
        dy = dy_model, ref_pos = dy_model.l, ctrl_interval = delta_t * ΔT, kp = 1.0, ki = 0.0, kd = 0.0)
    central = FormationControl(formation, RefK, dy_model)

    formation = rotate_formation(formation, 0°)
    init = map(1:N) do i 
        x = formation[i]
        x, zero(Z), zero(U)
    end

    world_model = WorldDynamics(fill((dy_model, StaticObsDynamics()), N))
    # framework = NaiveCF(X, Z, N, central, msg_queue_length(delays_model), ΔT)
    framework = let 
        x_weights = fill(X(x = 1, y = 1, θ = 1), N)
        u_weights = fill(U(v̂ = 1, ψ̂ = 1), N)
        OvCF(central, world_model, delays_model, x_weights, u_weights; X, Z, N, H)
    end

    comps = ["x", "y", "θ", "ψ", "v"]
    function record_f(xs, zs, us)
        comps = [:x, :y, :θ, :ψ, :v]
        hcat(((p -> getfield(p,c)).(xs) for c in comps)...)
    end

    world = ([(dy_actual, leader_z_dy); fill((dy_actual, StaticObsDynamics()), N-1)] 
            |> WorldDynamics)
    result, logs = simulate(
        world, delays_actual, framework, init, (comps, record_f), 1:t_end; xs_observer)
    # visualize(result; delta_t = 1 / freq) |> display
    if plot_result
        plot_formation(result, freq, central) |> display
    end
    result, logs
end

log_range(lb,ub,n) = exp.(range(log(lb), log(ub); length = n))
quadratic_range(ub,n; reverse=false) = let f(x) = sign(x) * x^2 * ub
    l, r = reverse ? (1,-1) : (-1,1)
    f.(range(l, r; length=n))
end

function refpoint_from_mouse(scene, clock) 
    last_pos = SVector{2, ℝ}(scene.events.mouseposition[])
    rp = Node((last_pos, zero(last_pos)))
    on(clock) do _
        x = SVector{2, ℝ}(scene.events.mouseposition[])
        v = x - last_pos
        rp[] = (x, v)
        last_pos = x
    end 
    rp
end


function live_demo(;dynamics_noise = 0.005, sensor_noise = dynamics_noise)
    X, U = CarX{ℝ}, CarU{ℝ}
    Z = U
    freq = 100.0
    delta_t = 1 / freq
    
    scene, layout = layoutscene(resolution = (1800, 1600))

    function mk_silder!(name, range, parent_layout, unit = "")
        ls = labelslider!(scene, name, range, format = x -> @sprintf("%.3g%s", x, unit))
        parent_layout[end+1,1] = ls.layout
        ls.slider
    end

    params_layout = layout[1,2] = GridLayout()
    ax_view = layout[1,1] = LAxis(
        scene, title = "Scene", aspect = DataAspect(), 
        backgroundcolor = RGBf0(0.98, 0.98, 0.98),
        xrectzoom = false, yrectzoom= false)
    controls_layout = layout[2,1] = GridLayout()
    colsize!(layout, 1, Relative(3/4))
    rowsize!(layout, 1, Relative(4/5))

    dy_noise_control = mk_silder!(
        "dynamics noise", log_range(1e-5, 1e-2, 100), params_layout)
    sensor_noise_control = mk_silder!(
        "sensor noise", log_range(1e-5, 1e-2, 100), params_layout)

    params = (
        dy_noise = dy_noise_control.value, 
        sensor_noise = sensor_noise_control.value)

    # Car dynamics parameters
    dy_model  = CarDynamics(;delta_t, max_ψ = 45°)
    rng = MersenneTwister(1234)
    function add_noise(x::X, t)::X where X
        x + params.dy_noise[] * CarX(
            x=0.0, y=0.0, θ=0.0, v = randn(rng, ℝ), ψ = randn(rng, ℝ))
    end
    dy_actual = @set dy_model.add_noise = add_noise
    
    function xs_observer(xs, t)
        (x -> x + randn(rng, X) * sensor_noise).(xs)
    end

    @unpack max_v, max_ψ = dy_actual
    max_ψ = rad2deg(max_ψ)
    slider_ticks = 500
    ext_v = mk_silder!(
        "v̂", quadratic_range(max_v, slider_ticks), controls_layout, "m/s")
    ext_ψ = mk_silder!(
        "ψ̂", quadratic_range(max_ψ, slider_ticks, reverse = true), controls_layout, "°")

    function change_turn(delta) 
        set_close_to!(ext_ψ, ext_ψ.value[] + 4delta * max_ψ / slider_ticks)
    end
    function change_speed(delta) 
        set_close_to!(ext_v, ext_v.value[] + 4delta * max_v / slider_ticks)
    end

    function external_control(x, t)
        CarU{ℝ}(ext_v.value[], deg2rad(ext_ψ.value[]))
    end
    leader_z_dy = FormationObsDynamics(external_control)

    # running at 20Hz
    delays_model  = DelayModel(obs = 3, act = 6, com = 13, ΔT = 5)
    delays_actual = delays_model
    H = 20
    ΔT = delays_model.ΔT

    N = 4
    formation = begin
        l = 0.5
        Δϕ = 360° / (N-1) 
        circle = [X(x = l * cos(Δϕ * i), y = l * sin(Δϕ * i), θ = 0) for i in 1:N-1]
        [[zero(X)]; circle]
    end

    RefK = RefPointTrackControl(;
        dy = dy_model, ref_pos = dy_model.l, ctrl_interval = delta_t * ΔT, 
        kp = 1.0, ki = 0, kd = 0)
    central = FormationControl(formation, RefK, dy_model)

    formation = rotate_formation(formation, 0°)
    init = map(1:N) do i 
        x = formation[i]
        x, zero(Z), zero(U)
    end

    world_model = WorldDynamics(fill((dy_model, StaticObsDynamics()), N))
    # framework = NaiveCF(X, Z, N, central, msg_queue_length(delays_model), ΔT)
    framework = let 
        x_weights = fill(X(x = 1, y = 1, θ = 1), N)
        u_weights = fill(U(v̂ = 1, ψ̂ = 1), N)
        OvCF(central, world_model, delays_model, x_weights, u_weights; X, Z, N, H)
    end

    xs_node = Node((p -> p[1]).(init))
    colors = [get(ColorSchemes.Spectral_10, i/N) for i in 1:N]
    begin
         # draw cars
        for id in 1:N
            car = @lift let 
                @unpack x, y, θ = $xs_node[id]
                car_triangle(x, y, θ)
            end
            poly!(ax_view, car; color = colors[id], strokecolor=:black, strokewidth=2)
        end
        # draw ref points
        refpoints = @lift let
            ctrl = central
            leader = $xs_node[1]
            @_ (ctrl.formation 
                |> map(ref_point(ctrl.K, _),__) 
                |> map(to_formation_frame(ctrl, leader), __))
        end
        points = @lift (x -> Point2f0(x[1])).($refpoints)
        directions = @lift (x -> Point2f0(x[2])).($refpoints)
        arrows!(ax_view, points, directions; 
            linecolor=:red, arrowcolor=:red, lengthscale=0.2, arrowsize=0.1)
        scatter!(ax_view, points; color=:red)
    end

    
    function callback(xs,zs,us,t)
        xs_node[] = xs

        let button = scene.events.keyboardbuttons[]
            ispressed(button, Keyboard.left) && change_turn(1)
            ispressed(button, Keyboard.right) && change_turn(-1)
            ispressed(button, Keyboard.up) && change_speed(1)
            ispressed(button, Keyboard.down) && change_speed(-1)
        end
        current_time = Dates.now()
        time = Dates.canonicalize(current_time - start_time)
        print("Interactive Simulation running ($time)...  \r"); flush(stdout)
            
        to_sleep = delta_t - (current_time - last_time).value/1000
        sleep(max(0, to_sleep))
        last_time = current_time
    end

    world = ([(dy_actual, leader_z_dy); fill((dy_actual, StaticObsDynamics()), N-1)] 
            |> WorldDynamics)
    scene |> display
    start_time = Dates.now()
    last_time = Dates.now()
    simulate(
        world, delays_actual, framework, init, callback, (1, typemax(𝕋)); xs_observer)
end

run_example = formation_example