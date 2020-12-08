log_range(lb,ub,n) = exp.(range(log(lb), log(ub); length = n))

quadratic_range(ub,n; reverse=false) = let f(x) = sign(x) * x^2 * ub
    l, r = reverse ? (1,-1) : (-1,1)
    f.(range(l, r; length=n))
end

function live_demo()
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
        "dynamics noise", log_range(1e-5, 1e-1, 100), params_layout)
    sensor_noise_control = mk_silder!(
        "sensor noise", log_range(1e-5, 1e-1, 100), params_layout)

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
        (x -> x + randn(rng, X) * params.sensor_noise[]).(xs)
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
    triangle_formation = let
        l = 0.5
        Δϕ = 360° / (N-1) 
        circle = [X(x = l * cos(Δϕ * i), y = l * sin(Δϕ * i), θ = 0) for i in 1:N-1]
        [[zero(X)]; circle]
    end
    horizontal_formation = let
        l = 0.5
        leader_idx = round_ceil(N/2)
        line = [X(x = 0, y = l * (i - leader_idx), θ = 0) for i in 1:N]
        [line[mod1(leader_idx + j - 1, N)] for j in 1:N]
    end
    vertical_formation = let
        l = 0.5
        leader_idx = round_ceil(N/2)
        line = [X(x = l * (i - leader_idx), y = 0, θ = 0) for i in 1:N]
        [line[mod1(leader_idx + j - 1, N)] for j in 1:N]
    end
    formation_names = ["triangle", "horizontal", "vertical"]
    formation_map = Dict{String, Formation{ℝ}}(zip(formation_names, 
        [triangle_formation, horizontal_formation, vertical_formation]))
    formation_control = controls_layout[0, 1] = LMenu(
        scene, direction = :up, 
        options = formation_names, selection = formation_names[1])

    formation_f() = formation_map[formation_control.selection[]]

    RefK = RefPointTrackControl(;
        dy = dy_model, ref_pos = dy_model.l, ctrl_interval = delta_t * ΔT, 
        kp = 1.0, ki = 0, kd = 0)
    central = FormationControl((_, _) -> formation_f(), RefK, dy_model)

    init = let 
        formation = rotate_formation(formation_f(), 0°)
        map(1:N) do i 
            x = formation[i]
            x, zero(Z), zero(U)
        end
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
            @_ (formation_f()
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