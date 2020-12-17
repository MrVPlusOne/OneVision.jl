log_range(lb,ub,n) = exp.(range(log(lb), log(ub); length = n))

quadratic_range(ub,n; reverse=false) = let f(x) = sign(x) * x^2 * ub
    l, r = reverse ? (1,-1) : (-1,1)
    f.(range(l, r; length=n))
end

function build_ui(max_v, max_ψ, formation_names)
    scene, layout = layoutscene(resolution = (2000, 1600))
    
    function mk_silder!(name, range, parent_layout, unit = "")
        ls = labelslider!(scene, name, range, format = x -> @sprintf("%.3g%s", x, unit))
        parent_layout[end+1,1] = ls.layout
        ls.slider
    end

    ax_view = layout[1,1] = LAxis(
        scene, title = "Scene", aspect = DataAspect(), 
        backgroundcolor = RGBf0(0.98, 0.98, 0.98))
    display_layout = layout[1,2] = GridLayout()
    controls_layout = layout[2,1] = GridLayout()
    params_layout = layout[2,2] = GridLayout()
    colsize!(layout, 1, Relative(2/3))
    rowsize!(layout, 1, Relative(4/5))

    dy_noise_control = mk_silder!(
        "dynamics noise", log_range(1e-5, 1e-1, 100), params_layout)
    sensor_noise_control = mk_silder!(
        "sensor noise", log_range(1e-5, 1e-1, 100), params_layout)
    
    max_ψ = rad2deg(max_ψ)
    slider_ticks = 500
    v_slider = mk_silder!(
        "v̂", range(-max_v, max_v, length = slider_ticks), controls_layout, "m/s")
    ψ_slider = mk_silder!(
        "ψ̂", quadratic_range(max_ψ, slider_ticks, reverse = true), controls_layout, "°")
    formation_control = controls_layout[0, 1] = LMenu(
        scene, direction = :up, 
        options = formation_names, selection = formation_names[1])

    function handle_keyboard()
        function change_turn(delta) 
            set_close_to!(ψ_slider, ψ_slider.value[] + 4delta * max_ψ / slider_ticks)
        end
        function change_speed(delta) 
            set_close_to!(v_slider, v_slider.value[] + 4delta * max_v / slider_ticks)
        end
        function change_formation(i)
            formation_control.i_selected[] = i
        end

        let button = scene.events.keyboardbuttons[]
            ispressed(button, Keyboard.left) && change_turn(1)
            ispressed(button, Keyboard.right) && change_turn(-1)
            ispressed(button, Keyboard.up) && change_speed(1)
            ispressed(button, Keyboard.down) && change_speed(-1)
            ispressed(button, Keyboard._1) && change_formation(1)
            ispressed(button, Keyboard._2) && change_formation(2)
            ispressed(button, Keyboard._3) && change_formation(3)
        end
    end

    (
        scene = scene,
        ax_view = ax_view,
        display_layout = display_layout,
        dy_noise = dy_noise_control, 
        sensor_noise = sensor_noise_control,
        v_slider = v_slider,
        ψ_slider = ψ_slider,
        formation_control = formation_control,
        handle_keyboard = handle_keyboard,
    )
end

function tryset_ylims(scene, (lower, upper), margin = 0.1)
    Δ = upper - lower
    (Δ != 0) && ylims!(scene, (lower - margin * Δ, upper + margin * Δ))
end

function draw_view(ui, freq, init, central, dy_model, form_from_id)
    @unpack ax_view, display_layout, formation_control, scene = ui

    N = length(init)
    xs_node = Node((p -> p[1]).(init))
    traj_len = round_ceil(freq * 5)
    traj_qs = [Node(constant_queue(init[i][1], traj_len)) for i in 1:N]
    colors = [get(ColorSchemes.Spectral_10, i/N) for i in 1:N]

    loss_node = Node(AxisArray(zeros(ℝ, N, 2), id=1:N, comp=[:u, :x]))
    loss_len = traj_len
    loss_q = constant_queue(loss_node[], loss_len)
    u_loss = [Node(zeros(ℝ, loss_len)) for _ in 1:N]
    x_loss = [Node(zeros(ℝ, loss_len)) for _ in 1:N]
    total_loss = Node(zeros(ℝ, loss_len))

    # draw loss history
    on(loss_node) do loss
        pushpop!(loss_q, copy(loss))
        for i in 1:N
            u_loss[i][] = [l[comp=:u, id = i] for l in loss_q]
            x_loss[i][] = [l[comp=:x, id = i] for l in loss_q]
        end
        total_loss[] = map(sum, loss_q)
    end

    function loss_ax(title)
        display_layout[end+1,1] = LAxis(scene, title = title, 
            xzoomlock = true, xpanlock = true, yrectzoom = true)
    end
    ax_total, ax_x, ax_u = loss_ax.(["Total Loss", "X Loss", "U Loss"])
    trim!(display_layout)
    
    for i in 1:N
        lines!(ax_x, 1:loss_len, x_loss[i]; color=colors[i])
        lines!(ax_u, 1:loss_len, u_loss[i]; color=colors[i])
    end
    lines!(ax_total, 1:loss_len, total_loss; color=:red)
    on(total_loss) do loss
        tryset_ylims(ax_total, (minimum(loss), maximum(loss)))
        get_min(losses) = minimum(n -> minimum(n[]), losses)
        get_max(losses) = maximum(n -> maximum(n[]), losses)
        tryset_ylims(ax_x, (get_min(x_loss), get_max(x_loss)))
        tryset_ylims(ax_u, (get_min(u_loss), get_max(u_loss)))
    end
    
    function draw_refs(ctrl::FormationControl{RefPointTrackControl})
        refpoints = @lift let
            leader = $xs_node[1]
            @_ (form_from_id(formation_control.i_selected[])
                |> map(ref_point(ctrl.K, _),__) 
                |> map(to_formation_frame(ctrl, leader), __))
        end
        points = @lift (x -> Point2f0(x[1])).($refpoints)
        directions = @lift (x -> Point2f0(x[2])).($refpoints)
        arrows!(ax_view, points, directions; 
            linecolor=:red, arrowcolor=:red, lengthscale=0.2, arrowsize=0.1)
        scatter!(ax_view, points; color=:red)
    end

    function draw_refs(ctrl::FormationControl{ConfigTrackControl})
        ref_cars = @lift let
            leader = $xs_node[1]
            center, ω = center_of_rotation(leader, dy_model.l)
            form = form_from_id(formation_control.i_selected[])
            s′ = form[1]
            rot = rotation2D(leader.θ - s′.θ)
            offset = get_pos(leader) - get_pos(s′)
            transform = s -> rot * s + offset
            @_ (form
                |> map(transform ∘ get_pos, __) 
                |> map(rotate_around(center, ω, dy_model.l, _), __))
        end
        for id in 1:N
            car = @lift let 
                @unpack x, y, θ = $ref_cars[id]
                car_triangle(x, y, θ)
            end
            poly!(ax_view, car; color = with_alpha(colors[id], 0.3))
        end
    end

    # draw trajectories
    for id in 1:N
        q = traj_qs[id]
        traj_x = @lift [x.x for x in $q]
        traj_y = @lift [x.y for x in $q]
        on(xs_node) do xs
            pushpop!(q[], xs[id])
            q[] = q[]
        end
        c = colors[id]
        cmap = [with_alpha(c, 0.4), with_alpha(c, 1)]
        color = range(0, 1, length = traj_len)
        lines!(ax_view, traj_x, traj_y; color, colormap = cmap, linewidth=3)
    end
    # draw cars
    for id in 1:N
        car = @lift let 
            @unpack x, y, θ = $xs_node[id]
            car_triangle(x, y, θ)
        end
        poly!(ax_view, car; color = colors[id], strokecolor=:black, strokewidth=2)
    end
    # draw ref points
    draw_refs(central)

    display(ui.scene)
    xs_node, loss_node
end

function live_demo()
    X, U = CarX{ℝ}, CarU{ℝ}
    Z = HVec{U, ℕ}
    ΔT = 5
    freq = 20.0 * ΔT
    delta_t = 1 / freq

    max_v = 5.0
    max_ψ = 45°
    formation_names = ["triangle", "horizontal", "vertical"]
    ui = build_ui(max_v, max_ψ, formation_names)

    # Car dynamics parameters
    dy_model = CarDynamics(;delta_t, max_ψ = 45°, integrator_samples = round_ceil(5 / ΔT))
    rng = MersenneTwister(1234)
    function add_noise(x::X, t)::X where X
        x + ui.dy_noise.value[] * CarX(
            x=0.0, y=0.0, θ=0.0, v = randn(rng, ℝ), ψ = randn(rng, ℝ))
    end
    dy_actual = @set dy_model.add_noise = add_noise
    
    function xs_observer(xs, t)
        (x -> x + randn(rng, X) * ui.sensor_noise.value[]).(xs)
    end

    N = 4
    triangle_formation = let
        l = 0.8
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
        l = 0.6
        leader_idx = round_ceil(N/2)
        line = [X(x = l * (i - leader_idx), y = 0, θ = 0) for i in 1:N]
        [line[mod1(leader_idx + j - 1, N)] for j in 1:N]
    end
    formation_map = Dict{String, Formation{ℝ}}(zip(formation_names, 
        [triangle_formation, horizontal_formation, vertical_formation]))
    form_from_id(i) = formation_map[formation_names[i]]

    function external_control(x, t)
        CarU{ℝ}(ui.v_slider.value[], deg2rad(ui.ψ_slider.value[]))
    end
    function external_formation(x, t)
        ui.formation_control.i_selected[]
    end
    leader_z_dy = FormationObsDynamics(external_control, external_formation)

    delays_model  = DelayModel(obs = 2, act = 4, com = 6, ΔT = ΔT)
    # delays_model  = DelayModel(obs = 0, act = 0, com = 4ΔT, ΔT = ΔT)
    delays_actual = delays_model
    H = 20

    RefK = RefPointTrackControl(;
        dy = dy_model, ref_pos = dy_model.l, ctrl_interval = delta_t * ΔT, 
        kp = 1.0, ki = 0, kd = 0)
    # RefK = ConfigTrackControl(;dy = dy_model, k1 = 2.0, k2 = 0.5, k3 = 1.0)
    avoidance = CollisionAvoidance(scale=1.0, min_r=dy_model.l, max_r=3*dy_model.l)
    central = FormationControl((_, zs, _) -> form_from_id(zs[1].d), 
        RefK, dy_model, avoidance)

    init = let 
        formation = rotate_formation(form_from_id(1), 0°)
        map(1:N) do i 
            x = formation[i]
            x, HVec(zero(U), 1), zero(U)
        end
    end

    world_model = WorldDynamics(fill((dy_model, StaticObsDynamics()), N))
    x_weights = SVector{N}(fill(X(x = 1, y = 1, θ = 1), N))
    u_weights = SVector{N}(fill(U(v̂ = 1, ψ̂ = 1), N))
    loss_model = RegretLossModel(central, world_model, x_weights, u_weights)

    # framework = NaiveCF(X, Z, N, central, msg_queue_length(delays_model), ΔT)
    # framework = LocalCF(central, world_model, delays_model; X, Z)
    framework = OvCF(loss_model, delays_model; Z, H)
    world = ([(dy_actual, leader_z_dy); fill((dy_actual, StaticObsDynamics()), N-1)] 
            |> WorldDynamics)

    xs_node, loss_node = draw_view(ui, freq, init, central, dy_model, form_from_id)
    start_time = Dates.now()
    last_time = Dates.now()
    function callback(xs,zs,us,loss,t)
        xs_node[] = xs
        loss_node[] = loss
        ui.handle_keyboard()

        current_time = Dates.now()
        time = Dates.canonicalize(current_time - start_time)
        print("Interactive Simulation running ($time)...  \r"); flush(stdout)
            
        to_sleep = delta_t - (current_time - last_time).value/1000
        sleep(max(0, to_sleep))
        last_time = current_time
    end
    simulate(
        world, delays_actual, framework, init, callback, (1, typemax(𝕋)); 
        loss_model, xs_observer)
end