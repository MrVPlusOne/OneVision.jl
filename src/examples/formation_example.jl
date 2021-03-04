function plot_formation(data::TrajectoryData, freq::ℝ, ctrl::FormationControl, formation_at_t)
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
        @_ (formation_at_t($t) 
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

struct FormationWallObsDynamic{Fu, Ff} <: ObsDynamics 
    "`external_u(x, z, t) -> u`"
    external_u::Fu
    "`external_formation(x, t) -> formation_idx`"
    external_formation::Ff
end

function OneVision.obs_forward(dy::FormationWallObsDynamic, x, z, t::𝕋)
    HVec(dy.external_u(x, z, t), dy.external_formation(x, t))
end

function formation_example(;time_end = 20.0, freq = 100.0, 
        noise = 0.005, sensor_noise = noise, 
        delays = default_delays,
        CF::CFName = onevision_cf,
        switch_formation = true,
        track_config = false,
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
        return U(v̂ = 0.0, ψ̂ = 0.0)
        (if t ≤ 3 * freq
            U(v̂ = 1.0, ψ̂ = 0.0)
        elseif t ≤ 5 * freq
            U(v̂ = 1.0, ψ̂ = 8°)
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
    H = 20
    ΔT = delays_model.ΔT

    N = 2 # number of car
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

    world = ([(dy_actual, leader_z_dy); fill((dy_actual, StaticObsDynamics()), N-1)] 
            |> WorldDynamics)
    result, (logs, loss) = simulate(
        world, delays_actual, framework, init, (comps, record_f), 1:t_end
        ; xs_observer, loss_model)
    if plot_result
        visualize(result; delta_t = 1 / freq, loss) |> display
        plot_formation(result, freq, central, 
            t -> form_from_id(external_formation(missing, t))) |> display
    end
    loss
end

function formation_ros_example(;time_end = 20.0, freq = 100.0, 
    noise = 0.005, sensor_noise = noise, 
    delays = default_delays,
    CF::CFName = onevision_cf,
    switch_formation = true,
    track_config = false,
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
        (if t ≤ 3 * freq
            U(v̂ = 1.0, ψ̂ = 0.0)
        elseif t ≤ 5 * freq
            U(v̂ = 1.0, ψ̂ = 8°)
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
    H = 20
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

    world = ([(dy_actual, leader_z_dy); fill((dy_actual, StaticObsDynamics()), N-1)] 
            |> WorldDynamics)
    result, (logs, loss) = ros_simulate(
        world, delays_actual, framework, init, (comps, record_f), 1:t_end
        ; xs_observer, loss_model)
    if plot_result
        visualize(result; delta_t = 1 / freq, loss) |> display
        plot_formation(result, freq, central, 
            t -> form_from_id(external_formation(missing, t))) |> display
    end
    loss
end


"""
TODO: change to file operation
"""

"""
Open loop simulation
"""

function run_open_example(car_id::Integer, fleet_size::Integer, freq::Int32, preheat::Bool)
    N::Int64 = fleet_size
    #init_status = get_initial_states(N)
    println("in open loop")
    framework, init_status, X, Z, U, track_config = get_framework(car_id, fleet_size, freq)

    port::Integer = car_id + 5000

    """
    Socket Communication only
    """
    function parse_state(result::Dict{String, Any})
        x = result["x"]
        return x
    end

    function parse_obs(result::Dict{String, Any}) 
        @debug result
        z = 
        if track_config == "wall_obs"
            Z((result["z"]["c"][1], result["z"]["c"][2]), result["z"]["d"])
        else
            HVec{U, ℕ}(result["z"]["c"], result["z"]["d"])
        end
        return z
    end

    function parse_msg(result::Dict{String, Any}, M_TYPE)::Pair{Each{ℕ}, Each{M_TYPE}}
        #println("msgs are", result["msgs"])
        msg_array::Each{Vector{UInt8}} = result["msgs"]
        msg_senders::Each{ℕ} = result["msgs_senders"]
        msgs::Each{M_TYPE} = [deserialize_from_b_array(m) for m in msg_array] #[deserialize(msg) for msg in result["msgs"]]
        return Pair(msg_senders, msgs)
    end

    start_framework(framework, init_status, car_id, port, fleet_size, freq, preheat, parse_state, parse_obs, parse_msg)
end

"""
Open loop simulation
return framework
"""
function get_framework(    
    car_id :: Integer,
    fleet_size ::Integer,
    freq :: Int32,
    CF::CFName = onevision_cf,
    switch_formation::Bool = false,
    fn::String = "config/params.ini"
)
    # read config from file
    conf = ConfParse("config/params.ini")
    parse_conf!(conf)
    track_config::String = retrieve(conf, "controller", "track_config")
    # delays
    obs_delay = retrieve(conf, "delay", "obs", 𝕋)
    act_delay = retrieve(conf, "delay", "act", 𝕋)
    comm_delay = retrieve(conf, "delay", "comm", 𝕋)
    ctrl_interval = retrieve(conf, "delay", "ctrl", 𝕋)
    delays = DelayModel(obs = obs_delay, act = act_delay, com = comm_delay, ΔT = ctrl_interval)
    
    #cf = naive_cf local_cf const_u_cf onevision_cf
    cfname = retrieve(conf, "controller", "cf")
    CF::CFName = if cfname == "onevision_cf"
        onevision_cf
    elseif cfname == "const_u_cf"
        const_u_cf
    elseif cfname == "local_cf"
        local_cf
    elseif cfname == "naive_cf"
        naive_cf
    else
        @error "CFName not valid, current input is $cfname"
        exit(1)
    end

    X, U = CarX{ℝ}, CarU{ℝ}
    Z, init_z = if(track_config == "wall_obs")
        HVec{Tuple{ℝ, ℝ}, ℕ}, ()->Z((Inf, Inf), retrieve(conf, "formation", "default_formation_id", Int64)) # stop distance, formation
    else
        HVec{U, ℕ}, ()->Z(U(0.0, 0.0), retrieve(conf, "formation", "default_formation_id", Int64))# action, formation
    end
    delta_t = 1.0 / freq

    # Car dynamics parameters
    dy_model = CarDynamics(;   
    delta_t=delta_t,
    max_v = retrieve(conf, "dynamics", "max_v", ℝ),
    max_ψ = retrieve(conf, "dynamics", "max_psi", ℝ),
    l = retrieve(conf, "dynamics", "l", ℝ) ,
    k_v = retrieve(conf, "dynamics", "k_v", ℝ),
    k_ψ = retrieve(conf, "dynamics", "k_psi", ℝ),
    k_max_a = retrieve(conf, "dynamics", "k_max_a", ℝ),
    k_max_ω = retrieve(conf, "dynamics", "k_max_omega", ℝ))
    @debug "$dy_model"

    delays_model = delays
    H = 20
    ΔT = delays_model.ΔT

    N::Int64 = fleet_size

    init_dir = homedir() * "/controllers/one_vision_harness/init_status/"
    init = Each{Tuple{X, Z, U}}()
    for i in 1:fleet_size
        open(init_dir * "init_status_$i") do io
            x = parse(ℝ, readline(io))
            y = parse(ℝ, readline(io))
            θ = parse(ℝ, readline(io))
            v = parse(ℝ, readline(io))
            s = parse(ℝ, readline(io))
            i_s = (X(x, y, θ, v, s), init_z(), U(0.0, 0.0))
            @assert typeof(i_s) == Tuple{X, Z, U}
            push!(init,  (X(x, y, θ, v, s), init_z(), U(0.0, 0.0)))
        end;
    end
    @debug "init status read"
    triangle_sep::Float64 = retrieve(conf, "formation", "triangle_sep", Float64)
    vertical_sep::Float64 = retrieve(conf, "formation", "vertical_sep", Float64)
    horizental_sep::Float64 = retrieve(conf, "formation", "horizental_sep", Float64)
    @debug "sep: $triangle_sep, $vertical_sep, $horizental_sep"
    triangle_formation = let
        if(N == 1)
            [[zero(X)]; []]
        elseif N == 3
            # custom 3 formation
            l = triangle_sep
            Δϕs= [135°, -135°]
            circle = [X(x = l * cos(ϕ), y = l * sin(ϕ), θ = 0.0) for ϕ in Δϕs]
            [[zero(X)]; circle]
        else
            l = triangle_sep
            Δϕ = 360° / (N-1) 
            circle = [X(x = l * cos(Δϕ * i), y = l * sin(Δϕ * i), θ = 0.0) for i in 1:N-1]
            [[zero(X)]; circle]
        end
    end
    vertical_formation = let
        l = vertical_sep
        line = [X(x = -l * (i-1), y = 0.0, θ = 0.0) for i in 1:N]
        
        """
        l = vertical_sep
        leader_idx = round_ceil(N/2)
        line = [X(x = l * (i - leader_idx), y = 0.0, θ = 0.0) for i in 1:N]
        [line[mod1(leader_idx + j - 1, N)] for j in 1:N]        
        """
        line
    end

    horizontal_formation = let
        l = horizental_sep
        leader_idx = round_ceil(N/2)
        line = [X(x = 0, y = l * (i - leader_idx), θ = 0) for i in 1:N]
        [line[mod1(leader_idx + j - 1, N)] for j in 1:N]
    end

    formations = [triangle_formation, horizontal_formation, vertical_formation]
    form_from_id(i) = formations[i]

    kp::Float64 = retrieve(conf, "pid", "kp", Float64)
    ki::Float64 = retrieve(conf, "pid", "ki", Float64)
    kd::Float64 = retrieve(conf, "pid", "kd", Float64)
    kv::Float64 = retrieve(conf, "pid", "kv", Float64)

    @debug "kp $kp ki $ki kd $kd"

    RefK = if track_config == "config_track"
        ConfigTrackControl(dy_model, 100.0, 0.0, 3.0)
    elseif track_config == "track_ref"
        RefPointTrackControl(;
            dy = dy_model, ref_pos = dy_model.l, ctrl_interval = delta_t * ΔT, 
            kp = kp, ki = ki, kd = kd, kv=kv)
    elseif track_config == "wall_obs"
        WallObsControl(;
        dy = dy_model, ref_pos = dy_model.l, ctrl_interval = delta_t * ΔT, 
        kp = kp, ki = ki, kd = kd, kv=kv,
        stop_distance = retrieve(conf, "wall", "stop_distance", Float64), 
        target_v = retrieve(conf, "wall", "target_v", Float64),
        p1 = [parse(ℝ, i) for i in retrieve(conf, "wall", "p1")],
        p2 = [parse(ℝ, i) for i in retrieve(conf, "wall", "p2")],
        r = retrieve(conf, "wall", "r", ℝ),
        kp_leader = retrieve(conf, "wall", "kp_leader", ℝ),
        ki_leader = retrieve(conf, "wall", "ki_leader", ℝ),
        kd_leader = retrieve(conf, "wall", "kd_leader", ℝ),
        kv_leader = retrieve(conf, "wall", "kv_leader", ℝ),
        )
    else
        error("Tracking configuration not found! Current value is $track_config")
        exit(0)
    end
    avoidance = CollisionAvoidance(scale=1.0, min_r=dy_model.l*1.2, max_r=1.5*dy_model.l)
    central = FormationControl((_, zs, _) -> form_from_id(zs[1].d),
        RefK, dy_model, avoidance)

    world_model = WorldDynamics(fill((dy_model, StaticObsDynamics()), N))

    loss_s_x::Float64 = retrieve(conf, "loss", "s_x", Float64)
    loss_s_y::Float64 = retrieve(conf, "loss", "s_y", Float64)
    loss_s_θ::Float64 = retrieve(conf, "loss", "s_theta", Float64)
    loss_s_v::Float64 = retrieve(conf, "loss", "s_v", Float64)
    loss_s_ψ::Float64 = retrieve(conf, "loss", "s_psi", Float64)
    loss_u_v::Float64 = retrieve(conf, "loss", "u_v", Float64)
    loss_u_ψ::Float64 = retrieve(conf, "loss", "u_psi", Float64)
    @debug "loss: [$loss_s_x, $loss_s_y, $loss_s_θ, $loss_s_v, $loss_s_ψ], [$loss_u_v, $loss_u_ψ]"
    loss_model = let 
        x_weights = SVector{N}(fill(X(x = loss_s_x, y = loss_s_y, θ = loss_s_θ, v=loss_s_v, ψ=loss_s_ψ), N))
        u_weights = SVector{N}(fill(U(v̂ = loss_u_v, ψ̂ = loss_u_ψ), N))
        RegretLossModel(central, world_model, x_weights, u_weights)
    end


    # logging function

    log_interval = retrieve(conf, "log", "log_interval", Int64)
    function f_log(x::Tuple{ℕ,𝕋,X,Z})::Bool where {X,Z}
        if log_interval == 0
            return false
        else
            return x[2]%log_interval == 0
        end
    end
    framework = mk_cf(CF, world_model, central, delays, loss_model, FuncT(f_log, Tuple{ℕ,𝕋,X,Z}, Bool); X, Z, H)

    @info "init status is $init"
    #exit(0)
    return framework,  init, X, Z, U, track_config
end

# TODO: change to distributed setting
#function OneVision.parse_obs(result::Dict{String, Any})
#    return [HVec{CarU{ℝ}, ℕ}(CarU{ℝ}(i["c"]), i["d"]) for i in result["zs"]]
#end
