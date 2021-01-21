module Car1DExample
export CarX, CarZ, CarU, car_system, WallObsDynamics, LeaderFollowerControl

using OneVision
using OneVision: ℝ, 𝕋, ℕ, @kwdef
using OneVision.Examples
using Random
using StaticArrays
using Plots
using Serialization

import OneVision

struct CarX{R} <: FieldVector{2,R}
    pos::R
    velocity::R
end

struct CarU{R} <: FieldVector{1,R}
    acc::R
end

struct CarZ{R} <: FieldVector{2,R}
    detected::R
    distance::R
end


const car_A = @SMatrix [0.0 1.0; 0.0 0.0]
const car_B = @SMatrix [0.0; 1.0]


function car_system(delta_t::ℝ, noise::Function = _ -> CarX(0.0, 0.0))
    A′, B′ = discretize(car_A, car_B, delta_t)
    SysDynamicsLTI(A′, B′, noise)
end
    
"The wall observation model used for simulation.\n"
@kwdef struct WallObsDynamics <: ObsDynamics
    wall_position::Union{ℝ,Nothing}
    detector_range::ℝ
end

function OneVision.obs_forward(
    dy::WallObsDynamics, x::CarX, z::CarZ{R}, t::𝕋
)::CarZ{R} where R
    if Bool(z.detected)
        z
    elseif (isnothing(dy.wall_position) 
            || x.pos + dy.detector_range < dy.wall_position)
        [0.0, 0.0]
    else
        [1.0, dy.wall_position]
    end
end


"""
The simple wall dynamics model used by OneVision.
"""
struct WallObsModel <: ObsDynamics end

OneVision.obs_forward(dy::WallObsModel, x::CarX, z::CarZ, t::𝕋) = z

"""
A controller struct - holds all necessary information 
"""
@kwdef struct LeaderFollowerControl <: CentralControlStateless{CarU{ℝ}}
    "Will output u=0 before this time"
    warm_up_time::𝕋  
    "Will only use bang bang control when difference is larger than this"
    bang_bang_tol::ℝ = Inf
    k_v::ℝ = 10.0#3.0
    k_x::ℝ = 2.0
    stop_distance::ℝ = 4.0
    target_v::ℝ = 0.75
end




OneVision.control_one(
    lf::LeaderFollowerControl, xs,zs, t::𝕋, id::ℕ
)::CarU{ℝ} = begin
    tol = lf.bang_bang_tol
    function bang_bang(x̂, x, k, tol)
        if abs(x̂ - x) ≤ tol
            (x̂ - x) * k
        else
            sign(x̂ - x) * tol * k
        end
    end
    
    x, z = xs[id], zs[id]
    if t ≤ lf.warm_up_time
        acc = 0.0
    elseif Bool(round(z.detected)) && z.distance - x.pos ≤ lf.stop_distance
        acc = bang_bang(0.0, x.velocity, lf.k_v, tol)
    elseif id == 1
        # the leader
        acc = bang_bang(lf.target_v, x.velocity, lf.k_v, tol)
    else
        # a follower
        leader = xs[1]
        acc = (control_one(lf, xs, zs, t, 1).acc # mimic the leader
                + bang_bang(leader.velocity, x.velocity, lf.k_v, Inf)
                + bang_bang(leader.pos, x.pos, lf.k_x, Inf))
    end
    CarU(acc) # TODO: change to pid
end

function run_example(;time_end = 20.0, freq::ℝ = 100.0, 
        delays = default_delays,
        CF::CFName = onevision_cf,
        noise = 0.0, 
        has_obstacle = true,
        use_bang_bang = true,
        seed = 1, plot_result = true, log_prediction = false)
    t_end = 𝕋(ceil(time_end * freq))
    delta_t = 1 / freq
    times::Vector{𝕋} = collect(1:t_end)
    idx_to_time(xs) = (xs .- 1) .* delta_t
    rng = MersenneTwister(seed)
    
    agent_info(id) = begin
        acc_noise = randn(rng, ℝ, t_end) * noise
        sys_dy = car_system(delta_t, t -> CarX(0.0, acc_noise[t]))
        obs_dy = 
            if id == 1 && has_obstacle; WallObsDynamics(wall_position = 30.0, detector_range = 8.0)
            else WallObsModel() end
        
        sys_dy, obs_dy
    end
    
    N = 2
    world_dynamics = WorldDynamics([agent_info(i) for i in 1:N])
    init_states = fill((CarX(0.0, 0.0), CarZ(0.0, 0.0), CarU(0.0)), N)
    ΔT = delays.ΔT
    world_model = WorldDynamics(
        fill((car_system(delta_t), WallObsModel()), N))

    comps = ["pos", "velocity", "acceleration", "obstacle"]
    function record_f(xs, zs, us)
        [(x -> x.pos).(xs) (x -> x.velocity).(xs) (u -> u.acc).(us) (z -> z.distance).(zs)]
    end
    
    H = 20
    central = LeaderFollowerControl(
        warm_up_time = delays.act, 
        bang_bang_tol = use_bang_bang ? 1.0 : Inf)
    x_weights = SVector{N}(fill(CarX(10.0, 1.0), N))
    u_weights = SVector{N}(fill(CarU(0.1), N))
    loss_model = RegretLossModel(central, world_model, x_weights, u_weights)
    framework = mk_cf(
        CF, world_model, central, delays, loss_model; 
        X = CarX{ℝ}, Z = CarZ{ℝ}, H)

    result, (logs, loss) = simulate(
        world_dynamics, 
        delays,
        framework,
        init_states,
        (comps, record_f),
        times;
        loss_model,
    )

    if log_prediction
        t0, t1 = (0.0, 11.0)
        id = 1
        x_indices = [t for t in result.times if t0 ≤ (t - 1) * delta_t ≤ t1]
        sorted_log = sort!([x for x in logs[id]], by = x -> x[1])
        δv_x = idx_to_time((x -> x[1]).(sorted_log))
        anim = @gif for (t, log) in sorted_log
            ps = []
            for (c, label) in enumerate(["δx", "δv"])
                δv_y = [log.δxz[1][c] for (_, log) in sorted_log]
                p = plot(idx_to_time(x_indices), 
                        result.values[x_indices,:,c], label = ["agent 1" "agent 2"])
                plot!(p, δv_x, δv_y, label = label)
                t -= 1
                local ts = idx_to_time(t:t + size(log.x̃, 1) - 1)
                plot!(p, ts, (x -> x[c]).(log.x̃); label = ["central 1" "central 2"])
                push!(ps, p)
            end
            plot(ps...; layout = (2, 1))
        end
        anim |> display
    end
    plot_result && display(visualize(result; delta_t, loss))
    
    loss
end

"""
Placed here for reuse of state/initial var definition
"""
function run_ros_example(;time_end = 20.0, freq::ℝ = 20.0, 
    delays = default_delays,
    CF::CFName = onevision_cf,
    noise = 0.0, 
    has_obstacle = true,
    use_bang_bang = true,
    seed = 1, plot_result = true, log_prediction = false)

    #delays = DelayModel(obs = 100, act = 1, com = 6, ΔT = 5)

    t_end = 𝕋(ceil(time_end * freq)) # integer value 
    delta_t = 1 / freq
    times::Vector{𝕋} = collect(1:t_end) # convert to array
    idx_to_time(xs) = (xs .- 1) .* delta_t # convert integer index to discrete timestep. -1 is to ensure start with 0
    rng = MersenneTwister(seed) # rng to generator

    # generate information for each agent
    agent_info(id) = begin
        acc_noise = randn(rng, ℝ, t_end) * noise 
        sys_dy = car_system(delta_t, t -> CarX(0.0, acc_noise[t])) # construct system dynamics
        obs_dy = 
            if id == 1 && has_obstacle; WallObsDynamics(wall_position = 30.0, detector_range = 8.0)
            else WallObsModel() end
        # construct observation dynamics
        sys_dy, obs_dy
    end

    N = 2 # number of robot 
    world_dynamics = WorldDynamics([agent_info(i) for i in 1:N]) # construct world dynamics (state, obs) for each car
    init_states = fill((CarX(0.0, 0.0), CarZ(0.0, 0.0), CarU(0.0)), N) # initialize state for each car
    ΔT = delays.ΔT # DelayModel 
    world_model = WorldDynamics(
        fill((car_system(delta_t), WallObsModel()), N)) # knowledge of the world

    comps = ["pos", "velocity", "acceleration", "obstacle"]
    function record_f(xs, zs, us)
        [(x -> x.pos).(xs) (x -> x.velocity).(xs) (u -> u.acc).(us) (z -> z.distance).(zs)]
    end # used for log purpose

    H = 20
    central = LeaderFollowerControl(
        warm_up_time = delays.act, 
        bang_bang_tol = use_bang_bang ? 1.0 : Inf) # initialize control of the robot
    x_weights = SVector{N}(fill(CarX(10.0, 1.0), N)) # state weight (hyperparam)
    u_weights = SVector{N}(fill(CarU(0.1), N)) # actuation weight (hyperparam)
    loss_model = RegretLossModel(central, world_model, x_weights, u_weights) # regression loss
    framework = mk_cf(
        CF, world_model, central, delays, loss_model; 
        X = CarX{ℝ}, Z = CarZ{ℝ}, H) # construct a controller framework - bundle of information needed for simulation

    result, (logs, loss) = ros_simulate(
        world_dynamics, 
        delays,
        framework,
        init_states,
        (comps, record_f),
        times;
        loss_model,
    )

    if log_prediction
        t0, t1 = (0.0, 11.0)
        id = 1
        x_indices = [t for t in result.times if t0 ≤ (t - 1) * delta_t ≤ t1]
        sorted_log = sort!([x for x in logs[id]], by = x -> x[1])
        δv_x = idx_to_time((x -> x[1]).(sorted_log))
        anim = @gif for (t, log) in sorted_log
            ps = []
            for (c, label) in enumerate(["δx", "δv"])
                δv_y = [log.δxz[1][c] for (_, log) in sorted_log]
                p = plot(idx_to_time(x_indices), 
                        result.values[x_indices,:,c], label = ["agent 1" "agent 2"])
                plot!(p, δv_x, δv_y, label = label)
                t -= 1
                local ts = idx_to_time(t:t + size(log.x̃, 1) - 1)
                plot!(p, ts, (x -> x[c]).(log.x̃); label = ["central 1" "central 2"])
                push!(ps, p)
            end
            plot(ps...; layout = (2, 1))
        end
        anim |> display
    end
    plot_result && display(visualize(result; delta_t, loss))

    loss
end
"""
TODO: change to file operation
"""
function get_initial_states(N)
    return fill((CarX(0.0, 0.0), CarZ(0.0, 0.0), CarU(0.0)), N) # initialize state for each car
end
"""
Open loop simulation
"""
function run_open_example(car_id::Integer, fleet_size::Integer)
    N = fleet_size
    init_status = get_initial_states(N)
    framework, world_dynamics = get_framework(init_status, car_id, fleet_size, has_obstacle = true)

    port::Integer = car_id + 5000

    """
    Socket Communication only
    """
    function parse_state(result::Dict{String, Any})
        x = result["x"]
        return x
    end
    function parse_obs(result::Dict{String, Any}) 
        z = result["z"]
        return z
    end

    function parse_msg(result::Dict{String, Any})::Each{OvMsg}
        println("msgs are", result["msgs"])
        msg_array::Each{Vector{UInt8}} = result["msgs"]
        msgs::Each{OvMsg} = [deserialize_from_b_array(m) for m in msg_array] #[deserialize(msg) for msg in result["msgs"]]
        return msgs
    end

    start_framework(world_dynamics, framework, init_status, car_id, port, fleet_size, parse_state, parse_obs, parse_msg)
end

"""
Open loop simulation
return framework
"""
function get_framework(    
    init_status :: Each{Tuple{X,Z,U}},
    car_id :: Integer,
    fleet_size ::Integer
    ;freq::ℝ = 20.0, 
    delays = default_delays,
    CF::CFName = onevision_cf,
    has_obstacle = true,
    use_bang_bang = true,
    )where {X, Z, U}
    N::Int64 = fleet_size
    #delays = DelayModel(obs = 100, act = 1, com = 6, ΔT = 5)

    #t_end = 𝕋(ceil(time_end * freq)) # integer value 
    delta_t = 1 / freq
    #times::Vector{𝕋} = collect(1:t_end) # convert to array
    #idx_to_time(xs) = (xs .- 1) .* delta_t # convert integer index to discrete timestep. -1 is to ensure start with 0
    #rng = MersenneTwister(seed) # rng to generator

    # generate information for each agent
    agent_info(id) = begin
        sys_dy = car_system(delta_t, t -> CarX(0.0, 0.0)) # construct system dynamics
        obs_dy = 
            if id == 1 && has_obstacle; WallObsDynamics(wall_position = 30.0, detector_range = 8.0)
            else WallObsModel() end
        # construct observation dynamics
        sys_dy, obs_dy
    end

    world_dynamics = WorldDynamics([agent_info(i) for i in 1:N]) # construct world dynamics (state, obs) for each car
    ΔT = delays.ΔT # DelayModel 
    world_model = WorldDynamics(
        fill((car_system(delta_t), WallObsModel()), N)) # knowledge of the world

    H = 20 # number of horizen
    central = LeaderFollowerControl(
        warm_up_time = delays.act, 
        bang_bang_tol = use_bang_bang ? 1.0 : Inf) # initialize control of the robot
    x_weights = SVector{N}(fill(CarX(10.0, 1.0), N)) # state weight (hyperparam)
    u_weights = SVector{N}(fill(CarU(0.1), N)) # actuation weight (hyperparam)
    loss_model = RegretLossModel(central, world_model, x_weights, u_weights) # regression loss
    framework = mk_cf(
        CF, world_model, central, delays, loss_model; 
        X = CarX{ℝ}, Z = CarZ{ℝ}, H) # construct a controller framework - bundle of information needed for simulation

    return framework, world_dynamics
end




end# Car1DExampleLabeled
