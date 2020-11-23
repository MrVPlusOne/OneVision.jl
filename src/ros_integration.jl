module ROSCar1D
export main

using RobotOS
using PyCall
roslib = pyimport("roslib")
roslib.load_manifest("amrl_msgs")
@rosimport amrl_msgs.msg: AckermannCurvatureDriveMsg
@rosimport nav_msgs.msg: Odometry
rostypegen(@__MODULE__)
#import .amrl_msgs.msg: AckermannCurvatureDriveMsg
#import .nav_msgs.msg: Odometry
using .amrl_msgs.msg: AckermannCurvatureDriveMsg
using .nav_msgs.msg: Odometry
using StaticArrays
using Unrolled
using Random

using OneVision
using OneVision.Car1DExample

global x_cur = CarX(0.0, 0.0) # CarX needs to be immutalble - will find a container to wrap this around.

function odom_callback(msg::Odometry)
    x_cur.pos = sqrt(square(msg.position.position.x) + square(msg.position.position.y))
    x_cur.velocity = 2.0 * atan(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
end

function ros_publish_actuation(pub::Publisher, act::CarU, state::CarX)
    # interperate 
    next_vel = state.velocity + act.acc*0.05
    msg = AckermannCurvatureDriveMsg()
    msg.velocity = next_vel
    publish(pub, msg)
    publish(pub, msg)
    publish(pub, msg)
end


"The simple wall dynamics model used by OneVision."
struct WallObsModel <: ObsDynamics end

OneVision.obs_forward(dy::WallObsModel, x::CarX, z::CarZ, t::𝕋) = z

function make_ros_controllers(times, freq::ℝ; noise=0.0, plot_result=false, log_prediction=false, N=1)
    delta_t = 1 / freq
    times::Vector{𝕋} = collect(times)
    idx_to_time(xs) = (xs .- 1) .* delta_t
    t0, t_end = times[1], times[end]
    rng = MersenneTwister(1234)
    
    agent_info(id) = begin
        acc_noise = randn(rng, ℝ, 1 + t_end - t0) * noise
        sys_dy = car_system(delta_t, t -> CarX(0.0, acc_noise[1 + t - t0]))
        obs_dy = 
            if id == 1; WallObsDynamics(wall_position=30.0, detector_range=8.0)
            else WallObsModel() end
        
        sys_dy, obs_dy
    end
    
    world_dynamics = WorldDynamics([agent_info(i) for i in 1:N])
    init_states = fill((CarX(0.0, 0.0), CarZ(0.0, 0.0), CarU(0.0)), N)
    delays = DelayModel(obs=2, act=3, com=4)  # DelayModel(obs=1, act=2, com=5)
    world_model = WorldDynamics(
        fill((car_system(delta_t), WallObsModel()), N))

    comps = ["pos", "velocity", "acceleration", "obstacle"]
    function record_f(xs, zs, us)
        [(x -> x.pos).(xs) (x -> x.velocity).(xs) (u -> u.acc).(us) (z -> z.distance).(zs)]
    end
    
    H = 20
    πc = LeaderFollowerControl(warm_up_time=delays.act)


    u_weights = fill(CarU(1.0), N)
    x_weights = fill(CarX(1.0, 1.0), N)
    
    controller_framework = OvCF{N,CarX{ℝ},CarZ{ℝ},CarU{ℝ},H}(
            πc, 
            world_model, delays, x_weights, u_weights,
            FuncT(Tuple{ℕ,𝕋,CarX{ℝ},CarZ{ℝ}}, Bool) do (id, t, _, _) 
                false
            end
        )
    return world_dynamics, delays, controller_framework, init_states, times
end



function loop(pub::Publisher)
    freq = 20.0
    wd, delay_model, cf, init_status, times = make_ros_controllers(1:20, freq)
    (controllers, msg_qs) = make_controllers(cf, init_status, times[1])
    println(controllers)
    make_agent(id::ℕ)::AgentState = begin
        (x₀, z₀, u₀) = init_status[id]
        AgentState(
            state_queue=constant_queue(x₀, delay_model.obs),
            obs_queue=constant_queue(z₀, delay_model.obs),
            act_queue=constant_queue(u₀, delay_model.act),
            msg_queue=msg_qs[id],
            controller=controllers[id],
        )
    end
    N = 1
    agents = map(make_agent, SVector{N}(1:N))
    init_states = SVector{N}(init_status)
    #a::SVector{N,Tuple{CarX,CarZ,CarU}} = init_states
    loop_rate = Rate(freq)
    tmp = first(agents[1].msg_queue)
    transmition = MMatrix{N,N,typeof(tmp)}(undef)  
    #print(a) 
    Each = MVector{N}
    xs, zs, us = @unzip(MVector(init_states), Each{Tuple{CarX{Float64},CarZ{Float64},CarU{Float64}}})
    
    for t in 1:20
        if is_shutdown()
            error("FATAL: ROS node has been shutdown")
            exit(0)
        end
        rossleep(loop_rate)
        for i in 1:length(agents)  # unroll away dynamic dispatch at compile time!
            a = agents[i]
            x = xs[i]
            u = us[i]
            z = zs[i]
            ms = first(a.msg_queue)
            u, ms′ = control!(a.controller, x, z, ms)
            #transmition[:,i] = ms′
            ros_publish_actuation(pub, u, x)
        end

    end
    """
    loop_rate = Rate(20.0)
    for t in 1:20
        if is_shutdown()
            error("FATAL: ROS node has been shutdown")
        end
        rossleep(loop_rate) # handle callbacks here - busy loop. Potentially very inefficient
        # at this point, messages should be handled, - run the simulation framework
        print(xs[0])
        xs[0] = x_cur # assign the current observation

        # observe, control, and send messages
        transmition = MMatrix{N,N,Msg}(undef)  # indexed by (receiver, sender)
        for i in 1:length(agents)  # unroll away dynamic dispatch at compile time!
            a = agents[i]
            x = pushpop!(a.state_queue, xs[i])
            z = pushpop!(a.obs_queue, zs[i])
            ms = first(a.msg_queue)
            u, ms′ = control!(a.controller, x, z, ms)
            us[i] = pushpop!(a.act_queue, u)
            transmition[:,i] = ms′
        end
        # receive messagees and update world states
        for i in 1:length(agents)
            a = agents[i]
            pushpop!(a.msg_queue, convert(Vector{Msg}, transmition[i,:]))
            ros_publish_actuation(pub, act, cur_state) # needs to be obtained from ROS 
            zs[i] = zs[i] # do not update the wall pos for now #obs_forward(world_dynamics.obs_dynamics[i], xs[i], zs[i], t)
        end
    end
    """
    print("finished loop")
end

function main()
    # ROS related initialization
    init_node("rosjl_example")
    pub = Publisher{AckermannCurvatureDriveMsg}("ackermann_curvature_drive", queue_size=1)
    sub = Subscriber{Odometry}("odom", odom_callback,  queue_size=10)
    # simulation initialization
    loop(pub)
end
end
