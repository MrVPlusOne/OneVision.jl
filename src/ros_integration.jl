using RobotOS
using PyCall
roslib = pyimport("roslib")
roslib.load_manifest("amrl_msgs")
@rosimport amrl_msgs.msg: AckermannCurvatureDriveMsg
@rosimport nav_msgs.msg: Odometry
rostypegen(@__MODULE__)
import .amrl_msgs.msg: AckermannCurvatureDriveMsg
import .nav_msgs.msg: Odometry
using Plots: Plot, plot
using StaticArrays
using Unrolled


global x_cur: CarX = CarX(0.0, 0.0)# sadly there is no way around a global variable as of now to pass information from callback

function odom_callback(msg::Odometry)
    x_cur.pos = sqrt(square(msg.position.position.x) + square(msg.position.position.y))
    x_cur.velocity = 2.0 * atan(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
end

function ros_publish_actuation(pub::Publisher, act::CarU, state::CarX)
    # interperate 
    next_vel = state.velocity + act.acceleration*0.05
    msg = AckermannCurvatureDriveMsg()
    msg.velocity = next_vel
    publish(pub, msg)
end

function loop(pub::Publisher
    )
    N = 2
    world_dynamics = WorldDynamics([agent_info(i) for i in 1:N])
    init_states = fill((CarX(0.0, 0.0), CarZ(0.0, 0.0), CarU(0.0)), N)
    delays = DelayModel(obs=2, act=3, com=4)  # DelayModel(obs=1, act=2, com=5)
    world_model = WorldDynamics(
        fill((car_system(delta_t), WallObsModel()), N))
    xs, zs, us = @unzip(MVector(init_states), Each{Tuple{X,Z,U}}

    assert(length(agents) == 1)

    #loop_rate = Rate(20.0)
    for t in times[1]:times[end] 
        if is_shutdown()
            error("FATAL: ROS node has been shutdown")
        end
        rossleep(loop_rate) # handle callbacks here - busy loop. Potentially very inefficient
        # at this point, messages should be handled, - run the simulation framework
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
            xs[i] = ros_publish_actuation(pub, act, cur_state) # needs to be obtained from ROS 
            zs[i] = obs_forward(world_dynamics.obs_dynamics[i], xs[i], zs[i], t)
        end
    end
    print("finished loop")
end

function main()
    # ROS related initialization
    init_node("rosjl_example")
    pub = Publisher{AckermannCurvatureDriveMsg}("robot_1/ackermann_curvature_drive", queue_size=1)
    sub = Subscriber{Odometry}("robot_1/odom", odom_callback, queue_size=10)
    # simulation initialization
    
    loop(pub)
end

if ! isinteractive()
    main()
else
    print("Warning - running in intreactive mode - this may result in unexpected behavior, use with caution")
    main()
end
