export send_and_get_states, init_socket, start_framework
using JSON
using StaticArrays

function get_states(conn, x_old::X, u_old::U, z_old::Z, msgs::Each{Msgs}, t::Int64) where {X, Z, U, N, Msgs}
    result = readline(conn)
    result_dict :: Dict{String, Any} = JSON.parse(result) 
    println("recieved ", result_dict)

    # convert to state, obs, and msg
    #x::X = parse_state(result_dict)
    #z::Z = parse_obs(result_dict)
    #msgs::Each{Msg} = parse_msg(result_dict)
    #return x, z, msgs
end
"""
Simulation purpose only:

Send the current known states and actuation for ROS to execute. Block untill a result
    
Due to communication need - now
"""
function send_and_get_states(conn, x_old::X, u_old::U, z_old::Z, msgs::Each{Msg}, t::Int64, f_state, f_obs, f_msg) where {X, Z, U, N, Msg}
    s = JSON.json(Dict("states"=> x_old, "actuation" => u_old, "time" => t, "observation" => z_old, "msgs" => [serialize_to_b_array(m) for m in msgs] )) * "\n"
    #println("sending $s")
    write(conn, s)
    result = readline(conn)
    result_dict :: Dict{String, Any} = JSON.parse(result) 
    #println("recieved ", result_dict)

    # convert to state, obs, and msg
    x::X = f_state(result_dict)
    z::Z = f_obs(result_dict)
    msgs::Each{Msg} = f_msg(result_dict)
    return x, z, msgs, t

    """
    # state: StaticArrays.MArray{Tuple{2},OneVision.Examples.Car1DExample.CarX{Float64},1,2}
    # obs: StaticArrays.MArray{Tuple{2},OneVision.Examples.Car1DExample.CarZ{Float64},1,2}
    # actuation: StaticArrays.MArray{Tuple{2},OneVision.Examples.Car1DExample.CarU{Float64},1,2}
    
    xs1::MArray{Tuple{N}, X} = result_dict["xs"]
    zs1::MArray{Tuple{N}, Z} = parse_obs(result_dict)#[h_vec_from_dict(i) for i in result_dict["zs"]]
    return xs1, zs1
    """

end
"""
takes in a port number and initialize a socket client
"""
function init_socket(port_number::Integer) :: TCPSocket
    return connect(port_number)
end

"""
This function starts the distributed process.
"""
function start_framework(world_dynamics::WorldDynamics{N},  framework::ControllerFramework{X,Z,U,Msg,Log}, 
    init_status::Each{Tuple{X,Z,U}}, car_id::Integer, port_number::Integer, fleet_size::Integer, freq, f_state, f_obs, f_msg) where {X, Z, U, Msg, Log, N}
    t0::𝕋 = 1
    t = t0
    dt = 1.0/freq
    conn = init_socket(port_number)
    # unzip the initial states
    xs, zs, us = @unzip(MVector{N}(init_status), MVector{N}{Tuple{X,Z,U}})
    (controllers, msg_qs) = make_controllers(framework, init_status, t0) # start at time = 1 
    ms_recieved = first(msg_qs[car_id]) # we wont be using the queue impl as delay is not handled here 
    x, z, u = xs[car_id], zs[car_id], us[car_id]

    # start the looping process
    while true
        # start controlling
        u, ms_new = control!(controllers[car_id], x, z, ms_recieved)
        # limit control TODO: change to c++ 
        u = limit_control(world_dynamics.dynamics[car_id], u, x, t)
        # send and get state
        x, z, ms_recieved, t = send_and_get_states(conn, x, u, z, ms_new, controllers[car_id].τ, f_state, f_obs, f_msg)
        t += dt
    end

    

    # initialize the controller framework
    #framework = mk_cf(CF, world_model, central, delays, loss_model; X, Z, H)


end
