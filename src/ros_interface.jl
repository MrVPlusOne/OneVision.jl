export send_and_get_states, init_socket
using JSON
using StaticArrays

"""
Simulation purpose only:

Send the current known states and actuation for ROS to execute. Block untill a result
    
Due to communication need - now
"""
function send_and_get_states(conn, xs::MArray{Tuple{N}, X}, us::MArray{Tuple{N}, U}, zs::StaticArrays.MArray{Tuple{N}, Z}, t::Int64) where {X, Z, U, N}
    s = JSON.json(Dict("states"=> xs, "actuation" => us, "time" => t, "observation" => zs)) * "\n"
    # print(s)
    println("sending $s")
    write(conn, s)
    result = readline(conn)
    result_dict :: Dict{String, Any} = JSON.parse(result) 
    # state: StaticArrays.MArray{Tuple{2},OneVision.Examples.Car1DExample.CarX{Float64},1,2}
    # obs: StaticArrays.MArray{Tuple{2},OneVision.Examples.Car1DExample.CarZ{Float64},1,2}
    # actuation: StaticArrays.MArray{Tuple{2},OneVision.Examples.Car1DExample.CarU{Float64},1,2}
    println("recieved ", result_dict)
    
    xs1::MArray{Tuple{N}, X} = result_dict["xs"]
    zs1::MArray{Tuple{N}, Z} = parse_obs(result_dict)#[h_vec_from_dict(i) for i in result_dict["zs"]]
    return xs1, zs1
end
function init_socket(port_number::Integer) :: TCPSocket
    return connect(port_number)
end

"""
This function starts the distributed process.
function start_framework(framework::ControllerFramework{X,Z,U,Msg,Log}, 
    init_status::Each{Tuple{X,Z,U}}, socket::TCPSocket, car_id::Integer) 
    where{X, Z, U, Msg, Log}
    # unzip the initial states
    xs, zs, us = @unzip(MVector{N}(init_status), MVector{N}{Tuple{X,Z,U}})
    (controllers, msg_qs) = make_controllers(framework, init_status, t0) 
    ms = first(msg_qs[car_id]) # we wont be using the queue impl as delay is not handled here 
    x, z, u = xs[car_id], zs[car_id], us[car_id]

    # start the looping process
    while true
        # start controlling
        u, ms_new = control!(controllers[car_id], x, z, ms)
        # send 
    end

    

    # initialize the controller framework
    #framework = mk_cf(CF, world_model, central, delays, loss_model; X, Z, H)


end
"""
