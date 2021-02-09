export send_and_get_states, init_socket, start_framework
using JSON
using StaticArrays

function get_states(conn, x_old::X, u_old::U, z_old::Z, msgs::Each{Msgs}, t::Int64) where {X,Z,U,N,Msgs}
    result = readline(conn)
    result_dict::Dict{String,Any} = JSON.parse(result) 
    println("recieved ", result_dict)

    # convert to state, obs, and msg
    # x::X = parse_state(result_dict)
    # z::Z = parse_obs(result_dict)
    # msgs::Each{Msg} = parse_msg(result_dict)
    # return x, z, msgs
end
"""
Simulation purpose only:

Send the current known states and actuation for ROS to execute. Block untill a result
    
Due to communication need - now
"""
function send_and_get_states(conn, x_old::X, u_old::U, z_old::Z, msgs::Each{Msg}, t::Int64, f_state, f_obs, f_msg) where {X,Z,U,N,Msg}
    s = JSON.json(Dict("states" => x_old, "actuation" => u_old, "time" => t, "observation" => z_old, "msgs" => [serialize_to_b_array(m) for m in msgs])) * "\n"
    # println("sending $s")
    write(conn, s)
    result = readline(conn)
    result_dict::Dict{String,Any} = JSON.parse(result) 
    # println("recieved ", result_dict)

    # convert to state, obs, and msg
    x::X = f_state(result_dict)
    z::Z = f_obs(result_dict)
    msgs::Pair{Each{ℕ}, Each{Msg}} = f_msg(result_dict)
    t::Float64 = result_dict["time"]
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
function init_socket(port_number::Integer)::TCPSocket
    return connect(port_number)
end

"""
TODO (Tongrui): Create an interface for common cache
"""
function create_cache(msg_qs::AbstractVector{Msg})::Dict{𝕋,Msg} where Msg
    msg_ds::Dict{T,Msg} = Dict()
    for m in msg_qs
        msg_ds[m.δx.time] = m 
    end
    return msg_ds
end

function create_cache_from_vec(msg_qss::FixedQueue{Each{Msg}}, fleet_size::Int64)::Vector{Dict{𝕋,Msg}} where Msg
    msg_ds_arr  = [Dict{𝕋,Msg}() for _ in 1:fleet_size]
    
    for msgs in msg_qss.vec
        # this iterates over timesteps
        # gives a vector of msg where index = car_id
        for (i, m) in enumerate(msgs)
            # this iterates over the car
            println("current msg time is $(m.δx.time)")
            msg_ds_arr[i][m.δx.time] = m
        end
    end
    return msg_ds_arr
end
function add_to_cache!(msg_ds::Dict{𝕋,Msg}, msg::Msg) where Msg
    t = msg.δx.time
    @debug "adding to cache time: $t"
    @assert t<10 || (t>=10 && t ∉ keys(msg_ds))
    msg_ds[t] = msg
end

function remove_from_cache!(msg_ds::Dict{𝕋,Msg}, t::𝕋) where Msg
    @assert t ∈ keys(msg_ds)
    delete!(msg_ds, t)
end

"""
returns the message vector where index = car id
"""
function get_vec_from_cache(msg_ds_arr::AbstractVector{Dict{𝕋,Msg}}, t::𝕋) where Msg
    return [msg_ds[t] for msg_ds in msg_ds_arr]
end

"""
Add a message array with arbitrary timestep and car id to the cache
"""
function add_vec_to_cache!(msg_ds_arr::AbstractVector{Dict{𝕋,Msg}}, msg_id:: AbstractVector{ℕ}, msg_vs::AbstractVector{Msg}) where Msg
    for (from, msg) in zip(msg_id, msg_vs)
        @debug "recieved message from $from"
        add_to_cache!(msg_ds_arr[from], msg)
    end
end

"""
Removes message at the specific time
"""
function remove_vec_from_cache!(msg_ds_arr::AbstractVector{Dict{𝕋,Msg}}, t::𝕋) where Msg
    for msg_ds in msg_ds_arr
        remove_from_cache!(msg_ds, t)
    end
end

"""
This function starts the distributed process.
"""
function start_framework(framework::ControllerFramework{X,Z,U,Msg,Log}, 
    init_status::Each{Tuple{X,Z,U}}, car_id::Integer, port_number::Integer, fleet_size::Integer, freq, f_state, f_obs, f_msg) where {X,Z,U,Msg,Log}
    t0::𝕋 = 1

    dt = 1# 1.0/freq
    N::Int64 = fleet_size
    MsgCache = Dict{𝕋,Msg}
    """
    The cache is a array of timed dictionary of message
    where the index of the array corresponds to the car id
    and t corresponds to the time at which the message is recieved
    """
    # 1. 
    # unzip the initial states
    xs, zs, us = @unzip(MVector{N}(init_status), MVector{N}{Tuple{X,Z,U}})
    (controllers, msg_qs) = make_controllers(framework, init_status, t0) # start at time = 1 
    msg_ds_arr::Vector{MsgCache} = create_cache_from_vec(msg_qs[car_id], convert(Int64, fleet_size))
    x, z, u = xs[car_id], zs[car_id], us[car_id]

    t_msg::𝕋 = maximum( m.δx.time for m in msg_qs[1].vec[1])
    t = t0
    prev_t = t
    t_msg_offset = t0 - t_msg
    # fp = open("car$car_id.csv", )
    # socket comm should initialize last
    conn = init_socket(port_number)
    #println("origional msg queue is $(msg_qs)\n\n current msg cache is $(msg_ds_arr)")
    # start the looping process
    fp_log = open("logs/car$(car_id).csv", "w")
    while true
        controller_t = controllers[car_id].τ
        
        ms_recieved = get_vec_from_cache(msg_ds_arr, t_msg)
        @assert length(ms_recieved) == fleet_size
        # println("[$t], x: $x, z:$z, dt:$dt")
        # start controlling
        u, ms_new = control!(controllers[car_id], x, z, ms_recieved)
        for (i, c) in enumerate(controllers)
            #print("[id:$i] ")
            log_str = "[id:$i] "
            for (k, v) in c.logs
                log_str = log_str * "key: $k, value: $v"
                #@debug "key: $k, value: $v"
            end
            #println()
            write(fp_log, log_str)
        end
        # limit control TODO: change to c++ 
        # u = limit_control(world_dynamics.dynamics[car_id], u, x, t)
        # send and get state
        @debug "[$t], state is $x, action is $u"
        x, z, msg_recieved_pair, t_diff = send_and_get_states(conn, x, u, z, ms_new, t, f_state, f_obs, f_msg)

        add_vec_to_cache!(msg_ds_arr, msg_recieved_pair.first, msg_recieved_pair.second)
        # parse the message
        prev_t = t
        t = t + dt#𝕋(round(t_diff/freq))
        t_msg = t_msg + dt #t + t_msg_offset #+= dt
        @debug "current msg time is $t_msg, offset is $t_msg_offset recieved msg time is $(msg_recieved_pair.second[1].δx.time) new msg time is $(ms_new[1].δx.time), $(ms_new[2].δx.time)"
    end

    

    # initialize the controller framework
    # framework = mk_cf(CF, world_model, central, delays, loss_model; X, Z, H)


end
