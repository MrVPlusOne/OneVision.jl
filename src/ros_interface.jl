export send_and_get_states
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
    print(s)
    write(conn, s)
    result = readline(conn)
    result_dict :: Dict{String, Any} = JSON.parse(result) 
    # state: StaticArrays.MArray{Tuple{2},OneVision.Examples.Car1DExample.CarX{Float64},1,2}
    # obs: StaticArrays.MArray{Tuple{2},OneVision.Examples.Car1DExample.CarZ{Float64},1,2}
    # actuation: StaticArrays.MArray{Tuple{2},OneVision.Examples.Car1DExample.CarU{Float64},1,2}
    print(typeof(result_dict))
    xs1::MArray{Tuple{N}, X} = result_dict["xs"]
    zs1::MArray{Tuple{N}, X} = result_dict["zs"]
    return xs1, zs1
end

