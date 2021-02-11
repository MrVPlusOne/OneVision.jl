using DrWatson
@quickactivate "OneVision"
using ArgParse
using OneVision.ROS_SIM_RUNNER

function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table s begin
        "--car_id"
            help = "current id of the car"
            arg_type = Int32
            required = true
        "--fleet_size"
            help = "number of car"
            arg_type = Int32
            required = true
        "--freq"
            help = "frequency of controller"
            arg_type = Int32
            required = true
        "--preheat"
            help = "true to allow ahead of time compilation"
            arg_type = Bool
            required = true
    end

    return parse_args(s)
end

function run()
    parsed_args = parse_commandline()
    println("Parsed args:")
    for (arg,val) in parsed_args
        println("  $arg  =>  $val")
    end
    car_id::Int32 = parsed_args["car_id"]
    fleet_size::Int32 = parsed_args["fleet_size"]
    freq::Int32 = parsed_args["freq"]
    preheat::Bool = parsed_args["preheat"]
    run_open_loop(car_id, fleet_size, freq, preheat)
end

run()


