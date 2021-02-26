

a = ["--car_id=2", "--fleet_size=3", "--freq=50", "--preheat=true"]
b = ["--car_id=2", "--fleet_size=3", "--freq=50", "--preheat=false"]

empty!(ARGS)
append!(ARGS, b)