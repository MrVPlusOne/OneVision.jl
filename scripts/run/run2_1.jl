

a = ["--car_id=1", "--fleet_size=2", "--freq=50", "--preheat=true"]
b = ["--car_id=1", "--fleet_size=2", "--freq=50", "--preheat=false"]

empty!(ARGS)
append!(ARGS, a)