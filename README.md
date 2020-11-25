# OneVision

## Installation
Note that the instruction below are prelimary and are intended to be used as internal reference only.

### Download and install julia
#### Linux ARM
```wget https://julialang-s3.julialang.org/bin/linux/aarch64/1.5/julia-1.5.3-linux-aarch64.tar.gz
tar zxvf julia-1.5.3-linux-aarch64.tar.gz
mv julia-788b2c77c1/ julia-153
sudo mv julia-153 /opt/
```

and then add the linking to user profile:
```
export PATH="$PATH:/opt/julia-153/bin"
```
### Reproduce
This code base is using the Julia Language and [DrWatson](https://juliadynamics.github.io/DrWatson.jl/stable/)
to make a reproducible scientific project named
> OneVision

It is authored by Jiayi Wei, Tongrui Li.

To (locally) reproduce this project, do the following:

0. Download this code base. Notice that raw data are typically not included in the
   git-history and may need to be downloaded independently.
1. Open a Julia console and do:
   ```
   julia> using Pkg
   julia> Pkg.activate("path/to/this/project")
   julia> Pkg.instantiate()
   ```

This will install all necessary packages for you to be able to run the scripts and
everything should work out of the box.

### Installing necessary dependencies
#### arm modification (jetson tx2 specific)
Due to package compatibiliies, some package will have to be installed in an alternative manner. Note that the following are only validated on Jetson TX2 with 18.04 - your milage may vary. 

OSQP:
```
] add https://github.com/TongruiLi/OSQP.jl
```



#### Installation
In the top directory of the cloned repo, press ] and then do 
```
activate .
instantiate
```
### running

```
julia --sysimage <path to image>
```

# Quickstart
## Convension
In correspondance to the paper, without further context the following references applies:
- X: robot state
- Z: robot observation
- U: robot actuation


## Usage
The main framework depends on the control! function, which has the following signature:
```
function OneVision.control!(
    π::OvController{N,X,Z,U,H},
    x::X,
    z::Z,
    msgs::Each{OvMsg{X,Z}},
)::Tuple{U,Each{OvMsg{X,Z}}} where {N,X,Z,U,H}
```

x and z are the current state and observation, and msgs are the messages recieved from other robot

### Making a OneVision Controller
We provide the following function to make a OneVision controller:
```
function OneVision.make_controllers(
    cf::OvCF{N,X,Z,U,H},
    init_status::Each{Tuple{X,Z,U}},
    t0::𝕋,
)::Tuple where {N,X,Z,U,H}
```

To make a controller framework, the following needs to be defined
#### Central Control (πc, OneVision.control_one)
Two steps needs to happen:
- A struct that extends the CentralControl class needs to be defined. It must contain all relavant information that is needed for a centralized controller to execute. It alows needs to be parameterized by U.
- control_one: A function that takes the struct, current X and Z, time, and carID needs to utilize all information available and outputs an approporate actuation


####  World Dynamics
Takes in a tuple of system and observational dynamics:
- system dynamics - given current actuation and state, predict actuation at next timestep
- observational dynamic - given current state and obs, predict next obs

#### Delay
DelayModel, takes in an observation, actuation, and comm delay.

#### importance weight
Only used for linear systems - defins the associated weight for each given part

#### save log
A function that determines whether a log should be saved given the current car, time, state, and obs. 

Initial state can just be made from a tuple of associated initial car state, obs, and actuation.






