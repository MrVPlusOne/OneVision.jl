# OneVision: Centralized  to  Distributed  Controller  Synthesis


## Reproduce
This code base is using the Julia Language and [DrWatson](https://juliadynamics.github.io/DrWatson.jl/stable/)
to make a reproducible scientific project named
> OneVision

It is authored by Jiayi Wei, Tongrui Li, Swarat Chaudhuri, Isil Dillig, and Joydeep Biswas.

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


### Generate System Image
This step can significantly reduce Julia compilation time.

To generate a system image, from command line, run
```
julia scripts/precompile/generate_img.jl
```
It will take several minutes for this step to complete, after which a system image called "JuliaSysimage.dylib" will be created under the project root. To use this image, run all Julia commands with an additional `sysimage` flag like below
```
julia --sysimage <path to image>
```

## Results
Full simulation results can be found [here](papers/SimulationResults.md).

## Usages
*Documentation coming soon.*

### Convension
In correspondance to the paper, without further context the following references applies:
- X: robot state
- Z: robot observation
- U: robot actuation


### Programming Interface (Outdated)
The main framework depends on the control! function, which has the following signature:
```julia
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
```julia
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






