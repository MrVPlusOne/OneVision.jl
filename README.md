# OneVision: Centralized  to  Distributed  Controller  Synthesis

This repo contains the code for the paper [OneVision: Centralized to Distributed Controller Synthesis with Delay Compensation](https://arxiv.org/abs/2104.06588). 

Supplementary video can be watched [here on Youtube](https://youtu.be/xY4o7f2dp4M).


## Reproduce
This code base is using the Julia Language and [DrWatson](https://juliadynamics.github.io/DrWatson.jl/stable/)
to make a reproducible scientific project.

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


### Generate System Image (Optional)
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






