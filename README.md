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
