wget https://julialang-s3.julialang.org/bin/linux/aarch64/1.5/julia-1.5.3-linux-aarch64.tar.gz
tar -xvzf julia-1.5.3-linux-aarch64.tar.gz
sudo mv julia-1.5.3/ /opt/
sudo ln -s /opt/julia-1.5.3/bin/julia /usr/local/bin/julia
