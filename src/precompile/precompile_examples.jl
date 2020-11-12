# This file is used as an example workload to generate a local system image for fast loading time

include("pkg_list.jl")
for p in pkg_list
    println("loading $(string(p))")
    @time eval(:(using $p))
end

using LinearAlgebra
using JuMP, OSQP
@elapsed let
    local x, Min
    ℝ = Float64
    model = Model(OSQP.Optimizer)
    n_x = 4
    H = 20
    
    x_weights = [0,1,2,3]
    @variables model begin
        x[1:n_x, 1:H + 1]
    end

    # initial conditions
    @constraint model x[:,1] .== [1,2,3,4]
    
    A(t) = randn(n_x, n_x)
    w(t) = ones(ℝ, n_x, 1)
    
    # dynamics constraints
    @constraint(model, [t = 1:H], 
        A(t) * x[:,t] .+ w(t) .== x[:,t + 1]
    )

    @objective(model, Min, 
        sum((x[:,2:end] .- 1.0).^2 .* x_weights)
    )

    optimize!(model)
    
    value.(x[:, 1:H]), objective_value(model)
end

using Plots
@elapsed gr()
@elapsed (p = plot(rand(5), rand(5)); display(p))
@gif for i in 1:5
    plot(rand(5), rand(5))
    plot!(rand(5), rand(5))
end
@elapsed plotlyjs()
@elapsed (p = plot(rand(5), rand(5)); display(p))
@elapsed @MArray [1 5;6 3]