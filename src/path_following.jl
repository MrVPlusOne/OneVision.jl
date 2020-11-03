module PathFollowing
export PathFollowingProblem, follow_path

using OneVision: ℝ, 𝕋
using OneVision
using JuMP
using OSQP
using MathOptInterface: AbstractOptimizer
using Random, LinearAlgebra
using StaticArrays

const SVec = StaticVector
const SMat = StaticMatrix

struct PathFollowingProblem{n_x,n_u,H}
    dy::SysDynamicsLinear
    x_weights::StaticVector{n_x}
    u_weights::StaticVector{n_u}
    optimizer::AbstractOptimizer
end

function follow_path(
    p::PathFollowingProblem{n_x,n_u,H}, 
    x0::SVec{n_x},
    x_path::SMat{n_x,H}, 
    u_path::SMat{n_u,H},
    τ::𝕋
) where {n_x,n_u,H}
    model = Model(p.optimizer)
    @variables model begin
        x[1:n_x, τ:τ + H]
        u[1:n_u, τ:τ + H - 1]
    end

    # initial conditions
    @constraint model x[:,τ] .== x0
    
    A(t) = convert(SMat{n_x,n_x}, sys_A(p.dy, t))
    B(t) = convert(SMat{n_x,n_u}, sys_B(p.dy, t))
    w(t) = convert(SMat{n_x,1}, sys_w(p.dy, t))
    # dynamics constraints
    @constraint(model, dy_cons[t=τ:τ + H - 1], 
        A(t) * x[:,t] + B(t) * u[:,t] + w(t) == x[:,t + 1]
    )

    @objective(model, Min, 
        sum((x - x_path).^2 .* p.x_weights) + sum((u - u_path).^2 * p.u_weights)
    )

    optimize!(model)
    
    value(u), value(x), objective_value(model)
end

end # module

using .PathFollowing