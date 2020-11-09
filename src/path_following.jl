module PathFollowing
export PathFollowingProblem, follow_path

using OneVision: ℝ, 𝕋
using OneVision
using JuMP
using OSQP
using MathOptInterface: AbstractOptimizer
using Random, LinearAlgebra
using StaticArrays
import MutableArithmetics

struct PathFollowingProblem{n_x,n_u,H,Dy <: SysDynamicsLinear}
    horizon::Val{H}
    dy::Dy
    x_weights::StaticVector{n_x,ℝ}
    u_weights::StaticVector{n_u,ℝ}
    make_optimizer
end

function MutableArithmetics.undef_array(::Type{Array{T,N}}, ::StaticArrays.SOneTo{M}) where {T,N,M}
    return Array{T,N}(undef, M)
end

"""
Returns `(u⋆[t in τ: τ+H-1], x⋆[t in τ+1: τ+H], objective_value)`.

# Arguments
 - `x0[τ]`: state at time τ
 - `x_path[t in τ+1: τ+H]`: state trajectory ∈ [τ+1, τ+H]
 - `u_path[t in τ: τ+H-1]`: action trajectory ∈ [τ, τ+H-1]
"""
function follow_path(
    p::PathFollowingProblem{n_x,n_u,H},
    x0::SVector{n_x},
    x_path::SMatrix{n_x,H},
    u_path::SMatrix{n_u,H},
    τ::𝕋
)::Tuple where {n_x,n_u,H}
    local x, u, Min

    model = Model(p.make_optimizer)
    @variables model begin
        x[1:n_x, 1:H + 1]
        u[1:n_u, 1:H]
    end

    # initial conditions
    @constraint model x[:,1] .== x0

    A(t)::SMatrix{n_x,n_x,ℝ} = sys_A(p.dy, τ + t - 1)
    B(t)::SMatrix{n_x,n_u,ℝ} = sys_B(p.dy, τ + t - 1)
    w(t)::SMatrix{n_x,1,ℝ} = sys_w(p.dy,  τ + t - 1)

    # dynamics constraints
    @constraint(model, [t = 1:H],
        A(t) * x[:,t] .+ B(t) * u[:,t] .+ w(t) .== x[:,t + 1]
    )

    @objective(model, Min,
        sum((x[:,2:end] - x_path).^2 .* p.x_weights) + sum((u - u_path).^2 .* p.u_weights)
    )

    optimize!(model)

    value.(u), value.(x[:, 1:H]), objective_value(model)
end

end # module

@reexport using .PathFollowing
