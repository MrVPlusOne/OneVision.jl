module PathFollowing
export PathFollowingProblem, follow_path

using OneVision: ℝ, 𝕋
using OneVision
using JuMP
import OSQP
# import COSMO
using MathOptInterface: AbstractOptimizer
using Random, LinearAlgebra
using StaticArrays
import MutableArithmetics

struct PathFollowingProblem{H,n_x,n_u,Dy <: SysDynamicsLinear}
    horizon::Val{H}
    dy::Dy
    x_weights::StaticVector{n_x,ℝ}
    u_weights::StaticVector{n_u,ℝ}
    cache::Ref{Any}
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
    p::PathFollowingProblem{H,n_x,n_u},
    x0::SVector{n_x},
    x_path::SMatrix{n_x,H},
    u_path::SMatrix{n_u,H},
    τ::𝕋
)::Tuple where {n_x,n_u,H}
    local x, u, Min

    model = Model(() -> OSQP.Optimizer())
    # model = Model(() -> COSMO.Optimizer())
    set_silent(model)

    # set_optimizer_attribute(model, "warm_start", true)

    @variables model begin
        x[1:n_x, 1:H + 1]
        u[1:n_u, 1:H]
    end

    # initial conditions
    @constraint model x[:,1] .== x0

    A(t)::SMatrix{n_x,n_x,ℝ} = sys_A(p.dy, τ + t - 1)
    B(t)::SMatrix{n_x,n_u,ℝ} = sys_B(p.dy, τ + t - 1)
    w(t)::SMatrix{n_x,1,ℝ} = sys_w(p.dy, τ + t - 1)

    # dynamics constraints
    @constraint(model, [t = 1:H],
        A(t) * x[:,t] .+ B(t) * u[:,t] .+ w(t) .== x[:,t + 1]
    )

    @objective(model, Min,
        sum((x[:,2:end] - x_path).^2 .* p.x_weights) + sum((u - u_path).^2 .* p.u_weights)
    )

    use_warmstart = false
    if use_warmstart && !ismissing(p.cache[])
        x0, u0 = p.cache[]
        set_start_value.(x[1:n_x, 2:H], x0)
        set_start_value.(u[1:n_u, 1:H-1], u0)
    end

    optimize!(model)

    if use_warmstart
        p.cache[] = value.(x[1:n_x, 3:H + 1]), value.(u[1:n_u, 2:H])
    end
        
    @assert (obj = objective_value(model)) ≥ -sqrt(eps()) "negative objective: $(obj)"
    value.(u), value.(x[:, 2:end]), obj
end

end # module

@reexport using .PathFollowing
