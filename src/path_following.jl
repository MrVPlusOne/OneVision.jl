module PathFollowing
export PathFollowingProblem, follow_path_optim, x_path_from_u

using OneVision: ℝ, 𝕋, @kwdef
using OneVision
using Random, LinearAlgebra
using StaticArrays
import Optim


@kwdef struct PathFollowingProblem{H,n_x,n_u,Dy <: SysDynamics}
    horizon::Val{H}
    dy::Dy
    x_weights::SVector{n_x,ℝ}
    u_weights::SVector{n_u,ℝ}
    cache::Ref{Any}
    u_tol::ℝ = 0.0
    loss_tol::ℝ = 0.0
    loss_rel_tol::ℝ = 0.0
end

PathFollowingProblem(H::𝕋, dy, x_weights, u_weights; 
    u_tol = 0.0, loss_tol = 0.0, loss_rel_tol = 0.0) =
    PathFollowingProblem(
        Val(H), dy, x_weights, u_weights, Ref{Any}(missing), u_tol, loss_tol, loss_rel_tol)

function x_path_from_u(x0::SVector{n_x,ℝ}, t0::𝕋, û::SMatrix{n_u,H,ℝ}, dy) where {n_x,n_u,H}
    x̂ = MMatrix{n_x,H,ℝ}(undef)
    x::SVector{n_x,ℝ} = x0
    for t = 1:H
        x = sys_forward(dy, x, û[:,t], t0 + t - 1)
        x̂[:,t] = x
    end
    x̂
end

"""
Returns `(u⋆[t in τ: τ+H-1], objective_value)`.

# Arguments
 - `x0[τ]`: state at time τ
 - `x_path[t in τ+1: τ+H]`: state trajectory ∈ [τ+1, τ+H]
 - `u_path[t in τ: τ+H-1]`: action trajectory ∈ [τ, τ+H-1]
"""
function follow_path_optim(
    p::PathFollowingProblem{H,n_x,n_u,Dy},
    x0::SVector{n_x},
    x_path::SMatrix{n_x,H},
    u_path::SMatrix{n_u,H},
    τ::𝕋
)::Tuple where {n_x,n_u,H,Dy}
    n_u_H = n_u * H

    function loss(uvec)
        R = eltype(uvec)
        u::SMatrix{n_u,H,R} = reshape(uvec, (n_u, H))
        x::SVector{n_x,R} = x0
        l_x = @SVector zeros(R, n_x)
        l_u = @SVector zeros(R, n_u)

        for t = 1:H
            x = sys_forward(p.dy, x, u[:,t], τ + t - 1)
            l_x += (x .- x_path[:, t]).^2 
            l_u += (u[:,t] .- u_path[:,t]).^2 
        end
        sum(l_x .* p.x_weights) + sum(l_u .* p.u_weights)
    end

    use_warmstart = true

    u0::Vector{ℝ} = (use_warmstart && !ismissing(p.cache[])) ? p.cache[] : zeros(n_u_H)
    
    res = Optim.optimize(
        loss,
        u0, Optim.LBFGS(),
        Optim.Options(
            x_abstol = p.u_tol,
            f_abstol = p.loss_tol,
            f_reltol = p.loss_rel_tol,
            iterations = 100,
        ),
        autodiff = :forward,
    )
    @assert Optim.converged(res) "Optim not converged: $res"

    uvec = Optim.minimizer(res)
    û::SMatrix{n_u,H,ℝ} = reshape(uvec, (n_u, H))

    if use_warmstart
        uvec[1:end - n_u] = uvec[1 + n_u:end]
        p.cache[] = uvec
    end

    @assert (obj = Optim.minimum(res)) ≥ -sqrt(eps()) "negative objective: $(obj)"
    û, obj
end


end # module

@reexport using .PathFollowing
