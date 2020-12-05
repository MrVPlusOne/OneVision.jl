module TrajPlanning

export TrajPlanningProblem, plan_trajectory, x_path_from_u

using OneVision
using OneVision: ℝ, ℕ, 𝕋, @kwdef
using StaticArrays
using Optim

struct TrajPlanningProblem{H,ΔT,X,U,Dy <: SysDynamics}
    horizon::Val{H}
    control_inverval::Val{ΔT}
    dy::Dy
    x_weights::X
    u_weights::U
    optim_options::Optim.Options
    cache::Ref{Any}
end

TrajPlanningProblem(H::ℕ, ΔT::𝕋, dy, x_weights, u_weights; 
    optim_options = Optim.Options(iterations = 100)) = 
    TrajPlanningProblem(Val(H), Val(ΔT), dy, x_weights, u_weights, 
        optim_options, Ref{Any}(missing))


"""
Returns `(u⋆, objective_value)` where `u⋆[1] = u⋆(τ)`, `length(u⋆) = H`.

# Arguments
    - `x0[τ]`: state at time τ
    - `x_path[t in τ+1: τ+HΔT]`: state trajectory ∈ [τ+1, τ+HΔT]
    - `u_path[t in τ: τ+HΔT-1]`: action trajectory ∈ [τ, τ+HΔT-1]
"""
function plan_trajectory(
    p::TrajPlanningProblem{H,ΔT,X,U,Dy},
    x0::X,
    x_path::SVector{H_ΔT,X},
    u_path::SVector{H_ΔT,U},
    τ::𝕋
)::Tuple where {H,ΔT,X,U,Dy,H_ΔT}
    @assert H_ΔT == H * ΔT

    function loss(uvec::AbstractVector{R})::R where R
        n_u = length(U)
        us::SMatrix{n_u,H,R} = reshape(uvec, (n_u, H))
        XR, UR = similar_type(X, R), similar_type(U, R)
        x::XR = x0
        l_x = zero(XR)
        l_u = zero(UR)

        for j = 0:H - 1
            u::UR = us[:,j + 1]
            u = limit_control(p.dy, u, x, τ + j * ΔT)
            for k = 0:ΔT - 1
                t = j * ΔT + k
                x = sys_forward(p.dy, x, u, τ + t)
                l_x += (x .- x_path[t + 1]).^2 
                l_u += (us[:,j + 1] .- u_path[t + 1]).^2
            end
        end
        sum(l_x .* p.x_weights) + sum(l_u .* p.u_weights)
    end

    use_warmstart = true

    u0::Vector{ℝ} = (use_warmstart && !ismissing(p.cache[])) ? 
                    p.cache[] : zeros(length(U) * H)
    
    res = Optim.optimize(
        loss,
        u0, Optim.LBFGS(),
        p.optim_options,
        autodiff = :forward,
    )
    @assert Optim.converged(res) "Optim not converged: $res"

    uvec = Optim.minimizer(res)
    n_u = length(U)
    # û = @SVector[U(uvec[1+n_u*i:n_u*(i+1)]) for i in 1:H]
    û = SVector{H}([U(uvec[i:i+n_u-1]...) for i in 1:n_u:H*n_u])

    if use_warmstart
        uvec[1:end - n_u] = uvec[1 + n_u:end]
        p.cache[] = uvec
    end

    @assert (obj = Optim.minimum(res)) ≥ -sqrt(eps()) "negative objective: $(obj)"
    û, obj
end

function x_path_from_u(x0::X, t0::𝕋, û::SVector{H,U}, dy, ::Val{ΔT}) where {X,U,H,ΔT}
    H_ΔT = H * ΔT
    x̂ = MVector{H_ΔT,X}(undef)
    x::X = x0
    for i = 0:H - 1
        for j = 0:ΔT - 1
            t = t0 + i * ΔT + j
            x = sys_forward(dy, x, û[i + 1], t)
            x̂[1 + i * ΔT + j] = x
        end
    end
    x̂
end


end # module TrajPlanning

@reexport using .TrajPlanning