using StaticArrays

export ForwardPredictProblem, forward_predict

struct ForwardPredictProblem{N,H,U,Dy <: WorldDynamics{N},Ctrl <: CentralControl{U}}
    horizon::Val{H}
    world_dynamics::Dy
    π::Ctrl
end


"""
Forward-predict the ideal fleet trajectory. 
Returns `(ũ[t in τ: τ+H-1], x̃[t in τ+1: τ+H])`

# Argument Details
 - `x0[τ]`: state at time τ
 - `x_path[t in τ+1: τ+H]`: state trajectory ∈ [τ+1, τ+H]
 - `u_path[t in τ: τ+H-1]`: action trajectory ∈ [τ, τ+H-1]
"""
function forward_predict(
    prob::ForwardPredictProblem{N,H,U},
    δx::MMatrix{H,N,X},
    δz::MMatrix{H,N,Z},
    init_state::Each{Tuple{X,Z}},
    τ0::𝕋,
) where {X,Z,U,N,H}
    x_traj = MMatrix{H,N,X}(undef)
    u_traj = MMatrix{H,N,U}(undef)

    xs = MVector{N,X}(first.(init_state))
    zs = MVector{N,Z}(last.(init_state))
    x_dy = prob.world_dynamics.dynamics
    z_dy = prob.world_dynamics.obs_dynamics

    for t in 1:H
        us = control_all(prob.π, xs, zs, τ0 + t - 1, 1:N)
        @inbounds for i in 1:N
            xs[i] = sys_forward(x_dy[i], xs[i], us[i], N) + δx[t, i]
            zs[i] = obs_forward(z_dy[i], xs[i], zs[i], N) + δz[t, i]
        end
        x_traj[t,:] = xs
    end
    u_traj, x_traj
end
