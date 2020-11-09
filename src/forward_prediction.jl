using StaticArrays

export ForwardPredictProblem, forward_predict, self_estimate

struct ForwardPredictProblem{N,H,X,Z,U,Dy,Ctrl}
    world_dynamics::Dy
    π::Ctrl
    δx::MMatrix{H,N,X}
    δz::MMatrix{H,N,Z}

    function ForwardPredictProblem(
        world_dynamics::Dy, π::Ctrl; H, X, Z,
    ) where {N,U, Dy <: WorldDynamics{N},Ctrl <: CentralControl{U}}
        new{N,H,X,Z,U,Dy,Ctrl}(
            world_dynamics, π, 
            MMatrix{H,N,X}(undef) ,MMatrix{H,N,Z}(undef))
    end
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
    prob::ForwardPredictProblem{N,H,X,Z,U},
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
            xs[i] = sys_forward(x_dy[i], xs[i], us[i], N) + prob.δx[t, i]
            zs[i] = obs_forward(z_dy[i], xs[i], zs[i], N) + prob.δz[t, i]
        end
        x_traj[t,:] = xs
    end
u_traj, x_traj
end

"""
Estimate the current state from an old state and the actuation history since then.
"""
function self_estimate(
    self_dynamics::SysDynamics,
    s0::Tuple{X,𝕋},
    u_history,
)::X where {X}
    x, τ = s0
    for (t, u) in enumerate(u_history)
        x = sys_forward(self_dynamics, x, u, τ + t - 1)
    end
    x
end
