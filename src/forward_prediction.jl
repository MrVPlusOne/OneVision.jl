using StaticArrays

export ForwardPredictProblem, forward_predict, self_estimate

struct ForwardPredictProblem{N,H,X,Z,U,Dy,Ctrl}
    world_dynamics::Dy
    π::Ctrl
    δx::MMatrix{H,N,X}
    δz::MMatrix{H,N,Z}

    """
        ForwardPredictProblem(world_dynamics, π; H, X, Z)
    """
    function ForwardPredictProblem(
        world_dynamics::Dy, π::Ctrl, x_zero::X, z_zero::Z; H,
    ) where {N,U,Dy <: WorldDynamics{N},Ctrl <: CentralControl{U}, X, Z}
        new{N,H,X,Z,U,Dy,Ctrl}(
            world_dynamics, π, 
            MMatrix{H,N,X}(fill(x_zero, H, N)),
            MMatrix{H,N,Z}(fill(z_zero, H, N)))
    end
end


"""
Forward-predict the ideal fleet trajectory.
Returns `(ũ[t in t0: t0+H-1], x̃[t in t0+1: t0+H], z̃[t in t0+1: t0+H])`

# Argument Details
 - `x0[t0]`: state at time t0
 - `prob.δx`: state disturbance ∈ [t0, t0+H-1]
 - `prob.δz`: observation disturbance ∈ [t0, t0+H-1]
"""
function forward_predict(
    prob::ForwardPredictProblem{N,H,X,Z,U},
    init_state::Each{Tuple{X,Z}},
    τ0::𝕋,
) where {X,Z,U,N,H}
    x_traj = MMatrix{H,N,X}(undef)
    z_traj = MMatrix{H,N,Z}(undef)
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
        z_traj[t,:] = zs
        u_traj[t,:] = us
    end
    u_traj, x_traj, z_traj
end


"""
Estimate the self state history using an old state and the actuation history.
Returns `x_history`.
"""
function self_estimate(
    self_dynamics::SysDynamics,
    s0::Tuple{X,𝕋},
    u_history,
)::Vector{X} where {X}
    x, τ = s0
    x_history = Vector{X}(undef, length(u_history))
    for (t, u) in enumerate(u_history)
        x = sys_forward(self_dynamics, x, u, τ + t - 1)
        x_history[t] = x
    end
    x_history
end
