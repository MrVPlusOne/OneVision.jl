using StaticArrays

export ForwardPredictProblem, forward_predict!, self_estimate

struct ForwardPredictProblem{N,Hf,X,Z,U,Dy,Ctrl}
    world_dynamics::Dy
    π::Ctrl
    δx::MMatrix{Hf,N,X}
    z_obs::SizedMatrix{Hf,N,Optional{Z}}

    function ForwardPredictProblem(
        world_dynamics::Dy, π::Ctrl; X, Z, Hf,
    ) where {N,U,Dy <: WorldDynamics{N},Ctrl <: CentralControl{U}}
        @assert isbitstype(X) "X = $X is not of bits type"
        @assert isbitstype(U) "U = $U is not of bits type"
        new{N,Hf,X,Z,U,Dy,Ctrl}(
            world_dynamics, π, 
            MMatrix{Hf,N,X}(fill(zero(X), Hf, N)),
            SizedMatrix{Hf,N,Optional{Z}}(fill(missing, Hf, N)))
    end
end


"""
Forward-predict the ideal fleet trajectory.
Returns `(ũ[t in t0: t0+H-1], x̃[t in t0+1: t0+H], z̃[t in t0+1: t0+H])`.
Note that `init_s` will be modified from `t=t0` to `t=t0+ΔT`. (It will then be deep copied)

# Argument Details
- `prob.δx`: state disturbance ∈ [t0, t0+H-1]
- `prob.z_obs`: observation ∈ [t0, t0+H-1]
- `init_xz`: state `x` and observation `z` at time t0
- `init_s`: central control state at time t0, will be modified to t0+1.
"""
function forward_predict!(
    prob::ForwardPredictProblem{N,Hf,X,Z,U},
    init_xz::Each{Tuple{X,Z}},
    init_s::S,
    τ0::𝕋,
    ΔT::𝕋,
) where {X,Z,U,S,N,Hf}
    x_traj = MMatrix{Hf,N,X}(undef)
    z_traj = SizedMatrix{Hf,N,Z}(undef)
    u_traj = MMatrix{Hf,N,U}(undef)

    xs = MVector{N,X}(first.(init_xz))
    zs = SizedVector{N,Z}(last.(init_xz))
    x_dy = prob.world_dynamics.dynamics
    z_dy = prob.world_dynamics.obs_dynamics

    s_c = init_s

    for t in 1:Hf
        us = control_all(prob.π, s_c, xs, zs, τ0 + t - 1, SOneTo(N))
        us = limit_control.(x_dy, us, xs, τ0 + t - 1)
        @inbounds for i in 1:N
            xs[i] = sys_forward(x_dy[i], xs[i], us[i], N) + prob.δx[t, i]
            z = prob.z_obs[t, i]
            zs[i] = (z === missing) ? obs_forward(z_dy[i], xs[i], zs[i], N) : z
        end
        x_traj[t,:] = xs
        z_traj[t,:] = zs
        u_traj[t,:] = us
        if t == ΔT
            s_c = deepcopy(s_c)
        end
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

function self_estimate(
    self_dynamics::SysDynamics,
    s0::Timed{X},
    u_history::TimedQueue,
)::Vector{Timed{X}} where {X}
    x, τ = s0.value, s0.time
    x_history = Vector{Timed{X}}(undef, length(u_history))
    for (t, u) in enumerate(u_history.queue)
        x = sys_forward(self_dynamics, x, u[τ + t - 1], τ + t - 1)
        x_history[t] = Timed(τ + t, x)
    end
    x_history
end