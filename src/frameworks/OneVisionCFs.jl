export OvCF, OvController, OvMsg

import OSQP

struct OvMsg{X,Z}
    xz::Tuple{X,Z}
    δxz::Tuple{X,Z}
end

"""
The OneVision Controller Framework.
"""
struct OvCF{N,X,Z,U,H} <: ControllerFramework{X,Z,U,OvMsg{X,Z}}
    central::CentralControl
    world_model::WorldDynamics
    delay_model::DelayModel
    x_weights::Each{Vector{ℝ}}
    u_weights::Each{Vector{ℝ}}
end

@kwdef mutable struct OvController{N,X,Z,U,H,Hf,Dy,Ctrl} <: Controller{X,Z,U,OvMsg{X,Z}}
    id::ℕ
    cf::OvCF{N,X,Z,U,H}
    τ::𝕋
    u_history::FixedQueue{U}  # t ∈ [τ-Tx,τ+Tu-1]
    pred_xz::Tuple{X,Z}  # predicted x, z for t = τ-Tx
    self_δxz::FixedQueue{Tuple{X,Z}}  # t ∈ [τ-Tx-Tc-1, τ-Tx-1]
    ideal_xz::Each{Tuple{X,Z}}  # for t = τ-Tx-Tc-1
    fp_prob::ForwardPredictProblem{N,Hf,X,Z,U,Dy,Ctrl}
    pf_prob::PathFollowingProblem{H}
end

"""
# Arguments
- `init_status`: The initial (x,z,u) of the agent. x and z are assumed to be 
measured at `t0-Tx`.
"""
function OneVision.make_controllers(
    cf::OvCF{N,X,Z,U,H},
    init_status::Each{Tuple{X,Z,U}},
    t0::𝕋,
)::Tuple where {N,X,Z,U,H}
    dm = cf.delay_model
    function mk_controller(id)
        x0, z0, u0 = init_status[id]
        u_history = constant_queue(u0, dm.obs + dm.act)
        x_dy = cf.world_model.dynamics[id]
        pred_xz = let 
            x1 = sys_forward(x_dy, x0, u0, t0 - dm.obs)
            z_dy = cf.world_model.obs_dynamics[id]
            z1 = obs_forward(z_dy, x0, z0, t0 - dm.obs)
            (x1, z1)
        end
        self_δxz = constant_queue((zero(x0), zero(z0)), dm.com + 1)
        ideal_xz = [(x,z) for (x,z,_) in init_status]

        fp_prob = let H = H + dm.total 
            ForwardPredictProblem(cf.world_model, cf.central, zero(x0), zero(z0); H)
        end
        x_weights = let v = cf.x_weights[id]; SVector{length(v)}(v) end
        u_weights = let v = cf.u_weights[id]; SVector{length(v)}(v) end
        pf_prob = PathFollowingProblem(
            Val(H), x_dy, x_weights, u_weights, 
            () -> OSQP.Optimizer(verbose=false))
        OvController(;
            id, cf, τ=t0, u_history, pred_xz, self_δxz, ideal_xz, fp_prob, pf_prob,
        )
    end

    function mk_q(_)
        receives = [OvMsg((x, z), (zero(x), zero(z))) for (x, z, u) in init_status]
        constant_queue(receives, dm.com)
    end

    ctrls::NTuple{N,OvController} = ntuple(mk_controller, N)
    msg_qs::NTuple{N,MsgQueue} = ntuple(mk_q, N)
    ctrls, msg_qs
end

function OneVision.control!(
    π::OvController{N,X,Z,U,H},
    x::X,
    z::Z,
    msgs::Each{OvMsg{X,Z}},
)::Tuple{U,Each{OvMsg{X,Z}}} where {N,X,Z,U,H}
    id = π.id
    π.τ += 1
    world = π.cf.world_model
    dm = π.cf.delay_model
    δxz = (x, z) .- π.pred_xz
    pushpop!(π.self_δxz, δxz)
    new_msg = OvMsg((x, z), δxz)

    x_self = self_estimate(
        world.dynamics[id],
        (x, π.τ - dm.obs),
        π.u_history
    )[end]  # self state at τ+Tu

    # forward predict the ideal fleet trajectory from the state at t-Tx-Tc-1
    for j in 1:N
        if j==id continue end
        δx,δz = msgs[j].δxz
        π.fp_prob.δx[1, j] = δx
        π.fp_prob.δz[1, j] = δz
    end
    for (t, (δx,δz)) in enumerate(π.self_δxz)
        π.fp_prob.δx[t,id] = δx
        π.fp_prob.δz[t,id] = δz
    end

    ũ, x̃, z̃ = forward_predict(π.fp_prob, π.ideal_xz, π.τ - dm.obs - dm.com - 1)

    # local planning using path following
    
    u_plan, x_plan = let 
        n_x, n_u = length(X), length(U)
        x0 = SVector{n_x}(x_self)
        x_path = SMatrix{n_x, H}(hcat(x̃[dm.total+1:end,id]...))
        u_path = SMatrix{n_u, H}(hcat(ũ[dm.total+1:end,id]...))
        follow_path(π.pf_prob, x0, x_path, u_path, π.τ+dm.act)
    end
    
    u = (U <: FieldVector) ? U(u_plan[:,1]...) : convert(U, u_plan[:,1])
    pushpop!(π.u_history, u)
    π.ideal_xz = [(x̃[1,j], z̃[1,j]) for j in 1:N]
    π.pred_xz = let 
        x_dy = world.dynamics[id]
        x1 = sys_forward(x_dy, x, u, π.τ - dm.obs)
        z_dy = world.obs_dynamics[id]
        z1 = obs_forward(z_dy, x, z, π.τ - dm.obs)
        (x1, z1)
    end

    u, fill(new_msg, N)
end
