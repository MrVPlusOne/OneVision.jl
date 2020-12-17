export OvCF, OvController, OvMsg

import Optim

struct OvMsg{X,Z}
    xz::Tuple{X,Z}
    δxz::Tuple{X,Z}
end

"""
Matrices are indexed by [time, id].
"""
struct OvLog{X,Z,U}
    ũ::Matrix{U}
    x̃::Matrix{X}
    z̃::Matrix{Z}
    δxz::Tuple{X,Z}
end

function to_array(m::MMatrix{n1,n2,T}) where {n1,n2,T}
    r = Matrix{T}(undef, n1, n2)
    copyto!(r, m)
end

"""
The OneVision Controller Framework.
"""
@kwdef struct OvCF{N,X,Z,U,S,H} <: ControllerFramework{X,Z,U,OvMsg{X,Z},OvLog{X,Z,U}}
    central::CentralControl{U,S}
    world_model::WorldDynamics{N}
    delay_model::DelayModel
    x_weights::SVector{N,X}
    u_weights::SVector{N,U}
    optim_options::Optim.Options
    save_log::FuncT{Tuple{ℕ,𝕋,X,Z},Bool} = FuncT(x -> false, Tuple{ℕ,𝕋,X,Z}, Bool)
end

function OvCF(central::CentralControl{U,S}, world_model, delay_model, x_weights, u_weights; 
    X, Z, N, H, 
    u_tol = 1e-4, loss_tol = 1e-6, loss_rel_tol = 1e-6, max_iters = 100,
    save_log = FuncT(x -> false, Tuple{ℕ,𝕋,X,Z}, Bool)
) where {U,S}
    OvCF{N,X,Z,U,S,H}(;central, world_model, delay_model, x_weights, u_weights, save_log,
        optim_options = Optim.Options(
            x_abstol = u_tol, f_abstol = loss_tol, f_reltol = loss_rel_tol, 
            iterations = max_iters))
end

function OvCF(loss_model::RegretLossModel{N,X,U,S}, delay_model; 
    Z, H, 
    optim_options = Optim.Options(
        x_abstol=1e-4, f_abstol=1e-6, f_reltol=1e-6, iterations=100),
    save_log = FuncT(x -> false, Tuple{ℕ,𝕋,X,Z}, Bool)
) where {N,X,U,S}
    @unpack central, world_model, x_weights, u_weights = loss_model
    OvCF{N,X,Z,U,S,H}(;central, world_model, delay_model, 
        x_weights, u_weights, save_log, optim_options)
end


@kwdef mutable struct OvController{N,X,Z,U,S,H,ΔT,Hf,WDy,Ctrl} <: Controller{X,Z,U,OvMsg{X,Z},OvLog{X,Z,U}}
    id::ℕ
    cf::OvCF{N,X,Z,U,S,H}
    τ::𝕋                                # -- At the start of each controll step --
    u_history::FixedQueue{U}            # t ∈ [τ-Tx,τ+Tu-1]
    "predicted x and z made from last control step, used to measure disturbance."
    pred_xz::Tuple{X,Z}                 # t = τ-Tx
    "self disturbance history"
    self_δxz::FixedQueue{Tuple{X,Z}}    # t ∈ [τ-Tx-Tc-ΔT-1, τ-Tx-2]
    ideal_xz::Each{Tuple{X,Z}}          # t = τ-Tx-Tc-ΔT
    ideal_s::S                          # t = τ-Tx-Tc-ΔT
    fp_prob::ForwardPredictProblem{N,Hf,X,Z,U,WDy,Ctrl}
    plan_prob::TrajPlanningProblem{H,ΔT,X,U}
    logs::Dict{𝕋,OvLog{X,Z,U}}
end

OneVision.write_logs(ctrl::OvController) = ctrl.logs

"""
# Arguments
- `init_status`: The initial (x,z,u) of the agent. x and z are assumed to be 
measured at `t0-Tx`.
"""
function OneVision.make_controllers(
    cf::OvCF{N,X,Z,U,S,H},
    init_status::Each{Tuple{X,Z,U}},
    t0::𝕋,
)::Tuple where {N,X,Z,U,S,H}
    dm = cf.delay_model
    ΔT = dm.ΔT
    function mk_controller(id)
        x0, z0, u0 = init_status[id]
        u_history = constant_queue(u0, dm.obs + dm.act)
        x_dy = cf.world_model.dynamics[id]
        pred_xz = (x0, z0)
        self_δxz = constant_queue((zero(X), zero(Z)), dm.com + ΔT)
        ideal_xz = [(x, z) for (x, z, _) in init_status]
        ideal_s = init_state(cf.central, t0)

        fp_prob = let Hf = H * ΔT + dm.total + ΔT
            ForwardPredictProblem(cf.world_model, cf.central; X, Z, Hf)
        end
        x_weights = cf.x_weights[id]
        u_weights = cf.u_weights[id]
        plan_prob = TrajPlanningProblem(
            H, ΔT, x_dy, x_weights, u_weights, optim_options = cf.optim_options,
        )

        OvController(;
            id, cf, τ = t0 - 1, u_history, pred_xz, self_δxz, ideal_xz, ideal_s, 
            fp_prob, plan_prob, logs = Dict{𝕋,OvLog{X,Z,U}}(),
        )
    end

    function mk_q(_)
        receives = [OvMsg((x, z), (zero(x), zero(z))) for (x, z, u) in init_status]
        constant_queue(receives, msg_queue_length(dm))
    end

    ctrls::NTuple{N,OvController} = ntuple(mk_controller, N)
    msg_qs::NTuple{N,MsgQueue} = ntuple(mk_q, N)
    ctrls, msg_qs
end

function OneVision.control!(
    π::OvController{N,X,Z,U,S,H,ΔT},
    x::X,
    z::Z,
    msgs::Each{OvMsg{X,Z}},
)::Tuple{U,Each{OvMsg{X,Z}}} where {N,X,Z,U,S,H,ΔT}
    id = π.id
    world = π.cf.world_model
    dm = π.cf.delay_model
    fp_prob = π.fp_prob

    π.τ += ΔT
    δxz = ((x, z) .- π.pred_xz)  # δxz: t = τ-Tx-1
    foreach(1:ΔT-1) do _; pushpop!(π.self_δxz, (zero(x), zero(z))) end  
    pushpop!(π.self_δxz, δxz) # π.self_δxz: t ∈ [τ-Tx-1-Tc, τ-Tx-1]
    new_msg = OvMsg((x, z), δxz)
    should_log = π.cf.save_log((id, π.τ, x, z))

    x_self = isempty(π.u_history) ? x : self_estimate(
        world.dynamics[id],
        (x, π.τ - dm.obs),
        π.u_history
    )[end]  # x_self: t = τ+Tu

    # forward predict the ideal fleet trajectory from the state at t-Tx-Tc-1
    for j in 1:N
        if j == id continue end
        for t in 1:ΔT - 1
            fp_prob.δx[t, j] = zero(X)
            fp_prob.δz[t, j] = zero(Z)
        end
        δx, δz = msgs[j].δxz
        fp_prob.δx[ΔT, j] = δx
        fp_prob.δz[ΔT, j] = δz
    end
    for (t, (δx, δz)) in enumerate(π.self_δxz)
        fp_prob.δx[t,id] = δx
        fp_prob.δz[t,id] = δz
    end

    # x̃[t in τ-Tx-Tc-ΔT+1: τ+Tu+HΔT], ũ[t in τ-Tx-Tc-ΔT: τ+Tu+HΔT-1]
    ũ, x̃, z̃ = forward_predict!(
        fp_prob, π.ideal_xz, π.ideal_s, π.τ - dm.obs - dm.com - ΔT, ΔT)
    π.ideal_xz = [(x̃[ΔT,j], z̃[ΔT,j]) for j in 1:N]  # π.ideal_xz: t = τ-Tx-Tc

    # local planning using path following
    
    # u_plan: [τ+Tu:τ+Tu+HΔT-1], x_plan: [τ+Tu+1:τ+Tu+HΔT]
    u_plan, loss = let 
        n_x, n_u = length(X), length(U)
        x0 = x_self
        x_path = SVector{H * ΔT}(x̃[1 + ΔT + dm.total:end,id])  # [τ+Tu+1:τ+Tu+HΔT]
        u_path = SVector{H * ΔT}(ũ[1 + ΔT + dm.total:end,id])  # [τ+Tu:τ+Tu+HΔT-1]
        plan_trajectory(π.plan_prob, x0, x_path, u_path, π.τ + dm.act)
    end
    
    u = u_plan[1]

    π.pred_xz = let x1 = x, z1 = z
        x_dy = world.dynamics[id]
        z_dy = world.obs_dynamics[id]
        for t = (π.τ - dm.obs):(π.τ - dm.obs + ΔT - 1)
            u_old = pushpop!(π.u_history, u)  # u_old at t = τ-Tx
            x1 = sys_forward(x_dy, x1, u_old, t)
            z1 = obs_forward(z_dy, x1, z1, t)
        end
        (x1, z1)
    end

    if should_log 
        π.logs[π.τ - dm.obs - dm.com] = OvLog(to_array.((ũ, x̃, z̃))..., δxz)
    end

    u, fill(new_msg, N)
end
