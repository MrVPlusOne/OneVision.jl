export OvCF, OvController, OvMsg

import Optim

struct OvMsg{X,Z}
    δx::Timed{X}
    z::Timed{Optional{Z}}
end

to_optional(x::Timed{X}) where X = convert(Timed{Optional{X}}, x)

"""
Matrices are indexed by [time, id].
"""
struct OvLog{X,Z,U}
    ũ::Matrix{U}
    x̃::Matrix{X}
    z̃::Matrix{Z}
    δx::X
    z::Z
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
    u_history::TimedQueue{U}            # t ∈ [τ-Tx,τ+Tu-1]
    "predicted x from last control step, used to measure disturbance."
    pred_x::Timed{X}                    # t = τ-Tx
    "self disturbance history"
    self_δx::TimedQueue{X}              # t ∈ [τ-Tx-Tc-ΔT-1, τ-Tx-1-ΔT]
    self_z::TimedQueue{Optional{Z}}     # t ∈ [τ-Tx-Tc-ΔT, τ-Tx-ΔT]
    ideal_xz::Timed{Each{Tuple{X,Z}}}   # t = τ-Tx-Tc-ΔT
    ideal_s::Timed{S}                   # t = τ-Tx-Tc-ΔT
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
    τ::𝕋,
)::Tuple where {N,X,Z,U,S,H}
    dm = cf.delay_model
    @unpack Tx, Tu, Tc, ΔT = short_delay_names(dm)
    function mk_controller(id)
        x0, z0, u0 = init_status[id]
        u_history = TimedQueue(u0, τ-Tx, τ+Tu-1)
        x_dy = cf.world_model.dynamics[id]
        pred_x = Timed(τ-Tx, x0)
        self_δx = TimedQueue(zero(X), τ-Tx-Tc-ΔT-1, τ-Tx-ΔT-1)
        self_z = TimedQueue(missing, τ-Tx-Tc-ΔT, τ-Tx-ΔT, eltype=Optional{Z})
        ideal_xz = Timed(τ-Tx-Tc-ΔT, [(x, z) for (x, z, _) in init_status])
        ideal_s = Timed(τ-Tx-Tc-ΔT, init_state(cf.central, τ))

        fp_prob = let Hf = H * ΔT + dm.total + ΔT
            ForwardPredictProblem(cf.world_model, cf.central; X, Z, Hf)
        end
        x_weights = cf.x_weights[id]
        u_weights = cf.u_weights[id]
        plan_prob = TrajPlanningProblem(
            H, ΔT, x_dy, x_weights, u_weights, optim_options = cf.optim_options,
        )

        OvController(;
            id, cf, τ, u_history, pred_x, 
            self_δx, self_z, ideal_xz, ideal_s, 
            fp_prob, plan_prob, logs = Dict{𝕋,OvLog{X,Z,U}}(),
        )
    end

    function mk_q(_)
        receives = [[OvMsg(Timed(t-1, zero(x)), Timed{Optional{Z}}(t, missing)) 
                    for (x, z, u) in init_status
                    ] for t in range(τ-Tc-Tx, step = ΔT, 
                                    length = msg_queue_length(dm))]
        @show range(τ-Tc-Tx, step = ΔT, length = msg_queue_length(dm))
        FixedQueue(receives)
    end

    ctrls::NTuple{N,OvController} = ntuple(mk_controller, N)
    msg_qs::NTuple{N,MsgQueue} = ntuple(mk_q, N)
    ctrls, msg_qs
end

function OneVision.control!(
    π::OvController{N,X,Z,U,S,H,ΔT},
    x::X,
    z0::Z,
    msgs::Each{OvMsg{X,Z}},
)::Tuple{U,Each{OvMsg{X,Z}}} where {N,X,Z,U,S,H,ΔT}
    @unpack id, τ = π
    world = π.cf.world_model
    dm = π.cf.delay_model
    fp_prob = π.fp_prob
    @unpack Tx, Tu, Tc, Ta = short_delay_names(dm)
    z = Timed(τ-Tx, z0)

    δx = x - π.pred_x[τ-Tx] |> attime(τ-Tx-1)   # δx: t = τ-Tx-1
    for t in τ-Tx-ΔT+1:τ-Tx-1
        pushpop!(π.self_δx, Timed(t-1, zero(x)))
        pushpop!(π.self_z, Timed(t, missing))
    end  
    pushpop!(π.self_δx, δx)     # π.self_δx: t ∈ [τ-Tx-1-Tc, τ-Tx-1]
    pushpop!(π.self_z, z)       # π.self_z: t ∈ [τ-Tx-Tc, τ-Tx]
    new_msg = OvMsg(δx, to_optional(z))
    should_log = π.cf.save_log((id, π.τ, x, z.value))

    x_self::Timed{X} = isempty(π.u_history) ? x : self_estimate(
        world.dynamics[id],
        Timed(τ - Tx, x),
        π.u_history
    )[end]  # x_self: t = τ+Tu

    # forward predict the ideal fleet trajectory from the state at t-Tx-Tc-1
    ptime = τ-Tx-Tc-ΔT  # the starting time of forward_predict!
    for j in 1:N
        if j == id continue end
        for t in 1:ΔT - 1
            fp_prob.δx[t, j] = zero(X)
            fp_prob.z_obs[t, j] = missing
        end
        @unpack δx, z = msgs[j]
        fp_prob.δx[ΔT, j] = δx[ptime-1]
        fp_prob.z_obs[ΔT, j] = z[ptime]
    end
    fp_prob.δx[1:length(π.self_δx),id] = collect(x.value for x in π.self_δx.queue)
    fp_prob.z_obs[1:length(π.self_z),id] = collect(x.value for x in π.self_z.queue)

    # x̃[t in τ-Tx-Tc-ΔT+1: τ+Tu+HΔT], ũ[t in τ-Tx-Tc-ΔT: τ+Tu+HΔT-1]
    @asserteq π.self_δx[1].time ptime
    ũ, x̃, z̃ = forward_predict!(
        fp_prob, π.ideal_xz[ptime], π.ideal_s[ptime], ptime, ΔT)
    π.ideal_xz = [(x̃[ΔT,j], z̃[ΔT,j]) for j in 1:N] |> attime(ptime+ΔT)  # π.ideal_xz: t = τ-Tx-Tc

    # local planning using path following
    
    # u_plan: [τ+Tu:τ+Tu+HΔT-1], x_plan: [τ+Tu+1:τ+Tu+HΔT]
    u_plan, loss = let 
        n_x, n_u = length(X), length(U)
        x_path = SVector{H * ΔT}(x̃[1 + ΔT + Ta : end, id])  # [τ+Tu+1:τ+Tu+HΔT]
        u_path = SVector{H * ΔT}(ũ[1 + ΔT + Ta : end, id])  # [τ+Tu:τ+Tu+HΔT-1]
        plan_trajectory(π.plan_prob, x_self[τ+Tu], x_path, u_path, τ+Tu)
    end
    
    u = Timed(τ+Tu, u_plan[1])

    π.pred_x = let x1 = x, z1 = z[τ-Tx]
        x_dy = world.dynamics[id]
        for t = τ-Tx : (τ - Tx + ΔT - 1)
            u_old = pushpop!(π.u_history, u)[τ-Tx]  # u_old at t = τ-Tx
            x1 = sys_forward(x_dy, x1, u_old, t)
        end
        Timed(x1, τ-Tx+ΔT)
    end

    if should_log 
        π.logs[π.τ - dm.obs - dm.com] = OvLog(to_array.((ũ, x̃, z̃))..., δx, z)
    end

    π.τ += ΔT
    u[τ+Tu], fill(new_msg, N)
end
