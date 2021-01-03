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
    τ0::𝕋
    τ::𝕋                                # -- At the start of each controll step --
    u_last::U                           # t = τ+Tu-1
    u_history::TimedQueue{U}            # t ∈ [τ-Tx,τ+Tu-1]
    "predicted x from last control step, used to measure disturbance."
    pred_x::Timed{X}                    # t = τ-Tx
    "self disturbance history"
    self_δx::TimedQueue{X}              # t ∈ [τ-Tx-Tc-ΔT-1, τ-Tx-2]
    self_z::TimedQueue{Optional{Z}}     # t ∈ [τ-Tx-Tc-ΔT, τ-Tx-1]
    ideal_xz::Timed{Each{Tuple{X,Z}}}   # t = τ-Tx-Tc-ΔT
    ideal_s::S                   # t = τ-Tx-Tc-ΔT
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
    @unpack Tx, Tu, Tc, Ta, ΔT = short_delay_names(dm)
    function mk_controller(id)
        x0, z0, u0 = init_status[id]
        u_history = TimedQueue(u0, τ-Tx, τ+Tu-1)
        x_dy = cf.world_model.dynamics[id]
        pred_x = Timed(τ-Tx, x0)
        self_δx = TimedQueue(zero(X), τ-Tx-Tc-ΔT-1, τ-Tx-2)
        self_z = TimedQueue(missing, τ-Tx-Tc-ΔT, τ-Tx-1, eltype=Optional{Z})
        ideal_xz = Timed(τ-Tx-Tc-ΔT, [(x, z) for (x, z, _) in init_status])
        ideal_s = init_state(cf.central, τ)

        fp_prob = let Hf = H * ΔT + Ta + ΔT
            ForwardPredictProblem(cf.world_model, cf.central; X, Z, Hf)
        end
        x_weights = cf.x_weights[id]
        u_weights = cf.u_weights[id]
        plan_prob = TrajPlanningProblem(
            H, ΔT, x_dy, x_weights, u_weights, optim_options = cf.optim_options,
        )

        OvController(;
            id, cf, τ0 = τ, τ, u_last = u0, u_history, pred_x, 
            self_δx, self_z, ideal_xz, ideal_s, 
            fp_prob, plan_prob, logs = Dict{𝕋,OvLog{X,Z,U}}(),
        )
    end

    function mk_q(_)
        receives = [[OvMsg(Timed(t-1, zero(x)), Timed{Optional{Z}}(t, missing)) 
                    for (x, z, u) in init_status
                    ] for t in range(τ-Tc-Tx, length = msg_queue_length(dm))]
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
    @unpack id, τ, τ0 = π
    world = π.cf.world_model
    fp_prob = π.fp_prob
    @unpack Tx, Tu, Tc, Ta = short_delay_names(π.cf.delay_model)
    z = Timed(τ-Tx, z0)

    δx = x - π.pred_x[τ-Tx] |> attime(τ-Tx-1)   # δx: t = τ-Tx-1
    pushpop!(π.self_δx, δx)     # π.self_δx: t ∈ [τ-Tx-1-Tc, τ-Tx-1]
    pushpop!(π.self_z, z)       # π.self_z: t ∈ [τ-Tx-Tc, τ-Tx]
    new_msg = OvMsg(δx, to_optional(z))
    should_log = π.cf.save_log((id, π.τ, x, z.value))

    x_self::Timed{X} = isempty(π.u_history) ? x : self_estimate(
        world.dynamics[id],
        Timed(τ - Tx, x),
        π.u_history
    )[end]  # x_self: t = τ+Tu

    "the time offset within the ΔT period"
    t = mod1(τ - τ0, ΔT)

    # forward predict the ideal fleet trajectory from the state at t-Tx-Tc-1
    for j in 1:N
        if j == id continue end
        @unpack δx, z = msgs[j]
        fp_prob.δx[t, j] = δx[τ-Tx-Tc-1]
        fp_prob.z_obs[t, j] = z[τ-Tx-Tc]
    end

    if t == ΔT
        fp_prob.δx[1:length(π.self_δx),id] = collect(x.value for x in π.self_δx.queue)
        fp_prob.z_obs[1:length(π.self_z),id] = collect(x.value for x in π.self_z.queue)

        # x̃[t in τ-Tx-Tc-ΔT+1: τ+Tu+HΔT], ũ[t in τ-Tx-Tc-ΔT: τ+Tu+HΔT-1]
        ptime = τ-Tx-Tc-ΔT  # the starting time of forward_predict!
        @asserteq π.self_δx[1].time ptime
        ũ, x̃, z̃ = forward_predict!(
            fp_prob, π.ideal_xz[ptime], π.ideal_s, ptime, ΔT)
        π.ideal_xz = [(x̃[ΔT,j], z̃[ΔT,j]) for j in 1:N] |> attime(ptime+ΔT)  # π.ideal_xz: t = τ-Tx-Tc

        # local planning using path following
        
        # u_plan: [τ+Tu:τ+Tu+HΔT-1], x_plan: [τ+Tu+1:τ+Tu+HΔT]
        u_plan, loss = let 
            n_x, n_u = length(X), length(U)
            x_path = SVector{H * ΔT}(x̃[1 + ΔT + Ta : end, id])  # [τ+Tu+1:τ+Tu+HΔT]
            u_path = SVector{H * ΔT}(ũ[1 + ΔT + Ta : end, id])  # [τ+Tu:τ+Tu+HΔT-1]
            plan_trajectory(π.plan_prob, x_self[τ+Tu], x_path, u_path, τ+Tu)
        end
        
        π.u_last = u_plan[1]
    end
    u = π.u_last

    π.pred_x = let
        x_dy = world.dynamics[id]
        u_old = pushpop!(π.u_history, Timed(τ+Tu, u))[τ-Tx]  # u_old at t = τ-Tx
        x1 = sys_forward(x_dy, x, u_old, τ-Tx)
        Timed(τ-Tx+1, x1)
    end

    if should_log 
        π.logs[τ - Tx - Tc] = OvLog(to_array.((ũ, x̃, z̃))..., δx, z)
    end

    π.τ += 1
    u, fill(new_msg, N)
end
