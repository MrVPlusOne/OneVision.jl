export OvCF, OvController, OvMsg

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
struct OvCF{N,X,Z,U,H} <: ControllerFramework{X,Z,U,OvMsg{X,Z},OvLog{X,Z,U}}
    central::CentralControl
    world_model::WorldDynamics
    delay_model::DelayModel
    x_weights::Each{Vector{ℝ}}
    u_weights::Each{Vector{ℝ}}
    save_log::FuncT{Tuple{ℕ,𝕋,X,Z},Bool}
end


@kwdef mutable struct OvController{N,X,Z,U,H,Hf,Dy,Ctrl} <: Controller{X,Z,U,OvMsg{X,Z},OvLog{X,Z,U}}
    id::ℕ
    cf::OvCF{N,X,Z,U,H}
    τ::𝕋                                # -- At the start of each controll loop --
    u_history::FixedQueue{U}            # t ∈ [τ-Tx,τ+Tu-1]
    pred_xz::Tuple{X,Z}                 # t = τ-Tx
    self_δxz::FixedQueue{Tuple{X,Z}}    # t ∈ [τ-Tx-Tc-2, τ-Tx-2]
    ideal_xz::Each{Tuple{X,Z}}          # t = τ-Tx-Tc-1
    fp_prob::ForwardPredictProblem{N,Hf,X,Z,U,Dy,Ctrl}
    pf_prob::PathFollowingProblem{H}
    logs::Dict{𝕋,OvLog{X,Z,U}}
end

OneVision.write_logs(ctrl::OvController) = ctrl.logs

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
        pred_xz = (x0, z0)
        self_δxz = constant_queue((zero(x0), zero(z0)), dm.com + 1)
        ideal_xz = [(x, z) for (x, z, _) in init_status]

        fp_prob = let H = H + dm.total + 1
            ForwardPredictProblem(cf.world_model, cf.central, zero(x0), zero(z0); H)
        end
        x_weights = let v = cf.x_weights[id]; SVector{length(v)}(v) end
        u_weights = let v = cf.u_weights[id]; SVector{length(v)}(v) end
        pf_prob = PathFollowingProblem(Val(H), x_dy, x_weights, u_weights, Ref{Any}(missing))
        OvController(;
            id, cf, τ=t0, u_history, pred_xz, self_δxz, ideal_xz, fp_prob, pf_prob,
            logs=Dict{𝕋,OvLog{X,Z,U}}(),
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
    δxz = (x, z) .- π.pred_xz  # δxz: t = τ-Tx-1
    pushpop!(π.self_δxz, δxz)  # π.self_δxz: t ∈ [τ-Tx-1-Tc, τ-Tx-1]
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
        δx, δz = msgs[j].δxz
        π.fp_prob.δx[1, j] = δx
        π.fp_prob.δz[1, j] = δz
    end
    for (t, (δx, δz)) in enumerate(π.self_δxz)
        π.fp_prob.δx[t,id] = δx
        π.fp_prob.δz[t,id] = δz
    end

    # x̃[t in τ-Tx-Tc: τ+Tu+H], ũ[t in τ-Tx-Tc-1: τ+Tu+H-1]
    ũ, x̃, z̃ = forward_predict(π.fp_prob, π.ideal_xz, π.τ - dm.obs - dm.com - 1)
    π.ideal_xz = [(x̃[1,j], z̃[1,j]) for j in 1:N]  # π.ideal_xz: t = τ-Tx-Tc

    # local planning using path following
    
    # u_plan: [τ+Tu:τ+Tu+H-1], x_plan: [τ+Tu+1:τ+Tu+H]
    u_plan, loss = let 
        n_x, n_u = length(X), length(U)
        x0 = SVector{n_x}(x_self)
        x_path = SMatrix{n_x,H}(hcat(x̃[2 + dm.total:end,id]...))  # [τ+Tu+1:τ+Tu+H]
        u_path = SMatrix{n_u,H}(hcat(ũ[2 + dm.total:end,id]...))  # [τ+Tu:τ+Tu+H-1]
        follow_path_optim(π.pf_prob, x0, x_path, u_path, π.τ + dm.act)
    end
    
    u = (U <: FieldVector) ? U(u_plan[:,1]...) : convert(U, u_plan[:,1])
    u_old = pushpop!(π.u_history, u)  # u at t = τ-Tx

    π.pred_xz = let 
        x_dy = world.dynamics[id]
        x1 = sys_forward(x_dy, x, u_old, π.τ - dm.obs)
        z_dy = world.obs_dynamics[id]
        z1 = obs_forward(z_dy, x, z, π.τ - dm.obs)
        (x1, z1)
    end

    if should_log 
        π.logs[π.τ - dm.obs - dm.com] = OvLog(to_array.((ũ, x̃, z̃))..., δxz)
    end

    u, fill(new_msg, N)
end
