export LocalCF, LocalCFMsg, LocalController

const LocalCFMsg = NaiveMsg

"""
A controller framework that only performs local delay compensation without 
considering communication delays.
"""
struct LocalCF{N,X,Z,U,S,XDy,ZDy} <: ControllerFramework{X,Z,U,NaiveMsg{X,Z},Nothing}
    central::CentralControl{U,S}
    world_model::WorldDynamics{N,XDy,ZDy}
    delay_model::DelayModel
end

function LocalCF(
    central::CentralControl{U,S}, world_model::WorldDynamics{N,XDy,ZDy}, delay_model; 
    X, Z
) where {N,U,S,XDy,ZDy}
    LocalCF{N,X,Z,U,S,XDy,ZDy}(central, world_model, delay_model)
end

mutable struct LocalController{id,N,X,Z,U,S,XDy,ZDy} <: Controller{X,Z,U,LocalCFMsg,Nothing}
    cf::LocalCF{N,X,Z,U,S,XDy,ZDy}
    "Central control state."
    s_c::S
    τ::𝕋
    u_history::FixedQueue{U}            # t ∈ [τ-Tx,τ+Tu-1]
end

function LocalController(
    id,cf::LocalCF{N,X,Z,U,S,XDy,ZDy}, s_c, t, u_history
) where {N,X,Z,U,S,XDy,ZDy}
    LocalController{id,N,X,Z,U,S,XDy,ZDy}(cf, s_c, t, u_history)
end

function OneVision.control!(
    ctrl::LocalController{id,N,X,Z,U},
    x::X,
    z::Z,
    msgs::Each{LocalCFMsg{X,Z}},
)::Tuple{U,Each{LocalCFMsg{X,Z}}} where {id,N,X,Z,U}
    @unpack central, world_model, delay_model = ctrl.cf
    Tx = delay_model.obs
    Tu = delay_model.act
    ΔT = delay_model.ΔT

    ctrl.τ += ΔT

    x_self = isempty(ctrl.u_history) ? x : self_estimate(
        world_model.dynamics[id],
        (x, ctrl.τ - Tx),
        ctrl.u_history
    )[end]  # x_self: t = τ+Tu

    msg = LocalCFMsg(x_self, z)
    msgs[id] = msg
    xs = [m.x for m in msgs]
    zs = [m.z for m in msgs]
    u = control_one(central, ctrl.s_c, xs, zs, ctrl.τ + Tu, id)
    for _ in 1:ΔT
        pushpop!(ctrl.u_history, u)
    end
    msgs′ = fill(msg, N)
    u, msgs′
end

function OneVision.make_controllers(
    cf::LocalCF{N,X,Z,U,S},
    init_status::Each{Tuple{X,Z,U}},
    t0::𝕋,
)::Tuple where {N,X,Z,U,S}
    dm = cf.delay_model
    ΔT = dm.ΔT
    function mk_controller(id)
        x0, z0, u0 = init_status[id]
        u_history = constant_queue(u0, dm.obs + dm.act)
        ideal_s = init_state(cf.central, t0)

        LocalController(id, cf, ideal_s, t0-1, u_history)
    end
    function mk_q(_)
        receives = [LocalCFMsg(x,z) for (x, z, u) in init_status]
        constant_queue(receives, msg_queue_length(dm))
    end

    ctrls = ntuple(mk_controller, N)
    msg_qs = ntuple(mk_q, N)
    ctrls, msg_qs
end