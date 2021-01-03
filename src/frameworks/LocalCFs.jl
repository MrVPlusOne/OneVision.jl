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

struct LocalController{id,N,X,Z,U,S,XDy,ZDy} <: Controller{X,Z,U,LocalCFMsg,Nothing}
    cf::LocalCF{N,X,Z,U,S,XDy,ZDy}
    "Central control state."
    s_c::Ref{S}
    τ::Ref{𝕋}
    u::Ref{U}                           # t = τ-1
    u_history::TimedQueue{U}            # t ∈ [τ-Tx,τ+Tu-1]
    t0::𝕋
end

function LocalController(
    id,cf::LocalCF{N,X,Z,U,S,XDy,ZDy}, s_c, t, u0, u_history
) where {N,X,Z,U,S,XDy,ZDy}
    LocalController{id,N,X,Z,U,S,XDy,ZDy}(cf, Ref(s_c), Ref(t), Ref(u0), u_history, t)
end

function OneVision.control!(
    ctrl::LocalController{id,N,X,Z,U},
    x::X,
    z::Z,
    msgs::Each{LocalCFMsg{X,Z}},
)::Tuple{U,Each{LocalCFMsg{X,Z}}} where {id,N,X,Z,U}
    @unpack central, world_model, delay_model = ctrl.cf
    @unpack Tx, Tu, ΔT = short_delay_names(delay_model)
    τ = ctrl.τ[]

    x_history = self_estimate(
        world_model.dynamics[id],
        Timed(τ - Tx, x),
        ctrl.u_history,
    )
    z_history = self_z_estimate(
        world_model.obs_dynamics[id], 
        Timed(τ - Tx, z),
        [Timed(τ - Tx, x); x_history[1:end-1]],
    )

    x_self, z_self = isempty(ctrl.u_history) ? (x, z) : (x_history[end], z_history[end])
    msg = LocalCFMsg(x_self[τ+Tu], z_self[τ+Tu])
    if mod(τ - ctrl.t0, ΔT) == 0
        msgs[id] = msg
        xs = [m.x for m in msgs]
        zs = [m.z for m in msgs]
        ctrl.u[] = control_one(central, ctrl.s_c[], xs, zs, τ + Tu, id)
    end
    msgs′ = fill(msg, N)
    pushpop!(ctrl.u_history, Timed(τ + Tu, ctrl.u[]))
    ctrl.τ[] += 1
    ctrl.u[], msgs′
end

function OneVision.make_controllers(
    cf::LocalCF{N,X,Z,U,S},
    init_status::Each{Tuple{X,Z,U}},
    t0::𝕋,
)::Tuple where {N,X,Z,U,S}
    @unpack Tx, Tu, ΔT = short_delay_names(cf.delay_model)
    function mk_controller(id)
        x0, z0, u0 = init_status[id]
        u_history = TimedQueue(u0, t0-Tx, t0+Tu-1)
        ideal_s = init_state(cf.central, t0)

        LocalController(id, cf, ideal_s, t0, u0, u_history)
    end
    function mk_q(_)
        receives = [LocalCFMsg(x,z) for (x, z, u) in init_status]
        constant_queue(receives, msg_queue_length(cf.delay_model))
    end

    ctrls = ntuple(mk_controller, N)
    msg_qs = ntuple(mk_q, N)
    ctrls, msg_qs
end

# LocalCF has nothing to log
OneVision.write_logs(::LocalController) = Dict{𝕋,Nothing}()