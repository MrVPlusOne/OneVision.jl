export NaiveMsg, NaiveCF, NaiveController

struct NaiveMsg{X,Z}
    x::X
    z::Z
end

struct NaiveCF{X,Z,U,S} <: ControllerFramework{X,Z,U,NaiveMsg{X,Z},Nothing}
    num_agents::ℕ
    central::CentralControl{U,S}
    msg_queue_len::ℕ
    ΔT::𝕋
end

NaiveCF(X, Z, N, central::CentralControl{U,S}, msg_queue_len, ΔT) where {U,S} = 
    NaiveCF{X,Z,U,S}(N, central, msg_queue_len, ΔT)

struct NaiveController{X,Z,U,S} <: Controller{X,Z,U,NaiveMsg{X,Z},Nothing}
    self::ℕ
    cf::NaiveCF{X,Z,U,S}
    "Central control state."
    s_c::Ref{S}
    "Current action"
    u::Ref{U}
    t::Ref{𝕋}
    t0::𝕋
end

# NaiveCF has nothing to log
OneVision.write_logs(::NaiveController) = Dict{𝕋,Nothing}()

function OneVision.control!(
    ctrl::NaiveController{X,Z,U},
    x::X,
    z::Z,
    msgs::Each{NaiveMsg{X,Z}},
)::Tuple{U,Each{NaiveMsg{X,Z}}} where {X,Z,U}
    msgs[ctrl.self] = NaiveMsg(x, z)
    @unpack ΔT, central = ctrl.cf
    xs = [m.x for m in msgs]
    zs = [m.z for m in msgs]
    if mod(ctrl.t[] - ctrl.t0, ΔT) == 0
        ctrl.u[] = control_one(central, ctrl.s_c[], xs, zs, ctrl.t[], ctrl.self)
    end
    msgs′ = fill(NaiveMsg(x, z), length(msgs))
    ctrl.t[] += 1
    ctrl.u[], msgs′
end

function OneVision.make_controllers(
    framework::NaiveCF{X,Z,U,S},
    init_status::Each{Tuple{X,Z,U}},
    init_t::𝕋,
) where {X,Z,U,S}
    central = framework.central
    s_c = init_state(central, init_t)
    ctrls = ntuple(framework.num_agents) do i
        u = init_status[i][3]
        NaiveController(i, framework, Ref(s_c), Ref(u), Ref(init_t), init_t)
    end
    init_msg() = begin
        receives = [NaiveMsg(x, z) for (x, z, u) in init_status]
        constant_queue(receives, framework.msg_queue_len)
    end
    msgs = ntuple(_ -> init_msg(), framework.num_agents)
    ctrls, msgs
end
