export NaiveMsg, NaiveCF, NaiveController

struct NaiveMsg{X,Z}
    x::X
    z::Z
end

struct NaiveCF{X,Z,U,S} <: ControllerFramework{X,Z,U,NaiveMsg{X,Z},Nothing}
    num_agents::ℕ
    central::CentralControl{U,S}
    msg_delay::ℕ
end

mutable struct NaiveController{X,Z,U,S} <: Controller{X,Z,U,NaiveMsg{X,Z},Nothing}
    self::ℕ
    central::CentralControl{U,S}
    "Central control state."
    s_c::S
    t::𝕋
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
    xs = [m.x for m in msgs]
    zs = [m.z for m in msgs]
    ctrl.t += 1
    u = control_one(ctrl.central, ctrl.s_c, xs, zs, ctrl.t, ctrl.self)
    msgs′ = fill(NaiveMsg(x, z), length(msgs))
    u, msgs′
end

function OneVision.make_controllers(
    framework::NaiveCF{X,Z,U,S},
    init_status::Each{Tuple{X,Z,U}},
    init_t::𝕋,
) where {X,Z,U,S}
    central = framework.central
    s_c = init_state(central)
    ctrls = ntuple(framework.num_agents) do i
        NaiveController{X,Z,U,S}(i, central, s_c, init_t - 1)
    end
    init_msg() = begin
        receives = [NaiveMsg(x, z) for (x, z, u) in init_status]
        constant_queue(receives, framework.msg_delay)
    end
    msgs = ntuple(_ -> init_msg(), framework.num_agents)
    ctrls, msgs
end
