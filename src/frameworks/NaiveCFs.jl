module NaiveCFs

export NaiveCF, NaiveMsg, control!, make_controllers

using OneVision
using OneVision: ℕ
import OneVision: control!, make_controllers

struct NaiveMsg{X,Z}
    x::X
    z::Z
end

struct NaiveCF{X,Z,U} <: ControllerFramework{X,Z,U,NaiveMsg{X,Z}}
    num_agents::ℕ
    central::CentralControl{X,Z,U}
    msg_delay::ℕ
end

mutable struct NaiveController{X,Z,U} <: Controller{X,Z,U,NaiveMsg{X,Z}}
    self::ℕ
    central::CentralControl{X,Z,U}
end

function control!(
    ctrl::NaiveController{X,Z,U},
    x::X,
    z::Z,
    msgs::Each{NaiveMsg{X,Z}},
)::Tuple{U,Each{NaiveMsg}} where {X,Z,U}
    msgs[ctrl.self] = NaiveMsg(x, z)
    xs = [m.x for m in msgs]
    os = [m.z for m in msgs]
    u = control_one(ctrl.central, xs, os, ctrl.self)
    msgs′ = fill(NaiveMsg(x, z), length(msgs))
    u, msgs′
end

function make_controllers(
    framework::NaiveCF{X,Z,U},
    init_status::Each{Tuple{X,Z,U}}
)::Tuple{Each{NaiveController{X,Z,U}},Each{MsgQueue{NaiveMsg{X,Z}}}} where {X,Z,U}
    ctrls = [NaiveController(i, framework.central) for i in 1:framework.num_agents]
    init_msg() = begin
        receives = [NaiveMsg(x, z) for (x, z, u) in init_status]
        constant_queue(receives, framework.msg_delay) 
    end
    msgs = [init_msg() for _ in init_status]
    ctrls, msgs
end


end # NaiveCFs