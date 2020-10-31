module NaiveCFs

export NaiveCF, control!, make_controllers

using OneVision
import OneVision: control!, make_controllers

struct NaiveMsg
    x::State
    z::Obs
end

struct NaiveCF <: ControllerFramework{NaiveMsg}
    num_agents::ℕ
    central::CentralControl
    msg_delay::ℕ
end

mutable struct NaiveController <: Controller{NaiveMsg}
    self::ℕ
    central::CentralControl
end

function control!(
    ctrl::NaiveController,
    x::State,
    z::Obs,
    msgs::Each{NaiveMsg},
)::Tuple{Act,Each{NaiveMsg}} 
    msgs[ctrl.self] = NaiveMsg(x, z)
    xs = [m.x for m in msgs]
    os = [m.z for m in msgs]
    u = control_one(ctrl.central, xs, os, ctrl.self)
    msgs′ = fill(NaiveMsg(x, z), length(msgs))
    u, msgs′
end

function make_controllers(
    framework::NaiveCF,
    init_status::Each{Tuple{State,Obs,Act}}
)::Tuple{Each{Controller{NaiveMsg}},Each{MsgQueue{NaiveMsg}}}
    ctrls = [NaiveController(i, framework.central) for i in 1:framework.num_agents]
    init_msg() = begin
        receives = [NaiveMsg(x, z) for (x, z, u) in init_status]
        constant_queue(receives, framework.msg_delay) 
    end
    msgs = [init_msg() for _ in init_status]
    ctrls, msgs
end


end # NaiveCFs