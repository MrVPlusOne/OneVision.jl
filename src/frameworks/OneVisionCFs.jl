export OvCF, OvController, OvMsg

struct OvMsg{X,Z}
    x::X
    z::Z
    δx::X
    δz::Z
end


struct OvCF{N,X,Z,U} <: ControllerFramework{X,Z,U,OvMsg{X,Z}}
    central::CentralControl
    world_model::WorldDynamics
    delay_model::DelayModel
end

struct OvController{N,X,Z,U,H,Dy} <: Controller{X,Z,U,OvMsg{X,Z}}
    id::ℕ
    cf::OvCF{N,X,Z,U}
    time::Ref{𝕋}
    u_history::Queue{U}
    prob::ForwardPredictProblem{N,H,U,Dy}
    δx::MMatrix{H,N,X}
    δz::MMatrix{H,N,Z}
end # struct

function OneVision.make_controllers(
    cf::OvCF{N,X,Z,U},
    init_status::Each{Tuple{X,Z,U}},
    init_t::𝕋,
)::Tuple where {N,X,Z,U}
    dm = cf.delay_model
    function mk_controller(id)
        u_history = constant_queue(init_status[id][3], dm.obs + dm.act)
        OvController(id, cf, init_t, u_history)
    end

    function mk_q(_)
        receives = [OvMsg(x, z, zero(x), zero(z)) for (x, z, u) in init_status]
        constant_queue(receives, dm.com)
    end

    ctrls::NTuple{N,OvController} = ntuple(mk_controller, N)
    msg_qs::NTuple{N,MsgQueue} = ntuple(mk_q, N)
    ctrls, msg_qs
end

function OneVision.control!(
    ctrl::OvController{N,X,Z,U},
    x::X,
    z::Z,
    msgs::Each{OvMsg{X,Z}},
)::Tuple{U,Each{OvMsg{X,Z}}} where {N,X,Z,U}
    world = ctrl.cf.world_model
    delay::DelayModel = ctrl.cf.delay_model
    ctrl.t += 1

    x_self = self_estimate(
        world.dynamics[ctrl.id],
        (x, ctrl.t - delay.obs),
        ctrl.u_history
    )  # self state at t+Tᵘ

    
    forward_predict(ctrl.prob, ctrl.δx, ctrl.δz, s0,τ0)

    x_self, []
end
