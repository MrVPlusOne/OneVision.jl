# module ActHolderCFs

# export ActHolderCF, ActHolderMsg, control!, make_controllers

# using OneVision
# using OneVision: State, Obs, Act, ℕ, 𝕋
# import OneVision: control!, make_controllers

# struct Msg
#     x::State
#     z::Obs
#     u::Act
#     t::𝕋
# end

# const ActHolderMsg = Msg

# struct ActHolderCF <: ControllerFramework{Msg}
#     num_agents::ℕ
#     central::CentralControl
#     delay_model::DelayModel
#     world_model::WorldDynamics
# end

# mutable struct ActHolderController <: Controller{Msg}
#     self::ℕ
#     cf::ActHolderCF
#     u_history::Each{Queue{Tuple{𝕋,Act}}}
# end

# function control!(
#     ctrl::ActHolderController,
#     x::State,
#     z::Obs,
#     msgs::Each{Msg},
# )::Tuple{Act,Each{Msg}} 
#     systems = ctrl.cf.world_model.dynamics
#     function predict(id::ℕ, old_state::Msg, t_pred::𝕋)
#         x = old_state.x
#         for t in old_state.t:t_pred - 1
#             u = get(ctrl.u_history[id], t)
#             x = forward(systems[id], x, u, t)
#         end
#     end

#     msgs[ctrl.self] = Msg(x, z)
#     xs = [m.x for m in msgs]
#     os = [m.z for m in msgs]
#     u = control_one(ctrl.central, xs, os, ctrl.self)
#     msgs′ = fill(Msg(x, z), length(msgs))
#     u, msgs′
# end

# # function make_controllers(
# #     framework::ActHolderCF,
# #     init_status::Each{Tuple{State,Obs,Act}}
# # )::Tuple{Each{Controller{Msg}},Each{MsgQueue{Msg}}}
# #     ctrls = [NaiveController(i, framework.central) for i in 1:framework.num_agents]
# #     init_msg() = begin
# #         receives = [Msg(x, z) for (x, z, u) in init_status]
# #         constant_queue(receives, framework.msg_delay) 
# #     end
# #     msgs = [init_msg() for _ in init_status]
# #     ctrls, msgs
# # end


# end # ActHolderCFs