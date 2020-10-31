
export simulate, SimulationResult, visualize

using Plots

"Results are indexed by (time, agent)"
@kwdef struct SimulationResult
    times::Vector{𝕋}
    xs::Matrix{State}
    zs::Matrix{Obs}
end

SimulationResult(times::Vector{𝕋}, num_agents::ℕ) = begin
    xs = Matrix{State}(undef, length(times), num_agents)
    zs = Matrix{Obs}(undef, length(times), num_agents)
    SimulationResult(times, xs, zs) 
end 

"A generic result visualization function that draws multiple line plots, 
one for each component of the state vector.\n"
visualize(result::SimulationResult, world::WorldDynamics; delta_t=nothing) = begin
    if delta_t === nothing
        times = result.times
        xlabel = "time step"
    else
        times = result.times .* delta_t 
        xlabel = "time (s)"
    end
    ## currently assume all agents have the same type of states and observations
    for (x_id, x_name) in enumerate(state_names(world.dynamics[1]))
        ys::Matrix{ℝ} = (x -> x[x_id]).(result.xs)
        labels = ["agent $i" for _ in 1:1, i in 1:size(ys, 2)]
        display(plot(times, ys; title=x_name, label=labels, xlabel))
    end
end

        
@kwdef struct AgentState{Msg}
    state_queue::Queue{State}
    obs_queue::Queue{Obs}
    act_queue::Queue{Act}
    msg_queue::MsgQueue{Msg}
    controller::Controller{Msg}
end
  

"""
Simulate multiple distributed agents with the given initial states and controllers and 
return the resulting trajectory as a `Simulation` struct.

# Arguments
- `times::Vector{𝕋}`: the time instances at which the simulation should save the states in 
the result. The first element in this sequence should be the time of the given initial states. 
"""
function simulate(
    world_dynamics::WorldDynamics,
    delay_model::DelayModel,
    framework::ControllerFramework{Msg},
    init_status::Each{Tuple{State,Obs,Act}},
    times::Vector{𝕋},
)::SimulationResult where Msg
    (controllers, msg_qs) = make_controllers(framework, init_status)
    
    make_agent(id::ℕ)::AgentState = begin
        (x₀, z₀, u₀) = init_status[id]
        AgentState(
            state_queue=constant_queue(x₀, delay_model.obs),
            obs_queue=constant_queue(z₀, delay_model.obs),
            act_queue=constant_queue(u₀, delay_model.act),
            msg_queue=msg_qs[id],
            controller=controllers[id],
        )
    end
    simulate_impl(
        world_dynamics,
        map(make_agent, 1:world_dynamics.num_agents),
        init_status,
        times,
    )
end

function simulate_impl(
    world_dynamics::WorldDynamics,
    agents::Each{AgentState{Msg}},
    init_status::Each{Tuple{State,Obs,Act}},
    times,
) where Msg
    @assert !isempty(times)
    N = world_dynamics.num_agents
    @assert length(agents) == N

    xs = first.(init_status)
    zs = (x -> x[2]).(init_status)
    result = SimulationResult(times, N)
    data_idx = 1
    for t in times[1]:times[end]
        # observe, control, and send messages
        transmition = Array{Msg}(undef, N, N)  # indexed by (receiver, sender)
        for (i, a) in enumerate(agents)
            enqueue!(a.state_queue, xs[i])
            enqueue!(a.obs_queue, zs[i])
            x = dequeue!(a.state_queue)
            z = dequeue!(a.obs_queue)
            ms = dequeue!(a.msg_queue)
            u, ms′ = control!(a.controller, x, z, ms)
            enqueue!(a.act_queue, u)
            transmition[:,i] = ms′
        end
        # receive messagees and update world states
        for (i, a) in enumerate(agents)
            enqueue!(a.msg_queue, transmition[i,:])
            u = dequeue!(a.act_queue)
            xs[i] = forward(world_dynamics.dynamics[i], xs[i], u, t)
            zs[i] = forward(world_dynamics.obs_dynamics[i], xs[i], zs[i], t)
        end
        # record results
        if t == times[data_idx]
            result.xs[data_idx,:] = xs
            result.zs[data_idx,:] = zs
            data_idx += 1
        end
    end
    result
end