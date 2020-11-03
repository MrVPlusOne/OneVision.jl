export simulate, SimulationResult, visualize

using Plots: Plot, plot

"Results are indexed by (time, agent, component)"
@kwdef struct SimulationResult
    times::Vector{𝕋}
    values::Array{ℝ,3}
    components::Vector{String}
end

SimulationResult(times::Vector{𝕋}, num_agents::ℕ, components::Vector{String}) = begin
    values = Array{ℝ}(undef, length(times), num_agents, length(components))
    SimulationResult(times, values, components) 
end 

"A generic result visualization function that draws multiple line plots, 
one for each component of the state vector.\n"
function visualize(
    result::SimulationResult; delta_t=nothing
)::Vector{Plot}
    if delta_t === nothing
        times = result.times
        xlabel = "time step"
    else
        times = result.times .* delta_t 
        xlabel = "time (s)"
    end
    ## currently assume all agents have the same type of states and observations
    labels = reshape(["agent $i" for i in 1:size(result.values, 2)], (1, :))
    map(enumerate(result.components)) do (c_id, comp)
        ys = result.values[:,:,c_id]
        plot(times, ys; title=comp, label=labels, xlabel)
    end
end

        
@kwdef struct AgentState{X,Z,U,Msg}
    state_queue::Queue{X}
    obs_queue::Queue{Z}
    act_queue::Queue{U}
    msg_queue::MsgQueue{Msg}
    controller::Controller{X,Z,U,Msg}
end
  

"""
Simulate multiple distributed agents with the given initial states and controllers and 
return the resulting trajectory as a `Simulation` struct.

# Arguments
- `times::Vector{𝕋}`: the time instances at which the simulation should save the states in 
the result. The first element in this sequence should be the time of the given initial states. 
- `recorder`: a tuple of the shape `(components::Vector{String}, record_f)`, where 
   `record_f` should takes in `(xs, zs, us)` and returns a real matrix of the size 
   `(num agents, length(components))`.
"""
function simulate(
    world_dynamics::WorldDynamics,
    delay_model::DelayModel,
    framework::ControllerFramework{X,Z,U,Msg},
    init_status::Each{Tuple{X,Z,U}},
    recorder::Tuple{Vector{String},Function},
    times::Vector{𝕋},
)::SimulationResult where {X,Z,U,Msg}
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
        recorder,
        times,
    )
end

function simulate_impl(
    world_dynamics::WorldDynamics,
    agents::Each{AgentState{X,Z,U,Msg}},
    init_status::Each{Tuple{X,Z,U}},
    recorder::Tuple{Vector{String},Function},
    times,
) where {X,Z,U,Msg}
    @assert !isempty(times)
    N = world_dynamics.num_agents
    @assert length(agents) == N

    xs, zs, us = unzip(init_status)
    result = SimulationResult(times, N, recorder[1])
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
            us[i] = u
            xs[i] = sys_forward(world_dynamics.dynamics[i], xs[i], u, t)
            zs[i] = obs_forward(world_dynamics.obs_dynamics[i], xs[i], zs[i], t)
        end
        # record results
        if t == times[data_idx]
            data = recorder[2](xs, zs, us)
            @assert size(data) == (N, length(recorder[1])) ("The recorder should "
               * "return a matrix of size (N * Num curves), got size $(size(data))")
            result.values[data_idx, :, :] = data
            data_idx += 1
        end
    end
    result
end