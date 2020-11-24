export simulate, TrajectoryData, visualize, AgentState

using Plots: Plot, plot
using StaticArrays
using Unrolled

@kwdef struct TrajectoryData
    times::Vector{𝕋}
    "values are indexed by (time, agent, component)"
    values::Array{ℝ,3}
    components::Vector{String}
    function TrajectoryData(times::AbstractVector{𝕋}, num_agents::ℕ, components::Vector{String})
        values = Array{ℝ}(undef, length(times), num_agents, length(components))
        new(times, values, components) 
    end
end

function Base.getindex(data::TrajectoryData, comp::String)
    c_id = indexin([comp],data.components)[1]
    @assert c_id !== nothing "component '$comp' not found."
    data.values[:,:,(c_id::Int)]
end



"A generic result visualization function that draws multiple line plots, 
one for each component of the state vector.\n"
function visualize(
    result::TrajectoryData; delta_t=nothing
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

        
@kwdef struct AgentState{X,Z,U,Msg,Log}
    state_queue::FixedQueue{X}
    obs_queue::FixedQueue{Z}
    act_queue::FixedQueue{U}
    msg_queue::MsgQueue{Msg}
    controller::Controller{X,Z,U,Msg,Log}
end
  

"""
Simulate multiple distributed agents with the given initial states and controllers and 
return `(trajectory data, snapshots)`.

# Arguments
- `times::Vector{𝕋}`: the time instances at which the simulation should save the states in 
the result. The first element in this sequence should be the time of the given initial states. 
- `recorder`: a tuple of the shape `(components::Vector{String}, record_f)`, where 
   `record_f` should takes in `(xs, zs, us)` and returns a real matrix of the size 
   `(num agents, length(components))`.
- `save_snapshot`: takes in `(id, time, x, z, u)` and returns a bool to indicate whether to
take a detailed snapshot for the current controller status.
"""
function simulate(
    world_dynamics::WorldDynamics{N},
    delay_model::DelayModel,
    framework::ControllerFramework{X,Z,U,Msg,Log},
    init_status::Each{Tuple{X,Z,U}},
    recorder::Tuple{Vector{String},Function},
    times::AbstractVector{𝕋},
)::Tuple{TrajectoryData,Dict{ℕ,Dict{𝕋,Log}}} where {X,Z,U,Msg,Log,N}
    (controllers, msg_qs) = make_controllers(framework, init_status, times[1])
    
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
        map(make_agent, SVector{N}(1:N)),
        SVector{N}(init_status),
        recorder,
        times,
    )
end

"""
A sumulation implementation. It takes in information regarding the simulation and
return `(results, logs)`.

# Arguments

"""
function simulate_impl(
    world_dynamics::WorldDynamics{N},
    agents::SVector{N,AgentState{X,Z,U,Msg,Log}},
    init_status::SVector{N,Tuple{X,Z,U}},
    recorder::Tuple{Vector{String},RF},
    times,
) where {X,Z,U,Msg,Log,RF,N}
    @assert !isempty(times)
    @assert isbitstype(Msg) "Msg = $Msg"
    Each = MVector{N}

    xs, zs, us = @unzip(MVector(init_status), Each{Tuple{X,Z,U}})
    result = TrajectoryData(times, N, recorder[1])
    data_idx = 1
    for t in times[1]:times[end]
        # observe, control, and send messages
        transmition = MMatrix{N,N,Msg}(undef)  # indexed by (receiver, sender)
        for i in 1:length(agents)  # unroll away dynamic dispatch at compile time!
            a = agents[i]
            x = pushpop!(a.state_queue, xs[i])
            z = pushpop!(a.obs_queue, zs[i])
            ms = first(a.msg_queue)
            u, ms′ = control!(a.controller, x, z, ms)
            us[i] = pushpop!(a.act_queue, u)
            transmition[:,i] = ms′
        end
        # receive messagees and update world states
        for i in 1:length(agents)
            a = agents[i]
            pushpop!(a.msg_queue, convert(Vector{Msg}, transmition[i,:]))
            xs[i] = sys_forward(world_dynamics.dynamics[i], xs[i], us[i], t)
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
    logs = Dict(i => write_logs(agents[i].controller) for i in 1:N)
    result, logs
end