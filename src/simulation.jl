export simulate, TrajectoryData, visualize

# using Plots: Plot, plot
using Makie
using AbstractPlotting.MakieLayout
using StaticArrays
using Unrolled
using Base.Iterators: drop
using ColorSchemes

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
    c_id = indexin([comp], data.components)[1]
    @assert c_id !== nothing "component '$comp' not found."
    data.values[:,:,(c_id::Int)]
end



"A generic result visualization function that draws multiple line plots, 
one for each component of the state vector.\n"
function visualize(
    result::TrajectoryData; delta_t = nothing
)::Scene
    if delta_t === nothing
        times = result.times
        xlabel = "time step"
    else
        times = result.times .* delta_t 
        xlabel = "time (s)"
    end
    ## currently assume all agents have the same type of states and observations
    labels = reshape(["agent $i" for i in 1:size(result.values, 2)], (1, :))
    n = length(result.components)
    scene, layout = layoutscene(resolution = (1200, 500 * n))
    axes = map(enumerate(result.components)) do (c_id, comp)
        ax = LAxis(scene; title = comp, xlabel)
        ys = result.values[:,:,c_id]
        N = size(ys, 2)
        for j in 1:N
            lines!(ax, times, ys[:,j], linewidth=2,
                color = get(ColorSchemes.rainbow, j / N))
        end
        layout[c_id,1] = ax
    end
    linkxaxes!(axes...)
    scene
end


function shorten_queue(q, len)
    @assert(len ≤ length(q), 
        "Actual communication delay ($len) should ≤ the modeled delay ($(length(q))).")
    to_drop = length(q) - len
    FixedQueue(collect(drop(collect(q), to_drop)))
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
"""
function simulate(
    world_dynamics::WorldDynamics{N},
    delay_model::DelayModel,
    framework::ControllerFramework{X,Z,U,Msg,Log},
    init_status::Each{Tuple{X,Z,U}},
    recorder::Tuple{Vector{String},Function},
    times::AbstractVector{𝕋},
)::Tuple{TrajectoryData,Dict{ℕ,Dict{𝕋,Log}}} where {X,Z,U,Msg,Log,N}
    @assert !isempty(times)

    result = TrajectoryData(times, N, recorder[1])
    data_idx = 1

    function callback(xs,zs,us,t)
        if t == times[data_idx]
            data = recorder[2](xs, zs, us)
            @assert size(data) == (N, length(recorder[1])) ("The recorder should "
               * "return a matrix of size (N * Num curves), got size $(size(data))")
            result.values[data_idx, :, :] = data
            data_idx += 1
        end
    end

    tspan = times[1], times[end]
    logs = simulate(world_dynamics, delay_model, framework, init_status, callback, tspan)
    result, logs
end
  

"""
Simulate multiple distributed agents with delayed communication.

# Arguments
- `callback`: a function of the form `callback(xs,zs,us,t) -> nothing` that runs at
every time step.
"""
function simulate(
    world_dynamics::WorldDynamics{N},
    delay_model::DelayModel,
    framework::ControllerFramework{X,Z,U,Msg,Log},
    init_status::Each{Tuple{X,Z,U}},
    callback::Function,
    tspan::Tuple{𝕋, 𝕋},
)::Dict{ℕ,Dict{𝕋,Log}} where {X,Z,U,Msg,Log,N}
    @assert isbitstype(Msg) "Msg = $Msg"

    t0, tf = tspan
    ΔT = delay_model.ΔT
    is_control_time(t) = mod(t - t0, ΔT) == 0
    is_obs_time(t) = mod(t + delay_model.obs - t0, ΔT) == 0
    is_act_time(t) = mod(t - delay_model.act - t0, ΔT) == 0
    is_msg_time(t) = mod(t - delay_model.com - t0, ΔT) == 0

    xs, zs, us = @unzip(MVector{N}(init_status), MVector{N}{Tuple{X,Z,U}})

    (controllers, msg_qs) = make_controllers(framework, init_status, t0)
    # the head of these queues corresponding to values that are currently taking effect
    msg_qs = shorten_queue.(msg_qs, delay_model.com ÷ ΔT + 1)
    state_qs = SVector{N}(constant_queue.(xs, delay_model.obs ÷ ΔT + 1))
    obs_qs = SVector{N}(constant_queue.(zs, delay_model.obs ÷ ΔT + 1))
    act_qs = SVector{N}(constant_queue.(us, delay_model.act ÷ ΔT + 1))

    # we store the newest messages/actions into these caches and wait until 
    # the next control/actuation step to push them into the queues.
    msg_cache = MMatrix{N,N,Msg}(hcat(first.(msg_qs)...)) # [sender, receiver]
    act_cache = MVector{N, U}(first.(act_qs))
    for t in t0:tf
        if is_obs_time(t)
            # we don't need cahces here since they will only be used in the next
            # control step anyway.
            pushpop!.(state_qs, xs)
            pushpop!.(obs_qs, zs)
        end
        if is_msg_time(t)
            for i in SOneTo(N)
                pushpop!(msg_qs[i], convert(Vector{Msg}, msg_cache[:, i]))
            end
        end
        if is_control_time(t)
            for i in SOneTo(N)
                x, z, ms = first.((state_qs[i], obs_qs[i], msg_qs[i]))
                u, ms′ = control!(controllers[i], x, z, ms)
                u = limit_control(world_dynamics.dynamics[i], u, x, t)
                msg_cache[i, :] = ms′
                act_cache[i] = u
            end
        end
        if is_act_time(t)
            pushpop!.(act_qs, act_cache)
            us .= first.(act_qs)
        end

        # record results
        callback(xs,zs,us,t)

        # update physics
        xs .= sys_forward.(world_dynamics.dynamics, xs, us, t)
        zs .= obs_forward.(world_dynamics.obs_dynamics, xs, zs, t)
    end
    logs = Dict(i => write_logs(controllers[i]) for i in 1:N)
    logs
end