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
    result::TrajectoryData; delta_t = nothing, loss = nothing,
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
        layout[c_id, 1] = ax
    end

    if loss !== nothing
        let ts = collect(AxisArrays.axes(loss)[1])
            (delta_t !== nothing) && (ts = ts .* delta_t)
            for (label, name) in ((:u, "Action"), (:x, "State"))
                ys = collect(loss[comp=label])
                ax = LAxis(scene; title = "$name Loss", xlabel)
                N = size(ys, 2)
                for j in 1:N
                    lines!(ax, ts, ys[:,j], linewidth=2,
                    color = get(ColorSchemes.rainbow, j / N))
                end
                layout[end+1, 1] = ax
                push!(axes, ax)
            end
        end
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
Simulate multiple distributed agents with delayed communication.

Returns `(logs, loss_history)` or just `logs` if `loss_model` is `nothing`.

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
    (t0, tf)::Tuple{𝕋, 𝕋};
    loss_model::Union{RegretLossModel, Nothing} = nothing,
    xs_observer = (xs, t) -> xs,
    zs_observer = (zs, t) -> zs,
) where {X,Z,U,Msg,Log,N}
    @assert isbitstype(Msg) "Msg = $Msg"
    @assert tf ≥ t0

    ΔT = delay_model.ΔT
    is_control_time(t) = mod(t - t0, ΔT) == 0
    is_obs_time(t) = mod(t + delay_model.obs - t0, ΔT) == 0
    is_act_time(t) = mod(t - delay_model.act - t0, ΔT) == 0
    is_msg_time(t) = mod(t - delay_model.com - t0, ΔT) == 0

    if loss_model !== nothing
        # the *ideal* state and observation at the current time step
        xs_idl, zs_idl, us_idl = @unzip(MVector{N}(init_status), MVector{N}{Tuple{X,Z,U}})
        @unpack central, x_weights, u_weights = loss_model
        modeled_dynamics = loss_model.world_model
        s_c = init_state(central, t0)
        loss_history = AxisArray(-ones(ℝ, tf-t0+1, N, 2), time=t0:tf, id=1:N, comp=[:u, :x])
    end

    # the *actual* state, observation, and actuation at the current time step
    xs, zs, us = @unzip(MVector{N}(init_status), MVector{N}{Tuple{X,Z,U}})

    (controllers, msg_qs) = make_controllers(framework, init_status, t0)
    # the head of these queues corresponding to values that are currently taking effect
    msg_qs = shorten_queue.(msg_qs, delay_model.com ÷ ΔT + 1)
    state_qs = SVector{N}(constant_queue.(xs, delay_model.obs ÷ ΔT + 1))
    obs_qs = SVector{N}(constant_queue.(zs, delay_model.obs ÷ ΔT + 1))
    act_qs = SVector{N}(constant_queue.(us, delay_model.act ÷ ΔT + 1))

    # we store the newest messages/actions into these caches and wait until 
    # the next control/actuation step to push them into the queues to take effect.
    msg_cache = MMatrix{N,N,Msg}(hcat(first.(msg_qs)...)) # [sender, receiver]
    act_cache = MVector{N, U}(first.(act_qs))
    for t in t0:tf
        if is_obs_time(t)
            # we don't need cahces here since they will only be used in the next
            # control step anyway.
            pushpop!.(state_qs, xs_observer(xs, t))
            pushpop!.(obs_qs, zs_observer(zs, t))
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

        # record loss
        if loss_model !== nothing
            if is_act_time(t)
                us_idl .= control_all(central, s_c, xs_idl, zs_idl, t, SOneTo(N))
            end
            for i in 1:N
                x_loss = sum((xs[i] .- xs_idl[i]).^2 .* x_weights[i])
                u_loss = sum((collect(us[i]) .- us_idl[i]).^2 .* u_weights[i])
                loss_history[time=atvalue(t), id = i, comp=:u] = u_loss
                loss_history[time=atvalue(t), id = i, comp=:x] = x_loss
            end
        end

        # update physics
        if loss_model === nothing
            xs .= sys_forward.(world_dynamics.dynamics, xs, us, t)
            zs .= obs_forward.(world_dynamics.obs_dynamics, xs, zs, t)
        else
            xs1 = sys_forward.(world_dynamics.dynamics, xs, us, t)
            zs1 = obs_forward.(world_dynamics.obs_dynamics, xs, zs, t)
            xs2 = sys_forward.(modeled_dynamics.dynamics, xs, us, t)
            zs2 = obs_forward.(modeled_dynamics.obs_dynamics, xs, zs, t)
            xs_idl .= sys_forward.(modeled_dynamics.dynamics, xs_idl, us_idl, t) .+ (xs1.-xs2)
            zs_idl .= obs_forward.(modeled_dynamics.obs_dynamics, xs_idl, zs_idl, t) .+ (zs1-zs2)
            xs .= xs1
            zs .= zs1
        end
    end
    logs = Dict(i => write_logs(controllers[i]) for i in 1:N)
    if loss_model === nothing
        return logs
    else
        return logs, loss_history
    end
end

"""
Simulate multiple distributed agents with the given initial states and controllers and 
return `(trajectory data, (logs [, loss_history]))`.

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
    times::AbstractVector{𝕋};
    kwargs...
) where {X,Z,U,Msg,Log,N}
    @assert !isempty(times)

    comps, record_f = recorder
    result = TrajectoryData(times, N, comps)
    data_idx = 1
    tspan = times[1], times[end]

    function callback(xs,zs,us,t)
        if t == times[data_idx]
            data = record_f(xs, zs, us)
            @assert size(data) == (N, length(comps)) ("The recorder should "
               * "return a matrix of size (N * Num curves), got size $(size(data))")
            result.values[data_idx, :, :] = data
            data_idx += 1
        end
    end

    sim_out = simulate(world_dynamics, delay_model, framework, init_status, 
        callback, tspan; kwargs...)
    
    result, sim_out
end