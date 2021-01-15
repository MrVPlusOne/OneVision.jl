export simulate, TrajectoryData, visualize, ros_simulate

import Plots
if false
    using GLMakie
else
    using WGLMakie
end
using AbstractPlotting.MakieLayout
using StaticArrays
using Unrolled
using Base.Iterators: drop
using ColorSchemes
using Sockets
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

"""
A generic result visualization function that draws multiple line plots, 
one for each component of the state vector.
"""
function visualize(
    result::TrajectoryData; delta_t = nothing, loss = nothing
)::Plots.Plot

    if delta_t === nothing
        times = result.times
        xlabel = "time step"
    else
        times = result.times .* delta_t 
        xlabel = "time (s)"
    end
    ## currently assume all agents have the same type of states and observations
    N = size(result.values, 2)
    labels = reshape(["agent $i" for i in 1:N], (1, :))
    n = length(result.components)
    colors = [get(ColorSchemes.rainbow, j / N) for j in 1:N]
    ps = map(enumerate(result.components)) do (c_id, comp)
        ys = result.values[:,:,c_id]
        Plots.plot(times, ys, label=labels, title=comp)
    end

    if loss !== nothing
        let ts = collect(AxisArrays.axes(loss)[1])
            (delta_t !== nothing) && (ts = ts .* delta_t)
            ps2 = map([(:u, "Action"), (:x, "State")]) do (label, name)
                ys = collect(loss[comp=label])
                Plots.plot(ts, ys, label=name, title=name)
            end
            append!(ps, ps2)
        end
    end
    
    #Plots.plotlyjs()
    Plots.plot(ps..., layout=(length(ps), 1), size=(500, length(ps) * 200), legend=false)
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
- `callback`: a function of the form `callback(xs,zs,us,loss,t) -> nothing` that runs at
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
    @assert tf ≥ t0
    @unpack Tx, Tu, Tc = short_delay_names(delay_model)
    @assert Tc > 0

    if loss_model !== nothing
        # the *ideal* state and observation at the current time step
        xs_idl, _, us_idl = @unzip(MVector{N}(init_status), MVector{N}{Tuple{X,Z,U}})
        @unpack central, x_weights, u_weights = loss_model
        modeled_dynamics = loss_model.world_model
        s_c = init_state(central, t0)
        loss_cache = AxisArray(-ones(ℝ, N, 2), id=1:N, comp=[:u, :x])
    else
        loss_cache = nothing
    end

    # the *actual* state, observation, and actuation at the current time step
    xs, zs, us = @unzip(MVector{N}(init_status), MVector{N}{Tuple{X,Z,U}})

    (controllers, msg_qs) = make_controllers(framework, init_status, t0)
    # the head of these queues corresponding to values that are currently taking effect
    msg_qs = shorten_queue.(msg_qs, Tc)
    state_qs = SVector{N}(constant_queue.(xs, Tx))
    obs_qs = SVector{N}(constant_queue.(zs, Tx))
    act_qs = SVector{N}(constant_queue.(us, Tu))

    # we store the newest messages/actions into these caches and wait until 
    # the next control/actuation step to push them into the queues to take effect.
    msg_cache = SizedMatrix{N,N,Msg}(hcat(first.(msg_qs)...)) # [sender, receiver]
    act_cache = MVector{N, U}(us)
    for t in t0:tf
        xs_read = pushpop!.(state_qs, xs_observer(xs, t))
        zs_read = pushpop!.(obs_qs, zs_observer(zs, t))
        # run controllers
        for i in SOneTo(N)
            x, z = xs_read[i], zs_read[i]
            ms = first(msg_qs[i])
            u, ms′ = control!(controllers[i], x, z, ms)
            u = limit_control(world_dynamics.dynamics[i], u, x, t)
            msg_cache[i, :] = ms′
            act_cache[i] = u # from c++
        end
        us .= pushpop!.(act_qs, act_cache)
        for i in SOneTo(N)
            pushpop!(msg_qs[i], convert(Vector{Msg}, msg_cache[:, i]))
        end

        # record loss
        if loss_model !== nothing
            us_idl .= control_all(central, s_c, xs_idl, zs, t, SOneTo(N))
            us_idl .= limit_control.(modeled_dynamics.dynamics, us_idl, xs_idl, t)
            for i in 1:N
                x_loss = sum((xs[i] .- xs_idl[i]).^2 .* x_weights[i])
                u_loss = sum((collect(us[i]) .- us_idl[i]).^2 .* u_weights[i])
                loss_cache[id = i, comp=:u] = u_loss
                loss_cache[id = i, comp=:x] = x_loss
            end
        end

        # record results
        callback(xs, zs, us, loss_cache, t)

        # update physics
        xs1 = sys_forward.(world_dynamics.dynamics, xs, us, t)
        zs1 = obs_forward.(world_dynamics.obs_dynamics, xs, zs, t)
        if loss_model !== nothing
            xs2 = sys_forward.(modeled_dynamics.dynamics, xs, us, t)
            xs_idl .= sys_forward.(modeled_dynamics.dynamics, xs_idl, us_idl, t) .+ (xs1.-xs2)
            xs .= xs1
        end
        xs .= xs1
        zs .= zs1
        # add step - sim - send next comd, actual is dummy
    end
    logs = Dict(i => write_logs(controllers[i]) for i in 1:N)
    return logs
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
    loss_model::Union{RegretLossModel, Nothing} = nothing,
    kwargs...
) where {X,Z,U,Msg,Log,N}
    @assert !isempty(times)

    comps, record_f = recorder
    result = TrajectoryData(times, N, comps)
    data_idx = 1
    t0, tf = times[1], times[end]
    if loss_model !== nothing
        loss_history = AxisArray(-ones(ℝ, tf-t0+1,  N, 2), time=t0:tf, id=1:N, comp=[:u, :x])
    end

    function callback(xs,zs,us,loss,t)
        if t == times[data_idx]
            data = record_f(xs, zs, us)
            @assert size(data) == (N, length(comps)) ("The recorder should "
               * "return a matrix of size (N * Num curves), got size $(size(data))")
            result.values[data_idx, :, :] = data
            data_idx += 1
        end
        if loss_model !== nothing
            @assert loss !== nothing
            loss_history[atvalue(t), :, :] = loss
        end
    end

    logs = simulate(world_dynamics, delay_model, framework, init_status, 
        callback, (t0, tf); loss_model, kwargs...)
    sim_out = (loss_model !== nothing) ? (logs, loss_history) : logs
    
    result, sim_out
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
function ros_simulate(
    world_dynamics::WorldDynamics{N},
    delay_model::DelayModel,
    framework::ControllerFramework{X,Z,U,Msg,Log},
    init_status::Each{Tuple{X,Z,U}},
    recorder::Tuple{Vector{String},Function},
    times::AbstractVector{𝕋};
    loss_model::Union{RegretLossModel, Nothing} = nothing,
    kwargs...
) where {X,Z,U,Msg,Log,N}
    @assert !isempty(times)

    comps, record_f = recorder
    result = TrajectoryData(times, N, comps)
    data_idx = 1
    t0, tf = times[1], times[end]
    if loss_model !== nothing
        loss_history = AxisArray(-ones(ℝ, tf-t0+1,  N, 2), time=t0:tf, id=1:N, comp=[:u, :x])
    end

    function callback(xs,zs,us,loss,t)
        if t == times[data_idx]
            data = record_f(xs, zs, us)
            @assert size(data) == (N, length(comps)) ("The recorder should "
               * "return a matrix of size (N * Num curves), got size $(size(data))")
            result.values[data_idx, :, :] = data
            data_idx += 1
        end
        if loss_model !== nothing
            @assert loss !== nothing
            loss_history[atvalue(t), :, :] = loss
        end
    end

    logs = ros_simulate(world_dynamics, delay_model, framework, init_status, 
        callback, (t0, tf); loss_model, kwargs...)
    sim_out = (loss_model !== nothing) ? (logs, loss_history) : logs
    
    result, sim_out
end

"""
[WIP] Utilize ROS simulator
"""
function ros_simulate(
    world_dynamics::WorldDynamics{N},
    delay_model::DelayModel,
    framework::ControllerFramework{X,Z,U,Msg,Log},
    init_status::Each{Tuple{X,Z,U}},
    callback::Function,
    (t0, tf)::Tuple{𝕋, 𝕋};
    loss_model::Union{RegretLossModel, Nothing} = nothing,
    xs_observer = (xs, t) -> xs,
    zs_observer = (zs, t) -> zs,
    port::Integer = 5001,
) where {X,Z,U,Msg,Log,N}
    @assert tf ≥ t0
    @unpack Tx, Tu, Tc = short_delay_names(delay_model)
    @assert Tc > 0

    # initialize server for ROS communication
    ros_conn = connect(port)
    

    if loss_model !== nothing
        # the *ideal* state and observation at the current time step
        xs_idl, _, us_idl = @unzip(MVector{N}(init_status), MVector{N}{Tuple{X,Z,U}})
        @unpack central, x_weights, u_weights = loss_model
        modeled_dynamics = loss_model.world_model
        s_c = init_state(central, t0)
        loss_cache = AxisArray(-ones(ℝ, N, 2), id=1:N, comp=[:u, :x])
    else
        loss_cache = nothing
    end

    # the *actual* state, observation, and actuation at the current time step
    xs, zs, us = @unzip(MVector{N}(init_status), MVector{N}{Tuple{X,Z,U}})

    (controllers, msg_qs) = make_controllers(framework, init_status, t0)
    # the head of these queues corresponding to values that are currently taking effect
    msg_qs = shorten_queue.(msg_qs, Tc)
    state_qs = SVector{N}(constant_queue.(xs, 0))
    obs_qs = SVector{N}(constant_queue.(zs, 0))
    act_qs = SVector{N}(constant_queue.(us, 0))

    # we store the newest messages/actions into these caches and wait until 
    # the next control/actuation step to push them into the queues to take effect.
    msg_cache = SizedMatrix{N,N,Msg}(hcat(first.(msg_qs)...)) # [sender, receiver]
    act_cache = MVector{N, U}(us)
    for t in t0:tf
        xs_read = pushpop!.(state_qs, xs_observer(xs, t))
        zs_read = pushpop!.(obs_qs, zs_observer(zs, t))
        # run controllers
        for i in SOneTo(N)
            x, z = xs_read[i], zs_read[i]
            ms = first(msg_qs[i])
            println("message is $ms \n")
            println("type is", typeof(ms))
            u, ms′ = control!(controllers[i], x, z, ms)
            u = limit_control(world_dynamics.dynamics[i], u, x, t)
            msg_cache[i, :] = ms′
            act_cache[i] = u
        end
        us .= pushpop!.(act_qs, act_cache)
        for i in SOneTo(N)
            pushpop!(msg_qs[i], convert(Vector{Msg}, msg_cache[:, i]))
        end

        # record loss
        if loss_model !== nothing
            us_idl .= control_all(central, s_c, xs_idl, zs, t, SOneTo(N))
            us_idl .= limit_control.(modeled_dynamics.dynamics, us_idl, xs_idl, t)
            for i in 1:N
                x_loss = sum((xs[i] .- xs_idl[i]).^2 .* x_weights[i])
                u_loss = sum((collect(us[i]) .- us_idl[i]).^2 .* u_weights[i])
                loss_cache[id = i, comp=:u] = u_loss
                loss_cache[id = i, comp=:x] = x_loss
            end
        end

        # record results
        callback(xs, zs, us, loss_cache, t)

        # update physics 
        @assert isopen(ros_conn) # make sure connection is still open
        xs1, zs1 = send_and_get_states(ros_conn, xs, us, zs, t)
        #xs1 = ros_sys_forward.(ross_conn, world_dynamics.dynamics, xs, us, t)
        #zs1 = obs_forward.(world_dynamics.obs_dynamics, xs, zs, t)

        if loss_model !== nothing
            xs2 = sys_forward.(modeled_dynamics.dynamics, xs, us, t)
            xs_idl .= sys_forward.(modeled_dynamics.dynamics, xs_idl, us_idl, t) .+ (xs1.-xs2)
            xs .= xs1
        end
        xs .= xs1
        zs .= zs1
    end
    logs = Dict(i => write_logs(controllers[i]) for i in 1:N)
    return logs
end

