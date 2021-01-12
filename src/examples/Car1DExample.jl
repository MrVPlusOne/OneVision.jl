module Car1DExample
export CarX, CarZ, CarU, car_system, WallObsDynamics, LeaderFollowerControl

using OneVision
using OneVision: ℝ, 𝕋, ℕ, @kwdef, @unpack, @asserteq
using OneVision.Examples
using Random
using StaticArrays
using Plots
using Statistics: mean

import OneVision

@kwdef struct CarX{R} <: FieldVector{3,R}
    pos::R
    velocity::R
    "Acceleration from the last time step"
    acc::R
end

StaticArrays.similar_type(::Type{CarX{A}}, ::Type{B}) where {A, B} = CarX{B}

struct CarU{R} <: FieldVector{1,R}
    acc::R
end

StaticArrays.similar_type(::Type{CarU{A}}, ::Type{B}) where {A, B} = CarU{B}

CarU{X}(x::Tuple{X}) where X = CarU(x[1])

struct CarZ{R}
    detected::Bool
    distance::R
end

struct CarSystem{Noise} <: SysDynamics
    dt::ℝ
    v_noise::Noise
end


function car_system(dt::ℝ, v_noise::Function = _ -> 0.0)
    CarSystem(dt, v_noise)
end

function OneVision.sys_forward(dy::CarSystem, x::CarX{R}, u, t::𝕋)::CarX{R} where R
    @unpack dt, v_noise = dy
    CarX(
        pos = x.pos + x.velocity * dt + 0.5 * u.acc * dt^2, 
        velocity = x.velocity + u.acc * dt + v_noise(t), 
        acc = u.acc,
    )
end
    
"The wall observation model used for simulation.\n"
@kwdef struct WallObsDynamics <: ObsDynamics
    wall_position::Union{ℝ,Nothing}
    detector_range::ℝ
end

function OneVision.obs_forward(
    dy::WallObsDynamics, x::CarX, z::CarZ{R}, t::𝕋
)::CarZ{R} where R
    if z.detected
        z
    elseif (isnothing(dy.wall_position) 
            || x.pos + dy.detector_range < dy.wall_position)
        CarZ(false, 0.0)
    else
       CarZ(true, dy.wall_position)
    end
end


"""
The simple wall dynamics model used by OneVision.
"""
struct WallObsModel <: ObsDynamics end

OneVision.obs_forward(dy::WallObsModel, x::CarX, z::CarZ, t::𝕋) = z

"""
A controller struct - holds all necessary information 
"""
@kwdef struct LeaderFollowerControl <: CentralControlStateless{CarU{ℝ}}
    "Will output u=0 before this time"
    warm_up_time::𝕋  
    "Will only use bang bang control when difference is larger than this"
    bang_bang_tol::ℝ = Inf
    k_v::ℝ = 3.0
    k_x::ℝ = 2.0
    stop_distance::ℝ = 4.0
    target_v::ℝ = 2.0
end

OneVision.control_one(
    lf::LeaderFollowerControl, xs, zs, t::𝕋, id::ℕ
)::CarU{ℝ} = begin
    tol = lf.bang_bang_tol
    function bang_bang(x̂, x, k, tol)
        if abs(x̂ - x) ≤ tol
            (x̂ - x) * k
        else
            sign(x̂ - x) * tol * k
        end
    end
    
    x, z = xs[id], zs[id]
    if t ≤ lf.warm_up_time
        acc = 0.0
    elseif z.detected && z.distance - x.pos ≤ lf.stop_distance
        acc = bang_bang(0.0, x.velocity, lf.k_v, tol)
    elseif id == 1
        # the leader
        acc = bang_bang(lf.target_v, x.velocity, lf.k_v, tol)
    else
        # a follower
        leader = xs[1]
        acc = (leader.acc # mimic the leader
                + bang_bang(leader.velocity, x.velocity, lf.k_v, Inf)
                + bang_bang(leader.pos, x.pos, lf.k_x, Inf))
    end
    CarU(acc)
end

"""
Compute the time-averaged distance between the leader and the follower 
using a quadratic norm, i.e., √(1/T ∫ (x_1 - x_2)^2 dt) .
"""
function avg_distance(result::TrajectoryData)::ℝ
    pos_data = result["pos"]
    @asserteq size(pos_data, 2) 2 "there should only be 2 agents"
    dis = (view(pos_data, :, 1) .- view(pos_data, :, 2)).^2
    √(mean(dis))
end

function run_example(; 
        setting::ExampleSetting,
        CF::CFName = onevision_cf,
        has_obstacle = true, use_bang_bang = true,
        seed = 1, 
        plot_result = true, log_prediction = false)
    @unpack time_end, freq, noise, sensor_noise, delays, H = setting  #TODO: unpack more
    t_end = 𝕋(ceil(time_end * freq))
    delta_t = 1 / freq
    times::Vector{𝕋} = collect(1:t_end)
    idx_to_time(xs) = (xs .- 1) .* delta_t
    rng = MersenneTwister(seed)
    
    agent_info(id) = begin
        v_noise = randn(rng, ℝ, t_end) * noise
        sys_dy = car_system(delta_t, t -> v_noise[t])
        obs_dy = 
            if id == 1 && has_obstacle; WallObsDynamics(wall_position = 30.0, detector_range = 8.0)
            else WallObsModel() end
        
        sys_dy, obs_dy
    end
    
    N = 2
    world_dynamics = WorldDynamics([agent_info(i) for i in 1:N])
    init_states = fill((CarX(0.0, 0.0, 0.0), CarZ(false, 0.0), CarU(0.0)), N)
    ΔT = delays.ΔT
    world_model = WorldDynamics(
        fill((car_system(delta_t), WallObsModel()), N))
    function xs_observer(xs, t)
        (x -> x + randn(rng, CarX{ℝ}) * sensor_noise).(xs)
    end

    comps = ["pos", "velocity", "acceleration", "obstacle"]
    function record_f(xs, zs, us)
        [(x -> x.pos).(xs) (x -> x.velocity).(xs) (u -> u.acc).(us) (z -> z.distance).(zs)]
    end
    
    central = LeaderFollowerControl(
        warm_up_time = delays.act, 
        bang_bang_tol = use_bang_bang ? 1.0 : Inf)
    x_weights = SVector{N}(fill(CarX(100.0, 10.0, 0.0), N))
    u_weights = SVector{N}(fill(CarU(1.0), N))
    loss_model = RegretLossModel(central, world_model, x_weights, u_weights)
    framework = mk_cf(
        CF, world_model, central, delays, loss_model; 
        X = CarX{ℝ}, Z = CarZ{ℝ}, H)

    result, (logs, loss) = simulate(
        world_dynamics, 
        delays,
        framework,
        init_states,
        (comps, record_f),
        times;
        loss_model,
        xs_observer,
    )

    if log_prediction
        # this is out-dated
        t0, t1 = (0.0, 11.0)
        id = 1
        x_indices = [t for t in result.times if t0 ≤ (t - 1) * delta_t ≤ t1]
        sorted_log = sort!([x for x in logs[id]], by = x -> x[1])
        δv_x = idx_to_time((x -> x[1]).(sorted_log))
        anim = @gif for (t, log) in sorted_log
            ps = []
            for (c, label) in enumerate(["δx", "δv"])
                δv_y = [log.δxz[1][c] for (_, log) in sorted_log]
                p = plot(idx_to_time(x_indices), 
                        result.values[x_indices,:,c], label = ["agent 1" "agent 2"])
                plot!(p, δv_x, δv_y, label = label)
                t -= 1
                local ts = idx_to_time(t:t + size(log.x̃, 1) - 1)
                plot!(p, ts, (x -> x[c]).(log.x̃); label = ["central 1" "central 2"])
                push!(ps, p)
            end
            plot(ps...; layout = (2, 1))
        end
        anim |> display
    end
    plot_result && display(visualize(result; delta_t, loss))
    
    loss, Dict("avg distance" => avg_distance(result))
end

end # Car1DExampleLabeled
