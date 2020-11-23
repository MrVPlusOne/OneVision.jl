module Car1DExample
export CarX, CarZ, CarU, car_system, WallObsDynamics

using OneVision
using OneVision: ℝ, 𝕋, ℕ, @kwdef
using Random
using StaticArrays
using Plots

import OneVision

struct CarX{R} <: FieldVector{2,R}
    pos::R
    velocity::R
end

struct CarU{R} <: FieldVector{1,R}
    acc::R
end

struct CarZ{R} <: FieldVector{2,R}
    detected::R
    distance::R
end


const car_A = @SMatrix [0.0 1.0; 0.0 0.0]
const car_B = @SMatrix [0.0; 1.0]


function car_system(delta_t::ℝ, noise::Function=_ -> CarX(0.0, 0.0))
    A′, B′ = discretize(car_A, car_B, delta_t)
    SysDynamicsLTI(A′, B′, noise)
end
    
"The wall observation model used for simulation.\n"
@kwdef struct WallObsDynamics <: ObsDynamics
    wall_position::Union{ℝ,Nothing}
    detector_range::ℝ
end

function OneVision.obs_forward(
    dy::WallObsDynamics, x::CarX, z::CarZ{R}, t::𝕋
)::CarZ{R} where R
    if Bool(z.detected)
        z
    elseif (isnothing(dy.wall_position) 
            || x.pos + dy.detector_range < dy.wall_position)
        [0.0, 0.0]
    else
        [1.0, dy.wall_position]
    end
end


"""
The simple wall dynamics model used by OneVision.
"""
struct WallObsModel <: ObsDynamics end

OneVision.obs_forward(dy::WallObsModel, x::CarX, z::CarZ, t::𝕋) = z

@kwdef struct LeaderFollowerControl <: CentralControl{CarU{ℝ}}
    warm_up_time::𝕋  # Will output u=0 before this time
    k_v::ℝ = 3.0
    k_x::ℝ = 2.0
    stop_distance::ℝ = 4.0
    target_v::ℝ = 2.0
end

OneVision.control_one(
    lf::LeaderFollowerControl, xs,zs, t::𝕋, id::ℕ
)::CarU{ℝ} = begin
    tol = 0.5
    function bang_bang(x̂, x, k, tol)
        if abs(x̂ - x) ≤ tol
            (x̂ - x) * k
        else
            sign(x̂ - x) * 3
        end
    end
    
    x, z = xs[id], zs[id]
    if t ≤ lf.warm_up_time
        acc = 0.0
    elseif Bool(z.detected) && z.distance - x.pos ≤ lf.stop_distance
        acc = bang_bang(0.0, x.velocity, lf.k_v, tol)
    elseif id == 1
        # the leader
        acc = bang_bang(lf.target_v, x.velocity, lf.k_v, tol)
    else
        # a follower
        leader = xs[1]
        acc = (control_one(lf, xs, zs, t, 1).acc # mimic the leader
                + bang_bang(leader.velocity, x.velocity, lf.k_v, Inf)
                + bang_bang(leader.pos, x.pos, lf.k_x, Inf))
    end
    CarU(acc)
end

function run_example(times, freq::ℝ; noise=0.0, plot_result=true, log_prediction=false)
    delta_t = 1 / freq
    times::Vector{𝕋} = collect(times)
    idx_to_time(xs) = (xs .- 1) .* delta_t
    t0, t_end = times[1], times[end]
    rng = MersenneTwister(1234)
    
    agent_info(id) = begin
        acc_noise = randn(rng, ℝ, 1 + t_end - t0) * noise
        sys_dy = car_system(delta_t, t -> CarX(0.0, acc_noise[1 + t - t0]))
        obs_dy = 
            if id == 1; WallObsDynamics(wall_position=30.0, detector_range=8.0)
            else WallObsModel() end
        
        sys_dy, obs_dy
    end
    
    N = 2
    world_dynamics = WorldDynamics([agent_info(i) for i in 1:N])
    init_states = fill((CarX(0.0, 0.0), CarZ(0.0, 0.0), CarU(0.0)), N)
    delays = DelayModel(obs=2, act=3, com=4)  # DelayModel(obs=1, act=2, com=5)
    world_model = WorldDynamics(
        fill((car_system(delta_t), WallObsModel()), N))

    comps = ["pos", "velocity", "acceleration", "obstacle"]
    function record_f(xs, zs, us)
        [(x -> x.pos).(xs) (x -> x.velocity).(xs) (u -> u.acc).(us) (z -> z.distance).(zs)]
    end
    
    H = 20
    πc = LeaderFollowerControl(warm_up_time=delays.act)
    result, logs = simulate(
        world_dynamics, 
        delays,
        # NaiveCF{CarX,CarZ,CarU}(N, πc, delays.com),
        let x_weights = fill(CarX(1.0, 1.0), 2), u_weights = fill(CarU(1.0), 2)
            OvCF{N,CarX{ℝ},CarZ{ℝ},CarU{ℝ},H}(
                πc, 
                world_model, delays, x_weights, u_weights,
                FuncT(Tuple{ℕ,𝕋,CarX{ℝ},CarZ{ℝ}}, Bool) do (id, t, _, _) 
                    log_prediction && mod1(t, 2) == 2 && 0.0 ≤ (t - 1) * delta_t ≤ 10.0
                end
            )  
        end,
        init_states,
        (comps, record_f),
        times,
    )
    if log_prediction
        t0, t1 = (0.0, 11.0)
        id = 1
        x_indices = [t for t in result.times if t0 ≤ (t - 1) * delta_t ≤ t1]
        sorted_log = sort!([x for x in logs[id]], by=x -> x[1])
        δv_x = idx_to_time((x -> x[1]).(sorted_log))
        anim = @gif for (t, log) in sorted_log
            ps = []
            for (c, label) in enumerate(["δx", "δv"])
                δv_y = [log.δxz[1][c] for (_, log) in sorted_log]
                p = plot(idx_to_time(x_indices), 
                        result.values[x_indices,:,c], label=["agent 1" "agent 2"])
                plot!(p, δv_x, δv_y, label=label)
                t -= 1
                local ts = idx_to_time(t:t + size(log.x̃, 1) - 1)
                plot!(p, ts, (x -> x[c]).(log.x̃); label=["central 1" "central 2"])
                push!(ps, p)
            end
            plot(ps...; layout=(2, 1))
        end
        anim |> display
    end
    plot_result ? visualize(result; delta_t) : result
end

end # Car1DExampleLabeled
