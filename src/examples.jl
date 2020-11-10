module Car1DExample
export CarX, CarZ, CarU, car_system, WallObsDynamics

using OneVision
using OneVision: ℝ, 𝕋, ℕ, @kwdef
using Random
using StaticArrays

import OneVision

struct CarX <: FieldVector{2,ℝ}
    pos::ℝ
    velocity::ℝ
end

struct CarU <: FieldVector{1,ℝ}
    acc::ℝ
end

struct CarZ <: FieldVector{2,ℝ}
    detected::ℝ
    distance::ℝ
end


const car_A = @SMatrix [0.0 1.0; 0.0 0.0]
const car_B = @SMatrix [0.0; 1.0]


function car_system(delta_t::ℝ, noise::Function)
    A′, B′ = discretize(car_A, car_B, delta_t)
    SysDynamicsLTI(A′, B′, noise)
end
    
car_system(delta_t::ℝ) = car_system(delta_t, _ -> CarX(0.0, 0.0))

@kwdef struct WallObsDynamics <: ObsDynamics
    wall_position::Union{ℝ,Nothing}
    detector_range::ℝ
end

OneVision.obs_forward(dy::WallObsDynamics, x::CarX, z::CarZ, t::𝕋)::CarZ = begin
    if Bool(z.detected)
        z
    elseif (isnothing(dy.wall_position) 
            || x.pos + dy.detector_range < dy.wall_position)
        [0.0, 0.0]
    else
        [1.0, dy.wall_position]
    end
end

struct WallObsModel <: ObsDynamics end

"The model simply assumes the wall position stays the same\n"
OneVision.obs_forward(dy::WallObsModel, x::CarX, z::CarZ, t::𝕋) = z

@kwdef struct LeaderFollowerControl <: CentralControl{CarU}
    k_v::ℝ = 3.0
    k_x::ℝ = 2.0
    stop_distance::ℝ = 3.0
    target_v::ℝ = 2.0
end

OneVision.control_one(
    lf::LeaderFollowerControl, xs::AbstractVector{CarX},zs::AbstractVector{CarZ}, t::𝕋, id::ℕ
)::CarU = begin
    tol = 0.5
    function bang_bang(x̂, x, k, tol)
        if abs(x̂ - x) ≤ tol
            k * (x̂ - x)
        else
            sign(x̂-x) * 3
        end
    end

    x = xs[id]
    z = zs[id]
    if (z.detected ≉ 0) && x.pos ≤ z.distance - lf.stop_distance
        acc = bang_bang(0.0, x.velocity, lf.k_v, tol)
    elseif id == 1
        # the leader
        acc = bang_bang(lf.target_v, x.velocity, lf.k_v, tol)
    else
        # a follower
        leader = xs[1]
        acc = (bang_bang(leader.velocity, x.velocity, lf.k_v, tol)
                + bang_bang(leader.pos, x.pos, lf.k_x, tol))
    end
    CarU(acc)
end

function run_example(times, delta_t::ℝ; plot_result=true)
    times::Vector{𝕋} = collect(times)
    t0, t_end = times[1], times[end]
    rng = MersenneTwister(1234)
    
    agent_info(id) = begin
        # acc_noise = randn(rng, ℝ, 1 + t_end - t0)
        acc_noise = zeros(1 + t_end - t0)
        sys_dy = car_system(delta_t, t -> CarX(0, acc_noise[1 + t - t0]))
        obs_dy = 
            if id == 1; WallObsDynamics(wall_position=15.0, detector_range=6.0)
            else WallObsModel() end
        
        sys_dy, obs_dy
    end
    
    N = 2
    world_dynamics = WorldDynamics([agent_info(i) for i in 1:N])
    init_states = fill((CarX(0.0, 0.0), CarZ(0.0, 0.0), CarU(0.0)), N)
    delays = DelayModel(obs=1, act=2, com=5)
    world_model = WorldDynamics(
        fill((car_system(delta_t), WallObsModel()), N))

    comps = ["pos", "velocity", "acceleration"]
    function record_f(xs, zs, us)
        [(x -> x.pos).(xs) (x -> x.velocity).(xs) (u -> u.acc).(us)]
    end
    
    H = 20
    result = simulate(
        world_dynamics, 
        delays,
        # NaiveCF{CarX,CarZ,CarU}(N, LeaderFollowerControl(), delays.com),
        let x_weights = fill(CarX(1, 1), 2), u_weights = fill(CarU(1), 2)
            OvCF{N,CarX,CarZ,CarU,H}(
                LeaderFollowerControl(), world_model, delays, x_weights, u_weights)  
        end,
        init_states,
        (comps, record_f),
        times,
    )
    plot_result ? visualize(result; delta_t) : result
end

end # Car1DExampleLabeled
