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
    lf::LeaderFollowerControl, xs::Each{CarX},zs::Each{CarZ}, t::𝕋, id::ℕ
)::CarU = begin
    x = xs[id]
    z = zs[id]
    if Bool(z.detected) && x.pos <= z.distance - lf.stop_distance
        acc = -lf.k_v * x.velocity
    elseif id == 1
        # the leader
        acc = lf.k_v * (lf.target_v - x.velocity)
    else
        # a follower
        leader = xs[1]
        acc = (lf.k_v * (leader.velocity - x.velocity) 
               + lf.k_x * (leader.pos - x.pos))
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
        obs_dy = (id == 1 ? WallObsDynamics(wall_position=10.0, detector_range=6.0) 
                : WallObsModel())
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
    
    result = simulate(
        world_dynamics, 
        delays,
        # NaiveCF{CarX,CarZ,CarU}(N, LeaderFollowerControl(), delays.com),
        OvCF{N, CarX, CarZ, CarU}(LeaderFollowerControl(), world_model, delays),
        init_states,
        (comps, record_f),
        times,
    )
    plot_result ? visualize(result; delta_t) : result
end

end # Car1DExampleLabeled
