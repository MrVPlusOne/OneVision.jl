module Car1DExample
using OneVision: ℝ, 𝕋, ℕ, ObsDynamics, State, Obs, Act
using OneVision: @kwdef, Each, CentralControl, WorldDynamics
using OneVision: SysDynamicsLTI, discretize, DelayModel, visualize, simulate
using Random
using OneVision.NaiveCFs: NaiveCF
import OneVision

# state indices
const Pos, Velocity = 1, 2
# action indices
const Acc = 1
# obs indices
const Detected, Distance = 1, 2

const car_A = [0.0 1.0; 0.0 0.0]
const car_B = [0.0 1.0]'

car_system(delta_t::ℝ, noise::Function) = 
    discretize(
        SysDynamicsLTI(car_A, car_B, noise, ["Pos", "Velocity"], ["Acc"]),
        delta_t,
    )
    
car_system(delta_t::ℝ) = car_system(delta_t, _ -> [0.0 0.0]')

@kwdef struct WallObsDynamics <: ObsDynamics
    wall_position::Union{ℝ,Nothing}
    detector_range::ℝ
end

OneVision.obs_names(::WallObsDynamics)::Vector{String} = 
    ["Detected", "Distance"]

OneVision.forward(dy::WallObsDynamics, x::State, z::Obs, t::𝕋)::Obs = begin
    if Bool(z[Detected])
        z
    elseif (isnothing(dy.wall_position) 
            || x[Pos] + dy.detector_range < dy.wall_position)
        [0.0, 0.0]
    else
        [1.0, dy.wall_position]
    end
end

struct WallObsModel <: ObsDynamics end

"The model simply assumes the wall position stays the same\n"
OneVision.forward(dy::WallObsModel, x::State, z::Obs, t::𝕋)::Obs = z

@kwdef struct LeaderFollowerControl <: CentralControl
    k_v::ℝ = 2.0
    k_x::ℝ = 2.0
    stop_distance::ℝ = 3.0
    target_v::ℝ = 2.0
end

OneVision.control_one(
    lf::LeaderFollowerControl, xs::Each{State},zs::Each{Obs}, id::ℕ
)::Act = begin
    # note that this controller does not stabilize the system; to do that, 
    # in addition to track a speed target point, we will need to track 
    # a position target as well.
    x = xs[id]
    z = zs[id]
    if Bool(z[Detected]) && x[Pos] <= z[Distance] - lf.stop_distance
        acc = -lf.k_v * x[Velocity]
    elseif id == 1
        # the leader
        acc = lf.k_v * (lf.target_v - x[Velocity])
    else
        # a follower
        leader = xs[1]
        acc = (lf.k_v * (leader[Velocity] - x[Velocity]) 
               + lf.k_x * (leader[Pos] - x[Pos]))
    end
    [acc]
end

function run_example(times, delta_t::ℝ)
    times::Vector{𝕋} = collect(times)
    t0, t_end = times[1], times[end]
    rng = MersenneTwister(1234)
    agent_info(id) = begin
        # acc_noise = randn(rng, ℝ, 1 + t_end - t0)
        acc_noise = zeros(1 + t_end - t0)
        sys_dy = car_system(delta_t, t -> [0 acc_noise[1 + t - t0]]')
        obs_dy = (id == 1 ? WallObsDynamics(wall_position=10.0, detector_range=6.0) 
                : WallObsModel())
        sys_dy, obs_dy
    end
    
    N = 2
    world_dynamics = WorldDynamics([agent_info(i) for i in 1:N])
    init_states = fill(([0.0,0.0], [0.0,0.0], [0.0]), N)
    delays = DelayModel(obs=1, act=2, com=5)
    result = simulate(
        world_dynamics, 
        delays,
        NaiveCF(N, LeaderFollowerControl(), delays.com),
        init_states,
        times,
    )
    visualize(result, world_dynamics; delta_t)
end

end # CarExample
