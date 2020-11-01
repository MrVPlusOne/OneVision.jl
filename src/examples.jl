# module Car1DExampleOld
# using OneVision: ℝ, 𝕋, ℕ, ObsDynamics, State, Obs, Act
# using OneVision: @kwdef, Each, CentralControl, WorldDynamics
# using OneVision: SysDynamicsLTI, discretize, DelayModel, visualize, simulate
# using Random
# using OneVision.NaiveCFs: NaiveCF
# import OneVision

# # state indices
# const Pos, Velocity = 1, 2
# # action indices
# const Acc = 1
# # obs indices
# const Detected, Distance = 1, 2

# const car_A = [0.0 1.0; 0.0 0.0]
# const car_B = [0.0 1.0]'

# car_system(delta_t::ℝ, noise::Function) = 
#     discretize(
#         SysDynamicsLTI(car_A, car_B, noise, ["Pos", "Velocity"], ["Acc"]),
#         delta_t,
#     )
    
# car_system(delta_t::ℝ) = car_system(delta_t, _ -> [0.0 0.0]')

# @kwdef struct WallObsDynamics <: ObsDynamics
#     wall_position::Union{ℝ,Nothing}
#     detector_range::ℝ
# end

# OneVision.obs_names(::WallObsDynamics)::Vector{String} = 
#     ["Detected", "Distance"]

# OneVision.forward(dy::WallObsDynamics, x::State, z::Obs, t::𝕋)::Obs = begin
#     if Bool(z[Detected])
#         z
#     elseif (isnothing(dy.wall_position) 
#             || x[Pos] + dy.detector_range < dy.wall_position)
#         [0.0, 0.0]
#     else
#         [1.0, dy.wall_position]
#     end
# end

# struct WallObsModel <: ObsDynamics end

# "The model simply assumes the wall position stays the same\n"
# OneVision.forward(dy::WallObsModel, x::State, z::Obs, t::𝕋)::Obs = z

# @kwdef struct LeaderFollowerControl <: CentralControl
#     k_v::ℝ = 2.0
#     k_x::ℝ = 2.0
#     stop_distance::ℝ = 3.0
#     target_v::ℝ = 2.0
# end

# OneVision.control_one(
#     lf::LeaderFollowerControl, xs::Each{State},zs::Each{Obs}, id::ℕ
# )::Act = begin
#     # note that this controller does not stabilize the system; to do that, 
#     # in addition to track a speed target point, we will need to track 
#     # a position target as well.
#     x = xs[id]
#     z = zs[id]
#     if Bool(z[Detected]) && x[Pos] <= z[Distance] - lf.stop_distance
#         acc = -lf.k_v * x[Velocity]
#     elseif id == 1
#         # the leader
#         acc = lf.k_v * (lf.target_v - x[Velocity])
#     else
#         # a follower
#         leader = xs[1]
#         acc = (lf.k_v * (leader[Velocity] - x[Velocity]) 
#                + lf.k_x * (leader[Pos] - x[Pos]))
#     end
#     [acc]
# end

# function run_example(times, delta_t::ℝ)
#     times::Vector{𝕋} = collect(times)
#     t0, t_end = times[1], times[end]
#     rng = MersenneTwister(1234)
#     agent_info(id) = begin
#         # acc_noise = randn(rng, ℝ, 1 + t_end - t0)
#         acc_noise = zeros(1 + t_end - t0)
#         sys_dy = car_system(delta_t, t -> [0 acc_noise[1 + t - t0]]')
#         obs_dy = (id == 1 ? WallObsDynamics(wall_position=10.0, detector_range=6.0) 
#                 : WallObsModel())
#         sys_dy, obs_dy
#     end
    
#     N = 2
#     world_dynamics = WorldDynamics([agent_info(i) for i in 1:N])
#     init_states = fill(([0.0,0.0], [0.0,0.0], [0.0]), N)
#     delays = DelayModel(obs=1, act=2, com=5)
#     result = simulate(
#         world_dynamics, 
#         delays,
#         NaiveCF(N, LeaderFollowerControl(), delays.com),
#         init_states,
#         times,
#     )
#     visualize(result, world_dynamics; delta_t)
# end

# end # CarExample


module Car1DExample
using OneVision: ℝ, 𝕋, ℕ, ObsDynamics
using OneVision: @kwdef, Each, CentralControl, WorldDynamics
using OneVision: SysDynamicsLTI, discretize, DelayModel, visualize, simulate
using OneVision.NaiveCFs: NaiveCF
using Random
using LabelledArrays
using StaticArrays

import OneVision

const CarState = @SLVector ℝ (:pos, :velocity)
const CarAct = @SLVector ℝ (:acc,)
const CarObs = @SLVector ℝ (:detected, :distance)

const car_A = @SMatrix [0.0 1.0; 0.0 0.0]
const car_B = @SMatrix [0.0; 1.0]

const T_X = typeof(CarState(0.0, 0.0))
const T_U = typeof(CarAct(0.0))
const T_Z = typeof(CarObs(0.0, 0.0))

Base.show(io::IO, ::Type{T_X}) = show(io, "T_X")
Base.show(io::IO, ::Type{T_U}) = show(io, "T_U")
Base.show(io::IO, ::Type{T_Z}) = show(io, "T_Z")

function test_s_array()
    A = car_A
    x = CarState(1.0, 2.0)
    convert(CarState, sum(A * x for _ in 1:1000))
end

function test_array()
    A = randn(ℝ, (2, 2))
    x = [1.0,2.0]
    sum(A * x for _ in 1:1000)
end

function car_system(delta_t::ℝ, noise::Function)
    A′, B′ = discretize(car_A, car_B, delta_t)
    SysDynamicsLTI{T_X,T_U,typeof(car_A),typeof(car_B)}(
        A′, B′, noise, ["Pos", "Velocity"], ["Acc"]
    )
end
    
car_system(delta_t::ℝ) = car_system(delta_t, _ -> [0.0 0.0]')

@kwdef struct WallObsDynamics <: ObsDynamics{T_X,T_Z}
    wall_position::Union{ℝ,Nothing}
    detector_range::ℝ
end

OneVision.obs_names(::WallObsDynamics)::Vector{String} = 
    ["Detected", "Distance"]

OneVision.obs_forward(dy::WallObsDynamics, x::T_X, z::T_Z, t::𝕋)::T_Z = begin
    if Bool(z.detected)
        z
    elseif (isnothing(dy.wall_position) 
            || x.pos + dy.detector_range < dy.wall_position)
        [0.0, 0.0]
    else
        [1.0, dy.wall_position]
    end
end

struct WallObsModel <: ObsDynamics{T_X,T_Z} end

"The model simply assumes the wall position stays the same\n"
OneVision.obs_forward(dy::WallObsModel, x::T_X, z::T_Z, t::𝕋) = z

@kwdef struct LeaderFollowerControl <: CentralControl{T_X,T_Z,T_U}
    k_v::ℝ = 2.0
    k_x::ℝ = 2.0
    stop_distance::ℝ = 3.0
    target_v::ℝ = 2.0
end

OneVision.control_one(
    lf::LeaderFollowerControl, xs::Each{T_X},zs::Each{T_Z}, id::ℕ
)::T_U = begin
    # note that this controller does not stabilize the system; to do that, 
    # in addition to track a speed target point, we will need to track 
    # a position target as well.
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
    [acc]
end

function run_example(times, delta_t::ℝ)
    times::Vector{𝕋} = collect(times)
    t0, t_end = times[1], times[end]
    rng = MersenneTwister(1234)
    agent_info(id) = begin
        # acc_noise = randn(rng, ℝ, 1 + t_end - t0)
        acc_noise = zeros(1 + t_end - t0)
        sys_dy = car_system(delta_t, t -> CarState(0, acc_noise[1 + t - t0]))
        obs_dy = (id == 1 ? WallObsDynamics(wall_position=10.0, detector_range=6.0) 
                : WallObsModel())
        sys_dy, obs_dy
    end
    
    N = 2
    world_dynamics = WorldDynamics([agent_info(i) for i in 1:N])
    init_states = fill((CarState(0.0, 0.0), CarObs(0.0, 0.0), CarAct(0.0)), N)
    delays = DelayModel(obs=1, act=2, com=5)
    comps = ["pos", "velocity", "acceleration"]
    function record_f(xs, zs, us)
        [(x -> x.pos).(xs) (x -> x.velocity).(xs) (u -> u.acc).(us)]
    end
    
    result = simulate(
        world_dynamics, 
        delays,
        NaiveCF(N, LeaderFollowerControl(), delays.com),
        init_states,
        (comps, record_f),
        times,
    )
    visualize(result; delta_t)
end

end # Car1DExampleLabeled
