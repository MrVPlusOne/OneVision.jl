if isdefined(@__MODULE__, :LanguageServer)  # hack to make vscode linter work properly
    include("../src/OneVision.jl")
    using .OneVision
    using .OneVision.Car1DExample
else
    using OneVision
    using OneVision.Car1DExample
end

using StaticArrays
using OSQP
using Plots
using Test
using Statistics: mean

include("test_utils.jl")

@testset "Discretize a double integrator" begin
    A, B = [0.0 1.0; 0.0 0.0], colvec([0.0; 1.0])
    let dt = 1.23
        @show A′, B′ = discretize(A, B, dt)
        x0 = 1.1
        v0 = 1.4
        a = 1.45
        x1, v1 = A′ * [x0 v0]' + B′ * [a]'
        @test x1 == x0 + v0 * dt + a * dt * dt / 2
        @test v1 == v0 + a * dt
    end
end

let
    dt = 0.1
    H = 20  # horizon
    sys = car_system(dt)

    @testset "Path tracking test" begin
        x_weights = @SVector [5.0, 2.0]
        u_weights = @SVector [0.1]

        prob = PathFollowingProblem(
            Val(H),
            sys,
            x_weights,
            u_weights,
            () -> OSQP.Optimizer(verbose=false),
        )
        x0 = SVector(CarX(-1.0, -1.0))

        # x_path = SMatrix{2,H}(hcat(zeros(2, H ÷ 2), fill([1;0], H ÷ 2)...))
        x_path = SMatrix{2,H}(hcat(fill(CarX(1, 0), H)...))
        u_path = SMatrix{1,H}(hcat(fill(CarU(0), H)...))
        u, x, obj = follow_path(prob, x0, x_path, u_path, 0)
        times = (1:H) * dt
        plot(
            plot(times, u'; label="u"),
            plot(times, hcat(x', x_path'); label=["x" "v" "x*" "v*"]),
            layout=(2, 1),
            size=(500, 300 * 3),
        ) |> display
        @test true
    end

    struct NoObs <: ObsDynamics end
    OneVision.obs_forward(::NoObs, x, z, t::𝕋) = z

    struct RendezvousControl <: CentralControl{CarU} end

    OneVision.control_one(::RendezvousControl, xs, zs, t::𝕋, id::ℕ) = begin
        target = mean(xs)
        CarU(sum(target - xs[id]))
    end

    @testset "Forward prediciton" begin
        N = 2
        world = WorldDynamics(fill((sys, NoObs()), 2))
        s1 = (CarX(0.0, 0.0), CarZ(0.0, 0.0))
        s2 = (CarX(2.0, 1.0), CarZ(0.0, 0.0))
        prob = ForwardPredictProblem(world, RendezvousControl(), s1[1], s1[2]; H)
        u_traj, x_traj = forward_predict(prob, [s1,s2], 0)
        y_data = hcat(x_traj[:,1]...)'
        plot(1:H, y_data; label=["x1", "v1"]) |> display
        @test true
    end

    @testset "Self estimation" begin
        s0 = CarX(0, 0), 0
        u_history = [CarU(0.1i) for i in 1:10]
        xs = self_estimate(sys, s0, u_history)
        plot(1:10, hcat(xs...)'; label=["x" "v"]) |> display
        @test true
    end
end # Double integrater let
