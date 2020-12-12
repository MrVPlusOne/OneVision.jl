if isdefined(@__MODULE__, :LanguageServer)  # hack to make vscode linter work properly
    include("../src/OneVision.jl")
    using .OneVision
    using .OneVision.Car1DExample
    using .OneVision.Car2DExamples
else
    using OneVision
    using OneVision.Car1DExample
    using OneVision.Car2DExamples
end

using StaticArrays
using Plots
using Test
using Statistics: mean
using SimpleTypePrint: config_type_display
config_type_display()  # for simpler type error

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
    ΔT = 5  # control interval
    HΔT = H * ΔT
    sys = car_system(dt)

    @testset "Path tracking test" begin
        x_weights = CarX(5.0, 2.0)
        u_weights = CarU(0.1)

        prob = TrajPlanningProblem(
            H,
            ΔT,
            sys,
            x_weights,
            u_weights,
        )
        x0 = CarX(-1.0, -1.0)

        x_path = SVector{HΔT}(fill(CarX(1.0, 0.0), HΔT))
        u_path = SVector{HΔT}(fill(CarU(0.0), HΔT))
        t0 = 0
        u, obj = plan_trajectory(prob, x0, x_path, u_path, t0)
        x = x_path_from_u(x0, t0, u, prob.dy, Val(ΔT))
        u_times = (1:H)*(dt * ΔT)
        x_times = (1:HΔT) * dt
        x_data = hcat(to_matrix(x), to_matrix(x_path))
        plot(
            plot(u_times, first.(u); label = "u"),
            plot(x_times, x_data; label = ["x" "v" "x*" "v*"]),
            layout = (2, 1),
            size = (500, 300 * 3),
        ) |> display
        @test true
    end

    struct NoObs <: ObsDynamics end
    OneVision.obs_forward(::NoObs, x, z, t::𝕋) = z

    struct RendezvousControl <: CentralControlStateless{CarU{ℝ}} end

    OneVision.control_one(::RendezvousControl, xs, zs, t::𝕋, id::ℕ) = begin
        target = mean(xs)
        CarU(sum(target - xs[id]))
    end

    @testset "Forward prediciton" begin
        N = 2
        world = WorldDynamics(fill((sys, NoObs()), 2))
        s1 = (CarX(0.0, 0.0), CarZ(0.0, 0.0))
        s2 = (CarX(2.0, 0.0), CarZ(0.0, 0.0))
        prob = ForwardPredictProblem(world, RendezvousControl(); 
            X = CarX{ℝ}, Z = CarZ{ℝ}, Hf = HΔT)
        u_traj, x_traj = forward_predict!(prob, [s1,s2], nothing, 0, ΔT)
        plot(
            plot(1:HΔT, to_matrix(x_traj[:,1]); label = ["x1" "v1"]),
            plot(1:HΔT, to_matrix(u_traj[:,1]); label = ["u1"]),
            layout = (2, 1),
            size = (500, 300*2),
        ) |> display
        @test true
    end

    @testset "Self estimation" begin
        s0 = CarX(0.0, 0.0), 0
        times = 1:1000
        u_history = [CarU(sin(t/100)) for t in times]
        xs = self_estimate(sys, s0, u_history)
        plot(times * dt, to_matrix(xs); label = ["x" "v"]) |> display
        @test true
    end
end # Double integrater let

int_tests = (
    () -> Car1DExample.run_example(1:3 * 20, 20.0; noise = 0.01, plot_result = false),
    () -> Car2DExamples.tracking_example(time_end = 3, noise_level = 0.001, plot_result = false),
    () -> Car2DExamples.formation_example(time_end = 3, dynamics_noise = 0.001, plot_result = false),
)
@testset "Integration test $i" for (i, f) in enumerate(int_tests)
    f()
    @test true
end