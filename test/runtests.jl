module TestOneVision

# include("../src/OneVision.jl")
using OneVision
using OneVision.PathFollowing
using StaticArrays
using OSQP
using Plots
using Test

struct DoubleIntegrator <: SysDynamicsLinear end

const A, B = [0.0 1.0; 0.0 0.0], colvec([0.0; 1.0])
@testset "Discretize a double integrator" begin
    let dt = 1.23
        @show A′, B′ = discretize(A, B, dt)
        x0 = 1.1; v0 = 1.4; a = 1.45
        x1, v1 =  A′ * [x0 v0]' + B′ * [a]'
        @test x1 == x0 + v0 * dt + a * dt * dt / 2
        @test v1 == v0 + a * dt
    end
end


@testset "Double integrator path tracking" begin 
    let dt = 0.1
        H = 20  # horizon
        SA, SB = discretize(SMatrix{2,2}(A), SMatrix{2,1}(B), dt)
        Sw = @SVector [0.0,0.0]
        function OneVision.sys_A(::DoubleIntegrator, t) SA end
        function OneVision.sys_B(::DoubleIntegrator, t) SB end
        function OneVision.sys_w(::DoubleIntegrator, t) Sw end
        x_weights = @SVector [5.0,2.0]
        u_weights = @SVector [0.1]

        prob = PathFollowingProblem{2,1,H,DoubleIntegrator}(
            DoubleIntegrator(), x_weights, u_weights, OSQP.Optimizer
        )
        x0 = @SVector [-1.0, -1.0]
        
        # x_path = SMatrix{2,H}(hcat(zeros(2, H ÷ 2), fill([1;0], H ÷ 2)...))
        x_path = SMatrix{2,H}(hcat(fill([1;0], H)...))
        u_path = SMatrix{1,H}(zeros(1, H))
        u, x, obj = follow_path(prob, x0, x_path, u_path, 0)
        times = (1:H) * dt
        plot(
            plot(times, u'; label="u"),
            plot(times, hcat(x', x_path'); label=["x" "v" "x*" "v*"]),
            layout=(2, 1),
            size=(500, 300 * 3)
        ) |> display
    end
end

end # module
