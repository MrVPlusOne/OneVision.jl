module TestOneVision

include("../src/OneVision.jl")
using OneVision
using Test

@testset "OneVision.jl" begin
    # Write your tests here.
    @testset "Discretize a double integrator" begin
        A, B = [0.0 1.0; 0.0 0.0], colvec([0.0; 1.0])
        dt = 1.23
        @show A′, B′ = discretize(A, B, dt)
        x0 = 1.1; v0 = 1.4; a = 1.45
        x1, v1 =  A′ * [x0 v0]' + B′ * [a]'
        @test x1 == x0 + v0 * dt + a * dt * dt / 2
        @test v1 == v0 + a * dt
    end
end

end
