using Optim

f(x) = (1.0 - x[1])^2 + 100.0 * (x[2] - x[1]^2)^2

x0 = [0.0, 0.0]
optimize(f, x0, LBFGS(); autodiff = :forward)
optimize(f, x0, SimulatedAnnealing())

using StaticArrays
A = @SVector [1,2,3,4]
reshape(A, Size(2,2)) |> x -> reshape(x, Size(4))

resh(x::SVector{4}) = reshape(x, Size(2,2))
@code_warntype resh(A)
