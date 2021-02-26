export @todo, @require_impl, colvec2vec, colvec, @unzip, FuncT, <|
export FixedQueue, pushpop!, constant_queue, @get!
export °, rotation2D, to_matrix
export HVec, h_vec_from_dict
export @kwdef, @_, @set
export serialize_to_b_array, deserialize_from_b_array
export restrict, square, intersect_line_circle

import DataStructures

using Base: @kwdef
using Underscores: @_
using Setfield: @set
using QuadGK: quadgk
using Parameters: @unpack
using StaticArrays
using AxisArrays
using MacroTools: @capture
using Serialization

Optional{T} = Union{T, Missing}

function square(n)
    n*n
end

"A placeholder for unimplmeneted code."
macro todo()
    :(error("TODO: Not implemented."))
end

macro require_impl()
    :(error("Abstract method requires implementation."))
end

macro asserteq(left, right)
    ex = string(:($left == $right))
    quote       
        @assert(
            (l = $(esc(left))) == (r = $(esc(right))),
            "$l != $r in assertion: $($ex)")
    end
end

"""
A typed function with annotated input and output type. 
If `a` has type `FuncT{X,Y,F}`, then `a(x)` is equivalent to `a.f(x::X)::Y`.
"""
struct FuncT{X,Y,F} <: Function
    f::F

    FuncT(f::F, ::Type{X}, ::Type{Y}) where {X,Y,F} = new{X,Y,F}(f)
end

@inline function (f::FuncT{X,Y})(x::X)::Y where {X,Y}
    f.f(x)
end

"Convert a 2-D column vector to a 1-D vector.\n"
colvec2vec(x::AbstractMatrix) = begin
    @assert size(x, 2) == 1
    reshape(x, :)
end

colvec2vec(x::AbstractVector) = x

"""
Make a 2-D column vector from a vector.
"""
colvec(x::AbstractVector) = reshape(x, :, 1)

"""
Caller should annotate the return type to help the compiler's type inference
"""
function unzip_impl(a)
    map(x -> getfield.(a, x), fieldnames(eltype(a)))
end

function type_transpose(ty::Expr)::Expr
    @capture(ty, h1_{h2_{elems__}})
    rs = map(x -> :($h1{$x}), elems)
    :($h2{$(rs...)})
end

"""
    @unzip(xs, xs_type)
Unzip a list of tuples into tuples of lists. `xs_type` should match xs's type and must be
provided as a type literal. The macro then use `xs_type` to compute the correct return 
type and annotate the result. (This type annotation is usually crucial for the compiler 
to perform type inference and generate efficient code.)

```jldoctest
julia> using OneVision: @unzip
       @unzip([(1,"a"),(2,"b")],Vector{Tuple{Int, String}})
([1, 2], ["a", "b"])

julia> @macroexpand @unzip([(1, "a"),(2, "b")],Vector{Tuple{Int,String}})
:(OneVision.unzip_impl([(1, "a"), (2, "b")]::Vector{Tuple{Int, String}})::Tuple{Vector{Int}, Vector{String}})
```
"""
macro unzip(xs, xs_type::Expr)
    result_t = type_transpose(xs_type)
    :(unzip_impl($(esc(xs))::$(esc(xs_type)))::$(esc(result_t)))
    # :(unzip_impl($xs::$xs_type)::$result_t) |> esc
end

macro get!(collection, key, default)
    quote
        col = $(esc(collection))
        k = $(esc(key))
        r = get(col, k, nothing)
        if r === nothing
            col[k] = $(esc(default))
        else
            r
        end
    end
end

"""
A first-in-first-out queue with fixed length. Supporting a `push!` operation that 
simultaneously push a value to the end of the queue and pop the value at the head 
of the queue.

Implemented using an array with a head pointer.

# Examples
```jldoctest
julia> fq = FixedQueue(1:5)
FixedQueue(len=5, queue=[1, 2, 3, 4, 5])

julia> pushpop!(fq, 10), fq
(1, FixedQueue(len=5, queue=[3, 4, 5, 10, 10]))

julia> collect(fq)
5-element Array{Int64,1}:
  2
  3
  4
  5
 10

julia> foreach(11:20) do x
    pushpop!(fq, x); println(fq)
end
FixedQueue(len=5, queue=[3, 4, 5, 10, 11])
FixedQueue(len=5, queue=[4, 5, 10, 11, 12])
FixedQueue(len=5, queue=[5, 10, 11, 12, 13])
FixedQueue(len=5, queue=[10, 11, 12, 13, 14])
FixedQueue(len=5, queue=[11, 12, 13, 14, 15])
FixedQueue(len=5, queue=[12, 13, 14, 15, 16])
FixedQueue(len=5, queue=[13, 14, 15, 16, 17])
FixedQueue(len=5, queue=[14, 15, 16, 17, 18])
FixedQueue(len=5, queue=[15, 16, 17, 18, 19])
FixedQueue(len=5, queue=[16, 17, 18, 19, 20])
```

"""
mutable struct FixedQueue{T} 
    len::Int
    head::Int
    vec::Vector{T}

    function FixedQueue(init::AbstractVector{T}) where T
        vec = convert(Vector{T}, init)
        new{T}(length(vec), 1, vec)
    end
end

function Base.iterate(q::FixedQueue{T}, state::Int = 0) where T
    if state < q.len
        i = mod1(q.head + state, q.len)
        q.vec[i], state+1
    else
        nothing
    end
end

function constant_queue(value::T, q_size; eltype::Type{E} = T)::FixedQueue{E} where {T,E}
    FixedQueue(convert(Vector{E}, fill(value, q_size)))
end

Base.length(q::FixedQueue) = q.len

Base.lastindex(q::FixedQueue) = q.len

Base.eltype(::FixedQueue{T}) where T = T

Base.first(q::FixedQueue) = q.vec[q.head]

Base.getindex(q::FixedQueue, i::Integer) = q.vec[mod1(q.head + i-1, q.len)]
Base.getindex(q::FixedQueue, range::UnitRange) = 
    (q.vec[mod1(q.head + i-1, q.len)] for i in range)

# push x to the end of the queue(stack?)
function pushpop!(q::FixedQueue{T}, x)::T where T
    (q.len==0) && return x
    v = q.vec[q.head]
    q.vec[q.head] = x
    q.head = mod1(q.head + 1, q.len)
    v
end

function Base.show(io::IO, q::FixedQueue) 
    seq = [i for i in q]
    print(io, "FixedQueue(len=$(q.len), queue=$seq)")
end
    

module NumericalIntegration
    export integrate_forward, integrate_forward_invariant
    export Euler, RK38

    """
    Numerical integration for differential equation systems.
    """
    @inline function integrate_forward(f, x0::X, tspan::Tuple{T,T}, method, N) where {X,T}
        t0, t1 = tspan
        dt = (t1 - t0)/N
        x::X = x0
        t::T = t0
        for _ in 1:N
            x += method(f, x, dt, t)
            t += dt
        end
        x
    end

    """
    Numerical integration for time-invariant differential equation systems.
    """
    @inline function integrate_forward_invariant(f, x0::X, dt::AbstractFloat, method, N) where {X}
        x::X = x0
        for _ in 1:N
            x += method(f, x, dt)
        end
        x
    end


    @inline function Euler(f, x::X, dt::AbstractFloat)::X where X
        f(x)::X * dt
    end

    @inline function Euler(f, x::X, dt::AbstractFloat, t)::X where X
        f(x, t)::X * dt
    end

    """
    Runge–Kutta 3/8 rule. (4th order method)
    """
    @inline function RK38(f, x::X, dt::AbstractFloat)::X where X
        k1 = f(x)::X
        k2 = f(x+(1/3)k1*dt)::X
        k3 = f(x+dt*((-1/3)k1+k2))::X
        k4 = f(x+dt*(k1-k2+k3))::X
        dt * ((1/8)k1 + (3/8)k2 + (3/8)k3 + (1/8)k4)
    end

    @inline function RK38(f, x::X, dt::AbstractFloat, t)::X where X
        k1 = f(x, t)::X
        k2 = f(x+(1/3)k1*dt, t+(1/3)dt)::X
        k3 = f(x+dt*((-1/3)k1+k2), t+(2/3)dt)::X
        k4 = f(x+dt*(k1-k2+k3), t+dt)::X
        dt * ((1/8)k1 + (3/8)k2 + (3/8)k3 + (1/8)k4)
    end

end

"""
    f <| x = f(x)
"""
<|(f, x) = f(x)

const ° = pi/180

rotation2D(α) = 
    @SMatrix [cos(α) -sin(α)
              sin(α)  cos(α)]

to_matrix(xs::AbstractVector{X}) where X = 
    [x[i] for x in xs, i in 1:length(X)]


"""
Hybrid Vector with both continuous and discrete parts.
"""
struct HVec{C, D}
    c::C
    d::D
end

"""
Custom serialization!
"""
function serialize_to_b_array(a::Any)::Vector{UInt8}
    write_iob = IOBuffer()
    serialize(write_iob, a)
    seekstart(write_iob)
    content = take!(write_iob)
    return content #String(content)
end

function deserialize_from_b_array(s::Vector{UInt8})
    read_iob = IOBuffer(s)
    return deserialize(read_iob)
end

function restrict(theta)
    return theta #rem2pi(theta, RoundNearest)
end

"""
modified from https://stackoverflow.com/questions/30844482/what-is-most-efficient-way-to-find-the-intersection-of-a-line-and-a-circle-in-py
Find the intersection between line and circle
Line is defined by 2 points. Note that the line extends arbitrarly beyond the points
Circle is defined by the center and radius
"""
function intersect_line_circle(p1, p2, circle_center, r) ::Vector{Vector{Float64}}
    tang_tol = 1e-3
    p1x, p1y = p1
    p2x, p2y = p2
    cx, cy = circle_center
    x1, y1 = p1x -cx, p1y - cy
    x2, y2 = p2x -cx, p2y - cy
    dx, dy = x2-x1, y2 -y1
    dr = sqrt(dx*dx + dy*dy) 
    big_d = x1*y2 - x2*y1
    discriminant = square(r) *square(dr) - square(big_d)
    if discriminant < 0.0
        return Vector{Vector{Float64}}()
    else
        intersections = Vector{Vector{Float64}}()
        s = if (dy < 0.0) 
            [1.0, -1.0] 
        else 
            [-1.0, 1.0] 
        end
        for sign in s
            ix = cx + (big_d * dy + sign * (if dy < 0  -1 else 1 end) * dx * sqrt(discriminant)) / square(dr)
            iy = cy + (-big_d * dx + sign * abs(dy) * sqrt(discriminant)) / square(dr)
            i = [ix, iy]
            push!(intersections, i)
        end
        if length(intersections) == 2 && abs(discriminant) < tang_tol
            return [intersections[1]]
        else 
            return intersections
        end
    end
end

