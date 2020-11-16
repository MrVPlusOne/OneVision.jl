export @todo, @require_impl, colvec2vec, colvec, @unzip, FuncT
export FixedQueue, pushpop!, constant_queue

export @kwdef

import DataStructures
using Base: @kwdef
using Setfield: @set
using QuadGK: quadgk

"A placeholder for unimplmeneted code."
macro todo()
    :(error("TODO: Not implemented."))
end

macro require_impl()
    :(error("Abstract method requires implementation."))
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

function constant_queue(value::T, q_size)::FixedQueue{T} where {T}
    FixedQueue(fill(value, q_size))
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

# todo: add docs and tests
function type_transpose(ty::Expr)::Expr
    @assert ty.head == :curly
    h1 = ty.args[1]
    inner = ty.args[2]
    @assert inner.head == :curly
    h2 = inner.args[1]
    elems = inner.args[2:end]
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

Base.length(q::FixedQueue) = q.len

Base.lastindex(q::FixedQueue) = q.len

Base.eltype(::FixedQueue{T}) where T = T

Base.first(q::FixedQueue) = q.vec[q.head]

Base.getindex(q::FixedQueue, i::Integer) = q.vec[mod1(q.head + i-1, q.len)]
Base.getindex(q::FixedQueue, range::UnitRange) = 
    (q.vec[mod1(q.head + i-1, q.len)] for i in range)

# push x to the end of the queue(stack?)
function pushpop!(q::FixedQueue{T}, x::T)::T where T
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
    
