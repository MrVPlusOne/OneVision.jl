export @todo, @require_impl, constant_queue, colvec2vec, colvec, @unzip
export @kwdef, Queue, enqueue!, dequeue!

import DataStructures
using DataStructures: Queue, enqueue!, dequeue!
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

    FuncT(::Type{X}, ::Type{Y}, f::F) where {X,Y,F} = new{X,Y,F}(f)
end

@inline function (f::FuncT{X,Y})(x::X)::Y where {X,Y}
    f.f(x)
end

function DataStructures.Queue(xs::AbstractVector{T})::Queue{T} where T
    q = Queue{T}()
    foreach(xs) do x
        enqueue!(q, x)
    end
    q
end

function constant_queue(value::T, q_size)::Queue{T} where {T}
    Queue(repeat([value], q_size))
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
