export @todo, @require_impl, colvec2vec, colvec, @unzip
export FixedQueue, pushpop!, constant_queue
export print_multiple, show_type, repr_type, show_simple_type
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

    FuncT(::Type{X}, ::Type{Y}, f::F) where {X,Y,F} = new{X,Y,F}(f)
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
    i = mod1(q.head + state, q.len)
    if state < q.len
        q.vec[i], state+1
    else
        nothing
    end
end

Base.length(q::FixedQueue) = q.len

Base.eltype(::FixedQueue{T}) where T = T

Base.first(q::FixedQueue) = q.vec[q.head]

Base.getindex(q::FixedQueue, i) = q.vec[mod1(q.head + i-1, q.len)]

function pushpop!(q::FixedQueue{T}, x::T)::T where T
    v = q.vec[q.head]
    q.vec[q.head] = x
    q.head = mod1(q.head + 1, q.len)
    v
end

function Base.show(io::IO, q::FixedQueue) 
    seq = [i for i in q]
    print(io, "FixedQueue(len=$(q.len), queue=$seq)")
end

"""
# Examples
```jldoctest
julia> print_multiple(show, stdout, 1:10, "{", ",", "}")
{1,2,3,4,5,6,7,8,9,10}
```
"""
function print_multiple(f, io::IO, xs, left="{", sep=",", right="}")
    n = length(xs)::Int
    print(io, left)
    for (i, x) in enumerate(xs)
        f(io,x)
        i < n && print(io, sep)
    end
    print(io, right)
end

"""
    show_type(io, type; kwargs...)

# Keyword args
- `max_depth=3`: the maximal type AST depth to show. Type arguments deeper than this value
will be printed as `...`.
- `short_type_name=true`: when set to `true`, will print simple type names without their 
corresponding module path. e.g. "Name" instead of "ModuleA.ModuleB.Name". Note that the 
shorter name will always be used if the type is visible from the current scope. 
"""
function show_type(io::IO, @nospecialize(ty::Type); max_depth::Int = 3, short_type_name::Bool = true)
    t_var_scope::Set{Symbol} = Set{Symbol}()

    function rec(x::DataType, d)
        short_type_name ? print(io, nameof(x)) : Base.show_type_name(io, x.name)
        if !isempty(x.parameters)
            if d ≤ 1
                print(io, "{...}")
            else
                print_multiple((_,p) -> rec(p, d-1), io, x.parameters)
            end
        end
    end

    function rec(x::Union, d)
        print_multiple((_, p) -> rec(p, d-1), io, Base.uniontypes(x))
    end

    function show_tv_def(tv::TypeVar, d)
        function show_bound(io::IO, @nospecialize(b))
            parens = isa(b,UnionAll) && !Base.print_without_params(b)
            parens && print(io, "(")
            rec(b, d-1)
            parens && print(io, ")")
        end
        lb, ub = tv.lb, tv.ub
        if lb !== Base.Bottom
            if ub === Any
                Base.show_unquoted(io, tv.name)
                print(io, ">:")
                show_bound(io, lb)
            else
                show_bound(io, lb)
                print(io, "<:")
                Base.show_unquoted(io, tv.name)
            end
        else
            Base.show_unquoted(io, tv.name)
        end
        if ub !== Any
            print(io, "<:")
            show_bound(io, ub)
        end
        nothing
    end

    function rec(x::UnionAll, d, var_group::Vector{TypeVar}=TypeVar[])
        # rename tvar as needed
        var_symb = x.var.name
        if var_symb === :_ || var_symb ∈ t_var_scope
            counter = 1
            while true
                newname = Symbol(var_symb, counter)
                if newname ∉ t_var_scope
                    newtv = TypeVar(newname, x.var.lb, x.var.ub)
                    x = UnionAll(newtv, x{newtv})
                    break
                end
                counter += 1
            end
        end
        var_symb = x.var.name

        push!(var_group, x.var)
        push!(t_var_scope, var_symb)
        if x.body isa UnionAll
            # current var group continues
            rec(x.body, d, var_group)
        else
            # current var group ends
            rec(x.body, d)
            print(io, " where ")
            if length(var_group) == 1
                show_tv_def(var_group[1], d)
            else
                print_multiple((_, v) -> show_tv_def(v, d), io, var_group)
            end
        end
        delete!(t_var_scope, var_symb)
    end

    function rec(tv::TypeVar, d)
        Base.show_unquoted(io, tv.name)
    end

    function rec(ohter, d)
        @nospecialize(other)
        show(io, ohter)
    end

    rec(ty, max_depth)
end

repr_type(ty::Type; kwargs...) = sprint((io,t) -> show_type(io,t; kwargs...), ty)

"""
    show_simple_type(;kwargs...)

Replace `Base.show(io::IO, x::Type)` with the simpler type printing funciton 
`show_type`. See `show_type` for details about the available `kwargs`.
"""
function show_simple_type(;kwargs...)
    @eval Base.show(io::IO, x::Type) = show_type(io, x; $(kwargs)...)
end

@info "Replace Base.show(io::IO, x::Type) with show_type."
show_simple_type()
    