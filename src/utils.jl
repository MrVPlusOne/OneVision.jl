export @todo, @require_impl, constant_queue, col_vector, unzip
export @kwdef, Queue, enqueue!, dequeue!

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

function constant_queue(value::T, q_size)::Queue{T} where T
    q = Queue{T}(q_size)
    for _ in 1:q_size
        enqueue!(q, value)
    end
    q
end    

"Convert a 2-D column vector to a 1-D vector.\n"
col_vector(x::AbstractMatrix) = begin
    @assert size(x, 2) == 1
    x[:]
end
    
col_vector(x::AbstractVector) = x

unzip(a) = map(x -> getfield.(a, x), fieldnames(eltype(a)))