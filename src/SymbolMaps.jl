module SymbolMaps
export SymbolMap, submap
export integral!, derivative!

import Base./
using Base: @kwdef

function symbol_path(s1::Symbol, s2::Symbol)
    Symbol(string(s1, "/", s2))
end

"""
Append the symbol `s2` to symbol `s1`.
"""
@inline function /(s1::Symbol, s2::Symbol)
    symbol_path(s1, s2)
end

@kwdef struct SymbolMap
    base_path::Symbol = :/
    data::Dict{Symbol,Any} = Dict{Symbol,Any}()
end

Base.getindex(m::SymbolMap, path::Symbol) = m.data[m.base_path / path]
Base.setindex!(m::SymbolMap, v, path::Symbol) = 
    m.data[m.base_path / path] = v
Base.get!(m::SymbolMap, path::Symbol, default) = 
    get!(m.data, m.base_path / path, default)

"""
Creat a restricted view of the orginal SymbolMap. 
"""
function submap(m::SymbolMap, path::Symbol)::SymbolMap
    SymbolMap(m.base_path / path, m.data)
end

function integral!(m::SymbolMap, path::Symbol, delta_t, e::E)::E where E
    m[path] = get!(m, path, zero(E))::E + delta_t * e
end

function derivative!(m::SymbolMap, path::SymbolMap, delta_t, e::E)::E where E
    m[path] = (e - get!(m, path, e)::E) / delta_t
end

end # module SymbolMaps