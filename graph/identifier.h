#pragma once

#include <cstdint>
#include <functional>
#include <concepts>

namespace graph {

    
template <typename Identifier>
struct IdentifierAllocator;


template <typename Derived, typename T=uint64_t>
struct Identifier {
    using self_t = Derived;
private:
    
    static constexpr T _INVALID_ID = 0;
    
    Derived* derived() {
        return static_cast<Derived*>(this);
    }
    
    const Derived* derived() const {
        return static_cast<const Derived*>(this);
    }
    
    T _id = _INVALID_ID;
    
    friend struct IdentifierAllocator<Derived>;

public:

    using value_type = T;
    
    constexpr Identifier() = default;
    constexpr Identifier(const Identifier&) = default;
    constexpr explicit Identifier(T id) : _id(id) {}
    
    static constexpr Derived invalid() {
        return Derived{_INVALID_ID};
    }
    
    explicit constexpr operator T() const {
        return _id;
    }
    
    constexpr auto operator<=>(const Identifier& other) const = default;
    
    constexpr bool is_valid() const {
        return _id != _INVALID_ID;
    }
    
    operator bool() const {
        return is_valid();
    }
    
}; // struct Identifier


template <typename Identifier>
struct IdentifierAllocator {
private:
    typename Identifier::value_type _next_id = Identifier::_INVALID_ID + 1;

public:
    
    Identifier next() {
        return Identifier{_next_id++};
    }
    
};

}  // namespace graph


template <typename D>
requires std::derived_from<D, graph::Identifier<D,typename D::value_type>>
struct std::hash<D> {
    // we have to use a Concept here, because if we specialize
    // for `Identifier<D>` directly, template resolution will not
    // match derived classes.
    size_t operator()(const D& id) const {
        using T = typename D::value_type;
        return std::hash<T>{}(static_cast<T>(id));
    }
};

template <typename D, typename T>
requires std::derived_from<D, graph::Identifier<D,T>>
std::ostream& operator<<(std::ostream& os, const D& id) {
    os << static_cast<T>(id);
    return os;
}
