#pragma once

#include <graph/graph.h>

// todo: need inverse lookup somehow for keys
//   - could we fold it into the base class Value type?
//   - consider not having special iterators and adding key_for(iter | id)

namespace graph {

/**
 * @brief A digraph class that allows vertices and/or edges to be indexed by a custom type.
 * 
 * Because this is a subclass, verts and edges will also continue to be indexable by VertexId
 * and EdgeId, respectively.
 * 
 * It is permitted for one of the key types to be void, in which case the corresponding
 * indexing methods will not be available.
 * 
 * As in the base class, value types for vertices and edges may each be void / omitted.
 * 
 * @tparam Vk Key type for vertices.
 * @tparam Vv Value type for vertices.
 * @tparam Ek Key type for edges.
 * @tparam Ev Value type for edges.
 * @tparam Map Map type to use for storing vertices and edges. Must support the same interface
 *   as std::unordered_map.
 */
template <
    typename Vk,
    typename Vv,
    typename Ek,
    typename Ev,
    template <class...> class Map=std::unordered_map>
struct DigraphMap : public Digraph<Vv, Ev, Map> {
private:
    static constexpr bool HasVertKey() { return !std::is_same<Vk, void>::value; }
    static constexpr bool HasEdgeKey() { return !std::is_same<Ek, void>::value; }

    using Base       = Digraph<Vv, Ev, Map>;
    using VertKeyMap = Map<Vk, VertexId>;
    using EdgeKeyMap = Map<Ek, EdgeId>;
    
    using StoredVertKeyMap = std::conditional_t<HasVertKey(), VertKeyMap, int[0]>;
    using StoredEdgeKeyMap = std::conditional_t<HasEdgeKey(), EdgeKeyMap, int[0]>;
    
    StoredVertKeyMap _vert_ids_by_key;
    StoredEdgeKeyMap _edge_ids_by_key;
    
    Base*       _base()       { return static_cast<Base*>(this); }
    const Base* _base() const { return static_cast<const Base*>(this); }
    
public:

    using typename Base::vertex_iterator;
    using typename Base::const_vertex_iterator;
    using typename Base::edge_iterator;
    using typename Base::const_edge_iterator;
    using typename Base::incident_edge_iterator;
    using typename Base::const_incident_edge_iterator;
    
    using typename Base::vertex_ref;
    using typename Base::const_vertex_ref;
    using typename Base::edge_ref;
    using typename Base::const_edge_ref;
    
    using Base::HasVertexValue;
    using Base::HasEdgeValue;
    
    
    /**
     * @brief Returns a const reference to the vertex with the given key.
     * 
     * If the key is not present in the graph, a `std::out_of_range` exception is thrown.
     */
    const_vertex_ref operator[](const Vk& key) const requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            throw std::out_of_range("vertex key not found");
        }
        return {this, this->_verts.find(i->second)};
    }
    
    /**
     * @brief Returns a reference to the vertex with the given key.
     * 
     * If the key is not present in the graph, a new vertex is created with the given key
     * and a reference to it is returned.
     */
    vertex_ref operator[](const Vk& key) requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            auto v_i = _base()->insert_vertex();
            _vert_ids_by_key.insert({key, v_i->id()});
            return v_i->_i;
        } else {
            return this->_verts.find(i->second);
        }
    }
    
    /**
     * @brief Returns a reference to the vertex with the given Id.
     */
    vertex_ref operator[](VertexId v) {
        return _base()->operator[](v);
    }
    
    /**
     * @brief Returns a const reference to the vertex with the given Id.
     */
    const_vertex_ref operator[](VertexId v) const {
        return _base()->operator[](v);
    }
    
    /**
     * @brief Returns a const reference to the edge with the given key.
     * 
     * If the key is not present in the graph, a `std::out_of_range` exception is thrown.
     */
    const_edge_ref operator[](const Ek& key) const requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            throw std::out_of_range("edge key not found");
        }
        return {this, this->_edges.find(i->second)};
    }
    
    /**
     * @brief Returns a reference to the edge with the given key.
     * 
     * If the key is not present in the graph, a `std::out_of_range` exception is thrown.
     */
    edge_ref operator[](const Ek& key) requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            throw std::out_of_range("edge key not found");
        } else {
            return {this, this->_edges.find(i->second)};
        }
    }
    
    /**
     * @brief Returns a reference to the edge with the given Id.
     */
    edge_ref operator[](EdgeId e) {
        return _base()->operator[](e);
    }
    
    /**
     * @brief Returns a const reference to the edge with the given Id.
     */
    const_edge_ref operator[](EdgeId e) const {
        return _base()->operator[](e);
    }
    
    using Base::contains;
    
    /// Returns true if a vertex with the given key is present in the graph.
    bool contains(const Vk& key) const requires (HasVertKey()) {
        return _vert_ids_by_key.contains(key);
    }
    
    /// Returns true if an edge with the given key is present in the graph.
    bool contains(const Ek& key) const requires (HasEdgeKey()) {
        return _edge_ids_by_key.contains(key);
    }
    
    using Base::find_vertex;
    
    /**
     * @brief Returns an iterator to the vertex with the given key.
     * 
     * If the key is not present in the graph, returns `this->end_vertices()`.
     */
    vertex_iterator find_vertex(const Vk& key) requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            return this->end_vertices();
        } else {
            return {this, this->_verts.find(i->second)};
        }
    }
    
    /**
     * @brief Returns a const iterator to the vertex with the given key.
     * 
     * If the key is not present in the graph, returns `this->end_vertices()`.
     */
    const_vertex_iterator find_vertex(const Vk& key) const requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            return this->end_vertices();
        } else {
            return {this, this->_verts.find(i->second)};
        }
    }
    
    using Base::find_edge;
    
    /**
     * @brief Returns an iterator to the edge with the given key.
     * 
     * If the key is not present in the graph, returns `this->end_edges()`.
     */
    edge_iterator find_edge(const Ek& key) requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            return this->end_edges();
        } else {
            return {this, this->_edges.find(i->second)};
        }
    }
    
    /**
     * @brief Returns a const iterator to the edge with the given key.
     * 
     * If the key is not present in the graph, returns `this->end_edges()`.
     */
    const_edge_iterator find_edge(const Ek& key) const requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            return this->end_edges();
        } else {
            return {this, this->_edges.find(i->second)};
        }
    }
    
    using Base::vertex;
    
    /// Returns a reference to the vertex with the given key, or `std::nullopt` if not found.
    std::optional<vertex_ref> vertex(const Vk& key) requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            return std::nullopt;
        } else {
            return {this, this->_verts.find(i->second)};
        }
    }
    
    /// Returns a const reference to the vertex with the given key, or `std::nullopt`
    /// if not found.
    std::optional<const_vertex_ref> vertex(const Vk& key) const requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            return std::nullopt;
        } else {
            return {this, this->_verts.find(i->second)};
        }
    }
    
    /// Returns a reference to the edge with the given key, or `std::nullopt` if not found.
    std::optional<edge_ref> edge(const Ek& key) requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            return std::nullopt;
        } else {
            return {this, this->_edges.find(i->second)};
        }
    }
    
    /// Returns a const reference to the edge with the given key, or `std::nullopt` if
    /// not found.
    std::optional<const_edge_ref> edge(const Ek& key) const requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            return std::nullopt;
        } else {
            return {this, this->_edges.find(i->second)};
        }
    }
    
    /**
     * @brief Removes the vertex with the given key from the graph, if present.
     * 
     * Returns the number of vertices (0 or 1) removed from the graph.
     */
    size_t erase(const Vk& key) requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i != _vert_ids_by_key.end()) {
            _vert_ids_by_key.erase(i);
            this->erase(vertex_iterator{this, this->_verts.find(i->second)});
            return 1;
        } else {
            return 0;
        }
    }
    
    /**
     * @brief Removes the edge with the given key from the graph, if present.
     * 
     * Returns the number of edges (0 or 1) removed from the graph.
     */
    size_t erase(const Ek& key) requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i != _edge_ids_by_key.end()) {
            _edge_ids_by_key.erase(i);
            this->erase(edge_iterator{this, this->_edges.find(i->second)});
            return 1;
        } else {
            return 0;
        }
    }
    
    auto erase(incident_edge_iterator edge) requires (not HasEdgeKey()) {
        return _base()->erase(edge);
    }
    
    auto erase(edge_iterator edge) requires (not HasEdgeKey()) {
        return _base()->erase(edge);
    }
    
    auto erase(EdgeId edge) requires (not HasEdgeKey()) {
        return _base()->erase(edge);
    }
    
    /**
     * @brief Inserts a vertex with the given key into the graph.
     * 
     * Returns a pair of an iterator to the vertex with the given key, and a boolean
     * indicating whether the vertex was inserted (`true`) or already existed (`false`).
     * 
     * In the case where the vertex was inserted, the value is default-constructed.
     * Otherwise, the value is left unchanged.
     */
    std::pair<vertex_iterator, bool> insert_vertex(const Vk& key) requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i != _vert_ids_by_key.end()) {
            return {vertex_iterator{this, this->_verts.find(i->second)}, false};
        } else {
            auto v_i = _base()->insert_vertex();
            _vert_ids_by_key.insert({key, v_i->id()});
            return {v_i, true};
        }
    }
    
    /**
     * @brief Inserts a vertex with the given key and value into the graph.
     * 
     * Returns a pair of an iterator to the vertex with the given key, and a boolean
     * indicating whether the vertex was inserted (`true`) or already existed (`false`).
     * 
     * In the case where the vertex was inserted, the value is copy-constructed from the
     * given value. Otherwise, the value is left unchanged.
     */
    std::pair<vertex_iterator, bool> insert_vertex(const Vk& key, const Vv& value)
        requires (HasVertKey() and HasVertexValue())
    {
        auto i = _vert_ids_by_key.find(key);
        if (i != _vert_ids_by_key.end()) {
            auto j = this->_verts.find(i->second);
            return {vertex_iterator{this, j}, false};
        } else {
            auto v_i = _base()->insert_vertex(value);
            _vert_ids_by_key.insert({key, v_i->id()});
            return {v_i, true};
        }
    }
    
    vertex_iterator insert_vertex(const Vv& value) requires (not HasVertKey() and HasVertexValue()) {
        return _base()->insert_vertex(value);
    }
    
    vertex_iterator insert_vertex() requires (not HasVertKey() and not HasVertexValue()) {
        return _base()->insert_vertex();
    }
    
    /**
     * @brief Emplaces a vertex with the given key and value construction arguments
     * into the graph.
     * 
     * Returns a pair of an iterator to the vertex with the given key, and a boolean
     * indicating whether the vertex was inserted (`true`) or already existed (`false`).
     * 
     * In the case where the vertex was inserted, the value is constructed in-place
     * from the given arguments. Otherwise, the value is left unchanged.
     */
    template <typename... Args>
    std::pair<vertex_iterator, bool> emplace_vertex(const Vk& key, Args&&... args)
        requires (HasVertKey() and HasVertexValue())
    {
        auto i = _vert_ids_by_key.find(key);
        if (i != _vert_ids_by_key.end()) {
            auto j = this->_verts.find(i->second);
            return {vertex_iterator{this, j}, false};
        } else {
            auto v_i = _base()->emplace_vertex(std::forward<Args>(args)...);
            _vert_ids_by_key.insert({key, v_i->id()});
            return {v_i, true};
        }
    }
    
    /**
     * @brief Emplaces a vertex with the given value construction arguments into the graph,
     * returning an iterator to the new vertex.
     */
    template <typename... Args>
    vertex_iterator emplace_vertex(Args&&... args)
        requires (not HasVertKey() and HasVertexValue())
    {
        return _base()->emplace_vertex(std::forward<Args>(args)...);
    }
    
    /**
     * @brief Inserts a vertex with the given key and value into the graph.
     * 
     * Returns a pair of an iterator to the vertex with the given key, and a boolean
     * indicating whether the vertex was inserted (`true`) or already existed (`false`).
     */
    std::pair<vertex_iterator, bool> insert_or_assign_vertex(const Vk& key, const Vv& value)
        requires (HasVertKey() and HasVertexValue())
    {
        auto i = _vert_ids_by_key.find(key);
        if (i != _vert_ids_by_key.end()) {
            auto j = this->_verts.find(i->second);
            j->value() = value;
            return {vertex_iterator{this, j}, false};
        } else {
            auto v_i = _base()->insert_vertex(value);
            _vert_ids_by_key.insert({key, v_i->id()});
            return {v_i, true};
        }
    }
    
    /**
     * @brief Inserts a directed edge with the given key into the graph.
     * 
     * Returns a pair of an iterator to the edge with the given key, and a boolean
     * indicating whether the edge was inserted (`true`) or already existed (`false`).
     * 
     * In the case where the edge was inserted, the value is default-constructed.
     * Otherwise, the value is left unchanged.
     */
    std::pair<incident_edge_iterator, bool> insert_directed_edge(
            const Ek& new_key,
            vertex_iterator src,
            vertex_iterator dst)
        requires (HasEdgeKey())
    {
        auto i = _edge_ids_by_key.find(new_key);
        if (i != _edge_ids_by_key.end()) {
            // insertion blocked by existing key
            return {incident_edge_iterator{this, this->_edges.find(i->second)}, false};
        }
        auto e_i = _base()->insert_directed_edge(src, dst);
        _edge_ids_by_key.insert({new_key, e_i->id()});
        return e_i;
    }
    
    /**
     * @brief Inserts a directed edge with the given key and value into the graph.
     * 
     * Returns a pair of an iterator to the edge with the given key, and a boolean
     * indicating whether the edge was inserted (`true`) or already existed (`false`).
     * 
     * In the case where the edge was inserted, the value is copy-constructed from the
     * given value. Otherwise, the value is left unchanged.
     */
    std::pair<incident_edge_iterator, bool> insert_directed_edge(
            const Ek& new_key,
            vertex_iterator src,
            vertex_iterator dst,
            const Ev& value)
        requires (HasEdgeKey() and HasEdgeValue())
    {
        auto i = _edge_ids_by_key.find(new_key);
        if (i != _edge_ids_by_key.end()) {
            // insertion blocked by existing key
            return {incident_edge_iterator{this, this->_edges.find(i->second)}, false};
        }
        auto e_i = _base()->insert_directed_edge(src, dst, value);
        _edge_ids_by_key.insert({new_key, e_i->id()});
        e_i->value() = value;
        return e_i;
    }
    
    /**
     * @brief Inserts a directed edge with the given key and value into the graph.
     * 
     * Returns a pair of an iterator to the edge with the given key, and a boolean
     * indicating whether the edge was inserted (`true`) or already existed (`false`).
     * 
     * Whether or not the edge was newly inserted, the value is copy-constructed or
     * copy-assigned from the given value.
     */
    std::pair<incident_edge_iterator, bool> insert_or_assign_directed_edge(
            const Ek& new_key,
            vertex_iterator src,
            vertex_iterator dst,
            const Ev& value)
        requires (HasEdgeKey() and HasEdgeValue())
    {
        auto i = _edge_ids_by_key.find(new_key);
        if (i != _edge_ids_by_key.end()) {
            auto j = this->_edges.find(i->second);
            j->value() = value;
            return {incident_edge_iterator{this, j}, false};
        }
        auto e_i = _base()->insert_directed_edge(src, dst, value);
        _edge_ids_by_key.insert({new_key, e_i->id()});
        e_i->value() = value;
        return e_i;
    }
    
    incident_edge_iterator insert_directed_edge(
            vertex_iterator src,
            vertex_iterator dst)
        requires (not HasEdgeKey())
    {
        return _base()->insert_directed_edge(src, dst);
    }
    
    /**
     * @brief Inserts an undirected edge with the given key and value construction
     * arguments into the graph.
     * 
     * Returns a pair of an iterator to the edge with the given key, and a boolean
     * indicating whether the edge was inserted (`true`) or already existed (`false`).
     * 
     * In the case where the edge was inserted, the value is constructed in-place
     * from the given arguments. Otherwise, the value is left unchanged.
     */
    template <typename... Args>
    std::pair<incident_edge_iterator, bool> emplace_directed_edge(
            const Ek& new_key,
            vertex_iterator src,
            vertex_iterator dst,
            Args&&... args)
        requires (HasEdgeKey() and HasEdgeValue())
    {
        auto i = _edge_ids_by_key.find(new_key);
        if (i != _edge_ids_by_key.end()) {
            // insertion blocked by existing key
            return {incident_edge_iterator{this, this->_edges.find(i->second)}, false};
        }
        auto e_i = _base()->emplace_directed_edge(src, dst, std::forward<Args>(args)...);
        _edge_ids_by_key.insert({new_key, e_i->id()});
        return e_i;
    }
    
    template <typename... Args>
    std::pair<incident_edge_iterator, bool> emplace_directed_edge(
            vertex_iterator src,
            vertex_iterator dst
            Args&&... args)
        requires (not HasEdgeKey() and HasEdgeValue())
    {
        return _base()->emplace_directed_edge(src, dst, std::forward<Args>(args)...);
    }
    
    std::pair<incident_edge_iterator, incident_edge_iterator> insert_undirected_edge(
            vertex_iterator src,
            vertex_iterator dst)
        requires (not HasEdgeKey() and not HasEdgeValue())
    {
        return _base()->insert_undirected_edge(src, dst);
    }
    
private:
    // "delete" the undirected edge helper; we can't store multiple edges under the same key.
    using Base::insert_undirected_edge;
    
};

} // namespace graph
