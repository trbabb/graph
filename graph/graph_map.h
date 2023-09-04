#pragma once

#include <graph/graph.h>

// todo: key iteration
//   - make an iterator which derives from the raw vert_ and edge_iterator types;
//     have it hold a parallel iterator to the key map; change the increment/decrement
//     method

// we do a double lookup of key -> id -> value because:
//   - storing edge keys would be heavy (many duplications)
//   - a key -> value lookup is 2.5x the cost of a key -> id lookup, with key = std::string.
//     meaning edge iteration is much faster if edges refer to each other by ID.

namespace graph {

namespace detail {

#ifdef _GRAPH_TEST_HARNESS_INSTRUMENTATION
// class Instrumentation is a friend of DigraphMap, and can access its private members.
// this is used by the test classes to verify invariants about the internal state of
// the graph.
struct Instrumentation {
    template <
        typename A,
        typename B,
        typename C,
        typename D,
        template <typename...> typename Map
    >
    static auto keys_to_vert_id(const DigraphMap<A,B,C,D,Map>& map) {
        return map._vert_ids_by_key;
    }
    
    template <
        typename A,
        typename B,
        typename C,
        typename D,
        template <typename...> typename Map
    >
    static auto keys_to_edge_id(const DigraphMap<A,B,C,D,Map>& map) {
        return map._edge_ids_by_key;
    }
    
    template <
        typename A,
        typename B,
        typename C,
        typename D,
        template <typename...> typename Map
    >
    static auto vert_id_to_key(const DigraphMap<A,B,C,D,Map>& map) {
        return map._vert_keys_by_id;
    }
    
    template <
        typename A,
        typename B,
        typename C,
        typename D,
        template <typename...> typename Map
    >
    static auto edge_id_to_key(const DigraphMap<A,B,C,D,Map>& map) {
        return map._edge_keys_by_id;
    }
};
#endif

} // namespace detail


/**
 * @brief A digraph class that allows vertices and/or edges to be indexed by a custom type
 * in O(1) time.
 * 
 * Because this is a subclass, verts and edges will also continue to be indexable by VertexId
 * and EdgeId, respectively.
 * 
 * It is permitted for one of the key types to be void, in which case the corresponding
 * indexing methods will not be available.
 * 
 * As in the base class, value types for vertices and edges may each be void / omitted.
 * 
 * @tparam VertKey Key type for vertices.
 * @tparam VertVal Value type for vertices.
 * @tparam EdgeKey Key type for edges.
 * @tparam EdgeVal Value type for edges.
 * @tparam Map Map type to use for storing vertices and edges. Must support the same interface
 *   as std::unordered_map.
 */
template <
    typename VertKey,
    typename VertVal,
    typename EdgeKey,
    typename EdgeVal,
    template <class...> class Map=std::unordered_map>
struct DigraphMap : public Digraph<VertVal, EdgeVal, Map> {
private:
    static constexpr bool HasVertKey() { return !std::is_same<VertKey, void>::value; }
    static constexpr bool HasEdgeKey() { return !std::is_same<EdgeKey, void>::value; }
    
#ifdef _GRAPH_TEST_HARNESS_INSTRUMENTATION
    
    friend struct detail::Instrumentation;
    
#endif

    using Base       = Digraph<VertVal, EdgeVal, Map>;
    using VertKeyMap = Map<VertKey, VertexId>;
    using EdgeKeyMap = Map<EdgeKey, EdgeId>;
    using KeyVertMap = Map<VertexId, VertKey>;
    using KeyEdgeMap = Map<EdgeId,   EdgeKey>;
    
    using StoredVertKeyMap = std::conditional_t<HasVertKey(), VertKeyMap, int[0]>;
    using StoredEdgeKeyMap = std::conditional_t<HasEdgeKey(), EdgeKeyMap, int[0]>;
    using StoredKeyVertMap = std::conditional_t<HasVertKey(), KeyVertMap, int[0]>;
    using StoredKeyEdgeMap = std::conditional_t<HasEdgeKey(), KeyEdgeMap, int[0]>;
    
    StoredVertKeyMap _vert_ids_by_key;
    StoredEdgeKeyMap _edge_ids_by_key;
    
    StoredKeyVertMap _vert_keys_by_id;
    StoredKeyEdgeMap _edge_keys_by_id;
    
    Base*       _base()       { return static_cast<Base*>(this); }
    const Base* _base() const { return static_cast<const Base*>(this); }
    
    void _register_vert(VertexId v_id, const VertKey& key) requires (HasVertKey()) {
        _vert_ids_by_key.insert({key, v_id});
        _vert_keys_by_id.insert({v_id, key});
    }
    
    void _register_edge(EdgeId e_id, const EdgeKey& key) requires (HasEdgeKey()) {
        _edge_ids_by_key.insert({key, e_id});
        _edge_keys_by_id.insert({e_id, key});
    }
    
    void _unregister_vert(VertexId v_id) requires (HasVertKey()) {
        auto i = _vert_keys_by_id.find(v_id);
        if (i != _vert_keys_by_id.end()) {
            _vert_ids_by_key.erase(i->second);
            _vert_keys_by_id.erase(i);
        }
    }
    
    void _unregister_edge(EdgeId e_id) requires (HasEdgeKey()) {
        auto i = _edge_keys_by_id.find(e_id);
        if (i != _edge_keys_by_id.end()) {
            _edge_ids_by_key.erase(i->second);
            _edge_keys_by_id.erase(i);
        }
    }
    
    void _unregister_incident_edges(typename Base::Verts::iterator v_iter) {
        if constexpr (HasEdgeKey()) {
            // delete the keys for all edges incident to the deleted vert
            for (EdgeDir dir : {EdgeDir::Incoming, EdgeDir::Outgoing}) {
                auto e0 = this->begin_incident_edges({this, v_iter}, dir);
                if (e0 == this->end_incident_edges()) continue;
                EdgeId e0_id = e0;
                for (auto e = ++e0; true; ++e) {
                    EdgeId e_id = e;
                    _unregister_edge(e_id);
                    if (e_id == e0_id) break;
                }
            }
        }
    }
    
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
    const_vertex_ref operator[](const VertKey& key) const requires (HasVertKey()) {
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
    vertex_ref operator[](const VertKey& key) requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            auto v_i = _base()->insert_vertex();
            _register_vert(v_i->id(), key);
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
    const_edge_ref operator[](const EdgeKey& key) const requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            throw std::out_of_range("edge key not found");
        }
        return {this->_edges.find(i->second)};
    }
    
    /**
     * @brief Returns a reference to the edge with the given key.
     * 
     * If the key is not present in the graph, a `std::out_of_range` exception is thrown.
     */
    edge_ref operator[](const EdgeKey& key) requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            throw std::out_of_range("edge key not found");
        } else {
            return {this->_edges.find(i->second)};
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
    bool contains(const VertKey& key) const requires (HasVertKey()) {
        return _vert_ids_by_key.contains(key);
    }
    
    /// Returns true if an edge with the given key is present in the graph.
    bool contains(const EdgeKey& key) const requires (HasEdgeKey()) {
        return _edge_ids_by_key.contains(key);
    }
    
    const VertKey& key_for(VertexId v) const requires (HasVertKey()) {
        return _vert_keys_by_id.at(v);
    }
    
    const EdgeKey& key_for(EdgeId e) const requires (HasEdgeKey()) {
        return _edge_keys_by_id.at(e);
    }
    
    using Base::find_vertex;
    
    /**
     * @brief Returns an iterator to the vertex with the given key.
     * 
     * If the key is not present in the graph, returns `this->end_vertices()`.
     */
    vertex_iterator find_vertex(const VertKey& key) requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            return {this, this->_verts.end()};
        } else {
            return {this, this->_verts.find(i->second)};
        }
    }
    
    /**
     * @brief Returns a const iterator to the vertex with the given key.
     * 
     * If the key is not present in the graph, returns `this->end_vertices()`.
     */
    const_vertex_iterator find_vertex(const VertKey& key) const requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            return {this, this->_verts.end()};
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
    edge_iterator find_edge(const EdgeKey& key) requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            return {this, this->_edges.end()};
        } else {
            return {this, this->_edges.find(i->second)};
        }
    }
    
    /**
     * @brief Returns a const iterator to the edge with the given key.
     * 
     * If the key is not present in the graph, returns `this->end_edges()`.
     */
    const_edge_iterator find_edge(const EdgeKey& key) const requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            return {this, this->_edges.end()};
        } else {
            return {this, this->_edges.find(i->second)};
        }
    }
    
    using Base::vertex;
    
    /// Returns a reference to the vertex with the given key, or `std::nullopt` if not found.
    std::optional<vertex_ref> vertex(const VertKey& key) requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            return std::nullopt;
        } else {
            return {this, this->_verts.find(i->second)};
        }
    }
    
    /// Returns a const reference to the vertex with the given key, or `std::nullopt`
    /// if not found.
    std::optional<const_vertex_ref> vertex(const VertKey& key) const requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            return std::nullopt;
        } else {
            return {this, this->_verts.find(i->second)};
        }
    }
    
    /// Returns a reference to the edge with the given key, or `std::nullopt` if not found.
    std::optional<edge_ref> edge(const EdgeKey& key) requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            return std::nullopt;
        } else {
            return {this, this->_edges.find(i->second)};
        }
    }
    
    /// Returns a const reference to the edge with the given key, or `std::nullopt` if
    /// not found.
    std::optional<const_edge_ref> edge(const EdgeKey& key) const requires (HasEdgeKey()) {
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
    size_t erase(const VertKey& key) requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i != _vert_ids_by_key.end()) {
            auto v_iter = this->_verts.find(i->second);
            _unregister_incident_edges(v_iter);
            // delete the key for this vert
            VertexId v_id = i->second;
            _vert_ids_by_key.erase(i);
            _vert_keys_by_id.erase(v_id);
            // delete the vert
            _base()->erase(vertex_iterator{this, v_iter});
            return 1;
        } else {
            return 0;
        }
    }
    
    auto erase(vertex_iterator vert) {
        _unregister_incident_edges(vert.inner_iterator());
        if constexpr (HasVertKey()) {
            _unregister_vert(vert->id());
        }
        return _base()->erase(vert);
    }
    
    auto erase(VertexId vert) {
        return this->erase(find_vertex(vert));
    }
    
    /**
     * @brief Removes the edge with the given key from the graph, if present.
     * 
     * Returns the number of edges (0 or 1) removed from the graph.
     */
    size_t erase(const EdgeKey& key) requires (HasEdgeKey()) {
        auto i = _edge_ids_by_key.find(key);
        if (i != _edge_ids_by_key.end()) {
            EdgeId e_id = i->second;
            _edge_keys_by_id.erase(e_id);
            _edge_ids_by_key.erase(i);
            this->erase(edge_iterator{this, this->_edges.find(e_id)});
            return 1;
        } else {
            return 0;
        }
    }
    
    auto erase(incident_edge_iterator edge) {
        if constexpr (HasEdgeKey()) {
            _unregister_edge(edge->id());
        }
        return _base()->erase(edge);
    }
    
    auto erase(edge_iterator edge) {
        if constexpr (HasEdgeKey()) {
            _unregister_edge(edge->id());
        }
        return _base()->erase(edge);
    }
    
    auto erase(EdgeId edge) {
        return this->erase(find_edge(edge));
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
    std::pair<vertex_iterator, bool> insert_vertex(const VertKey& key) requires (HasVertKey()) {
        auto i = _vert_ids_by_key.find(key);
        if (i != _vert_ids_by_key.end()) {
            return {vertex_iterator{this, this->_verts.find(i->second)}, false};
        } else {
            auto v_i = _base()->insert_vertex();
            _register_vert(v_i->id(), key);
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
    template <Forwardable<VertVal> T>
    std::pair<vertex_iterator, bool> insert_vertex(const VertKey& key, T&& value)
        requires (HasVertKey() and HasVertexValue())
    {
        auto i = _vert_ids_by_key.find(key);
        if (i != _vert_ids_by_key.end()) {
            auto j = this->_verts.find(i->second);
            return {vertex_iterator{this, j}, false};
        } else {
            auto v_i = _base()->insert_vertex(std::forward<T>(value));
            _register_vert(v_i->id(), key);
            return {v_i, true};
        }
    }
    
    /**
     * @brief Inserts a vertex with the given value into the graph.
     * 
     * Returns an iterator to the new vertex.
     */
    template <Forwardable<VertVal> T>
    vertex_iterator insert_vertex(T&& value) requires (not HasVertKey() and HasVertexValue()) {
        return _base()->insert_vertex(std::forward<T>(value));
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
    std::pair<vertex_iterator, bool> emplace_vertex(const VertKey& key, Args&&... args)
        requires (HasVertKey() and HasVertexValue())
    {
        auto i = _vert_ids_by_key.find(key);
        if (i != _vert_ids_by_key.end()) {
            auto j = this->_verts.find(i->second);
            return {vertex_iterator{this, j}, false};
        } else {
            auto v_i = _base()->emplace_vertex(std::forward<Args>(args)...);
            _register_vert(v_i->id(), key);
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
    template <Forwardable<VertVal> T>
    std::pair<vertex_iterator, bool> insert_or_assign_vertex(const VertKey& key, T&& value)
        requires (HasVertKey() and HasVertexValue())
    {
        auto i = _vert_ids_by_key.find(key);
        if (i != _vert_ids_by_key.end()) {
            auto j = this->_verts.find(i->second);
            j->value() = value;
            return {vertex_iterator{this, j}, false};
        } else {
            auto v_i = _base()->insert_vertex(std::forward<T>(value));
            _register_vert(v_i->id(), key);
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
            const EdgeKey& new_key,
            vertex_iterator src,
            vertex_iterator dst)
        requires (HasEdgeKey())
    {
        auto i = _edge_ids_by_key.find(new_key);
        if (i != _edge_ids_by_key.end()) {
            // insertion blocked by existing key
            return {
                incident_edge_iterator {
                    this,
                    this->_edges.find(i->second),
                    EdgeDir::Outgoing
                },
                false
            };
        }
        auto e_i = _base()->insert_directed_edge(src, dst);
        _register_edge(e_i->id(), new_key);
        return { e_i, true };
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
    template <Forwardable<EdgeVal> T>
    std::pair<incident_edge_iterator, bool> insert_directed_edge(
            const EdgeKey& new_key,
            vertex_iterator src,
            vertex_iterator dst,
            T&& value)
        requires (HasEdgeKey() and HasEdgeValue())
    {
        auto i = _edge_ids_by_key.find(new_key);
        if (i != _edge_ids_by_key.end()) {
            // insertion blocked by existing key
            return {
                incident_edge_iterator {
                    this,
                    this->_edges.find(i->second),
                    EdgeDir::Outgoing
                },
                false
            };
        }
        auto e_i = _base()->insert_directed_edge(src, dst, std::forward<T>(value));
        _register_edge(e_i->id(), new_key);
        return { e_i, true };
    }
    
    template <Forwardable<EdgeVal> T>
    std::pair<incident_edge_iterator, bool> insert_directed_edge(
            const EdgeKey& new_key,
            const VertKey& src_key,
            const VertKey& dst_key,
            T&& value)
        requires (HasEdgeKey() and HasEdgeValue() and HasVertKey())
    {
        return insert_directed_edge(
            new_key,
            this->find_vertex(src_key),
            this->find_vertex(dst_key),
            std::forward<T>(value)
        );
    }
    
    template <Forwardable<EdgeVal> T>
    std::pair<incident_edge_iterator, bool> insert_directed_edge(
            const VertKey& src_key,
            const VertKey& dst_key,
            T&& value)
        requires (not HasEdgeKey() and HasEdgeValue() and HasVertKey())
    {
        return insert_directed_edge(
            this->find_vertex(src_key),
            this->find_vertex(dst_key),
            std::forward<T>(value)
        );
    }
    
    std::pair<incident_edge_iterator, bool> insert_directed_edge(
            const EdgeKey& new_key,
            const VertKey& src_key,
            const VertKey& dst_key)
        requires (HasEdgeKey() and HasVertKey())
    {
        return insert_directed_edge(
            new_key,
            this->find_vertex(src_key),
            this->find_vertex(dst_key)
        );
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
            const EdgeKey& new_key,
            vertex_iterator src,
            vertex_iterator dst,
            Args&&... args)
        requires (HasEdgeKey() and HasEdgeValue())
    {
        auto i = _edge_ids_by_key.find(new_key);
        if (i != _edge_ids_by_key.end()) {
            // key exists; insertion blocked; return existing edge
            return {
                incident_edge_iterator {
                    this,
                    this->_edges.find(i->second),
                    EdgeDir::Outgoing
                },
                false
            };
        }
        auto e_i = _base()->emplace_directed_edge(src, dst, std::forward<Args>(args)...);
        _register_edge(e_i->id(), new_key);
        return { e_i, true };
    }
    
    template <typename... Args>
    std::pair<incident_edge_iterator, bool> emplace_directed_edge(
            const EdgeKey& new_key,
            const VertKey& src_key,
            const VertKey& dst_key,
            Args&&... args)
        requires (HasEdgeKey() and HasEdgeValue() and HasVertKey())
    {
        return emplace_directed_edge(
            new_key,
            this->find_vertex(src_key),
            this->find_vertex(dst_key),
            std::forward<Args>(args)...
        );
    }
    
    template <typename... Args>
    std::pair<incident_edge_iterator, bool> emplace_directed_edge(
            vertex_iterator src,
            vertex_iterator dst,
            Args&&... args)
        requires (not HasEdgeKey() and HasEdgeValue())
    {
        return _base()->emplace_directed_edge(src, dst, std::forward<Args>(args)...);
    }
    
    template <typename... Args>
    std::pair<incident_edge_iterator, bool> emplace_directed_edge(
            const VertKey& src_key,
            const VertKey& dst_key,
            Args&&... args)
        requires (not HasEdgeKey() and HasEdgeValue() and HasVertKey())
    {
        return _base()->emplace_directed_edge(
            this->find_vertex(src_key),
            this->find_vertex(dst_key),
            std::forward<Args>(args)...
        );
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
