#pragma once

#include <graph/digraph.h>

// todo: key iteration
//   - make an iterator which wraps the underlying key-id iterators but only returns the key.
//     (I think the stdlib has some unities that will construct this easily).
//   - also: find_{vert,edge}(key_iter) should be overloaded to use the inner map iterator
//     directly, saving a key lookup.

// todo: we store the keys in two places. a careful implementation might be able to share them

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


enum struct EdgeChange {
    CREATED,
    REDIRECTED,
    MODIFIED
};


/**
 * @brief A digraph class that allows vertices and/or edges to be indexed by a custom type
 * in O(1) time.
 * 
 *
 *     #include <graph/digraph_map.h>
 * 
 * It is permitted for one of the key types to be void, in which case the corresponding
 * indexing methods will not be available.
 * 
 * Vertices and edges will additionally be indexable by VertexId and EdgeId, respectively.
 * 
 * As in the base class, value types for vertices and edges may each be void / omitted.
 * 
 * DigraphMap and Digraph inherit from the same (hidden) base class, so they have the same
 * interface. It is not possible to convert between the two types (even by pointer casting),
 * because the mutation methods of DigraphMap have been carefully overridden to update the key
 * maps.
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
struct DigraphMap : public detail::DigraphBase<VertVal, EdgeVal, Map> {
    
    using HasVertKey = std::bool_constant<not std::is_same<VertKey, void>::value>;
    using HasEdgeKey = std::bool_constant<not std::is_same<EdgeKey, void>::value>;
    
    // we have to typedef these because in the case where they're void, the functions
    // that use them will produce compile errors.
    using EdgeKeyRef = std::conditional_t<HasEdgeKey::value, EdgeKey, int>&;
    using VertKeyRef = std::conditional_t<HasVertKey::value, VertKey, int>&;
    using ConstEdgeKeyRef = std::conditional_t<HasEdgeKey::value, const EdgeKey, int>&;
    using ConstVertKeyRef = std::conditional_t<HasVertKey::value, const VertKey, int>&;
    
private:
    
#ifdef _GRAPH_TEST_HARNESS_INSTRUMENTATION
    friend struct detail::Instrumentation;
#endif

    using Base       = detail::DigraphBase<VertVal, EdgeVal, Map>;
    using VertKeyMap = Map<VertKey, VertexId>;
    using EdgeKeyMap = Map<EdgeKey, EdgeId>;
    using KeyVertMap = Map<VertexId, VertKey>;
    using KeyEdgeMap = Map<EdgeId,   EdgeKey>;
    
    using StoredVertKeyMap = std::conditional_t<HasVertKey::value, VertKeyMap, int[0]>;
    using StoredEdgeKeyMap = std::conditional_t<HasEdgeKey::value, EdgeKeyMap, int[0]>;
    using StoredKeyVertMap = std::conditional_t<HasVertKey::value, KeyVertMap, int[0]>;
    using StoredKeyEdgeMap = std::conditional_t<HasEdgeKey::value, KeyEdgeMap, int[0]>;
    
    StoredVertKeyMap _vert_ids_by_key;
    StoredEdgeKeyMap _edge_ids_by_key;
    
    StoredKeyVertMap _vert_keys_by_id;
    StoredKeyEdgeMap _edge_keys_by_id;
    
    Base*       _base()       { return static_cast<Base*>(this); }
    const Base* _base() const { return static_cast<const Base*>(this); }
    
    void _register_vert(VertexId v_id, ConstVertKeyRef key) requires (HasVertKey::value) {
        _vert_ids_by_key.insert({key, v_id});
        _vert_keys_by_id.insert({v_id, key});
    }
    
    void _register_edge(EdgeId e_id, ConstEdgeKeyRef key) requires (HasEdgeKey::value) {
        _edge_ids_by_key.insert({key, e_id});
        _edge_keys_by_id.insert({e_id, key});
    }
    
    void _unregister_vert(VertexId v_id) requires (HasVertKey::value) {
        auto i = _vert_keys_by_id.find(v_id);
        if (i != _vert_keys_by_id.end()) {
            _vert_ids_by_key.erase(i->second);
            _vert_keys_by_id.erase(i);
        }
    }
    
    void _unregister_edge(EdgeId e_id) requires (HasEdgeKey::value) {
        auto i = _edge_keys_by_id.find(e_id);
        if (i != _edge_keys_by_id.end()) {
            _edge_ids_by_key.erase(i->second);
            _edge_keys_by_id.erase(i);
        }
    }
    
    void _unregister_incident_edges(typename Base::Verts::iterator v_iter) {
        if constexpr (HasEdgeKey::value) {
            // delete the keys for all edges incident to the deleted vert
            for (EdgeDir dir : {EdgeDir::Incoming, EdgeDir::Outgoing}) {
                for (auto e = this->begin_incident_edges({this, v_iter}, dir);
                     e != this->end_edges();
                     ++e)
                {
                    _unregister_edge(e->id());
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
    
    using typename Base::edge_endpoint;
    using typename Base::const_edge_endpoint;
    
    using typename Base::HasVertexValue;
    using typename Base::HasEdgeValue;
    
    /**
     * @brief Returns a const reference to the vertex with the given key.
     * 
     * If the key is not present in the graph, a `std::out_of_range` exception is thrown.
     */
    const_vertex_ref operator[](ConstVertKeyRef key) const requires (HasVertKey::value) {
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
    vertex_ref operator[](ConstVertKeyRef key) requires (HasVertKey::value) {
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
    const_edge_ref operator[](ConstEdgeKeyRef key) const requires (HasEdgeKey::value) {
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
    edge_ref operator[](ConstEdgeKeyRef key) requires (HasEdgeKey::value) {
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
    bool contains(ConstVertKeyRef key) const requires (HasVertKey::value) {
        return _vert_ids_by_key.contains(key);
    }
    
    /// Returns true if an edge with the given key is present in the graph.
    bool contains(ConstEdgeKeyRef key) const requires (HasEdgeKey::value) {
        return _edge_ids_by_key.contains(key);
    }
    
    /**
     * @brief Returns the key for the vertex with the given Id.
     * 
     * Throws `std::out_of_range` if the vertex is not in the graph.
     */
    ConstVertKeyRef key_for(VertexId v) const requires (HasVertKey::value) {
        return _vert_keys_by_id.at(v);
    }
    
    /**
     * @brief Returns the key for the edge with the given Id.
     * 
     * Throws `std::out_of_range` if the edge is not in the graph.
     */
    ConstEdgeKeyRef key_for(EdgeId e) const requires (HasEdgeKey::value) {
        return _edge_keys_by_id.at(e);
    }
    
    using Base::find_vertex;
    
    /**
     * @brief Returns an iterator to the vertex with the given key.
     * 
     * If the key is not present in the graph, returns `this->end_vertices()`.
     */
    vertex_iterator find_vertex(ConstVertKeyRef key) requires (HasVertKey::value) {
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
    const_vertex_iterator find_vertex(ConstVertKeyRef key) const requires (HasVertKey::value) {
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
    edge_iterator find_edge(ConstEdgeKeyRef key) requires (HasEdgeKey::value) {
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
    const_edge_iterator find_edge(ConstEdgeKeyRef key) const requires (HasEdgeKey::value) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            return {this, this->_edges.end()};
        } else {
            return {this, this->_edges.find(i->second)};
        }
    }
    
    using Base::vertex;
    
    /// Returns a reference to the vertex with the given key, or `std::nullopt` if not found.
    std::optional<vertex_ref> vertex(ConstVertKeyRef key) requires (HasVertKey::value) {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            return std::nullopt;
        } else {
            return {this, this->_verts.find(i->second)};
        }
    }
    
    /// Returns a const reference to the vertex with the given key, or `std::nullopt`
    /// if not found.
    std::optional<const_vertex_ref> vertex(ConstVertKeyRef key) const
            requires (HasVertKey::value)
    {
        auto i = _vert_ids_by_key.find(key);
        if (i == _vert_ids_by_key.end()) {
            return std::nullopt;
        } else {
            return {this, this->_verts.find(i->second)};
        }
    }
    
    /// Returns a reference to the edge with the given key, or `std::nullopt` if not found.
    std::optional<edge_ref> edge(ConstEdgeKeyRef key) requires (HasEdgeKey::value) {
        auto i = _edge_ids_by_key.find(key);
        if (i == _edge_ids_by_key.end()) {
            return std::nullopt;
        } else {
            return {this, this->_edges.find(i->second)};
        }
    }
    
    /// Returns a const reference to the edge with the given key, or `std::nullopt` if
    /// not found.
    std::optional<const_edge_ref> edge(ConstEdgeKeyRef key) const requires (HasEdgeKey::value) {
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
     * This operation is `O(degree(v))`, where `v` is the vertex indexed by `key`.
     * 
     * Returns the number of vertices (0 or 1) removed from the graph.
     */
    size_t erase(ConstVertKeyRef key) requires (HasVertKey::value) {
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
        if constexpr (HasVertKey::value) {
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
     * Returns the value associated with the deleted edge, or `std::nullopt` if the edge
     * was not present in the graph.
     */
    std::optional<EdgeVal> erase(ConstEdgeKeyRef key)
        requires (HasEdgeKey::value and HasEdgeValue::value)
    {
        auto i = _edge_ids_by_key.find(key);
        if (i != _edge_ids_by_key.end()) {
            EdgeId e_id = i->second;
            _edge_keys_by_id.erase(e_id);
            _edge_ids_by_key.erase(i);
            auto e_iter = this->_edges.find(e_id);
            if (e_iter != this->_edges.end()) {
                EdgeVal ev {std::move(e_iter->second)};
                this->erase(edge_iterator{this, e_iter});
                return ev;
            } else {
                return std::nullopt;
            }
        } else {
            return std::nullopt;
        }
    }
    
    /**
     * @brief Removes the edge with the given key from the graph, if present.
     * 
     * Returns the number of edges (0 or 1) removed from the graph.
     */
    size_t erase(ConstEdgeKeyRef key) requires (HasEdgeKey::value and not HasEdgeValue::value) {
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
    
    /// Removes the given edge from the graph.
    auto erase(edge_iterator edge) {
        if constexpr (HasEdgeKey::value) {
            _unregister_edge(edge->id());
        }
        return _base()->erase(edge);
    }
    
    /// Removes the edge with the given ID from the graph.
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
    std::pair<vertex_iterator, bool> insert_vertex(ConstVertKeyRef key)
            requires (HasVertKey::value)
    {
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
    std::pair<vertex_iterator, bool> insert_vertex(ConstVertKeyRef key, T&& value)
        requires (HasVertKey::value and HasVertexValue::value)
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
    vertex_iterator insert_vertex(T&& value)
            requires (not HasVertKey::value and HasVertexValue::value)
    {
        return _base()->insert_vertex(std::forward<T>(value));
    }
    
    vertex_iterator insert_vertex()
            requires (not HasVertKey::value and not HasVertexValue::value)
    {
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
    std::pair<vertex_iterator, bool> emplace_vertex(ConstVertKeyRef key, Args&&... args)
        requires (HasVertKey::value and HasVertexValue::value)
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
        requires (not HasVertKey::value and HasVertexValue::value)
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
    std::pair<vertex_iterator, bool> insert_or_assign_vertex(ConstVertKeyRef key, T&& value)
        requires (HasVertKey::value and HasVertexValue::value)
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
            ConstEdgeKeyRef new_key,
            vertex_iterator src,
            vertex_iterator dst,
            Args&&... args)
        requires (HasEdgeKey::value)
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
    
    /// Inserts a new edge from an `EdgeKey` and two `VertKey`s.
    template <typename... Args>
    std::pair<incident_edge_iterator, bool> emplace_directed_edge(
            ConstEdgeKeyRef new_key,
            ConstVertKeyRef src_key,
            ConstVertKeyRef dst_key,
            Args&&... args)
        requires (HasEdgeKey::value and HasVertKey::value)
    {
        return emplace_directed_edge(
            new_key,
            this->find_vertex(src_key),
            this->find_vertex(dst_key),
            std::forward<Args>(args)...
        );
    }
    
    /**
     * @brief Inserts a new un-keyed edge from two vertex iterators.
     * 
     * Not available if `HasEdgeKey::value == true`.
     */
    template <typename... Args>
    std::pair<incident_edge_iterator, bool> emplace_directed_edge(
            vertex_iterator src,
            vertex_iterator dst,
            Args&&... args)
        requires (not HasEdgeKey::value)
    {
        return _base()->emplace_directed_edge(src, dst, std::forward<Args>(args)...);
    }
    
    /**
     * @brief Inserts a new un-keyed edge from two vertex keys.
     * 
     * Not available if `HasEdgeKey::value == true`.
     */
    template <typename... Args>
    std::pair<incident_edge_iterator, bool> emplace_directed_edge(
            ConstVertKeyRef src_key,
            ConstVertKeyRef dst_key,
            Args&&... args)
        requires (not HasEdgeKey::value and HasVertKey::value)
    {
        return _base()->emplace_directed_edge(
            this->find_vertex(src_key),
            this->find_vertex(dst_key),
            std::forward<Args>(args)...
        );
    }
    
    /**
     * @brief Insert a new edge with the given key into the graph before each given edge.
     * 
     * Each vertex argument can be either a vertex or an edge iterator. If it is a vertex, the
     * edge will be inserted before the first edge in the given direction. If it is
     * an edge, the new edge will be inserted before the given edge, and the vertex
     * will be taken from the same endpoint.
     * 
     * The `before_outgoing` edge provides the source vertex for the new edge, and the
     * `before_incoming` edge provides the target vertex.
     * 
     * If the graph has edge values, the new edge will be constructed in-place with the
     * given arguments.
     */
    template <typename... Args>
    std::pair<incident_edge_iterator, bool> emplace_directed_edge_before(
            ConstEdgeKeyRef new_key,
            edge_endpoint src,
            edge_endpoint dst,
            Args&&... args)
        requires (HasEdgeKey::value)
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
        auto e_i = _base()->emplace_directed_edge_before(
            src,
            dst,
            std::forward<Args>(args)...
        );
        _register_edge(e_i->id(), new_key);
        return { e_i, true };
    }
    
    /**
     * @brief Insert an un-keyed edge at a specific place in the edge sequence.
     * 
     * The arguments are the same as for `emplace_directed_edge_before()`, except that
     * the `new_key` argument is omitted.
     * 
     * Not available if `HasEdgeKey::value == true`.
     */
    template <typename... Args>
    std::pair<incident_edge_iterator, bool> emplace_directed_edge_before(
            edge_endpoint src,
            edge_endpoint dst,
            Args&&... args)
        requires (not HasVertKey::value)
    {
        return _base()->emplace_directed_edge_before(src, dst, std::forward<Args>(args)...);
    }
    
    std::pair<incident_edge_iterator, incident_edge_iterator> insert_undirected_edge(
            vertex_iterator src,
            vertex_iterator dst)
        requires (not HasEdgeKey::value and not HasEdgeValue::value)
    {
        return _base()->insert_undirected_edge(src, dst);
    }
    
    /**
     * @brief Associate the edge key with a directed edge between the given vertices.
     * 
     * If an edge with the given key already exists, replace it, and construct its
     * associated value from the given arguments.
     * 
     * If no such edge exists, emplace one using the given arguments.
     * 
     * Return an iterator to the edge, and a bool indicating whether the edge was inserted
     * (true) or existed already (false).
     * 
     * To prefer keeping the previous edge value in the case where it exists, use
     * `try_redirect_edge()`.
     * 
     * If the edge endpoints were changed, the EdgeId associated with the key will change
     * as well.
     */
    template <typename... Args>
    std::pair<edge_iterator, EdgeChange> redirect_edge(
            ConstEdgeKeyRef key,
            vertex_iterator src,
            vertex_iterator dst,
            Args... args)
        requires (HasEdgeKey::value)
    {
        auto e = this->find_edge(key);
        if (e != this->end_edges()) {
            if (e->source_id() == src->id() and e->target_id() == dst) {
                // edge already exists with this key and endpoints
                if constexpr (HasVertexValue::value) {
                    // replace the existing value
                    e->value() = VertVal{std::forward<Args>(args)...};
                }
                return {e, EdgeChange::MODIFIED};
            } else {
                // edge exists, but points to the wrong verts
                this->erase(e);
                return std::make_pair(
                    this->emplace_directed_edge(
                            key,
                            src,
                            dst,
                            std::forward<Args>(args)...
                        ).first,
                    EdgeChange::REDIRECTED
                );
            }
        } else {
            // edge does not exist. emplace it.
            return std::make_pair(
                this->emplace_directed_edge(key, src, dst, std::forward<Args>(args)...).first,
                EdgeChange::CREATED
            );
        }
    }
    
    /**
     * @brief Change the endpoints of the edge with the given key, preserving the edge's
     * value if one exists.
     * 
     * If the edge does not exist, insert one having a value constructed from the given
     * arguments.
     * 
     * If the edge does exist, it will be redirected, and the existing value will be preserved.
     * 
     * To unconditionally replace the existing value, use `redirect_edge()`.
     * 
     * If the edge endpoints were changed, the EdgeId associated with the key will change as
     * well.
     * 
     * If this graph does not contain edge values, this function is equivalent to
     * `redirect_edge()`, and is therefore not available.
     */
    template <typename... Args>
    std::pair<incident_edge_iterator, bool> try_redirect_edge(
            ConstEdgeKeyRef key,
            vertex_iterator src,
            vertex_iterator dst,
            Args... args)
        requires (HasEdgeKey::value and HasEdgeValue::value)
    {
        auto e = this->find_edge(key);
        if (e != this->end_edges()) {
            // edge exists; save the existing value
            EdgeVal v { std::move(e->value()) };
            this->erase(e);
            return 
                std::make_pair(
                    this->emplace_directed_edge(key, src, dst, std::move(v)),
                    false
                );
        } else {
            // edge does not exist, emplace one
            return std::make_pair(
                this->emplace_directed_edge(key, src, dst, std::forward<Args>(args)...).first,
                true
            );
        }
    }
    
private:
    // "delete" the undirected edge helper; we can't store multiple edges under the same key.
    using Base::insert_undirected_edge;
    
};

} // namespace graph
