#pragma once

#include <optional>
#include <type_traits>

// todo: I don't think we need the tail pointer in the edge list
// todo: I don't like the in-band signaling of INVALID_{VERTEX, EDGE}.
//    try moving back to std::optional
// todo: testing. exercise all the code paths
// todo: make Digraph and GraphMap inherit from a common [private] base class but not each other.
//    it must not be possible to slice a GraphMap to a Digraph and edit it; as this would
//    invalidate the key-value mapping.
//    > consider also the access granted by shared iterator and range classes.
// todo: wbn to have a pre-built class that indexes edges by Pair<VertexId>
// todo: subclass of graph_map which can do reverse indexing
// todo: expose pseudo-containers for the verts and edges?
//    would allow disambiguation for operator[], etc; and a lot of
//    cruft would be teased apart.
//    concern: you can construct an incident edge iter directly from a vertex_iterator,
//      but can't construct a range. the former is in some ways more efficient.
//      you could make another range around iterators instead of indices, but that raises
//      subtle questions of invalidation which we'd prefer not to introduce.
// todo: should it be possible to keep an edge Id but change the endpoints?
//   > feels like this is something for the K/V thing above
// todo: other methods:
//   - merge(v1, v2)
//   - split(edge) (adds vert)
// todo: incident ranges should default to limited; use views::take?
// todo: add an `edges_repeating()` which returns the kind of range the current one does
// todo: add an `incident_edges()` which concats incoming and outgoing edge ranges

namespace graph {

namespace detail {

template <typename Ref>
struct arrow_proxy {
    Ref ref;
    
    Ref* operator->() { return &ref; }
};

} // namespace detail


    
// concepts
template <typename B, typename T>
concept Forwardable = std::convertible_to<B, std::remove_reference_t<T>>;

/// @brief Unique identifier for a vertex in a graph.
enum struct VertexId : uint64_t {};
/// @brief Unique identifier for an edge in a graph.
enum struct EdgeId   : uint64_t {};

constexpr VertexId INVALID_VERTEX = static_cast<VertexId>(~(uint64_t) 0);
constexpr EdgeId   INVALID_EDGE   = static_cast<EdgeId>  (~(uint64_t) 0);

VertexId operator+(VertexId v, uint64_t n) {
    return static_cast<VertexId>(static_cast<uint64_t>(v) + n);
}

VertexId operator+(VertexId v0, VertexId v1) {
    return static_cast<VertexId>(static_cast<uint64_t>(v0) + static_cast<uint64_t>(v1));
}

VertexId& operator+=(VertexId& v, uint64_t n) {
    v = v + n;
    return v;
}

VertexId operator++(VertexId& v, int) {
    VertexId old = v;
    v = v + 1;
    return old;
}

EdgeId operator+(EdgeId e, uint64_t n) {
    return static_cast<EdgeId>(static_cast<uint64_t>(e) + n);
}

EdgeId operator+(EdgeId e0, EdgeId e1) {
    return static_cast<EdgeId>(static_cast<uint64_t>(e0) + static_cast<uint64_t>(e1));
}

EdgeId& operator+=(EdgeId& e, uint64_t n) {
    e = e + n;
    return e;
}

EdgeId operator++(EdgeId& e, int) {
    EdgeId old = e;
    e = e + 1;
    return old;
}

/**
 * @brief A directed edge between two vertices.
 */
struct Edge {
    /// @brief The source vertex of the edge.
    VertexId v0;
    /// @brief The target vertex of the edge.
    VertexId v1;
    
    constexpr bool operator==(const Edge& other) const {
        return v0 == other.v0 and v1 == other.v1;
    }
};

/**
 * @brief The direction of an edge.
 * 
 * Incoming edges are incident to the target vertex; outgoing edges are incident
 * to the source vertex.
 */
enum struct EdgeDir {
    Incoming = 0,
    Outgoing = 1,
};

EdgeDir operator~(EdgeDir dir) {
    return dir == EdgeDir::Incoming ? EdgeDir::Outgoing : EdgeDir::Incoming;
}
    
enum struct Constness {
    Mutable,
    Const,
};

constexpr Constness operator~(Constness c) {
    return (c == Constness::Mutable) ? Constness::Const : Constness::Mutable;
}

inline size_t hash_combine(size_t h0, size_t h1) {
    if constexpr (sizeof(size_t) == 8) {
        /*
        // this offers a solution which doesn't use
        // a variant of the shitty boost combine: 
        // https://stackoverflow.com/questions/8513911/
        // but doesn't justify it.
        constexpr size_t k = 0x9E3779B97F4A7C15ULL;
        size_t a = (h0 ^ h1) * k;
        a ^= (a >> 47);
        size_t b = (h0 ^ a) * k;
        b ^= (b >> 47);
        return b * k;
        */
       
        // constant from: https://stackoverflow.com/questions/5889238/
        h0 ^= h1 + 0x517cc1b727220a95 + (h0 << 12) + (h0 >> 4);
        return h0;
    } else if constexpr (sizeof(size_t) == 4) {
        // boost's hash_combine. everyone uses this even though it's known to be shit.
        // i am not a cryptographer, though, so I don't expect to do much better myself.
        // we should make this better, though
        h0 ^= h1 + 0x9e3779b9U + (h0 << 6) + (h0 >> 2);
        return h0;
    } else if constexpr (sizeof(size_t) == 2) {
        // same as above. ugh
        h0 ^= h1 + 0x9e37U + (h0 << 3) + (h0 >> 1);
        return h0;
    } else {
        static_assert(sizeof(size_t) > 2, "size_t size not supported for hashing");
    }
}


template <
        typename VertKey,
        typename VertVal,
        typename EdgeKey,
        typename EdgeVal,
        template <class...> class Map>
struct DigraphMap;


} // namespace graph

namespace std {

template <>
struct hash<graph::Edge> {
    size_t operator()(const graph::Edge& e) const {
        return graph::hash_combine(
            std::hash<uint64_t>{}(static_cast<uint64_t>(e.v0)),
            std::hash<uint64_t>{}(static_cast<uint64_t>(e.v1))
        );
    }
};
    
} // namespace std


namespace graph {

/**
 * @brief A class representing a directed graph.
 * 
 * Iterator invalidation follows the same rules as the underlying map type, since
 * all iterators wrap the iterators of the underlying map.
 * 
 * Indexing by a valid iterator will generally be more performant than indexing by ID.
 * 
 * Indexing by ID will always be stable, regardless of mutations to other vertices and edges.
 * IDs are 64-bit and are never re-used. Indexing by a disused ID will return the corresponding
 * `end()` iterator.
 * 
 * The IDs `graph::INVALID_EDGE` and `graph::INVALID_VERTEX` will never be assigned to
 * any vertices or edges. Indexing by these IDs, as with any other unassigned ID, will
 * return the corresponding `end()` iterator.
 * 
 * Iterators to vertices are independent of iterators to edges; i.e. mutations to the set of
 * edges will not invalidate vertex iterators. Additions to the set of vertices will not
 * invalidate edge iterators; however, deletion of vertices will also delete all edges
 * incident to that vertex, which in turn *may* invalidate edge iterators (depending on the
 * iterator stability guarantees of the underlying map).
 * 
 * @tparam V Type associated with vertices (may be `void`).
 * @tparam E Type associated with edges (may be `void`).
 * @tparam Map Template class of an associative map type used to store the graph.
 * Defaults to `std::unordered_map`.
 */
template <
    typename V=void,
    typename E=void,
    template<class...> class Map=std::unordered_map>
struct Digraph {
    
    static constexpr bool HasVertexValue() { return not std::same_as<V, void>; }
    static constexpr bool HasEdgeValue()   { return not std::same_as<E, void>; }
    
private:

    using EdgeData   = std::conditional_t<std::same_as<E, void>, int[0], E>;
    using VertexData = std::conditional_t<std::same_as<V, void>, int[0], V>;
    
    template <typename VertKey, typename VertVal, typename EdgeKey, typename EdgeVal, template <class...> class M>
    friend struct DigraphMap;
    
    /**
     * @brief A node in a linked list of edges.
     * 
     * Each edge node links to the next edge in two lists: One of the outgoing edges
     * around the source vertex, and one of the incoming edges around the target
     * vertex.
     * 
     * If the edge is the last in the list, it points to itself.
     * 
     * If this graph stores edge values, the edge data is stored in `data`.
     */
    struct EdgeNode {
        Edge edge;
        union {
            EdgeId next[2] = {INVALID_EDGE, INVALID_EDGE};
            struct {
                EdgeId next_incoming;
                EdgeId next_outgoing;
            };
        };
        EdgeData data;
    };
    
    /**
     * @brief The head and tail of a linked list of edges.
     */
    struct EdgeList {
        EdgeId head;
        EdgeId tail;
        size_t size;
    };
    
    /**
     * @brief A node holding information about a vertex.
     * 
     * Each vertex node carries the head and tail of two linked lists of edges:
     * One of the incoming edges around the vertex, and one of the outgoing edges.
     * 
     * If there are no edges in a list, the head and tail point to `std::nullopt`.
     * 
     * If this graph stores vertex values, the vertex data is stored in `data`.
     */
    struct VertexNode {
        union {
            EdgeList edges[2];
            struct {
                EdgeList incoming_edges;
                EdgeList outgoing_edges;
            };
        };
        VertexData data;
        
        VertexNode():
            incoming_edges{INVALID_EDGE, INVALID_EDGE, 0},
            outgoing_edges{INVALID_EDGE, INVALID_EDGE, 0} {}
        
        template <typename... Args>
        VertexNode(Args... args): VertexNode(), data(std::forward<Args>(args)...) {}
    };
    
    using Edges = Map<EdgeId,   EdgeNode>;
    using Verts = Map<VertexId, VertexNode>;
    
    Verts    _verts;
    Edges    _edges;
    VertexId _free_vertex_id = static_cast<VertexId>(0);
    EdgeId   _free_edge_id   = static_cast<EdgeId>(0);

public:

    // fwd decls

    template <Constness Const>
    struct EdgeIterator;

    template <Constness Const>
    struct IncidentEdgeIterator;
    
    template <Constness Const>
    struct EdgeRange;
    
    template <Constness Const>
    struct IncidentEdgeRange;
    
    template <Constness Const>
    struct VertexIterator;
    
    template <Constness Const>
    struct VertexRange;
    
    
    // reference types

    template <Constness Const>
    struct EdgeRef {
    private:
        
        constexpr static bool IsConst() { return Const == Constness::Const; }
        using Graph = std::conditional_t<IsConst(), const Digraph, Digraph>;
        using Iter  = std::conditional_t<
            IsConst(),
            typename Edges::const_iterator,
            typename Edges::iterator
        >;
        using Value = std::conditional_t<IsConst(), const E, E>;
        using Node  = std::conditional_t<IsConst(), const EdgeNode, EdgeNode>;
        
        friend IncidentEdgeIterator<Const>;
        friend IncidentEdgeIterator<~Const>;
        friend Graph;
        template <
            typename VertKey,
            typename VertVal,
            typename EdgeKey,
            typename EdgeVal,
            template <class...> class M>
        friend struct DigraphMap;
        
        Iter _i;
        
        EdgeRef(Iter i): _i(i) {}
        
        Node& node() const { return _i->second; }
        Iter  inner_iterator() const { return _i; }
        
    public:
        
        /// The ID of the edge.
        EdgeId   id()      const                         { return _i->first; }
        /// The vertex IDs of the edge endpoints.
        Edge     edge()    const                         { return _i->second.edge; }
        /// The value stored on the edge.
        Value&   value()   const requires (HasEdgeValue()) { return _i->second.data; }
        /// Returns `true` if the edge begins and ends at the same vertex.
        bool     is_loop() const { return _i->second.edge.v0 == _i->second.edge.v1;}
        /// The ID of the source vertex.
        VertexId source_id()  const { return _i->second.edge.v0; }
        /// The ID of the target vertex.
        VertexId target_id()  const { return _i->second.edge.v1; }
        /**
         * @brief Returns the ID of the vertex at the given end of the edge.
         * 
         * The `Outgoing` end is the source vertex, the `Incoming` end is the target.
         */
        VertexId vertex(EdgeDir dir) const { 
            return dir == EdgeDir::Outgoing ? source_id() : target_id();
        }
        
        Value& operator*()  const requires (HasEdgeValue()) { return  value(); }
        Value* operator->() const requires (HasEdgeValue()) { return &value(); }
        operator Value&()   const requires (HasEdgeValue()) { return  value(); }
        
        /// @brief Implicitly converts to the edge ID.
        operator EdgeId() const { return id(); }
        
        bool operator==(const EdgeRef& other) const { return _i == other._i; }
        
    };
    
    template <Constness Const>
    struct VertexRef {
    private:
        constexpr static bool IsConst() { return Const == Constness::Const; }
        using Graph = std::conditional_t<IsConst(), const Digraph,   Digraph>;
        using Iter  = std::conditional_t<
            IsConst(),
            typename Verts::const_iterator,
            typename Verts::iterator
        >;
        using Value = std::conditional_t<IsConst(), const V, V>;
        
        friend Graph;
        friend VertexIterator<Const>;
        friend VertexIterator<~Const>;
        template <
            typename VertKey,
            typename VertVal,
            typename EdgeKey,
            typename EdgeVal,
            template <class...> class M>
        friend struct DigraphMap;
        
        Iter _i;
        
        VertexRef(Iter i): _i(i) {}
        
        Iter inner_iterator() const { return _i; }
        VertexNode& node() const { return _i->second; }
        
    public:
        
        /// An ID for this vertex which is unique to the graph container it belongs to.
        VertexId id()    const                           { return _i->first; }
        /// The value stored in this vertex.
        Value&   value() const requires (HasVertexValue()) { return _i->second.data; }
        /// The number of edges incident to this vertex in the given direction.
        size_t   degree(EdgeDir dir) const {
            return _i->second.edges[(int) dir].size;
        }
        /// The total number of edges incident to this vertex.
        size_t degree() const {
            return degree(EdgeDir::Incoming) + degree(EdgeDir::Outgoing);
        }
        
        Value& operator*()  const requires (HasVertexValue()) { return  value(); }
        Value* operator->() const requires (HasVertexValue()) { return &value(); }
        operator Value&()   const requires (HasVertexValue()) { return  value(); }
        
        /// Implicit conversion to the vertex ID.
        operator VertexId() const { return id(); }
        
        bool operator==(const VertexRef& other) const { return _i   == other._i; }
        bool operator==(VertexId other_id)      const { return id() == other_id; }
    };
    
    // iterator types
    
    /**
     * @brief An iterator over all the edges in a graph.
     * 
     * @tparam Const 
     */
    template <Constness Const>
    struct EdgeIterator {
    private:
        constexpr static bool IsConst() { return Const == Constness::Const; }
        using Graph = std::conditional_t<IsConst(), const Digraph, Digraph>;
        using Iter  = std::conditional_t<
            IsConst(),
            typename Edges::const_iterator,
            typename Edges::iterator
        >;
        using PointerProxy = detail::arrow_proxy<EdgeRef<Const>>;
        
        friend Graph;    
        template <
            typename VertKey,
            typename VertVal,
            typename EdgeKey,
            typename EdgeVal,
            template <class...> class M>
        friend struct DigraphMap;
        
        Graph* _graph;
        Iter   _i;
        
        EdgeIterator(Graph* graph, Iter i): _graph(graph), _i(i) {}
        
        /// The graph this iterator belongs to.
        Graph& graph()          const { return *_graph; }
        Iter   inner_iterator() const { return _i; }
        
    public:
    
        using value_type = EdgeRef<Const>;
        
        EdgeIterator(const EdgeIterator& other) = default;
        EdgeIterator& operator=(const EdgeIterator& other) = default;
        
        
        EdgeRef<Const> operator*() const { return _i; }
        PointerProxy  operator->() const { return PointerProxy { EdgeRef<Const>{ _i } }; }
        
        /// Retrieve a reference to the source vertex of the referenced edge.
        VertexRef<Const> source() const { return {_graph->_verts.find(_i->second.edge.v0)}; }
        /// Retrieve a reference to the target vertex of the referenced edge.
        VertexRef<Const> target() const { return {_graph->_verts.find(_i->second.edge.v1)}; }
        
        /// Advance the iterator to the next edge in the graph.
        EdgeIterator& operator++() {
            ++_i;
            return *this;
        }
        
        /// Decrement the iterator to the previous edge in the graph.
        EdgeIterator& operator--() {
            --_i;
            return *this;
        }
        
        template <Constness K>
        bool operator==(const EdgeIterator<K>& other) const {
            return _i == other._i;
        }
        
        template <Constness K>
        bool operator==(const IncidentEdgeIterator<K>& other) const {
            return _i == other._i;
        }
        
        operator EdgeId() const {
            return _i->first;
        }
        
    };
    

    /**
     * @brief An iterator over the edges incident to a vertex.
     * 
     * @tparam Const Whether the referenced edges are mutable or const.
     */
    template <Constness Const>
    struct IncidentEdgeIterator {
    private:
        constexpr static bool IsConst() { return Const == Constness::Const; }
        using Graph = std::conditional_t<IsConst(), const Digraph,   Digraph>;
        using Iter  = std::conditional_t<
            IsConst(),
            typename Edges::const_iterator,
            typename Edges::iterator
        >;
        using PointerProxy = detail::arrow_proxy<EdgeRef<Const>>;
        
        friend Graph;
        template <
            typename VertKey,
            typename VertVal,
            typename EdgeKey,
            typename EdgeVal,
            template <class...> class M>
        friend struct DigraphMap;
        
        Graph*  _graph;
        Iter    _i;
        EdgeDir _dir;
        
        IncidentEdgeIterator(Graph* graph, Iter i, EdgeDir dir):
            _graph(graph),
            _i(i),
            _dir(dir) {}
        
        /// The graph this iterator belongs to.
        Graph&  graph()          const { return *_graph; }
        Iter    inner_iterator() const { return _i; }
        
    public:
    
        using value_type = EdgeRef<Const>;
        
        IncidentEdgeIterator(const IncidentEdgeIterator& other) = default;
        IncidentEdgeIterator& operator=(const IncidentEdgeIterator& other) = default;
        
        /// Implicit conversion to the edge ID.
        operator IncidentEdgeIterator<Constness::Const>() const
            requires (Const == Constness::Mutable)
        {
            return IncidentEdgeIterator<Constness::Const>(_graph, _i, _dir);
        }
        
        EdgeRef<Const> operator*() const { return _i; }
        PointerProxy  operator->() const { return PointerProxy { EdgeRef<Const>{_i} }; }
        /// Whether this iterator traverses incoming or outgoing edges.
        EdgeDir traversal_direction()  const { return _dir; }
        
        /// Retrieve a reference to the source vertex of the referenced edge.
        VertexRef<Const> source() const { return {_graph->_verts.find(_i->second.edge.v0)}; }
        /// Retrieve a reference to the target vertex of the referenced edge.
        VertexRef<Const> target() const { return {_graph->_verts.find(_i->second.edge.v1)}; }
        /// Retrieve a reference to the vertex about which this iterator is traversing.
        VertexRef<Const> pivot()  const {
            EdgeId eid = _dir == EdgeDir::Outgoing ? _i->second.edge.v0 : _i->second.edge.v1;
            return { _graph->_verts.find(eid) };
        }
        /**
         * @brief Returns an iterator with the given traversal direction.
         * 
         * Changing the traversal direction causes the iterator to traverse the edges
         * incident to the opposite vertex, changing the pivot.
         */
        IncidentEdgeIterator with_traversal_direction(EdgeDir dir) const {
            return IncidentEdgeIterator(_graph, _i, dir);
        }
        
        /// Advances the iterator to the next edge in the current edge direction.  
        IncidentEdgeIterator& operator++() {
            EdgeId next_id = _i->second.next[(int) _dir];
            _i = (next_id != INVALID_EDGE)
                ? _graph->_edges.find(next_id)
                : _graph->_edges.end();
            return *this;
        }
        
        template <Constness K>
        bool operator==(const IncidentEdgeIterator<K>& other) const {
            return _i == other._i;
        }
        
        template <Constness K>
        bool operator==(const EdgeIterator<K>& other) const {
            return _i == other._i;
        }
        
        operator EdgeId() const {
            return _i->first;
        }
    };
    
    /**
     * @brief An iterator over the vertices in a graph.
     */
    template <Constness Const>
    struct VertexIterator {
    private:
        constexpr static bool IsConst() { return Const == Constness::Const; }
        using Graph = std::conditional_t<IsConst(), const Digraph,  Digraph>;
        using Iter  = std::conditional_t<
            IsConst(),
            typename Verts::const_iterator,
            typename Verts::iterator
        >;
        using PointerProxy = detail::arrow_proxy<VertexRef<Const>>;
    
        template <
            typename VertKey,
            typename VertVal,
            typename EdgeKey,
            typename EdgeVal,
            template <class...> class M>
        friend struct DigraphMap;
        
        friend Graph;
        
        Graph* _graph;
        Iter   _i;
        
        VertexIterator(Graph* graph, Iter i): _graph(graph), _i(i) {}
        
        /// The graph this iterator is iterating over.
        Graph& graph()          const { return *_graph; }
        Iter   inner_iterator() const { return _i; }
        
    public:
    
        using value_type = VertexRef<Const>;
        
        VertexIterator(const VertexIterator& other) = default;
        VertexIterator& operator=(const VertexIterator& other) = default;
    
        /// Implicit conversion to the a const iterator.
        operator VertexIterator<Constness::Const>() const 
            requires (Const == Constness::Mutable)
        {
            return VertexIterator<Constness::Const>(_graph, _i);
        }
        
        VertexRef<Const> operator*()  const { return _i; }
        PointerProxy     operator->() const { return PointerProxy { VertexRef<Const>{_i} }; }
        
        /// Advance this iterator.
        VertexIterator& operator++() {
            ++_i;
            return *this;
        }
        
        /// Decrement this iterator.
        VertexIterator& operator--() {
            --_i;
            return *this;
        }
        
        template <Constness K>
        bool operator==(const VertexIterator<K>& other) const {
            return _i == other._i;
        }
        
        operator VertexId() const {
            return _i->first;
        }
    };
    
    template <typename T>
    struct EdgePair {
        T incoming;
        T outgoing;
    };
    
    using vertex_iterator              = VertexIterator<Constness::Mutable>;
    using const_vertex_iterator        = VertexIterator<Constness::Const>;
    
    using edge_iterator                = EdgeIterator<Constness::Mutable>;
    using const_edge_iterator          = EdgeIterator<Constness::Const>;
    
    using incident_edge_iterator       = IncidentEdgeIterator<Constness::Mutable>;
    using const_incident_edge_iterator = IncidentEdgeIterator<Constness::Const>;
    
    using vertex_range                 = VertexRange<Constness::Mutable>;
    using const_vertex_range           = VertexRange<Constness::Const>;
    
    using edge_range                   = EdgeRange<Constness::Mutable>;
    using const_edge_range             = EdgeRange<Constness::Const>;
    
    using incident_edge_range          = IncidentEdgeRange<Constness::Mutable>;
    using const_incident_edge_range    = IncidentEdgeRange<Constness::Const>;
    
    using vertex_ref                   = VertexRef<Constness::Mutable>;
    using const_vertex_ref             = VertexRef<Constness::Const>;
    
    using edge_ref                     = EdgeRef<Constness::Mutable>;
    using const_edge_ref               = EdgeRef<Constness::Const>;
    
    
    /**
     * @brief A range capturing all the edges incident to a vertex in a given direction.
     * 
     * @tparam Const Whether the referenced edges are const or mutable.
     */
    template <Constness Const>
    struct IncidentEdgeRange {
    private:
        constexpr static bool IsConst() { return Const == Constness::Const; }
        using Graph = std::conditional_t<IsConst(), const Digraph, Digraph>;
        
        VertexId _vertex;
        Graph*   _graph;
        EdgeDir  _dir;
        
        friend Graph;
        
        IncidentEdgeRange(VertexId vertex, Graph* graph, EdgeDir dir):
            _vertex(vertex),
            _graph(graph),
            _dir(dir) {}
    
    public:
        
        /**
         * @brief An iterator pointing to the first edge in the set  of vertices incident
         * to the given vertex in the current direction.
         * 
         * If the vertex does not exist in the graph, this iterator will be equal to end().
         */
        IncidentEdgeIterator<Const> begin() const {
            auto i = _graph->_verts.find(_vertex);
            if (i == _graph->_verts.end()) {
                return end();
            } else {
                const EdgeList& edges = i->second.edges[(int) _dir];
                EdgeId head_id = edges.head;
                if (head_id == INVALID_EDGE) {
                    return end();
                } else {
                    return IncidentEdgeIterator<Const>(
                        _graph,
                        _graph->_edges.find(head_id),
                        _dir
                    );
                }
            }
        }
        
        /**
         * @brief An iterator pointing to the end of the range.
         * 
         * This iterator can never be reached by incrementing a valid iterator.
         */
        IncidentEdgeIterator<Const> end() const {
            return _graph->end_incident_edges();
        }
        
        /**
         * @brief A const iterator pointing to the first edge in the set  of vertices incident
         * to the given vertex in the current direction.
         * 
         * If the vertex does not exist in the graph, this iterator will be equal to end().
         */
        IncidentEdgeIterator<Constness::Const> cbegin() const {
            const Graph* g = _graph;
            return g->edges(_vertex, _dir).begin();
        }
        
        /**
         * @brief A const iterator pointing to the end of the range.
         * 
         * This iterator can never be reached by incrementing a valid iterator.
         */
        IncidentEdgeIterator<Constness::Const> cend() const {
            const Graph* g = _graph;
            return g->edges(_vertex, _dir).end();
        }
        
    };
    
    /**
     * @brief A range capturing all the edges in a graph.
     * 
     * @tparam Const Whether the referenced edges are const or mutable.
     */
    template <Constness Const>
    struct VertexRange {
    private:
        constexpr static bool IsConst() { return Const == Constness::Const; }
        using Graph = std::conditional_t<IsConst(), const Digraph, Digraph>;
        
        Graph* _graph;
        
        friend Graph;
        
        VertexRange(Graph* graph): _graph(graph) {}
    
    public:
        
        /**
         * @brief An iterator pointing to the first vertex in the graph.
         * 
         * If the graph is empty, this iterator will be equal to end_vertices().
         */
        VertexIterator<Const> begin() const {
            return VertexIterator<Const>(_graph, _graph->_verts.begin());
        }
        
        /**
         * @brief A sentinel iterator pointing beyond the end of the range of vertices.
         */
        VertexIterator<Const> end() const {
            return VertexIterator<Const>(_graph, _graph->_verts.end());
        }
        
        /**
         * @brief A const iterator pointing to the first vertex in the graph.
         */
        VertexIterator<Constness::Const> cbegin() const {
            return VertexIterator<Constness::Const>(_graph, _graph->_verts.begin());
        }
        
        /**
         * @brief A const sentinel iterator pointing beyond the end of the range of vertices.
         */
        VertexIterator<Constness::Const> cend() const {
            return VertexIterator<Constness::Const>(_graph, _graph->_verts.end());
        }
        
        /**
         * @brief The number of vertices in the graph.
         */
        size_t size() const {
            return _graph->_verts.size();
        }
    };
    
    /**
     * @brief A range capturing all the edges in a graph.
     * 
     * @tparam Const Whether the referenced edges are const or mutable.
     */
    template <Constness Const>
    struct EdgeRange {
    private:
        constexpr static bool IsConst() { return Const == Constness::Const; }
        using Graph = std::conditional_t<IsConst(), const Digraph, Digraph>;
        
        Graph* _graph;
        
        friend Graph;
        
        EdgeRange(Graph* graph): _graph(graph) {}
        
    public:
        
        /**
         * @brief An iterator pointing to the first edge in the graph.
         */
        EdgeIterator<Const> begin() const {
            return EdgeIterator<Const>(_graph, _graph->_edges.begin());
        }
        
        /**
         * @brief A sentinel iterator pointing beyond the end of the range of edges.
         */
        EdgeIterator<Const> end() const {
            return EdgeIterator<Const>(_graph, _graph->_edges.end());
        }
        
        /**
         * @brief A const iterator pointing to the first edge in the graph.
         */
        EdgeIterator<Constness::Const> cbegin() const {
            return EdgeIterator<Constness::Const>(_graph, _graph->_edges.begin());
        }
        
        /**
         * @brief A const sentinel iterator pointing beyond the end of the range of edges.
         */
        EdgeIterator<Constness::Const> cend() const {
            return EdgeIterator<Constness::Const>(_graph, _graph->_edges.end());
        }
        
        /**
         * @brief The number of edges in the graph.
         */
        size_t size() const {
            return _graph->_edges.size();
        }
    };

private:
    
    // assumes the given edge is definitely in the ring.
    // returns the next edge ID in the ring.
    EdgeId _remove_from_list(
            vertex_iterator v,
            EdgeDir dir, 
            typename Edges::iterator edge)
    {
        EdgeList& list = v->node().edges[(int) dir];
        if (list.size == 1) {
            // removing the only edge
            list.head = INVALID_EDGE;
            list.tail = INVALID_EDGE;
            list.size = 0;
            return INVALID_EDGE;
        } else {
            EdgeList& list = v->node().edges[(int) dir];
            EdgeId edge_id = edge->first;
            EdgeId prev_id = INVALID_EDGE;
            EdgeId cur_id  = list.head;
            auto e_prev = _edges.end();
            for (auto e = _edges.find(list.head); cur_id != INVALID_EDGE; ) {
                e = _edges.find(cur_id);
                EdgeId next_id = e->second.next[(int) dir];
                if (cur_id == edge_id) {
                    // found the edge to remove; excise it from the list
                    if (prev_id == INVALID_EDGE) {
                        // removing the head
                        list.head = next_id;
                    } else {
                        // removing an interior or tail edge.
                        // make the previous edge skip the deleted one.
                        e_prev->second.next[(int) dir] = next_id;
                    }
                    if (list.tail == edge_id) {
                        // removed the tail
                        list.tail = prev_id;
                    }
                    list.size -= 1;
                    cur_id = next_id;
                    break;
                } else {
                    prev_id = cur_id;
                    cur_id  = next_id;
                    e_prev  = e;
                }
            }
            return cur_id;
        }
    }

public:
    
    /**
     * @brief Retrieve a const reference to the vertex with the given id.
     * 
     * If the vertex does not exist, `std::out_of_range` will be thrown.
     */
    const_vertex_ref operator[](VertexId v) const {
        auto i = _verts.find(v);
        if (i == _verts.end()) throw std::out_of_range("vertex not found");
        return const_vertex_ref {i};
    }
    
    /**
     * @brief Retrieve a reference to the vertex with the given id.
     * 
     * If the vertex does not exist, it will be created.
     */
    vertex_ref operator[](VertexId v) {
        auto i = _verts.find(v);
        if (i == _verts.end()) {
            std::tie(i, std::ignore) = _verts.insert({v, VertexNode{}});
            _free_vertex_id = std::max(_free_vertex_id, v + 1);
        }
        return vertex_ref {i};
    }
    
    /**
     * @brief Retrieve a const reference to the edge with the given id.
     * 
     * If the edge does not exist, `std::out_of_range` will be thrown.
     */
    const_edge_ref operator[](EdgeId e) const {
        auto i = _edges.find(e);
        if (i == _edges.end()) throw std::out_of_range("edge not found");
        return const_edge_ref {i};
    }
    
    /**
     * @brief Retrieve a reference to the edge with the given id.
     * 
     * If the edge does not exist, `std::out_of_range` will be thrown.
     */
    edge_ref operator[](EdgeId e) {
        auto i = _edges.find(e);
        if (i == _edges.end()) throw std::out_of_range("edge not found");
        return edge_ref {i};
    }
    
    /**
     * @brief Whether the graph contains a vertex with the given id.
     */
    bool contains(VertexId vid) const {
        return _verts.contains(vid);
    }
    
    /**
     * @brief Whether the graph contains an edge with the given id.
     */
    bool contains(EdgeId eid) const {
        return _edges.contains(eid);
    }
    
    /// @name Vertex modifiers
    /// @{
    
    /// Insert a new vertex into the graph, and return an iterator to it.
    vertex_iterator insert_vertex() {
        VertexId v = _free_vertex_id++;
        auto [out, created] = _verts.insert({v, VertexNode{}});
        return {this, out};
    }
    
    /// Insert a new vertex into the graph storing the given value, and return
    /// an iterator to it.
    template <Forwardable<V> T>
    vertex_iterator insert_vertex(T&& v) requires (HasVertexValue()) {
        VertexId vid = _free_vertex_id++;
        auto [out, created] = _verts.insert(
            {
                vid, 
                VertexNode {
                    std::forward<T>(v)
                }
            }
        );
        return {this, out};
    }
    
    /// Insert a new vertex into the graph and construct it in-place with the given
    /// arguments, and return an iterator to it.
    template <typename... Args>
    vertex_iterator emplace_vertex(Args&&... args) requires (HasVertexValue()) {
        VertexId vid = _free_vertex_id++;
        auto [out, created] = _verts.insert(
            {
                vid,
                VertexNode {
                    std::forward<Args>(args)...
                }
            }
        );
        return {this, out};
    }
    
    /**
     * @brief Remove the given vertex, and return an iterator to the following vertex.
     * 
     * All edges incident to the vertex will also be removed.
     */
    vertex_iterator erase(vertex_iterator v) {
        if (v == end_vertices()) {
            return end_vertices();
        }
        for (EdgeDir dir : {EdgeDir::Incoming, EdgeDir::Outgoing}) {
            for (auto e = begin_incident_edges(v, dir); e != end_incident_edges(); ) {
                // remove the edge from the other vertex's ring
                const Edge& this_edge = e->edge();
                // if we're iterating over this vertex's incoming edges, then this vertex is
                // the destination, and the other vertex is the edge's source. Vice versa for
                // the other direction.
                VertexId other_id = (dir == EdgeDir::Incoming) ? this_edge.v0 : this_edge.v1;
                vertex_iterator other = find_vertex(other_id);
                // remove the edge from the other vertex's (opposite-direction) ring
                _remove_from_list(other, ~dir, e.inner_iterator());
                
                // don't bother removing the edge from the current ring, because we're
                // about to erase it anyway. increment the iterator by following its link
                // to the next edge now; after we erase the edge, the iterator will
                // be invalid.
                auto e_iter = e.inner_iterator();
                EdgeId next_id = e_iter->second.next[(int) dir];
                // erase the edge
                _edges.erase(e_iter);
                if (next_id == INVALID_EDGE) break;
                e = incident_edge_iterator {this, _edges.find(next_id), dir};
            }
        }
        return {this, _verts.erase(v.inner_iterator())};
    }
    
    /**
     * @brief Remove the vertex with the given ID, and return an iterator to the following
     * vertex.
     * 
     * Alias for `erase(find_vertex(vid))`.
     */
    vertex_iterator erase(VertexId vid) {
        return erase(find_vertex(vid));
    }
    
    /// @}

private:
    
    template <typename... Args>
    incident_edge_iterator _emplace_directed_edge(
            vertex_iterator src,
            vertex_iterator dst,
            Args&&... args)
    {
        if (src == end_vertices() or dst == end_vertices()) {
            return end_incident_edges();
        }
        EdgeId eid = _free_edge_id++;
        Edge edge {src->id(), dst->id()};
        VertexNode& v0 = src->node();
        VertexNode& v1 = dst->node();
        
        auto [new_edge, _] = _edges.insert({
            eid,
            EdgeNode {
                edge,
                // point to the head of the incoming and outgoing edge rings.
                .next_incoming = v1.incoming_edges.head,
                .next_outgoing = v0.outgoing_edges.head,
                .data { std::forward<Args>(args)... }
            }
        });
    
        auto _prepend_head = [](VertexNode& v, EdgeDir dir, EdgeId eid) {
            EdgeList& edge_list = v.edges[(int) dir];
            edge_list.head = eid;
            if (edge_list.tail == INVALID_EDGE) edge_list.tail = eid;
            edge_list.size += 1;
        };
        
        _prepend_head(v0, EdgeDir::Outgoing, eid);
        _prepend_head(v1, EdgeDir::Incoming, eid);
        
        return incident_edge_iterator{this, new_edge, EdgeDir::Outgoing};
    }

public:

    /// @name Edge modifiers
    /// @{
    
    /**
     * @brief Insert a new edge into the graph connecting the two given vertices,
     * and return an iterator to it.
     * 
     * Edges may be duplicated; a new edge will be created whether or not one between
     * the two vertices already exists.
     */
    incident_edge_iterator insert_directed_edge(
            vertex_iterator src,
            vertex_iterator dst)
    {
        return _emplace_directed_edge(src, dst);
    }
    
    /**
     * @brief Insert a new edge into the graph connecting the two given vertices,
     * storing the given value, and return an iterator to it.
     * 
     * Edges may be duplicated; a new edge will be created whether or not one between
     * the two vertices already exists.
     */
    template <Forwardable<E> T>
    incident_edge_iterator insert_directed_edge(
            vertex_iterator src,
            vertex_iterator dst,
            T&& e)
        requires (HasEdgeValue())
    {
        return _emplace_directed_edge(src, dst, std::forward<T>(e));
    }
    
    /// Alias for `insert_directed_edge(find_vertex(src), find_vertex(dst))`.
    incident_edge_iterator insert_directed_edge(VertexId src, VertexId dst) {
        return insert_directed_edge(find_vertex(src), find_vertex(dst));
    }
    
    /// Alias for `insert_directed_edge(find_vertex(src), find_vertex(dst), e)`.
    template <Forwardable<E> T>
    incident_edge_iterator insert_directed_edge(VertexId src, VertexId dst, T&& e)
        requires (HasEdgeValue())
    {
        return insert_directed_edge(find_vertex(src), find_vertex(dst), std::forward<T>(e));
    }
    
    /**
     * @brief Insert a new edge into the graph connecting the two given vertices,
     * constructing a new edge value in-place with the given arguments, and return
     * an iterator to it.
     * 
     * Edges may be duplicated; a new edge will be created whether or not one between
     * the two vertices already exists.
     */
    template <typename... Args>
    incident_edge_iterator emplace_directed_edge(
            vertex_iterator src,
            vertex_iterator dst,
            Args&&... args)
    {
        return _emplace_directed_edge(src, dst, std::forward<Args>(args)...);
    }
    
    /**
     * @brief Insert an undirected edge between the two given vertices by
     * inserting two directed edges in opposing directions.
     * 
     * Return a pair of iterators to the two edges.
     */
    std::pair<incident_edge_iterator, incident_edge_iterator> insert_undirected_edge(
            VertexId a,
            VertexId b)
        requires (not HasEdgeValue())
    {
        incident_edge_iterator e0 = insert_directed_edge(a, b);
        incident_edge_iterator e1 = insert_directed_edge(b, a);
        return {e0, e1};
    }

private:

    std::pair<EdgePair<EdgeId>, typename Edges::iterator>
    _impl_erase_edge(typename Edges::iterator edge) {
        if (edge == _edges.end()) {
            return {{INVALID_EDGE, INVALID_EDGE}, _edges.end()};
        }
        EdgeNode& edge_node = edge->second;
        Edge              e = edge_node.edge;
        vertex_iterator  v0 = find_vertex(e.v0);
        vertex_iterator  v1 = find_vertex(e.v1);
        
        EdgeId out_id = _remove_from_list(v0, EdgeDir::Outgoing, edge);
        EdgeId  in_id = _remove_from_list(v1, EdgeDir::Incoming, edge);
        
        auto next_edge = _edges.erase(edge);
        return {{in_id, out_id}, next_edge};
    }

public:

    
    /**
     * @brief Remove the given edge, and return the IDs of edges that followed it in the
     * incoming and outgoing edge rings.
     * 
     * This is slightly more efficient than the overload of erase() which returns an
     * incident_edge_iterator.
     */
    EdgePair<EdgeId> erase(incident_edge_iterator edge) {
        return _impl_erase_edge(edge.inner_iterator()).first;
    }
    
    /**
     * @brief Remove the given edge, and return an iterator to the following edge in the
     * given direction.
     */
    incident_edge_iterator erase(incident_edge_iterator edge, EdgeDir dir) {
        auto [in_id, out_id] = erase(edge);
        auto next_id = (dir == EdgeDir::Incoming) ? in_id : out_id;
        if (next_id != INVALID_EDGE) {
            return find_edge(next_id, dir);
        } else {
            return end_incident_edges();
        }
    }
    
    /// @brief Remove the edge with the given id, returning an iterator to the following edge
    /// in the graph.
    edge_iterator erase(EdgeId eid) {
        return {this, _impl_erase_edge(_edges.find(eid)).second};
    }
    
    /// @brief Remove the given edge, returning an iterator to the following edge
    /// in the graph.
    edge_iterator erase(edge_iterator edge) {
        return {this, _impl_erase_edge(edge.inner_iterator()).second};
    }
    
    /// @}
    /// @name Vertex iteration
    /// @{
    
    /**
     * @brief A range of all the vertices in the graph.
     */
    VertexRange<Constness::Mutable> vertices() {
        return VertexRange<Constness::Mutable>(this);
    }
    
    /**
     * @brief An immutable range of all the vertices in the graph.
     */
    VertexRange<Constness::Const> vertices() const {
        return VertexRange<Constness::Const>(this);
    }
    
    /**
     * @brief An iterator pointing to the first vertex in the graph.
     * 
     * If the graph is empty, this iterator will be equal to end_vertices().
     */
    vertex_iterator begin_vertices() {
        return vertex_iterator(this, _verts.begin());
    }
    
    /**
     * @brief A sentinel iterator pointing beyond the end of the range of vertices.
     */
    vertex_iterator end_vertices() {
        return vertex_iterator(this, _verts.end());
    }
    
    /**
     * @brief A const iterator pointing to the first vertex in the graph.
     * 
     * If the graph is empty, this iterator will be equal to end_vertices().
     */
    const_vertex_iterator begin_vertices() const {
        return const_vertex_iterator(this, _verts.begin());
    }
    
    /// A const sentinel iterator pointing beyond the end of the range of vertices.
    const_vertex_iterator end_vertices() const {
        return const_vertex_iterator(this, _verts.end());
    }
    
    const_vertex_iterator cbegin_vertices() const {
        return const_vertex_iterator(this, _verts.begin());
    }
    
    const_vertex_iterator cend_vertices() const {
        return const_vertex_iterator(this, _verts.end());
    }
    
    /// Return the number of vertices in the graph.
    size_t vertices_size() const {
        return _verts.size();
    }
    
    /// @}
    /// @name Edge iteration
    /// @{
    
    /**
     * @brief A range of all the edges in the graph.
     */
    EdgeRange<Constness::Mutable> edges() {
        return EdgeRange<Constness::Mutable>(this);
    }
    
    /**
     * @brief An immutable range of all the edges in the graph.
     */
    EdgeRange<Constness::Const> edges() const {
        return EdgeRange<Constness::Const>(this);
    }
    
    /**
     * @brief An iterator pointing to the first edge in the graph.
     * 
     * If the graph has no edges, this iterator will be equal to end_edges().
     */
    edge_iterator begin_edges() {
        return edge_iterator(this, _edges.begin());
    }
    
    /// A sentinel iterator pointing beyond the end of the range of edges.
    edge_iterator end_edges() {
        return edge_iterator(this, _edges.end());
    }
    
    /**
     * @brief A const iterator pointing to the first edge in the graph.
     * 
     * If the graph has no edges, this iterator will be equal to end_edges().
     */
    const_edge_iterator begin_edges() const {
        return const_edge_iterator(this, _edges.begin());
    }
    
    /// A const sentinel iterator pointing beyond the end of the range of edges.
    const_edge_iterator end_edges() const {
        return const_edge_iterator(this, _edges.end());
    }
    
    /// Return the number of edges in the graph.
    size_t edges_size() const {
        return _edges.size();
    }
    
    /// @}
    /// @name Vertex access
    /// @{
    
    /**
     * @brief Find the vertex with the given id and return a mutable iterator to it.
     * 
     * If the vertex does not exist, end_vertices() will be returned.
     */
    vertex_iterator find_vertex(VertexId v) {
        return vertex_iterator(this, _verts.find(v));
    }
    
    /**
     * @brief Find the vertex with the given id and return a const iterator to it.
     * 
     * If the vertex does not exist, end_vertices() will be returned.
     */
    const_vertex_iterator find_vertex(VertexId v) const {
        return const_vertex_iterator(this, _verts.find(v));
    }
    
    /**
     * @brief Return a reference to the vertex with the given id, if it exists.
     * 
     * If the vertex does not exist, std::nullopt will be returned.
     */
    std::optional<vertex_ref> vertex(VertexId v) {
        auto it = _verts.find(v);
        if (it == _verts.end()) return std::nullopt;
        return vertex_ref{it};
    }
    
    /**
     * @brief Return a const reference to the vertex with the given id, if it exists.
     * 
     * If the vertex does not exist, std::nullopt will be returned.
     */
    std::optional<const_vertex_ref> vertex(VertexId v) const {
        auto it = _verts.find(v);
        if (it == _verts.end()) return std::nullopt;
        return const_vertex_ref{it};
    }
    
    /// @}
    
    /// @name Incident edge iteration
    /// @{
    
    /**
     * @brief Return a range of edges which traverses all the edges incident to `v` in the
     * given direction.
     * 
     * This range does not traverse any edges in the opposite incidence direction (excepting
     * self-loop edges).
     * 
     * If the vertex has no edges in the given direction, an empty range will be returned.
     */
    IncidentEdgeRange<Constness::Mutable> incident_edges(
            VertexId v,
            EdgeDir dir=EdgeDir::Outgoing)
    {
        return IncidentEdgeRange<Constness::Mutable>(v, this, dir);
    }
    
    /**
     * @brief Return a range of immuatble edges which traverses all the edges incident to `v`
     * in the given direction.
     * 
     * This range does not traverse any edges in the opposite incidence direction (excepting
     * self-loop edges).
     * 
     * If the vertex has no edges in the given direction, an empty range will be returned.
     */
    IncidentEdgeRange<Constness::Const> incident_edges(
            VertexId v,
            EdgeDir dir=EdgeDir::Outgoing) const
    {
        return IncidentEdgeRange<Constness::Const>(v, this, dir);
    }
    
    /**
     * @brief Return an iterator pointing to the first edge incident to `v` in the given
     * direction.
     * 
     * This iterator will not traverse any edges in the opposite incidence direction
     * (except for self-loop edges).
     * 
     * If the vertex has no edges in the given direction, end_incident_edges() will be
     * returned.
     */
    incident_edge_iterator begin_incident_edges(vertex_iterator v, EdgeDir dir) {
        if (v == end_vertices()) {
            return end_incident_edges();
        }
        const VertexNode& v_node = v._i->second;
        EdgeId first_edge_id = v_node.edges[(int) dir].head;
        return {
            this,
            first_edge_id != INVALID_EDGE ? _edges.find(first_edge_id) : _edges.end(),
            dir
        };
    }
    
    /**
     * @brief Return a const iterator pointing to the first edge incident to `v` in the given
     * direction.
     * 
     * This iterator will not traverse any edges in the opposite incidence direction
     * (except for self-loop edges).
     * 
     * If the vertex has no edges in the given direction, end_incident_edges() will be
     * returned.
     */
    const_incident_edge_iterator begin_incident_edges(vertex_iterator v, EdgeDir dir) const {
        if (v == end_vertices()) {
            return end_incident_edges();
        }
        const VertexNode& v_node = v._i->second;
        EdgeId first_edge_id = v_node.edges[(int) dir].head;
        return {
            this,
            first_edge_id != INVALID_EDGE ? _edges.find(first_edge_id) : _edges.end(),
            dir
        };
    }
    
    /// Alias for `begin_incident_edges(find_vertex(v), dir)`.
    incident_edge_iterator begin_incident_edges(VertexId v, EdgeDir dir) {
        return begin_incident_edges(find_vertex(v), dir);
    }
    
    /// Alias for `begin_incident_edges(find_vertex(v), dir)`.
    const_incident_edge_iterator begin_incident_edges(VertexId v, EdgeDir dir) const {
        return begin_incident_edges(find_vertex(v), dir);
    }
    
    /**
     * @brief A sentinel iterator indicating an empty incident edge range.
     * 
     * This iterator is unreachable by incrementing a valid iterator.
     */
    incident_edge_iterator end_incident_edges() {
        return {this, _edges.end(), EdgeDir::Outgoing};
    }
    
    const_incident_edge_iterator end_incident_edges() const {
        return {this, _edges.end(), EdgeDir::Outgoing};
    }
    
    const_incident_edge_iterator cbegin_incident_edges(VertexId v, EdgeDir dir) const {
        return begin_incident_edges(v, dir);
    }
    
    const_incident_edge_iterator cend_incident_edges() const {
        return end_incident_edges();
    }
    
    /// @}
    /// @name Edge access
    /// @{
    
    /**
     * @brief Return an iterator pointing to the vertex with the given id.
     * 
     * `dir` determines the direction of the iterator.
     */
    incident_edge_iterator find_edge(EdgeId e, EdgeDir dir=EdgeDir::Outgoing) {
        return incident_edge_iterator(this, _edges.find(e), EdgeDir::Outgoing);
    }
    
    /**
     * @brief Return a const iterator pointing to the vertex with the given id.
     * 
     * `dir` determines the direction of the iterator.
     */
    const_incident_edge_iterator find_edge(EdgeId e, EdgeDir dir=EdgeDir::Outgoing) const {
        return const_incident_edge_iterator(this, _edges.find(e), EdgeDir::Outgoing);
    }
    
    /**
     * @brief Find the edge connecting the two given vertices.
     * 
     * If no such edge exists, end_incident_edges() will be returned.
     * 
     * This operation is O(k), with k being the degree of the smaller vertex.
     * 
     * The direction of the iterator will be such that it is traversing the vertex
     * with smaller degree.
     */
    incident_edge_iterator find_edge(const_vertex_iterator v0, const_vertex_iterator v1) {
        if (v0 == end_vertices() or v1 == end_vertices()) {
            return end_incident_edges();
        }
        // search through the smaller vertex's edges
        EdgeDir dir = EdgeDir::Outgoing;
        if (v0->degree(EdgeDir::Outgoing) > v0->degree(EdgeDir::Incoming)) {
            dir = EdgeDir::Incoming;
            std::swap(v0, v1);
        }
        for (auto e = begin_incident_edges(v0, dir); e != end_incident_edges(); ++e) {
            // we're iterating around v0, we've found our edge when it connects to v1:
            if (e->vertex(~dir) == v1) {
                return e;
            }
        }
        return end_incident_edges();
    }
    
    inline incident_edge_iterator find_edge(vertex_iterator v0, vertex_iterator v1) {
        return find_edge(const_vertex_iterator(v0), const_vertex_iterator(v1));
    }
    
    /**
     * @brief Find the edge connecting the two given vertices.
     * 
     * If no such edge exists, end_incident_edges() will be returned.
     * 
     * This operation is O(k), with k being the degree of the smaller vertex.
     * 
     * The direction of the iterator will be such that it is traversing the
     * vertex with smaller degree.
     */
    const_incident_edge_iterator find_edge(
            const_vertex_iterator v0,
            const_vertex_iterator v1) const
    {
        if (v0 == end_vertices() or v1 == end_vertices()) {
            return end_incident_edges();
        }
        // search through the smaller vertex's edges
        EdgeDir dir = EdgeDir::Outgoing;
        if (v0->degree(EdgeDir::Outgoing) > v0->degree(EdgeDir::Incoming)) {
            dir = EdgeDir::Incoming;
            std::swap(v0, v1);
        }
        for (auto e = begin_incident_edges(v0, dir); e != end_incident_edges(); ++e) {
            // we're iterating around v0, we've found our edge when it connects to v1:
            if (e->vertex(~dir) == v1) {
                return e;
            }
        }
        return end_incident_edges();
    }
    
    /**
     * @brief Alias for `find_edge(find_vertex(v0), find_vertex(v1))`.
     */
    incident_edge_iterator find_edge(VertexId v0, VertexId v1) {
        return find_edge(find_vertex(v0), find_vertex(v1));
    }
    
    /**
     * @brief Alias for `find_edge(find_vertex(v0), find_vertex(v1))`.
     */
    const_incident_edge_iterator find_edge(VertexId v0, VertexId v1) const {
        return find_edge(find_vertex(v0), find_vertex(v1));
    }
    
    /// Alias for `find_edge(e.v0, e.v1)`.
    incident_edge_iterator find_edge(Edge e) {
        return find_edge(e.v0, e.v1);
    }
    
    /// Alias for `find_edge(e.v0, e.v1)`.
    const_incident_edge_iterator find_edge(Edge e) const {
        return find_edge(e.v0, e.v1);
    }
    
    /// Return a reference to the edge with the given id.
    std::optional<edge_ref> edge(EdgeId edge) {
        auto it = _edges.find(edge);
        if (it == _edges.end()) return std::nullopt;
        return edge_ref{it};
    }
    
    /// Return a const reference to the edge with the given id.
    std::optional<const_edge_ref> edge(EdgeId edge) const {
        auto it = _edges.find(edge);
        if (it == _edges.end()) return std::nullopt;
        return const_edge_ref{it};
    }
    
    /// @}
    
};

} // namespace graph
