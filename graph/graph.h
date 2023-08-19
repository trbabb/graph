#pragma once

#include <ranges>

#include <ankerl/unordered_dense.h>

namespace graph {
    
enum struct VertexId : size_t {};
enum struct EdgeId   : size_t {};

struct Edge {
    VertexId v0;
    VertexId v1;
    
    constexpr bool operator==(const Edge& other) const {
        return v0 == other.v0 and v1 == other.v1;
    }
};
    
enum struct EdgeDir {
    Incoming = 0,
    Outgoing = 1,
};

EdgeDir operator~(EdgeDir dir) {
    return dir == EdgeDir::Incoming ? EdgeDir::Outgoing : EdgeDir::Incoming;
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


} // namespace graph

namespace std {

template <>
struct hash<graph::Edge> {
    size_t operator()(const graph::Edge& e) const {
        return graph::hash_combine(e.v0, e.v1);
    }
};
    
} // namespace std


// todo: change to O(n) edge-search
// todo: wbn to be able to index edges/verts by a custom type.
//   this would mean maintaining a mapping of E -> Edge and V -> VertexId
//   > subclass and keep a mapping of K -> Id
// todo: iterators should return proxy objects
// todo: ranges over
//   - all edges
//   - all verts
//   - incoming edges
//   - outgoing edges
//   - incident edges

namespace graph {

template <typename V, typename E>
struct DirectedGraph {
protected:

    using EdgeData   = std::conditional_t<std::same_as<E, void>, int[0], E>;
    using VertexData = std::conditional_t<std::same_as<V, void>, int[0], V>;
    
    static constexpr bool HasVertexValue = not std::same_as<V, void>;
    static constexpr bool HasEdgeValue   = not std::same_as<E, void>;
    
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
            EdgeId next[2];
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
            std::optional<EdgeList> edges[2];
            struct {
                std::optional<EdgeList> incoming_edges;
                std::optional<EdgeList> outgoing_edges;
            };
        };
        VertexData data;
    };
    
    using Edges = ankerl::unordered_dense<EdgeId,   EdgeNode>;
    using Verts = ankerl::unordered_dense<VertexId, VertexNode>;
    
    Verts    _verts;
    Edges    _edges;
    VertexId _free_vertex_id = 0;
    EdgeId   _free_edge_id   = 0;
    
    enum struct Constness {
        Mutable,
        Const,
    };

public:

    template <Constness Const>
    struct EdgeIterator;

    template <Constness Const>
    struct EdgeRef {
    private:
        static constexpr bool IsConst = Const == Constness::Const;
        using Graph = std::conditional_t<IsConst, const DirectedGraph,   DirectedGraph>;
        using Iter  = std::conditional_t<IsConst, Edges::const_iterator, Edges::iterator>;
        using Value = std::conditional_t<IsConst, const E, E>;
        using Node  = std::conditional_t<IsConst, const EdgeNode, EdgeNode>;
        
        friend EdgeIterator<Const>;
        friend EdgeIterator<~Const>;
        friend Graph;
        
        Iter _i;
        
        Node& node() const requires () { return _i->second; }
        Iter  inner_iterator() const { return _i; }
        
    public:
        
        EdgeId   id()      const                         { return _i->first; }
        Edge     edge()    const                         { return _i->second.edge; }
        Value&   value()   const requires (HasEdgeValue) { return _i->second.data; }
        bool     is_loop() const { return _i->second.edge.v0 == _i->second.edge.v1;}
        VertexId source()  const { return _i->second.edge.v0; }
        VertexId target()  const { return _i->second.edge.v1; }
        
        Value& operator*()  const requires (HasEdgeValue) { return  value(); }
        Value* operator->() const requires (HasEdgeValue) { return &value(); }
        
        operator EdgeId() const { return id(); }
        
        bool operator==(const EdgeRef& other) const { return _i == other._i; }
        
    };
    
    template <Constness Const>
    struct VertexRef {
    private:
        static constexpr bool IsConst = Const == Constness::Const;
        using Graph = std::conditional_t<IsConst, const DirectedGraph,   DirectedGraph>;
        using Iter  = std::conditional_t<IsConst, Verts::const_iterator, Verts::iterator>;
        using Value = std::conditional_t<IsConst, const V, V>;
        
        friend Graph;
        friend VertexIterator<Const>;
        friend VertexIterator<~Const>;
        
        Iter _i;
        
        Iter iner_iterator() const { return _i; }
        
    public:
        
        VertexId id()    const                           { return _i->first; }
        Value&   value() const requires (HasVertexValue) { return _i->second.data; }
        size_t   degree(EdgeDir dir) const {
            if (auto edge_list = _i->second.edges[(int) dir]) {
                return edge_list->size;
            } else {
                return 0;
            }
        }
        size_t degree() const {
            return degree(EdgeDir::Incoming) + degree(EdgeDir::Outgoing);
        }
        
        Value& operator*()  const requires (HasVertexValue) { return  value(); }
        Value* operator->() const requires (HasVertexValue) { return &value(); }
        
        operator VertexId() const { return id(); }
        
        bool operator==(const VertexRef& other) const { return _i   == other._i; }
        bool operator==(VertexId id)            const { return id() == id; }
    };

    template <Constness Const>
    struct EdgeIterator {
    private:
        static constexpr bool IsConst = Const == Constness::Const;
        using Graph = std::conditional_t<IsConst, const DirectedGraph,   DirectedGraph>;
        using Iter  = std::conditional_t<IsConst, Edges::const_iterator, Edges::iterator>;
        
        friend Graph;
        
        Graph*  _graph;
        Iter    _i;
        EdgeDir _dir;
        
        EdgeIterator(Graph* graph, Iter i, EdgeDir dir): _graph(graph), _i(i), _dir(dir) {}
        
    public:
        
        using Ref = std::conditional_t<Const == Constness::Mutable, E&, const E&>;
        
        operator EdgeIterator<Constness::Const>() const requires (Const == Constness::Mutable) {
            return EdgeIterator<Constness::Const>(_graph, _i, _dir);
        }
        
        EdgeRef operator*()  const { return _i; }
        EdgeRef operator->() const { return _i; }
        EdgeDir direction()  const { return _dir; }
        Graph&  graph()      const { return *_graph; }
        
        EdgeIterator opposite_vertex_edges() const {
            return EdgeIterator(_graph, _i, ~_dir);
        }
        
        
        EdgeIterator& operator++() {
            EdgeId next_id = _i->second.next[_dir];
            _i = _graph->_edges.find(next_id);
            return *this;
        }
        
        template <Constness Const>
        bool operator==(const EdgeIterator<Const>& other) const {
            return _i == other._i;
        }
    };
    
    template <Constness Const>
    struct VertexIterator {
    private:
        using Graph = std::conditional_t<Const == Constness::Mutable, DirectedGraph,  const DirectedGraph>;
        using Iter  = std::conditional_t<Const == Constness::Mutable, Verts::iterator, Verts::const_iterator>;
        
        friend Graph;
        
        Graph* _graph;
        Iter   _i;
        
        VertexIterator(Graph* graph, Iter i): _graph(graph), _i(i) {}
        
    public:
        
        using Ref = std::conditional_t<Const == Constness::Mutable, V&, const V&>;
        
        VertexRef operator*()  const { return _i; }
        VertexRef operator->() const { return _i; }
        Graph& graph() const { return *_graph; }
        
        VertexIterator& operator++() {
            ++_i;
            return *this;
        }
        
        VertexIterator& operator--() {
            --_i;
            return *this;
        }
        
        template <Constness Const>
        bool operator==(const VertexIterator<Const>& other) const {
            return _i == other._i;
        }
    };
    
    using edge_iterator         = EdgeIterator<Constness::Mutable>;
    using const_edge_iterator   = EdgeIterator<Constness::Const>;
    
    using vertex_iterator       = VertexIterator<Constness::Mutable>;
    using const_vertex_iterator = VertexIterator<Constness::Const>;
    
    using edge_range            = EdgeRange<Constness::Mutable>;
    using const_edge_range      = EdgeRange<Constness::Const>;
    
    using vertex_range          = VertexRange<Constness::Mutable>;
    using const_vertex_range    = VertexRange<Constness::Const>;
    
    using vertex_ref            = VertexRef<Constness::Mutable>;
    using const_vertex_ref      = VertexRef<Constness::Const>;
    
    using edge_ref              = EdgeRef<Constness::Mutable>;
    using const_edge_ref        = EdgeRef<Constness::Const>;
    
    
    template <Constness Const>
    struct EdgeRange {
        using Graph = std::conditional_t<Const == Constness::Mutable, DirectedGraph,  const DirectedGraph>;
        
        VertexId _vertex;
        Graph*   _graph;
        EdgeDir  _dir;
        
        EdgeRange(VertexId vertex, Graph* graph, EdgeDir dir):
            _vertex(vertex),
            _graph(graph),
            _dir(dir) {}
        
        EdgeIterator<Const> begin() const {
            auto i = _graph->_verts.find(_vertex);
            if (i == _graph->_verts.end()) {
                return end();
            } else {
                const std::optional<EdgeList>& edges = i->second.edges[(int) _dir];
                if (not edges) return end();
                return EdgeIterator<Const>(
                    _graph,
                    _graph->_edges.find(edges->head),
                    _dir
                );
            }
        }
        
        EdgeIterator<Const> end() const {
            return EdgeIterator<Const>(_graph, _graph->_edges.end(), EdgeDir::Outgoing);
        }
        
    };
    
    template <Constness Const>
    struct VertexRange {
        using Graph = std::conditional_t<Const == Constness::Mutable, DirectedGraph,  const DirectedGraph>;
        
        Graph* _graph;
        
        VertexRange(Graph* graph): _graph(graph) {}
        
        VertexIterator<Const> begin() const {
            return VertexIterator<Const>(_graph, _graph->_verts.begin());
        }
        
        VertexIterator<Const> end() const {
            return VertexIterator<Const>(_graph, _graph->_verts.end());
        }
    };

private:
    
    // assumes the given edge is definitely in the ring
    void _remove_from_ring(
            vertex_iterator v,
            EdgeDir dir, 
            const_edge_iterator edge)
    {
        std::optional<EdgeList>& ring = v->node().edges[(int) dir];
        if (ring->size == 1) {
            // removing the only edge
            ring = std::nullopt;
        } else {
            EdgeId prev_id = ring->tail;
            EdgeId next_id = edge->node().next[(int) dir];
            auto prev = _edges.find(prev_id);
            size_t cur_i = 0;
            do {
                EdgeId cur_id = prev->second.next[(int) dir];
                if (cur_id == edge->id()) {
                    prev->second.next[(int) dir] = next_id;
                    break;
                } else {
                    prev_id = cur_id;
                    prev = _edges.find(cur_id);
                    cur_i += 1;
                }
            } while (prev_id != ring->tail);
            
            if (ring->head == edge->id()) ring->head = next_id; // removed the head
            if (ring->tail == edge->id()) ring->tail = prev_id; // removed the tail
            
            ring->size -= 1;
        }
    }

public:
    
    const V& operator[](VertexId v) const {
        return _verts[v].data;
    }
    
    V& operator[](VertexId v) {
        return _verts[v].data;
    }
    
    const E& operator[](EdgeId e) const {
        return _edges[e].data;
    }
    
    E& operator[](EdgeId e) {
        return _edges[e].data;
    }
    
    VertexRange<Constness::Mutable> vertices() {
        return VertexRange<Constness::Mutable>(this);
    }
    
    VertexRange<Constness::Const> vertices() const {
        return VertexRange<Constness::Const>(this);
    }
    
    vertex_iterator begin_vertices() {
        return vertex_iterator(this, _verts.begin());
    }
    
    vertex_iterator end_vertices() {
        return vertex_iterator(this, _verts.end());
    }
    
    const_vertex_iterator begin_vertices() const {
        return const_vertex_iterator(this, _verts.begin());
    }
    
    const_vertex_iterator end_vertices() const {
        return const_vertex_iterator(this, _verts.end());
    }
    
    const_vertex_iterator cbegin_vertices() const {
        return const_vertex_iterator(this, _verts.begin());
    }
    
    const_vertex_iterator cend_vertices() const {
        return const_vertex_iterator(this, _verts.end());
    }
    
    vertex_iterator find_vertex(VertexId v) {
        return vertex_iterator(this, _verts.find(v));
    }
    
    const_vertex_iterator find_vertex(VertexId v) const {
        return const_vertex_iterator(this, _verts.find(v));
    }
    
    std::optional<vertex_ref> vertex(VertexId v) {
        auto it = _verts.find(v);
        if (it == _verts.end()) return std::nullopt;
        return vertex_ref{it};
    }
    
    std::optional<const_vertex_ref> vertex(VertexId v) const {
        auto it = _verts.find(v);
        if (it == _verts.end()) return std::nullopt;
        return const_vertex_ref{it};
    }
    
    size_t vertices_size() const {
        return _verts.size();
    }
    
    size_t edges_size() const {
        return _edges.size();
    }
        
    /**
     * @brief Return an infinite range of edges cycling about the vertex `v`.
     */
    EdgeRange<Constness::Mutable> edges(VertexId v, EdgeDir dir) {
        return EdgeRange<Constness::Mutable>(v, this, dir);
    }
    
    /**
     * @brief Return an infinite range of const edges cycling about the vertex `v`.
     * 
     * @param v 
     * @param dir 
     * @return EdgeRange<Constness::Const> 
     */
    EdgeRange<Constness::Const> edges(VertexId v, EdgeDir dir) const {
        return EdgeRange<Constness::Const>(v, this, dir);
    }
    
    edge_iterator begin_edges(vertex_iterator v, EdgeDir dir) {
        if (v == end_vertices()) {
            return end_edges();
        }
        VertexNode& v_node = v->second;
        if (not v_node.edges[dir]) {
            return end_edges();
        }
        return EdgeIterator<Constness::Mutable> {
            this,
            _edges.find(v_node.outgoing_edges->head),
            dir
        };
    }
    
    const_edge_iterator begin_edges(vertex_iterator v, EdgeDir dir) const {
        if (v == end_vertices()) {
            return end_edges();
        }
        VertexNode& v_node = v->second;
        if (not v_node.edges[dir]) {
            return end_edges();
        }
        return EdgeIterator<Constness::Const> {
            this,
            _edges.find(v_node.outgoing_edges->head),
            dir
        };
    }
    
    edge_iterator begin_edges(VertexId v, EdgeDir dir) {
        return begin_edges(find_vertex(v), dir);
    }
    
    const_edge_iterator begin_edges(VertexId v, EdgeDir dir) const {
        return begin_edges(find_vertex(v), dir);
    }
    
    edge_iterator end_edges() {
        return {this, _edges.end(), EdgeDir::Outgoing};
    }
    
    const_edge_iterator end_edges() const {
        return {this, _edges.end(), EdgeDir::Outgoing};
    }
    
    const_edge_iterator cbegin_edges(VertexId v, EdgeDir dir) const {
        return begin_edges(v, dir);
    }
    
    const_edge_iterator cend_edges() const {
        return end_edges();
    }
    
    edge_iterator find_edge(EdgeId e, EdgeDir dir=EdgeDir::Outgoing) {
        return edge_iterator(this, _edges.find(e), EdgeDir::Outgoing);
    }
    
    const_edge_iterator find_edge(EdgeId e, EdgeDir dir=EdgeDir::Outgoing) const {
        return const_edge_iterator(this, _edges.find(e), EdgeDir::Outgoing);
    }
    
    edge_iterator find_edge(const_vertex_iterator v0, const_vertex_iterator v1) {
        if (v1 == end_vertices()) {
            return end_edges();
        }
        auto first_edge_it = begin_edges(v0, EdgeDir::Outgoing);
        if (first_edge_it == end_edges()) {
            return end_edges();
        }
        auto edge_it = first_edge_it;
        do {
            if (edge_it->target() == v1) {
                return edge_it;
            }
            ++edge_it;
        } while (edge_it != first_edge_it);
        return end_edges();
    }
    
    const_edge_iterator find_edge(const_vertex_iterator v0, const_vertex_iterator v1) const {
        if (v1 == end_vertices()) {
            return end_edges();
        }
        auto first_edge_it = begin_edges(v0, EdgeDir::Outgoing);
        if (first_edge_it == end_edges()) {
            return end_edges();
        }
        auto edge_it = first_edge_it;
        do {
            if (edge_it->target() == v1) {
                return edge_it;
            }
            ++edge_it;
        } while (edge_it != first_edge_it);
        return end_edges();
    }
    
    edge_iterator find_edge(VertexId v0, VertexId v1) {
        return find_edge(find_vertex(v0), find_vertex(v1));
    }
    
    const_edge_iterator find_edge(VertexId v0, VertexId v1) const {
        return find_edge(find_vertex(v0), find_vertex(v1));
    }
    
    std::optional<edge_ref> edge(EdgeId edge) {
        auto it = _edges.find(e);
        if (it == _edges.end()) return std::nullopt;
        return edge_ref{it};
    }
    
    std::optional<const_edge_ref> edge(EdgeId edge) const {
        auto it = _edges.find(e);
        if (it == _edges.end()) return std::nullopt;
        return const_edge_ref{it};
    }
    
    vertex_iterator add_vertex() {
        VertexId v = ((size_t) _free_vertex_id)++;
        _verts[v] = VertexNode{};
        return v;
    }
    
    vertex_iterator add_vertex(const V& v) requires (HasVertexValue) {
        VertexId v = _free_vertex_id++;
        _verts[v] = VertexNode{.data = v};
        return v;
    }
    
    vertex_iterator add_vertex(V&& v) requires (HasVertexValue) {
        VertexId v = _free_vertex_id++;
        _verts[v] = VertexNode{.data = std::move(v)};
        return v;
    }
    
    edge_iterator insert_directed_edge(VertexId src, VertexId dst) {
        return insert_directed_edge(find_vertex(src), find_vertex(dst));
    }
    
    edge_iterator insert_directed_edge(vertex_iterator src, vertex_iterator dst) {
        if (src == end_vertices() or dst == end_vertices()) {
            return end_edges();
        }
        EdgeId eid = ((size_t) _free_edge_id)++;
        Edge edge {src->id(), dst->id()};
        VertexNode& v0 = src.node();
        VertexNode& v1 = dst.node();
        
        auto [new_edge, std::ignore] = _edges.insert({
            eid,
            EdgeNode{
                edge, 
                // point to the head of the incoming and outgoing edge rings.
                // if this is the first edge in each ring, point to the edge itself.
                .next_incoming = v1.incoming_edges ? v1.incoming_edges->head : e,
                .next_outgoing = v0.outgoing_edges ? v0.outgoing_edges->head : e,
            }
        });
    
        auto _append_tail = [this](VertexNode& v, EdgeDir dir, EdgeId eid) {
            std::optional<EdgeList>& edge_list = v.edges[(int) dir];
            if (edge_list) {
                // point the tail at the inserted edge
                auto tail = this->_edges.find(edge_list->tail);
                tail->second.next[(int) dir] = eid;
                // the new edge is the new tail
                edge_list->tail  = eid;
                edge_list->size += 1;
            } else {
                // there were no edges in the ring, so this new edge is the only one.
                // also, it already points at itself.
                edge_list = EdgeList{.head = eid, .tail = eid, .size = 1};
            }
        }
        
        _append_tail(v0, EdgeDir::Outgoing, eid);
        _append_tail(v1, EdgeDir::Incoming, eid);
        
        return edge_iterator{this, new_edge, EdgeDir::Outgoing};
    }
    
    std::pair<edge_iterator, edge_iterator> insert_undirected_edge(VertexId a, VertexId b) {
        edge_iterator e0 = add_directed_edge(a, b);
        edge_iterator e1 = add_directed_edge(b, a);
        return {e0, e1};
    }
    
    std::optional<E> remove(EdgeId eid) {
        return remove(_edges.find(eid));
    }
    
    std::optional<E> remove(edge_iterator edge) {
        if (edge == end_edges()) {
            return std::nullopt;
        }
        EdgeNode& edge_node = edge->node();
        Edge         e = edge_node.edge;
        EdgeId     eid = edge->id();
        vertex_iterator v0 = find_vertex(e.v0);
        vertex_iterator v1 = find_vertex(e.v1);
        
        _remove_from_ring(v0, EdgeDir::Outgoing, edge);
        _remove_from_ring(v1, EdgeDir::Incoming, edge);
        
        E data = std::move(edge_node.data);
        _edges.erase(edge.inner_iterator());
        return data;
    }
    
    std::optional<V> remove(vertex_iterator v) {
        if (v == end_vertices()) {
            return std::nullopt;
        }
        
        for (EdgeDir dir : {EdgeDir::Incoming, EdgeDir::Outgoing}) {
            EdgeList& ring = v->edges[(int) dir];
            for (auto e : begin_edges(v, dir) | std::ranges::views::take(ring.size)) {
                // remove from the /other/ vertex's ring
                const Edge& this_edge = e->edge();
                // if we're iterating over this vertex's incoming edges, then this vertex is the
                // destination, and the other vertex is the edge's source. Vice versa for
                // the other direction.
                EdgeId other_id = dir == EdgeDir::Incoming ? this_edge.v0 : this_edge.v1;
                vertex_iterator other = find_vertex(other_id);
                // remove the edge from the other vertex's (opposite-direction) ring
                _remove_from_ring(other, ~dir, e);
            }
        }
        V data = std::move(v->value());
        _verts.erase(v.inner_iterator());
        return data;
    }
    
};

} // namespace graph

