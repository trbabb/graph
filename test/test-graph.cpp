#define TEST_MODULE_NAME test_graph

#include <iostream>
#include <random>
#include <chrono>

#include <pcg_random.hpp>
#include <gtest/gtest.h>
#include <ankerl/unordered_dense.h>

#define _GRAPH_TEST_HARNESS_INSTRUMENTATION

#include <graph/digraph.h>
#include <graph/digraph_map.h>

uint64_t rng_seed = 5441962910926078884ULL;
auto     rng_eng  = pcg64(rng_seed);

double _dummy = 0;

using namespace graph;

using perftimer_t = std::chrono::steady_clock;
using delta_t = std::chrono::duration<double, std::milli>;

template <typename K, typename V>
using Map = ankerl::unordered_dense::map<K, V>;

using Inspector = detail::Instrumentation;

std::ostream& operator<<(std::ostream& os, const VertexId& id) {
    os << (size_t)id;
    return os;
}

std::ostream& operator<<(std::ostream& os, const EdgeId& id) {
    os << (size_t)id;
    return os;
}

// todo: test bidirectional iteration of edges
// todo: more tests to shake out iterator invalidation: interleaved addition/deletion
//   of verts/edges
// todo: test that swap_edge_order() for edges that are not adjacent to the same vertex
//   returns false and does not change the graph
// todo: make a test with multiple edges between two vertices; try to delete some of them
// todo: test self-loop edges
// todo: test with each of the value types as void
// todo: test DigraphMap with each of the key types equal to void
// todo: test iterators and refs with const graphs


/// make a 'star' graph with a central vertex and `n_verts` adjacent to it in the
/// outgoing direction
template <template <typename...> typename Map>
auto make_star_graph(Digraph<int,float,Map>& g, size_t n_verts) {
    EXPECT_EQ(g.vertices_size(), 0);
    auto v0 = g.insert_vertex();
    v0->value() = 9999;
    VertexId v0_id = v0->id();
    for (int i = 0; i < n_verts; ++i) {
        auto v = g.insert_vertex();
        v->value() = i;
    }
    EXPECT_EQ(g.vertices_size(), n_verts + 1);
    EXPECT_EQ(g[v0_id], 9999);
    
    // connect the central vertex to every other vertex
    size_t ct = 0;
    int i = 0;
    for (auto v : g.vertices()) {
        ++ct;
        if (v != v0_id) {
            g.emplace_directed_edge(v0_id, v.id(), (float) i++);
        }
    }
    EXPECT_EQ(ct, n_verts + 1);
    EXPECT_EQ(g.edges_size(), n_verts);
    return g.find_vertex(v0_id);
}

// make a random graph, and then return an iterator to a random vertex.
// add `n_incident_edges` outgoing edges to the returned vertex.
template <template <typename...> typename Map>
auto make_random_graph(Digraph<int,float,Map>& g, size_t n_verts, size_t n_incident_edges) {
    auto rng   = std::uniform_int_distribution<size_t>(1, n_verts);
    auto rng_f = std::uniform_real_distribution<float>(0, 1);
    
    auto v = g.begin_vertices();
    for (size_t i = 0; i < n_verts; ++i) {
        v = g.insert_vertex();
        v->value() = i;
    }
    
    size_t n_edges = std::uniform_int_distribution<size_t>(0, n_verts * n_verts / 2)(rng_eng);
    for (size_t i = 0; i < n_edges; ++i) {
        VertexId v0 = (VertexId) rng(rng_eng);
        VertexId v1 = (VertexId) rng(rng_eng);
        g.emplace_directed_edge(v0, v1, rng_f(rng_eng));
    }
    for (size_t i = 0; i < n_incident_edges; ++i) {
        VertexId v_id = (VertexId) rng(rng_eng);
        auto v1 = g.find_vertex(v_id);
        g.emplace_directed_edge(v, v1, rng_f(rng_eng));
    }
    return v;
}


TEST(TEST_MODULE_NAME, test_create) {
    Digraph<int, float, Map> g;
    EXPECT_EQ(g.vertices_size(), 0);
    EXPECT_EQ(g.edges_size(), 0);
}


TEST(TEST_MODULE_NAME, test_access) {
    Digraph<int, float, Map> g;
    
    auto v0 = g.insert_vertex();
    v0->value() = 1;
    EXPECT_EQ(g.vertices_size(), 1);
    EXPECT_EQ(*v0, 1);
    VertexId v0_id = v0->id();
    
    auto vx = g.find_vertex(VertexId::invalid());
    EXPECT_EQ(vx, g.end_vertices());
    EXPECT_FALSE(vx);
    EXPECT_FALSE(g.contains(VertexId::invalid()));
    
    auto v1 = g.insert_vertex();
    v1->value() = 2;
    EXPECT_EQ(g.vertices_size(), 2);
    EXPECT_EQ(*v1, 2);
    VertexId v1_id = v1->id();
    EXPECT_EQ(g.edges_size(), 0);
    
    EXPECT_TRUE(g.contains(v0_id));
    EXPECT_TRUE(g.contains(v1_id));
    EXPECT_EQ(g.find_vertex(v0_id)->value(), 1);
    EXPECT_EQ(g.find_vertex(v1_id)->value(), 2);
    EXPECT_EQ(g[v0_id].value(), 1);
    EXPECT_EQ(g[v1_id].value(), 2);
    
    auto e0 = g.emplace_directed_edge(v0_id, v1_id);
    e0->value() = 3;
    EXPECT_EQ(g.edges_size(), 1);
    EXPECT_EQ(*e0, 3);
    EdgeId e0_id = e0->id();
    
    EXPECT_TRUE(g.contains(e0_id));
    EXPECT_EQ(g.find_edge(v0_id, v1_id), e0);
    EXPECT_EQ(g.find_edge(v1_id, v0_id), g.end_edges()); // xxx memory error here
    EXPECT_EQ(g[e0_id].value(), 3);
}


TEST(TEST_MODULE_NAME, test_iteration_and_edge_deletion) {
    Digraph<int, float, Map> g;
    size_t n_verts = 10;
    auto v0 = make_star_graph(g, n_verts);
    
    // the source of each edge should be v0
    for (auto e : g.edges()) {
        EXPECT_EQ(e.source_id(), *v0);
    }
    
    // verify that v0 iterates over all its incident edges
    for (auto e : g.incident_edges(v0)) {
        // the source of each edge should be v0
        EXPECT_EQ(e.source_id(), *v0);
    }
    
    // the edges were inserted in forward order
    size_t ct = 0;
    for (auto e : g.incident_edges(v0, EdgeDir::Outgoing)) {
        EXPECT_EQ(e.value(), ct++);
    }
    
    // verify that we can delete an edge
    auto e0 = g.begin_edges();
    g.erase(e0);
    EXPECT_EQ(g.edges_size(), n_verts - 1);
    
    // iteration should still terminate, and touch exactly the remaining edges
    ct = 0;
    for (auto e : g.incident_edges(v0, EdgeDir::Outgoing)) {
        EXPECT_EQ(e.source_id(), *v0);
        ++ct;
    }
    EXPECT_EQ(ct, n_verts - 1);
    
    // remove all of the edges incident to v0
    ct = 0;
    for (auto e = g.begin_incident_edges(v0, EdgeDir::Outgoing);
         e != g.end_edges(); )
    {
        e = g.erase(e);
        ++ct;
    }
    EXPECT_EQ(ct, n_verts - 1);
    // there should be no edges left
    EXPECT_EQ(g.edges_size(), 0);
}


TEST(TEST_MODULE_NAME, test_vtx_deletion) {
    Digraph<int, float, Map> g;
    size_t n_verts = 10;
    auto v0 = make_star_graph(g, n_verts);
    EXPECT_EQ(g.edges_size(), n_verts);
    EXPECT_EQ(g.vertices_size(), n_verts + 1);
    g.erase(v0);
    EXPECT_EQ(g.edges_size(), 0);
    EXPECT_EQ(g.vertices_size(), n_verts);
}


TEST(TEST_MODULE_NAME, graph_map) {
    DigraphMap<std::string, int, int, float> dgm;
    
    dgm["goof"].value() = 55;
    EXPECT_EQ(dgm["goof"], 55);
    EXPECT_EQ(dgm.vertices_size(), 1);
    EXPECT_EQ(Inspector::vert_id_to_key(dgm).size(), 1);
    EXPECT_EQ(Inspector::keys_to_vert_id(dgm).size(), 1);
    
    dgm["fleg"].value() = 99;
    EXPECT_EQ(dgm["fleg"], 99);
    EXPECT_EQ(dgm.vertices_size(), 2);
    EXPECT_EQ(Inspector::vert_id_to_key(dgm).size(), 2);
    EXPECT_EQ(Inspector::keys_to_vert_id(dgm).size(), 2);
    
    VertexId v0_id = dgm.find_vertex("goof")->id();
    VertexId v1_id = dgm.find_vertex("fleg")->id();
    
    auto [e, created] = dgm.emplace_directed_edge(99, "goof", "fleg");
    EXPECT_TRUE(created);
    e->value() = 77;
    EXPECT_EQ(dgm[99], 77);
    EXPECT_EQ(e->source_id(), v0_id);
    EXPECT_EQ(e->target_id(), v1_id);
    EXPECT_EQ(dgm.edges_size(), 1);
    EXPECT_EQ(Inspector::edge_id_to_key(dgm).size(), 1);
    EXPECT_EQ(Inspector::keys_to_edge_id(dgm).size(), 1);
    
    size_t deleted = dgm.erase("goof");
    EXPECT_EQ(deleted, 1);
    EXPECT_EQ(dgm.vertices_size(), 1);
    EXPECT_EQ(dgm.edges_size(), 0);
    EXPECT_EQ(Inspector::edge_id_to_key(dgm).size(), 0);
    EXPECT_EQ(Inspector::keys_to_edge_id(dgm).size(), 0);
    EXPECT_EQ(Inspector::vert_id_to_key(dgm).size(), 1);
    EXPECT_EQ(Inspector::keys_to_vert_id(dgm).size(), 1);
    
    EXPECT_EQ(dgm.find_edge(99), dgm.end_edges());
}


TEST(TEST_MODULE_NAME, test_reorder_edge) {
    Digraph<int, float, Map> g;
    size_t n_verts = 60;
    auto v = make_random_graph(g, n_verts, 10);
    auto range = g.incident_edges(v, EdgeDir::Outgoing);
    size_t n = range.size();
    EXPECT_GE(n, 10);
    float i = 0;
    std::vector<float> vals;
    vals.resize(n);
    for (auto e : range) {
        vals[i] = e.value() = i;
        i += 1;
    }
    size_t j = 0;
    for (auto e : range) {
        EXPECT_EQ(e.value(), vals[j]);
        j += 1;
    }
    
    std::uniform_int_distribution<size_t> rng(0, n - 1);
    for (size_t i = 0; i < n * 20; ++i) {
        size_t p = rng(rng_eng);
        size_t q = rng(rng_eng);
        auto e_p = range.begin();
        auto e_q = range.begin();
        for (int k = 0; k < p; ++k) ++e_p;
        for (int k = 0; k < q; ++k) ++e_q;
        EXPECT_TRUE(g.swap_edge_order(e_p, e_q, EdgeDir::Outgoing));
        std::swap(vals[p], vals[q]);
        j = 0;
        for (auto e : range) {
            EXPECT_EQ(e.value(), vals[j]);
            j += 1;
        }
    }
}


TEST(TEST_MODULE_NAME, test_insert_before) {
    Digraph<int, float, Map> g;
    size_t n_verts  = 60;
    size_t n_trials = 255;
    auto rng   = std::uniform_int_distribution<size_t>(1, n_verts);
    auto rng_f = std::uniform_real_distribution<float>(0, 1);
    
    for (auto trial = 0; trial < n_trials; ++trial) {
        auto v = make_random_graph(g, n_verts, 10);
        
        std::vector<float> vals;
        
        // force the existing edges to be in order
        auto edges = g.incident_edges(v, EdgeDir::Outgoing);
        int i = 0;
        float n = edges.size();
        for (auto e : edges) {
            float val = (i++ + rng_f(rng_eng)) / n;
            e.value() = val;
            vals.push_back(val);
        }
        
        int n_extra = 20;
        for (int i = 0; i < n_extra; ++i) {
            // pick a vertex to connect to
            size_t other_v = rng(rng_eng);
            auto v1 = g.find_vertex((VertexId) other_v);
            EXPECT_NE(v1, g.end_vertices());
            // pick a value for the new edge
            float val = rng_f(rng_eng);
            vals.push_back(val);
            
            // insert the edge in order
            auto edges = g.incident_edges(v, EdgeDir::Outgoing);
            auto e = edges.begin();
            while (e != edges.end() and e->value() < val) ++e;
            auto e_new = g.emplace_directed_edge_before(e, v1, val);
            
            EXPECT_TRUE(e_new);
            EXPECT_EQ(e_new->value(), val);
        }
        
        std::sort(vals.begin(), vals.end());
        
        EXPECT_EQ(edges.size(), n + n_extra);
        
        // validate that the edges are in sorted order
        float last = -100;
        size_t ct = 0;
        for (auto e : edges) {
            EXPECT_LE(last, e.value());
            last = e.value();
            ++ct;
        }
        
        EXPECT_EQ(ct, edges.size());
    }
}


TEST(TEST_MODULE_NAME, test_digraph_size) {
    using D          = Digraph<int, int, Map>;
    using DNoEdge    = Digraph<int, void, Map>;
    using DNoVert    = Digraph<void, int, Map>;
    using DKeyMap    = DigraphMap<int, int, void, int>;
    using DValMap    = DigraphMap<void, int, int, int>;
    using DKeyValMap = DigraphMap<int, int, int, int>;
    EXPECT_EQ(sizeof(D), sizeof(DNoEdge)); // this affects the internal node size;
    EXPECT_EQ(sizeof(D), sizeof(DNoVert)); // not the structure size
    
    EXPECT_LT(sizeof(D), sizeof(DKeyMap)); // digraph maps keep structure that map
    EXPECT_LT(sizeof(D), sizeof(DValMap)); // keys to vert and edge IDs, and back again
    
    EXPECT_EQ(sizeof(DKeyMap), sizeof(DValMap)); // a vert key index weighs the same
                                                 // as a vert value index
                                                 
    EXPECT_LT(sizeof(DValMap), sizeof(DKeyValMap)); // two indices weigh more than one
}


TEST(TEST_MODULE_NAME, test_double_indirect_perf) {
    size_t n = 10'000;
    auto rng   = std::uniform_int_distribution<size_t>();
    auto rng_d = std::uniform_real_distribution<double>(0, 1);
    Map<std::string, size_t> keys;
    Map<size_t, double> vals;
    std::string* key_arr = new std::string[n];
    size_t*      id_arr  = new size_t[n];
    for (size_t i = 0; i < 5000; ++i) {
        size_t id = rng(rng_eng);
        id_arr[i] = id;
        const std::string& k = key_arr[i] = std::to_string(id);
        keys[k] = id;
        vals[id] = rng_d(rng_eng);
    }
    
    double d = 0;
    auto t1 = perftimer_t::now();
    for (size_t i = 0; i < n; ++i) {
        auto it = keys.find(key_arr[i]);
        if (it != keys.end()) {
            d += vals[it->second];
        }
    }
    auto t2 = perftimer_t::now();
    _dummy = d;
    delta_t ms = t2 - t1;
    
    std::cout << "  key -> id -> val: " << ms.count() << " ms\n";
    
    d = 0;
    t1 = perftimer_t::now();
    for (size_t i = 0; i < n; ++i) {
        auto it = keys.find(key_arr[i]);
        if (it != keys.end()) {
            d += it->second;
        }
    }
    t2 = perftimer_t::now();
    _dummy = d;
    ms = t2 - t1;
    
    std::cout << "  key -> val: " << ms.count() << " ms\n";
    
    d = 0;
    t1 = perftimer_t::now();
    for (size_t i = 0; i < n; ++i) {
        auto it = vals.find(id_arr[i]);
        if (it != vals.end()) {
            d += it->second;
        }
    }
    t2 = perftimer_t::now();
    _dummy = d;
    ms = t2 - t1;
    
    std::cout << "  id -> val: " << ms.count() << " ms\n";
    
    delete[] key_arr;
    delete[] id_arr;
}
