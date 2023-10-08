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

// todo: test emplace_before()
// todo: test swap_edge_order()
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
    VertexId v0_id = v0;
    for (int i = 0; i < n_verts; ++i) {
        auto v = g.insert_vertex();
        v->value() = i;
    }
    EXPECT_EQ(g.vertices_size(), n_verts + 1);
    EXPECT_EQ(g[v0_id], 9999);
    
    // connect the central vertex to every other vertex
    size_t ct = 0;
    for (auto v : g.vertices()) {
        ++ct;
        if (v != v0_id) {
            g.emplace_directed_edge(v0_id, v.id());
        }
    }
    EXPECT_EQ(ct, n_verts + 1);
    EXPECT_EQ(g.edges_size(), n_verts);
    return g.find_vertex(v0_id);
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
    VertexId v0_id = v0;
    
    auto v1 = g.insert_vertex();
    v1->value() = 2;
    EXPECT_EQ(g.vertices_size(), 2);
    EXPECT_EQ(*v1, 2);
    VertexId v1_id = v1;
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
    EdgeId e0_id = e0;
    
    EXPECT_TRUE(g.contains(e0_id));
    EXPECT_EQ(g.find_edge(v0_id, v1_id), e0);
    EXPECT_EQ(g.find_edge(v1_id, v0_id), g.end_edges());
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
    
    // verify that we can delete an edge
    auto e0 = g.begin_edges();
    g.erase(e0);
    EXPECT_EQ(g.edges_size(), n_verts - 1);
    
    // iteration should still terminate, and touch exactly the remaining edges
    size_t ct = 0;
    for (auto e : g.incident_edges(v0, EdgeDir::Outgoing)) {
        EXPECT_EQ(e.source_id(), *v0);
        ++ct;
    }
    EXPECT_EQ(ct, n_verts - 1);
    
    // remove all of the edges incident to v0
    ct = 0;
    for (auto e = g.begin_incident_edges(v0, EdgeDir::Outgoing);
         e != g.end_incident_edges(); )
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
