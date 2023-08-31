#define TEST_MODULE_NAME test_graph

#include <iostream>
#include <random>
#include <pcg_random.hpp>
#include <gtest/gtest.h>
#include <chrono>

#include <ankerl/unordered_dense.h>

#include <graph/graph.h>
#include <graph/graph_map.h>


uint64_t rng_seed = 5441962910926078884ULL;
auto     rng_eng  = pcg64(rng_seed);

double _dummy = 0;

using namespace graph;

using perftimer_t = std::chrono::steady_clock;
using delta_t = std::chrono::duration<double, std::milli>;

template <typename K, typename V>
using Map = ankerl::unordered_dense::map<K, V>;

std::ostream& operator<<(std::ostream& os, const VertexId& id) {
    os << (size_t)id;
    return os;
}

TEST(TEST_MODULE_NAME, test_create) {
    Digraph<int, float, Map> g;
    EXPECT_EQ(g.vertices_size(), 0);
    EXPECT_EQ(g.edges_size(), 0);
}


TEST(TEST_MODULE_NAME, test_access) {
    Digraph<int, float> g;
    
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
    
    auto e0 = g.insert_directed_edge(v0_id, v1_id);
    e0->value() = 3;
    EXPECT_EQ(g.edges_size(), 1);
    EXPECT_EQ(*e0, 3);
    EdgeId e0_id = e0;
    
    EXPECT_TRUE(g.contains(e0_id));
    // todo: ambiguous overload-- either make explicit overloads for const and non-const
    // iterators so the 'right one' has no conversion, or remove auto-conversion of iters to other things
    EXPECT_EQ(g.find_edge(v0_id, v1_id), e0);
    EXPECT_EQ(g.find_edge(v1_id, v0_id), g.end_edges());
    EXPECT_EQ(g[e0_id].value(), 3);
}


TEST(TEST_MODULE_NAME, graph_map) {
    DigraphMap<std::string, int, int, float> dgm;
    
    auto i = dgm["goof"];
}


TEST(TEST_MODULE_NAME, test_double_indirect) {
    size_t n = 10'000;
    auto rng   = std::uniform_int_distribution<size_t>();
    auto rng_d = std::uniform_real_distribution<double>(0, 1);
    Map<std::string, size_t> keys;
    Map<size_t, double> vals;
    std::string* key_arr = new std::string[n];
    size_t* id_arr = new size_t[n];
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
    
    std::cout << "key -> id -> val: " << ms.count() << "\n";
    
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
    
    std::cout << "key -> val: " << ms.count() << "\n";
    
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
    
    std::cout << "id -> val: " << ms.count() << "\n";
    
    
    delete[] key_arr;
}