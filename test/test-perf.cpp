#include <iostream>
#include <random>
#include <chrono>

#include <lemon/list_graph.h>
#include <pcg_random.hpp>
#include <gtest/gtest.h>
#include <ankerl/unordered_dense.h>
// #include <robin_hood.h>

#include <graph/digraph.h>
#include <graph/digraph_map.h>

uint64_t rng_seed = 3976676893487096997ULL;
auto     rng_eng  = pcg64(rng_seed);

using namespace graph;

using perftimer_t = std::chrono::steady_clock;
using delta_t     = std::chrono::duration<double, std::milli>;

template <typename K, typename V>
using Map = ankerl::unordered_dense::map<K, V>;
// using Map = robin_hood::unordered_map<K, V>;
// using Map = std::unordered_map<K, V>;

using IntFloatGraph = Digraph<int, float, Map>;

void test_add_10k_nodes() {
    IntFloatGraph g;
    constexpr int N = 100'000;
    std::uniform_real_distribution<float> u_float(0.0f, 1.0f);
    std::uniform_int_distribution<int>    u_int(0, N - 1);
    auto start = perftimer_t::now();
    for (int i = 0; i < N; ++i) {
        g.emplace_vertex(u_int(rng_eng));
    }
    auto end = perftimer_t::now();
    delta_t delta_graph = end - start;
    std::cout << "Add 100k nodes (graph): " << delta_graph.count() << " ms" << std::endl;
    
    lemon::ListDigraph l;
    lemon::ListDigraph::NodeMap<int> l_map(l);
    start = perftimer_t::now();
    for (int i = 0; i < N; ++i) {
        l_map[l.addNode()] = u_int(rng_eng);
    }
    end = perftimer_t::now();
    delta_t delta_lemon = end - start;
    std::cout << "Add 100k nodes (lemon): " << delta_lemon.count() << " ms" << std::endl;
    std::cout << "  ratio: " << (delta_graph.count() / (double)delta_lemon.count()) << std::endl;
    
    start = perftimer_t::now();
    for (int i = 0; i < N * 5; ++i) {
        g.emplace_directed_edge((VertexId) u_int(rng_eng), (VertexId) u_int(rng_eng), u_float(rng_eng));
    }
    end = perftimer_t::now();
    delta_graph = end - start;
    std::cout << "Add 500k edges (graph): " << delta_graph.count() << " ms" << std::endl;
    
    lemon::ListDigraph::ArcMap<float> l_arc_map(l);
    start = perftimer_t::now();
    for (int i = 0; i < N * 5; ++i) {
        l_arc_map[l.addArc(l.nodeFromId(u_int(rng_eng)), l.nodeFromId(u_int(rng_eng)))] = u_float(rng_eng);
    }
    end = perftimer_t::now();
    delta_lemon = end - start;
    std::cout << "Add 500k edges (lemon): " << delta_lemon.count() << " ms" << std::endl;
    std::cout << "  ratio: " << (delta_graph.count() / (double)delta_lemon.count()) << std::endl;
}


int main() {
    test_add_10k_nodes();
    return 0;
}