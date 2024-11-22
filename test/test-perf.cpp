#include <iostream>
#include <random>
#include <chrono>

#include <lemon/list_graph.h>
#include <pcg_random.hpp>
#include <gtest/gtest.h>
#include <ankerl/unordered_dense.h>
#include <absl/container/flat_hash_map.h>
// #include <robin_hood.h>

#include <graph/digraph.h>
#include <graph/digraph_map.h>

uint64_t rng_seed = 3976676893487096997ULL;
auto     rng_eng  = pcg64(rng_seed);

using namespace graph;

using perftimer_t = std::chrono::steady_clock;
using delta_t     = std::chrono::duration<double, std::milli>;

template <typename K, typename V>
// using Map = robin_hood::unordered_map<K, V>;
// using Map = ankerl::unordered_dense::map<K, V>;
using Map = std::unordered_map<K, V>;
// using Map = absl::flat_hash_map<K, V>;

/*
 * something is wrong.
 * 
 * - graph is slower than lemon. fine. maybe i didn't do a good job, but
 * - std::unordered map is slightly _faster_ than ankerl or absl. that disagrees
 *   with all the benchmarks and common sense
 * - it's not likely due to work that graph is doing, because emplace_vertex
 *   is just a thin wrapper around the map's emplace
 * 
 * - guess: my node data structure is too big? (56 bytes)
 *   - could confirm this by doing a benchmark with a same-sized structure
 *     using just the maps and no graph code
 *   - 56 bytes is really not big, though
 * - guess: my node data structure is doing some heavy / redundant work in the ctor?
 * - guess: breaking "trivially constructible" is slowing things down?
 * - guess: nontrivial move/copy assignment is happening for some reason?
 * 
 * - observations:
 *   - thirdparty maps are mainly faster when the stored data is only a word.
 *     larger data sizes lose their advantage. in clang:
 *     - unordered_dense is ~1.6x slower than unordered_map with a simple 56-byte struct
 *     - flat_hash is ~3x slower than unordered_map
 *   - thirdparty advantage is less in clang than gcc
 *   - gcc is slower; unordered_dense is slightly faster in clang; unordered_map is much faster.
 *     flat_hash is about the same. (bizarrely slow)
 *  
 * - conclusion: a malloc/free'd node implementation is likely to be beneficial compared to
 *   multiple indirections through a map. if we want this to be really good, we may have
 *   to rewrite it.
 */

using IntFloatGraph = Digraph<int, float, Map>;

void test_add_10k_nodes() {
    IntFloatGraph g;
    constexpr int N = 100'000;
    std::uniform_real_distribution<float> u_float(0.0f, 1.0f);
    std::uniform_int_distribution<int> u_int(0, N - 1);
    auto start = perftimer_t::now();
    for (int i = 0; i < N; ++i) {
        g.emplace_vertex(i);
    }
    auto end = perftimer_t::now();
    delta_t delta_graph = end - start;
    std::cout << "Add 100k nodes (graph): " << delta_graph.count() << " ms" << std::endl;
    
    lemon::ListDigraph l;
    lemon::ListDigraph::NodeMap<int> l_map(l);
    start = perftimer_t::now();
    for (int i = 0; i < N; ++i) {
        l_map[l.addNode()] = i;
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

struct Thing {
    size_t data[7];
};


template <typename M>
delta_t time_add_10k_nodes_map(M& map) {
    constexpr int N = 100'000;
    auto start = perftimer_t::now();
    for (int i = 0; i < N; ++i) {
        map.emplace(std::make_pair(i, Thing{}));
    }
    auto end = perftimer_t::now();
    return end - start;
}


void test_bare_maps() {
    std::unordered_map<size_t, Thing> umap;
    ankerl::unordered_dense::map<size_t, Thing> admap;
    absl::flat_hash_map<size_t, Thing> aflatmap;
    
    delta_t t_umap     = time_add_10k_nodes_map(umap);
    delta_t t_admap    = time_add_10k_nodes_map(admap);
    delta_t t_aflatmap = time_add_10k_nodes_map(aflatmap);
    
    std::cout << "-           std::unordered_map: " << t_umap.count()     << " ms" << std::endl;
    std::cout << "- ankerl::unordered_dense::map: " << t_admap.count()    << " ms" << std::endl;
    std::cout << "-          absl::flat_hash_map: " << t_aflatmap.count() << " ms" << std::endl;
    std::cout << "  ratio (unordered_dense : unordered_map): " << (t_admap.count()    / (double)t_umap.count()) << std::endl;
    std::cout << "  ratio (flat_hash       : unordered_map): " << (t_aflatmap.count() / (double)t_umap.count()) << std::endl;
    std::cout << std::endl;
}


int main() {
    test_add_10k_nodes();
    test_bare_maps();
    return 0;
}
