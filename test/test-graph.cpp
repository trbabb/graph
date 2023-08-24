#define TEST_MODULE_NAME test_graph

#include <iostream>
#include <pcg_random.hpp>
#include <gtest/gtest.h>

#include <graph/graph.h>

using namespace graph;

TEST(TEST_MODULE_NAME, test_create) {
    DirectedGraph<int, float> g;
    EXPECT_EQ(g.vertices_size(), 0);
    EXPECT_EQ(g.edges_size(), 0);
}
