#define TEST_MODULE_NAME test_graph

#include <iostream>
#include <pcg_random.hpp>
#include <gtest/gtest.h>

#include <graph/graph.h>

using namespace graph;

std::ostream& operator<<(std::ostream& os, const VertexId& id) {
    os << (size_t)id;
    return os;
}

TEST(TEST_MODULE_NAME, test_create) {
    DirectedGraph<int, float> g;
    EXPECT_EQ(g.vertices_size(), 0);
    EXPECT_EQ(g.edges_size(), 0);
}


TEST(TEST_MODULE_NAME, test_access) {
    DirectedGraph<int, float> g;
    
    auto v0 = g.add_vertex();
    v0->value() = 1;
    EXPECT_EQ(g.vertices_size(), 1);
    EXPECT_EQ(*v0, 1);
    VertexId v0_id = v0;
    
    auto v1 = g.add_vertex();
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
}
