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
