#include "graph.h"
#include <algorithm>
#include <cstdio>


UndirectedGraph::UndirectedGraph(uint64_t num_vertices) : num_vertices_(num_vertices), vertices_() {
  vertices_.reserve(num_vertices);
  // Pre-fill the vertex vector so we don't need to deal with non-initialized members
  for (uint64_t i=0; i < num_vertices; ++i) {
    vertices_.emplace_back(new Vertex(i));
  }
}


void UndirectedGraph::AddEdge(VertexId a, VertexId b, Weight w) {
  auto &av = vertices_[a];
  auto &bv = vertices_[b];
  av->adjacent.emplace_front(bv, w);
  bv->adjacent.emplace_front(av, w);
}

void UndirectedGraph::Print() {
  std::for_each(vertices_.cbegin(), vertices_.cend(), [](const VertexVector::value_type &vertex){
      printf("\n Adjacency list of vertex %lu\n head ", vertex->id);
      std::for_each(vertex->adjacent.cbegin(), vertex->adjacent.cend(), [](const AdjacentNode &node) {
          printf("-> %lu", node.vertex->id);
          });
      });
}

