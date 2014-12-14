#include <glog/logging.h>
#include <algorithm>
#include <cstdio>
#include <queue>

#include "graph.h"
#include "dfs.h"

UndirectedGraph::UndirectedGraph(uint64_t num_vertices) : num_vertices_(num_vertices), vertices_(), valid_vertices_() {
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

void UndirectedGraph::Print() const {
  std::for_each(vertices_.cbegin(), vertices_.cend(), [](const VertexVector::value_type &vertex){
    if (vertex.get() == NULL) {
      return;
    }
    LOG(INFO) << "Adjacency list of vertex " << vertex->id << " head ";
    std::for_each(
      vertex->adjacent.cbegin(),
      vertex->adjacent.cend(),
      [](const AdjacentNode &node) {
        LOG(INFO) << "-> " << node.vertex->id;
      });
    }
  );
}

void UndirectedGraph::RemoveUnconnectedComponents() {
  std::unordered_set<VertexId> seen;

  typedef std::priority_queue<std::unordered_set<VertexId>,std::vector<std::unordered_set<VertexId>>,BiggestSetCompare> SetSizeQueue;
  SetSizeQueue biggest_sets;

  for (auto it = vertices_.cbegin(); it != vertices_.cend(); ++it) {
    if (!seen.count(it->get()->id)) {
      auto r = dfs_iterative(*it);
      seen.insert(r.cbegin(), r.cend());
      biggest_sets.push(std::move(r));
    }
  }

  // Store the biggest set in valid_vertices_
  valid_vertices_.clear();
  valid_vertices_.insert(biggest_sets.top().cbegin(), biggest_sets.top().cend());

  // Remove the biggest set from the queue
  biggest_sets.pop();

  // Rest of the sets, clear the shared_ptrs to the point to NULL
  while (!biggest_sets.empty()) {
    // Remove set from graph
    for (auto it = biggest_sets.top().cbegin(); it != biggest_sets.top().cend(); ++it) {
      vertices_[*it].reset();
    }
    biggest_sets.pop();
  }
}
