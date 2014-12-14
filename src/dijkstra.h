#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>

#include "graph.h"

std::unordered_map<VertexId, int64_t> dijkstra(const UndirectedGraph &g, const std::shared_ptr<Vertex> &v);

class DistCompare
{
 public:
  DistCompare() {}
  bool operator() (const std::unordered_set<VertexId>& lhs, const std::unordered_set<VertexId>&rhs) const
  {
    return lhs.size() < rhs.size();
  }
};
