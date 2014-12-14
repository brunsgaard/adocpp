#pragma once

#include "graph.h"
#include <cassert>
#include <stack>
#include <unordered_set>

inline std::unordered_set<VertexId> dfs_iterative(const std::shared_ptr<Vertex> &v) {
  std::stack<std::shared_ptr<Vertex>> s;
  s.emplace(v);
  std::unordered_set<VertexId> seen;
  while (!s.empty()) {
    auto a = s.top();
    s.pop();
    if (!seen.count(a->id)) {
      seen.emplace(a->id);
      for (auto it = a->adjacent.cbegin(); it != a->adjacent.cend(); ++it) {
        s.push(it->vertex);
      }
    }
  }
  return seen;
}
