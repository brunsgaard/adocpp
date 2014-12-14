#include <unordered_map>
#include <queue>
#include <algorithm>
#include "dijkstra.h"

std

std::unordered_map<VertexId, int64_t> dijkstra(const UndirectedGraph &g, const std::shared_ptr<Vertex> &s){
  std::unordered_map<VertexId, int64_t> dist;

  dist.reserve(g.Size());
  dist.emplace(s->id, 0);

  auto const& ids = g.ValidIds();
  auto const& vertices = g.Get();
  std::for_each(ids.cbegin(), ids.cend(), [&dist,s](VertexId v) {
      if (v != s->id) {
        dist.emplace(v, -1);
      }
  });

}

