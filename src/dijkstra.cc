#include <glog/logging.h>
#include <unordered_map>
#include <queue>
#include <list>
#include <algorithm>

#include "dijkstra.h"

std::pair<std::vector<Weight>,std::vector<VertexReference>> Dijkstra(const UndirectedGraph &g, const std::shared_ptr<Vertex> &source) {
  auto const &vertices = g.Get();
  std::vector<Weight> min_distance(vertices.size(), MaxWeight);
  min_distance[source->id] = 0.0;
  std::set<std::pair<Weight, VertexId> > vertex_queue;

  std::vector<VertexReference> previous(vertices.size(), VertexNone);

  vertex_queue.insert(std::make_pair(min_distance[source->id], source->id));
  while (!vertex_queue.empty()) {
    auto dist = vertex_queue.begin()->first;
    auto u = vertex_queue.begin()->second;
    vertex_queue.erase(vertex_queue.begin());
    // Visit each edge exiting u
    const std::forward_list<AdjacentNode> &neighbors = vertices[u]->adjacent;
    for (auto neighbor_iter = neighbors.cbegin();
         neighbor_iter != neighbors.cend(); neighbor_iter++) {
      VertexId v = neighbor_iter->vertex->id;
      Weight weight = neighbor_iter->weight;
      Weight distance_through_u = dist + weight;
      if (distance_through_u < min_distance[v]) {
        vertex_queue.erase(std::make_pair(min_distance[v], v));
        min_distance[v] = distance_through_u;
        previous[v] = u;
        vertex_queue.emplace(min_distance[v], v);
      }
    }
  }
  return std::make_pair(min_distance, previous);
}

std::unordered_map<VertexId, Weight> DijkstraModified(const UndirectedGraph &g, const AdoICenter &ic, const std::shared_ptr<Vertex> &source) {
  auto const &vertices = g.Get();
  std::unordered_map<VertexId, Weight> min_distance;
  min_distance.emplace(source->id, 0.0);

  std::set<std::pair<Weight, VertexId> > vertex_queue;

  //std::vector<VertexReference> previous(vertices.size(), VertexNone);

  std::unordered_set<VertexId> fully_relaxed;
  fully_relaxed.reserve(g.Size());

  vertex_queue.insert(std::make_pair(0.0, source->id));
  while (!vertex_queue.empty()) {
    auto dist = vertex_queue.begin()->first;
    auto u = vertex_queue.begin()->second;
    vertex_queue.erase(vertex_queue.begin());
    if (fully_relaxed.count(u) > 0) {
//      LOG(INFO) << "Already relaxed vertex " << u;
      continue;
    } else {
      fully_relaxed.insert(u);
    }
    // Visit each edge exiting u
    const std::forward_list<AdjacentNode> &neighbors = vertices[u]->adjacent;
    for (auto neighbor_iter = neighbors.cbegin();
         neighbor_iter != neighbors.cend(); neighbor_iter++) {
      VertexId v = neighbor_iter->vertex->id;
      Weight weight = neighbor_iter->weight;
      Weight distance_through_u = dist + weight;
      if (distance_through_u < ic.at(v).first) {
        auto it = min_distance.find(v);
        Weight w = (it == min_distance.end()) ? MaxWeight : it->second;
        vertex_queue.erase(std::make_pair(w, v));
        min_distance[v] = distance_through_u;
        //previous[v] = u;
        vertex_queue.emplace(w, v);
      }
    }
  }
  return min_distance;
}

std::list<VertexReference> DijkstraGetShortestPathTo(
    VertexReference vid, const std::vector<VertexReference> &previous) {
    std::list<VertexReference> path;
    for ( ; vid != VertexNone; vid = previous[vid]) {
        path.push_front(vid);
    }
    return path;
}

