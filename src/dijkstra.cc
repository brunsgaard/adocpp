#include <unordered_map>
#include <queue>
#include <list>
#include <algorithm>
#include "dijkstra.h"

std::vector<Weight> dijkstra(const UndirectedGraph &g,
                             const std::shared_ptr<Vertex> &source) {
  auto const &vertices = g.Get();
  std::vector<Weight> min_distance(vertices.size(), MaxWeight);
  min_distance[source->id] = 0.0;
  std::set<std::pair<Weight, VertexId> > vertex_queue;
  std::vector<VertexId> previous;
  previous.clear();
  previous.resize(vertices.size(), -1);


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
  return min_distance;
}

std::list<VertexId> DijkstraGetShortestPathTo(
    VertexId vid, const std::vector<VertexId> &previous)
{
    std::list<VertexId> path;
    for ( ; vid != -1; vid = previous[vid])
        path.push_front(vid);
    return path;
}

