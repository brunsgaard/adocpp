#pragma once

#include <memory>
#include <vector>
#include <utility>
#include <list>

#include "graph.h"
#include "ado.h"

std::pair<std::vector<Weight>,std::vector<VertexReference>> Dijkstra(const UndirectedGraph &g, const std::shared_ptr<Vertex> &source);

std::unordered_map<VertexId, Weight> DijkstraModified(const UndirectedGraph &g, const AdoICenter &ic, const std::shared_ptr<Vertex> &source);

std::list<VertexReference> DijkstraGetShortestPathTo(
    VertexReference vid, const std::vector<VertexReference> &previous);
