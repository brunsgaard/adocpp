#pragma once

#include <memory>
#include <vector>
#include <utility>
#include <list>

#include "graph.h"

std::pair<std::vector<Weight>,std::vector<VertexReference>> dijkstra(const UndirectedGraph &g, const std::shared_ptr<Vertex> &source);

std::list<VertexReference> DijkstraGetShortestPathTo(
    VertexReference vid, const std::vector<VertexReference> &previous);
