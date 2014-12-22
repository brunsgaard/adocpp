#pragma once

#include <list>
#include <memory>
#include <sparsehash/sparse_hash_map>
#include <utility>
#include <vector>

#include "graph.h"
#include "ado.h"

std::pair<std::vector<Weight>,std::vector<VertexReference>> Dijkstra(const UndirectedGraph &g, const std::shared_ptr<Vertex> &source);

AdoClusterEntry DijkstraModified(const UndirectedGraph &g, const AdoICenter &ic, const std::shared_ptr<Vertex> &source);

std::list<VertexReference> DijkstraGetShortestPathTo(
    VertexReference vid, const std::vector<VertexReference> &previous);
