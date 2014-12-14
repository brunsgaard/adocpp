#pragma once

#include <memory>
#include <vector>

#include "graph.h"

std::vector<Weight> dijkstra(const UndirectedGraph &g,
                             const std::shared_ptr<Vertex> &source);

