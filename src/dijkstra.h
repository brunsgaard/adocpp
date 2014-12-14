#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "graph.h"

std::vector<Weight> dijkstra(const UndirectedGraph &g, const std::shared_ptr<Vertex> &source);

