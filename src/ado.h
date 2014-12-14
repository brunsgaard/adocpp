#pragma once
#include <unordered_map>
#include <utility>
#include <cstdint>
#include "graph.h"

typedef std::pair<Weight, VertexReference> AdoLink;
typedef std::unordered_map<VertexId, AdoLink> AdoICenter;
typedef std::unordered_map<int, AdoICenter> AdoADict;

void PreProcess(UndirectedGraph &g, const int k);
