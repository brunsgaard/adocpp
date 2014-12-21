#pragma once
#include <tbb/tbb.h>
#include <unordered_map>
#include <utility>
#include <cstdint>
#include "graph.h"

typedef std::pair<Weight, VertexReference> AdoLink;
typedef std::unordered_map<VertexId, AdoLink> AdoICenter;
typedef std::unordered_map<int, AdoICenter> AdoADict;
typedef std::unordered_map<VertexId, std::unordered_map<VertexId, Weight>> AdoVertexDistMap;
typedef tbb::concurrent_hash_map<VertexId, std::unordered_map<VertexId, Weight>> AdoVertexConcurrentDistMap;

std::pair<AdoADict, AdoVertexDistMap> PreProcess(UndirectedGraph &g, const int k);

Weight Distk(const AdoADict &a, const AdoVertexDistMap &b, VertexId u, VertexId v);
