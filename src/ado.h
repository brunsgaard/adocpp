#pragma once
#include <tbb/tbb.h>
#include <cstdint>
#include <sparsehash/sparse_hash_map>
#include <unordered_map>
#include <utility>
#include "graph.h"

typedef std::pair<Weight, VertexReference> AdoLink;
typedef std::unordered_map<VertexId, AdoLink> AdoICenter;
typedef std::unordered_map<int, AdoICenter> AdoADict;
//typedef google::sparse_hash_map<VertexId, Weight> AdoClusterEntry;
typedef google::sparse_hash_map<uint32_t, float> AdoClusterEntry;
typedef std::unordered_map<VertexId, AdoClusterEntry> AdoVertexDistMap;
typedef tbb::concurrent_hash_map<VertexId, AdoClusterEntry> AdoVertexConcurrentDistMap;

std::pair<AdoADict, AdoVertexDistMap> PreProcess(UndirectedGraph &g, const int k);

Weight Distk(const AdoADict &a, const AdoVertexDistMap &b, VertexId u, VertexId v);
