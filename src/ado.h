#pragma once
#include <cstdint>
#include <sparsehash/sparse_hash_map>
#include <unordered_map>
#include <utility>
#include <memory>
#include <mutex>
#include "graph.h"

typedef std::pair<Weight, VertexReference> AdoLink;
typedef std::unordered_map<VertexId, AdoLink> AdoICenter;
typedef std::unordered_map<int, AdoICenter> AdoADict;
typedef std::unordered_map<uint32_t, float> AdoClusterEntry;
typedef google::sparse_hash_map<uint32_t, google::sparse_hash_map<uint32_t,float>> AdoVertexDistMap;

std::pair<AdoADict, std::shared_ptr<AdoVertexDistMap>> PreProcess(UndirectedGraph &g, const int k);

Weight Distk(const AdoADict &a, const AdoVertexDistMap &b, VertexId u, VertexId v);
