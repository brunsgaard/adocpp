#pragma once
#include <tbb/tbb.h>
#include <cstdint>
#include <sparsehash/sparse_hash_map>
#include <unordered_map>
#include <utility>
#include <cstdio>
#include <memory>
#include "graph.h"

typedef std::pair<Weight, VertexReference> AdoLink;
typedef std::unordered_map<VertexId, AdoLink> AdoICenter;
typedef std::unordered_map<int, AdoICenter> AdoADict;
//typedef google::sparse_hash_map<VertexId, Weight> AdoClusterEntry;
typedef google::sparse_hash_map<uint32_t, float> AdoClusterEntry;
typedef std::unordered_map<VertexId, AdoClusterEntry> AdoVertexDistMap;
typedef tbb::concurrent_unordered_map<VertexId, AdoClusterEntry> AdoVertexConcurrentDistMap;

void PreProcess(UndirectedGraph &g, const int k, const std::string &path);

Weight Distk(const AdoADict &a, const AdoVertexDistMap &b, VertexId u, VertexId v);

typedef std::unique_ptr<std::FILE, int (*)(std::FILE *)> unique_file_ptr;
void WritePreprocessedToFile(const std::string &path, const AdoADict &a_dict);
void WritePreprocessedToFile(const unique_file_ptr& fm, VertexId vid, AdoClusterEntry &cluster);
std::pair<AdoADict, AdoVertexDistMap> ReadPreprocessedFile(const std::string &path);
