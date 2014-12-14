#pragma once
#include <unordered_map>
#include <utility>
#include <cstdint>
#include "graph.h"

typedef int64_t VertexReference;
const VertexReference VertexNone = -1;
typedef std::unordered_map<
    int, std::unordered_map<VertexId, std::pair<Weight, VertexReference>>>
    AdoADict;

void prepro(const UndirectedGraph &g, const int k);
