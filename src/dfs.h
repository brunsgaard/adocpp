#pragma once

#include "graph.h"
#include <unordered_set>
#include <memory>

std::unordered_set<VertexId> dfs_iterative(const std::shared_ptr<Vertex> &v);
