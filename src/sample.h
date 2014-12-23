#pragma once

#include "graph.h"

//double GetStretch(const Weight, const Weight);
std::vector<std::pair<uint32_t, uint32_t>> GenerateSample(UndirectedGraph &g, uint32_t);
