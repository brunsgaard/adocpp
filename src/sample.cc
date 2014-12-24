#include <glog/logging.h>
#include <random>
#include <iterator>
#include <cstdint>

#include "sample.h"

template <typename Iter, typename RandomGenerator>
Iter select_randomly(Iter start, Iter end, RandomGenerator& g) {
  std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
  std::advance(start, dis(g));
  return start;
}

template <typename Iter>
Iter select_randomly(Iter start, Iter end) {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  return select_randomly(start, end, gen);
}

std::vector<std::pair<uint32_t, uint32_t>> GenerateSample(UndirectedGraph& g,
                                                          uint32_t n) {
  auto population = std::set<std::pair<uint32_t, uint32_t>>();
  auto vertices = g.ValidIds();
  while (population.size() < n) {
    uint32_t from = *select_randomly(vertices.begin(), vertices.end());
    uint32_t to = *select_randomly(vertices.begin(), vertices.end());
    population.emplace(from, to);
    // LOG(INFO) << "Added new pair to population: (" << from << "," << to
    // <<")";
  }

  return std::vector<std::pair<uint32_t, uint32_t>>(population.begin(),
                                                    population.end());
}
