#include <sys/time.h>
#include <sys/resource.h>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <chrono>
#include <algorithm>

#include "graph.h"

long getMemoryUsage() {
  struct rusage usage;
  if (0 == getrusage(RUSAGE_SELF, &usage))
    return usage.ru_maxrss;  // bytes
  else
    return 0;
}

int main(int argc, char** argv) {
  printf("File: %s\n", argv[1]);
  std::ifstream network(argv[1]);

  // Find the largest vertex id
  VertexId a, b, max_id = 0;
  while (network >> a >> b) {
    if (a > max_id) {
      max_id = a;
    }
    if (b > max_id) {
      max_id = b;
    }
  }

  // Does our file contain duplicated edges for undirected graphs
  bool duplicated = static_cast<bool>(std::stoi(argv[2]));

  network.clear();
  network.seekg(0, std::ios_base::beg);

  auto g = UndirectedGraph(max_id+1);

  auto start = std::chrono::system_clock::now();

  int i = 0;
  std::chrono::system_clock::time_point since_last = start;
  while (network >> a >> b) {
    if (duplicated) {
      g.AddAdjacentNode(a, b);
    } else {
      g.AddEdge(a, b);
    }
    if (++i % 50000 == 0) {
      auto current_time = std::chrono::system_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(current_time - start).count();
      auto iteration = std::chrono::duration_cast<std::chrono::seconds>(current_time - since_last).count();
      printf("i: %d - last iteration: %lld seconds - total %ld minutes run\n", i, iteration, elapsed);
      since_last = current_time;
    }
  }

  auto end = std::chrono::system_clock::now();
  printf("Time spent: %lld\n",
         std::chrono::duration_cast<std::chrono::seconds>(end - start).count());
  printf("Ram used: %f\n", static_cast<double>(getMemoryUsage()) / 1024 / 1024);
}
