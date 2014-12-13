#include <sys/time.h>
#include <sys/resource.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <memory>
#include <chrono>
#include <set>

#include "undirected_graph.h"
#include "dijkstra.h"

long getMemoryUsage() {
  struct rusage usage;
  if (0 == getrusage(RUSAGE_SELF, &usage))
    return usage.ru_maxrss;  // bytes
  else
    return 0;
}

int main(int argc, char** argv) {
  printf("%s\n", argv[1]);
  std::ifstream network(argv[1]);
  uint32_t a, b;
  std::set<uint32_t> seen_vertices;

  auto start = std::chrono::system_clock::now();
  std::auto_ptr<alg::UndirectedGraph> g(new alg::UndirectedGraph());
  int i = 0;
  std::chrono::system_clock::time_point since_last = start;
  while (network >> a >> b) {
    //    printf("a: %d, b: %d\n", a, b);
    if (seen_vertices.emplace(a).second) {
      g->add_vertex(a);
    }
    if (seen_vertices.emplace(b).second) {
      g->add_vertex(b);
    }
    g->add_edge(a, b, 1);
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

  auto di_start = std::chrono::high_resolution_clock::now();
  alg::Dijkstra::run(*g, 1);
  auto di_end = std::chrono::high_resolution_clock::now();
  printf("Time spent dijkstra: %lld\n",
         std::chrono::duration_cast<std::chrono::milliseconds>(di_end - di_start)
             .count());
}
