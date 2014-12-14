#include <gflags/gflags.h>
#include <glog/logging.h>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>

#include "graph.h"
#include "utils.h"
#include "dijkstra.h"

DEFINE_string(datafile, "", "File to load as graph data");
DEFINE_bool(duplicated, false, "Does data file contain duplicated graph data");

int main(int argc, char** argv) {
  FLAGS_logtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  LOG(INFO) << "File: " << FLAGS_datafile;
  std::ifstream network(FLAGS_datafile);
  if (!network.good()) {
    LOG(ERROR) << "Could not open file " << FLAGS_datafile;
    return 1;
  }

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

  network.clear();
  network.seekg(0, std::ios_base::beg);

  auto g = UndirectedGraph(max_id+1);

  auto start = std::chrono::system_clock::now();

  int i = 0;
  std::chrono::system_clock::time_point since_last = start;
  while (network >> a >> b) {
    if (FLAGS_duplicated) {
      g.AddAdjacentNode(a, b);
    } else {
      g.AddEdge(a, b);
    }
    if (++i % 500000 == 0) {
      auto current_time = std::chrono::system_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(current_time - start).count();
      auto iteration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - since_last).count();
      LOG(INFO) << "i: " << i << " - last iteration: " << iteration << " ms - total " << elapsed << " min run";
      since_last = current_time;
    }
  }

  auto end = std::chrono::system_clock::now();
  LOG(INFO) << "Time spent: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  LOG(INFO) << "Ram used: " << static_cast<double>(getMemoryUsage()) / 1024 / 1024;

  // Cleanup the graph
  auto cleanup_start = std::chrono::system_clock::now();
  g.RemoveUnconnectedComponents();
  auto cleanup_end = std::chrono::system_clock::now();
  LOG(INFO) << "Time spent during cleanup: " << std::chrono::duration_cast<std::chrono::seconds>(cleanup_end - cleanup_start).count();


  // Dijkstra test
  auto dijkstra_start = std::chrono::system_clock::now();

  auto dist = dijkstra(g, g.Get().front());
  LOG(INFO) << " Distances " << dist.size();

  auto dijkstra_end = std::chrono::system_clock::now();
  LOG(INFO) << "Dijkstra bench: " << std::chrono::duration_cast<std::chrono::milliseconds>(dijkstra_end - dijkstra_start).count() << " ms";


}
