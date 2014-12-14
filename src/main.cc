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
#include "ado.h"

DEFINE_string(datafile, "", "File to load as graph data");
DEFINE_bool(duplicated, false, "Does data file contain duplicated graph data");


UndirectedGraph loadWeightedGraph(std::ifstream &file) {
  // Find the largest vertex id
  VertexId a, b, max_id = 0;
  Weight w;
  while (file >> a >> b >> w) {
    if (a > max_id) {
      max_id = a;
    }
    if (b > max_id) {
      max_id = b;
    }
  }

  file.clear();
  file.seekg(0, std::ios_base::beg);

  UndirectedGraph g(max_id+1);

  while (file >> a >> b >> w) {
    if (FLAGS_duplicated) {
      g.AddAdjacentNode(a, b, w);
    } else {
      g.AddEdge(a, b, w);
    }
  }
  return g;
}

UndirectedGraph loadUnweightedGraph(std::ifstream &file) {
  // Find the largest vertex id
  VertexId a, b, max_id = 0;
  while (file >> a >> b) {
    if (a > max_id) {
      max_id = a;
    }
    if (b > max_id) {
      max_id = b;
    }
  }

  file.clear();
  file.seekg(0, std::ios_base::beg);

  UndirectedGraph g(max_id+1);

  while (file >> a >> b) {
    if (FLAGS_duplicated) {
      g.AddAdjacentNode(a, b);
    } else {
      g.AddEdge(a, b);
    }
  }
  return g;
}

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

  auto start = std::chrono::system_clock::now();

  // Test if our file is weighted
  std::string line;
  std::getline(network, line);
  auto tokens = countWordsInString(line);
  network.clear();
  network.seekg(0, std::ios_base::beg);
  UndirectedGraph g(0);
  LOG(INFO) << "Tokens " << tokens;
  if (tokens == 3) {
    LOG(INFO) << "Looks like a weighted graph";
    g = loadWeightedGraph(network);
  } else if (tokens == 2) {
    LOG(INFO) << "Looks like an un-weighted graph";
    g = loadUnweightedGraph(network);
  } else {
    LOG(ERROR) << "Did not understand format of file";
    return 1;
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

  // Get first valid vertex
  auto head = g.Get()[*(g.ValidIds().cbegin())];

  auto dist = dijkstra(g, head);
  LOG(INFO) << " Distances " << dist.size();

//  int i = 0;
//  for (auto it = dist.cbegin(); it != dist.cend(); ++it) {
//    LOG(INFO) << i++ << ": " << *it;
//  }

  auto dijkstra_end = std::chrono::system_clock::now();
  LOG(INFO) << "Dijkstra bench: " << std::chrono::duration_cast<std::chrono::milliseconds>(dijkstra_end - dijkstra_start).count() << " ms";

  prepro(g, 2);
}
