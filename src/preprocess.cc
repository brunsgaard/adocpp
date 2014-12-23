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
DEFINE_string(outfile, "", "Where to output preprocessed data");

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

  UndirectedGraph g(max_id+2);

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

  UndirectedGraph g(max_id+2);

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

  // Verify that the maximum vertex id fits in a uint32_t
  // To reduce memory use at the critical preprocessing stage uint32_t vertex ids are required
  CHECK_GT(std::numeric_limits<uint32_t>::max(), g.MaxVertexId());

  // Figure out where to output
  if (FLAGS_outfile.empty()) {
    auto dot = FLAGS_datafile.find_last_of(".");
    if (dot != std::string::npos) {
      FLAGS_outfile = FLAGS_datafile.substr(0,dot);
      LOG(INFO) << "Defaulting to output at " << FLAGS_outfile << ".adict, " << FLAGS_outfile << ".map" ;
    }
  }

  // Pre process bench
  auto prepro_start = std::chrono::system_clock::now();
  // Preprocess graph
  PreProcess(g, 2, FLAGS_outfile);
  auto prepro_end = std::chrono::system_clock::now();
  LOG(INFO) << "Preprocess bench: " << std::chrono::duration_cast<std::chrono::seconds>(prepro_end - prepro_start).count() << " seconds";

}
