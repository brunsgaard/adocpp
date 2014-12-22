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
DEFINE_string(infile, "", "Where to load preprocessed data");
DEFINE_int64(a, 2000, "From where?");
DEFINE_int64(b, 3000, "To where?");

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

  // Verify a and b exists
  CHECK_GT(g.ValidIds().count(FLAGS_a),0);
  CHECK_GT(g.ValidIds().count(FLAGS_b),0);

  // Verify that the maximum vertex id fits in a uint32_t
  // To reduce memory use at the critical preprocessing stage uint32_t vertex ids are required
  CHECK_GT(std::numeric_limits<uint32_t>::max(), g.MaxVertexId());

  // Dijkstra test
  auto dijkstra_start = std::chrono::system_clock::now();
  // Get first valid vertex
  auto head = g.Get()[*(g.ValidIds().cbegin())];
  auto dist = Dijkstra(g, head);
  LOG(INFO) << " Distances " << dist.first.size();
  auto dijkstra_end = std::chrono::system_clock::now();
  LOG(INFO) << "Dijkstra bench: " << std::chrono::duration_cast<std::chrono::milliseconds>(dijkstra_end - dijkstra_start).count() << " ms";

  // Input bench
  auto in_start = std::chrono::system_clock::now();

  // Figure out where to read
  if (FLAGS_infile.empty()) {
    auto dot = FLAGS_datafile.find_last_of(".");
    if (dot != std::string::npos) {
      FLAGS_infile = FLAGS_datafile.substr(0,dot);
      LOG(INFO) << "Defaulting to read from " << FLAGS_infile << ".adict, " << FLAGS_infile << ".map" ;
    }
  }

  // Read preprocessed data
  auto preprocessed = ReadPreprocessedFile(FLAGS_infile);
  auto in_end = std::chrono::system_clock::now();
  LOG(INFO) << "Read preprocessed data in: " << std::chrono::duration_cast<std::chrono::seconds>(in_end - in_start).count() << " seconds";

  LOG(INFO) << FLAGS_a << "->" << FLAGS_b << ":  " << Distk(preprocessed.first, preprocessed.second, FLAGS_a, FLAGS_b);
}
