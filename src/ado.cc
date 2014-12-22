#include <glog/logging.h>
#include <tbb/tbb.h>

#include <atomic>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <vector>
#include <unordered_set>
#include <random>
#include <ctime>

#include "ado.h"
#include "graph.h"
#include "dijkstra.h"
#include "utils.h"

bool TakeNode(const int n, const int k) {
  static std::mt19937 mt(9);
  float r = static_cast<double>(mt()) / static_cast<double>(mt.max());
  return r <= pow(n, -1.0 / static_cast<double>(k));
}

std::pair<AdoADict, AdoVertexDistMap> PreProcess(UndirectedGraph &g, const int k) {
  std::vector<std::unordered_set<VertexId>> a;
  // setup A1 - Ak
  do {
    a.clear();
    a.reserve(k);

    // setup A0 which is a copy of valid vertices.
    a.emplace_back();
    a.back().reserve(g.ValidIds().size());
    a.back().insert(g.ValidIds().cbegin(), g.ValidIds().cend());
    auto a_size = a[0].size();
    for (int i = 1; i < k; ++i) {
      std::unordered_set<VertexId> ax;
      std::copy_if(a[i - 1].begin(), a[i - 1].end(), std::inserter(ax, ax.begin()), [a_size, k](VertexId v) {
        auto t = TakeNode(a_size, k);
        return t;
      });
      a.push_back(ax);
    }
  } while (a.back().size() == 0);
  a.emplace_back();

  if (VLOG_IS_ON(2)) {
    int i = 0;
    std::for_each(a.cbegin(), a.cend(), [&i](const std::unordered_set<VertexId> &s) {
      VLOG(2) << "Set " << i++;
      std::for_each(s.cbegin(), s.cend(), [](VertexId vid) {
          VLOG(2) << "   " << vid;
      });
    });
  }

  AdoADict a_dist;
  a_dist.reserve(k+1);
  std::for_each(g.ValidIds().cbegin(), g.ValidIds().cend(), [&a_dist, k](VertexId vid){
      a_dist[k][vid] = std::make_pair(MaxWeight, VertexNone);
  });

  VLOG(1) << "Ram used: " << static_cast<double>(getMemoryUsage()) / 1024 / 1024;

  //AdoVertexDistMap clusters;
  auto concurrent_clusters = std::make_shared<AdoVertexConcurrentDistMap>();

  for(int i=k-1; i>=0; --i) {
    // Add a vertex connected to all vertices in our set
    auto vid_max = *g.ValidIds().rbegin()+1;
    g.InitVertex(vid_max);
    std::for_each(a[i].cbegin(), a[i].cend(), [&g, vid_max](VertexId vid) {
      g.AddEdge(vid_max, vid, 0);
    });
    // Run dijkstra from our added node
    auto r = Dijkstra(g, g.Get()[vid_max]);
    // Update a_dist
    std::for_each(g.ValidIds().cbegin(), g.ValidIds().cend(), [&a_dist,&r,i](VertexId vid) {
      auto dist = r.first[vid];
      auto path = DijkstraGetShortestPathTo(vid, r.second);
      auto path_it = path.cbegin();
      std::advance(path_it, 1);
      auto witness = *path_it;
      a_dist[i][vid] = std::make_pair(dist, witness);
      if (a_dist[i][vid].first == a_dist[i+1][vid].first) {
        a_dist[i][vid].second = a_dist[i+1][vid].second;
      }
    });
    // Remove node added above
    auto const &adjacent = g.Get()[vid_max]->adjacent;
    std::for_each(adjacent.cbegin(), adjacent.cend(), [&g, vid_max](AdjacentNode n) {
      CHECK_EQ(n.vertex->adjacent.front().vertex->id,vid_max);
      n.vertex->adjacent.pop_front();
    });
    g.ResetVertex(vid_max);

    std::unordered_set<VertexId> a_filtered;
    auto prev_a = a[i+1];
    std::copy_if(a[i].begin(), a[i].end(), std::inserter(a_filtered, a_filtered.begin()), [prev_a](VertexId v) {
      return prev_a.count(v) == 0;
    });
    std::vector<VertexId> a_vector(a_filtered.cbegin(), a_filtered.cend());

    std::atomic_uint_least64_t v_count(0);

    tbb::parallel_for(tbb::blocked_range<size_t>(0,a_vector.size()),
        [concurrent_clusters,&a_dist,&g,&a_vector,i,&v_count](const tbb::blocked_range<size_t>& r) {
      for(size_t n=r.begin(); n!=r.end(); ++n) {
        auto vid = a_vector[n];
        VLOG(2) << "Dijkstra mod for IC: " << i+1 << " Vertex " << vid;
        auto r = DijkstraModified(g, a_dist.at(i+1), g.Get()[vid]);
        AdoVertexConcurrentDistMap::accessor a;
        concurrent_clusters->insert(a, vid);
        a->second = r;
        // Output useful stats every 10 seconds
        FB_LOG_EVERY_MS(INFO,10000) << "Ram used: " << static_cast<double>(getMemoryUsage()) / 1024 / 1024;
        FB_LOG_EVERY_MS(INFO,10000) << "Clusters computed: " << concurrent_clusters->size();
        uint_least64_t counter = v_count.fetch_add(1);
        FB_LOG_EVERY_MS(INFO,10000) << "Current: " << (counter+1) << "/" << a_vector.size();
      }
    });

//    std::for_each(a_filtered.cbegin(), a_filtered.cend(), [&](VertexId vid) {
//      VLOG(2) << "Dijkstra mod for IC: " << i+1 << " Vertex " << vid;
//      auto r = DijkstraModified(g, a_dist.at(i+1), g.Get()[vid]);
//      clusters.insert(std::make_pair(vid, r));
//    });
    VLOG(1) << "Completed IC " << i;
    VLOG(1) << "Ram used: " << static_cast<double>(getMemoryUsage()) / 1024 / 1024;
  }


  AdoVertexDistMap bunch;
//  std::for_each(clusters.cbegin(), clusters.cend(), [&bunch](const std::pair<VertexId, const std::unordered_map<VertexId,Weight>&> c) {
//      auto vid = c.first;
//      std::for_each(c.second.cbegin(), c.second.cend(), [&bunch,vid](const std::pair<VertexId, Weight>& w) {
//          bunch[w.first][vid] = w.second;
//      });
//  });
  std::for_each(concurrent_clusters->begin(), concurrent_clusters->end(), [&bunch](const std::pair<VertexId, const google::sparse_hash_map<VertexId, Weight>&> c) {
      auto vid = c.first;
      std::for_each(c.second.begin(), c.second.end(), [&bunch,vid](const std::pair<VertexId, Weight>& w) {
          bunch[w.first][vid] = w.second;
      });
  });


  VLOG(1) << "Ram used: " << static_cast<double>(getMemoryUsage()) / 1024 / 1024;

  if (VLOG_IS_ON(2)) {
    std::for_each(a_dist.cbegin(), a_dist.cend(), [](const std::pair<int, const AdoICenter&> &i) {
      VLOG(2) << "ICenter for k:" << i.first;
      std::for_each(i.second.cbegin(), i.second.cend(), [](const std::pair<VertexId, const AdoLink&> l) {
        VLOG(2) << "  Vertex: " << l.first << " Dist: " << l.second.first << " Witness: " << l.second.second;
      });
    });
//    std::for_each(clusters.cbegin(), clusters.cend(), [](const std::pair<VertexId, const std::unordered_map<VertexId,Weight>&> c) {
    std::for_each(concurrent_clusters->begin(), concurrent_clusters->end(), [](const std::pair<VertexId, const google::sparse_hash_map<VertexId, Weight>&> c) {
        VLOG(2) << "Cluster for vertex " << c.first;
        std::for_each(c.second.begin(), c.second.end(), [](const std::pair<VertexId, Weight>& w) {
          VLOG(2) << "  " << w.first << ": " << w.second;
        });

    });
    std::for_each(bunch.cbegin(), bunch.cend(), [](const std::pair<VertexId, const std::unordered_map<VertexId,Weight>&> c) {
        VLOG(2) << "Bunch for vertex " << c.first;
        std::for_each(c.second.cbegin(), c.second.cend(), [](const std::pair<VertexId, Weight>& w) {
          VLOG(2) << "  " << w.first << ": " << w.second;
        });
    });
  }
  return std::make_pair(a_dist, bunch);
}

Weight Distk(const AdoADict &a, const AdoVertexDistMap &b, VertexId u, VertexId v) {
  auto w = u;
  int i = 0;
  while (b.at(v).count(w) == 0) {
    ++i;
    std::swap(v,u);
    w = a.at(i).at(u).second;
  }
  return a.at(i).at(u).first + b.at(v).at(w);
}
