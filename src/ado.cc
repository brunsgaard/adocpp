#include <glog/logging.h>

#include <unordered_map>
#include <queue>
#include <algorithm>
#include <vector>
#include <unordered_set>
#include <cstdlib>
#include <ctime>

#include "ado.h"
#include "graph.h"
#include "dijkstra.h"
#include "utils.h"

bool TakeNode(const int n, const int k) {
  static bool seeded = false;
  if (!seeded) {
    srand(9);
    seeded = true;
  }
  float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  return r <= pow(n, -1.0 / static_cast<double>(k));
}

void PreProcess(UndirectedGraph &g, const int k) {
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

  int i = 0;
  std::for_each(a.cbegin(), a.cend(), [&i](const std::unordered_set<VertexId> &s) {
    LOG(INFO) << "Set " << i++;
    std::for_each(s.cbegin(), s.cend(), [](VertexId vid) {
        LOG(INFO) << "   " << vid;
    });
  });

  AdoADict a_dist;
  a_dist.reserve(k+1);
  std::for_each(g.ValidIds().cbegin(), g.ValidIds().cend(), [&a_dist, k](VertexId vid){
      a_dist[k][vid] = std::make_pair(MaxWeight, VertexNone);
  });

  LOG(INFO) << "Ram used: " << static_cast<double>(getMemoryUsage()) / 1024 / 1024;

  std::unordered_map<VertexId, std::unordered_map<VertexId, Weight>> clusters;

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
    std::for_each(a_filtered.cbegin(), a_filtered.cend(), [&](VertexId vid) {
      LOG(INFO) << "Dijkstra mod for IC: " << i+1 << " Vertex " << vid;
      auto r = DijkstraModified(g, a_dist.at(i+1), g.Get()[vid]);
      clusters.insert(std::make_pair(vid, r));
    });
    LOG(INFO) << "Completed IC " << i;
    LOG(INFO) << "Ram used: " << static_cast<double>(getMemoryUsage()) / 1024 / 1024;
  }

  LOG(INFO) << "Ram used: " << static_cast<double>(getMemoryUsage()) / 1024 / 1024;

  std::for_each(a_dist.cbegin(), a_dist.cend(), [](const std::pair<int, const AdoICenter&> &i) {
    LOG(INFO) << "ICenter for k:" << i.first;
    std::for_each(i.second.cbegin(), i.second.cend(), [](const std::pair<VertexId, const AdoLink&> l) {
      LOG(INFO) << "  Vertex: " << l.first << " Dist: " << l.second.first << " Witness: " << l.second.second;
    });
  });

  std::for_each(clusters.cbegin(), clusters.cend(), [](const std::pair<VertexId, const std::unordered_map<VertexId,Weight>&> c) {
      LOG(INFO) << "Cluster for vertex " << c.first;
      int i = 0;
      std::for_each(c.second.cbegin(), c.second.cend(), [&i](const std::pair<VertexId, Weight>& w) {
        LOG(INFO) << "  " << w.first << ": " << w.second;
      });

  });

}
