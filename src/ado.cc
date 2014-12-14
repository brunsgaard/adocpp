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

bool takenode(const int n, const int k) {
  static bool seeded = false;
  if (!seeded) {
    srand(9);
    seeded = true;
  }
  float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  return r <= pow(n, -1 / k);
}

void prepro(const UndirectedGraph &g, const int k) {
  std::vector<std::unordered_set<VertexId>> a;
  // setup A1 - Ak
  do {
    a.clear();
    a.reserve(k);

    // setup A0 which is a copy of valid vertices.
    a.emplace_back();
    a.back().reserve(g.ValidIds().size());
    a.back().insert(g.ValidIds().cbegin(), g.ValidIds().cend());
    for (int i = 1; i < k; ++i) {
      std::unordered_set<VertexId> ax;
      auto a_size = a[0].size();
      std::copy_if(a[i - 1].cbegin(), a[i - 1].cend(), ax.begin(),
                   [a_size, k] { return takenode(a_size, k); });
      a.push_back(ax);
    }
  } while (a.back().size() == 0);
  a.emplace_back();

  AdoADict a_dist;
  std::for_each(g.ValidIds().cbegin(), g.ValidIds().cend(), [&a_dist, k](VertexId vid){
      a_dist[k][vid] = std::make_pair(MaxWeight, VertexNone);
  });

  for(int i=k-1; i<0; --i){



  }

}
