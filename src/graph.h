#pragma once

#include <cstdint>
#include <forward_list>
#include <memory>
#include <set>
#include <unordered_set>
#include <vector>

struct AdjacentNode;
struct Vertex;
typedef uint8_t Weight;
typedef std::vector<std::shared_ptr<Vertex>> VertexVector;
typedef VertexVector::size_type VertexId;

struct AdjacentNode
{
  std::shared_ptr<Vertex> vertex;
  Weight weight;
  AdjacentNode(std::shared_ptr<Vertex> v, Weight w) : vertex(v), weight(w) {

  }
};

struct Vertex
{
  std::forward_list<AdjacentNode> adjacent;
  VertexId id;
  Vertex(VertexId i) : adjacent(), id(i) {
  }
};


class UndirectedGraph
{
 protected:
  uint64_t num_vertices_;
  VertexVector vertices_;
  std::set<VertexId> valid_vertices_;

 public:
  UndirectedGraph(uint64_t num_vertices);

  void AddEdge(VertexId a, VertexId b, Weight w);
  inline void AddEdge(VertexId a, VertexId b) {
    AddEdge(a, b, 1);
  }
  inline void AddAdjacentNode(VertexId a, VertexId b, Weight w) {
    vertices_[a]->adjacent.emplace_front(vertices_[b], w);
  }
  void AddAdjacentNode(VertexId a, VertexId b) {
    AddAdjacentNode(a, b, 1);
  }

  void Print() const;
  inline const VertexVector& Get() const {
    return vertices_;
  }

  uint64_t Size() const {
    return valid_vertices_.size();
  }

  const std::set<VertexId>& ValidIds() const {
    return valid_vertices_;
  }

  class BiggestSetCompare
  {
   public:
    BiggestSetCompare() {}
    bool operator() (const std::unordered_set<VertexId>& lhs, const std::unordered_set<VertexId>&rhs) const
    {
      return lhs.size() < rhs.size();
    }
  };

  void RemoveUnconnectedComponents();
};
