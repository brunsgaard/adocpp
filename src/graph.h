#pragma once

#include <cstdint>
#include <forward_list>
#include <memory>
#include <set>
#include <unordered_set>
#include <vector>

struct AdjacentNode;
struct Vertex;
typedef double Weight;
const Weight MaxWeight = std::numeric_limits<double>::infinity();

typedef std::vector<std::shared_ptr<Vertex>> VertexVector;
typedef VertexVector::size_type VertexId;

typedef int64_t VertexReference;
const VertexReference VertexNone = -1;

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
  VertexId max_vertex_id_;

 public:
  UndirectedGraph(uint64_t num_vertices);

  void AddEdge(VertexId a, VertexId b, Weight w);
  inline void AddEdge(VertexId a, VertexId b) {
    AddEdge(a, b, 1.0);
  }
  inline void AddAdjacentNode(VertexId a, VertexId b, Weight w) {
    vertices_[a]->adjacent.emplace_front(vertices_[b], w);
  }
  void AddAdjacentNode(VertexId a, VertexId b) {
    AddAdjacentNode(a, b, 1.0);
  }

  void Print() const;

  inline const VertexVector& Get() const {
    return vertices_;
  }

  inline uint64_t Size() const {
    return valid_vertices_.size();
  }

  inline VertexId MaxVertexId() const {
    return max_vertex_id_;
  }

  inline const std::set<VertexId>& ValidIds() const {
    return valid_vertices_;
  }

  inline void ResetVertex(const VertexId vid) {
    vertices_[vid].reset();
  }
  inline void InitVertex(const VertexId vid) {
    vertices_[vid].reset(new Vertex(vid));
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
