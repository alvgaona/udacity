#ifndef GRAPHEDGE_H_
#define GRAPHEDGE_H_

#include <string>
#include <vector>

class GraphNode;  // forward declaration

class GraphEdge {
 private:
  // data handles (not owned)
  GraphNode *_childNode;
  GraphNode *_parentNode;

  // proprietary members
  int _id;
  std::vector<std::string> _keywords;  // list of topics associated with this
  // edge

 public:
  // constructor / desctructor
  GraphEdge(int id);
  GraphEdge(GraphEdge &source);                       // copy constructor
  GraphEdge(GraphEdge &&source) noexcept;            // move constructor
  GraphEdge& operator=(GraphEdge const& source);      // copy assignment
  // constructor
  GraphEdge& operator=(GraphEdge&& source) noexcept;  // move assignment
  // constructor

  // getter / setter
  int GetID() { return _id; }
  void SetChildNode(GraphNode *childNode);
  void SetParentNode(GraphNode *parentNode);
  GraphNode *GetChildNode() { return _childNode; }
  std::vector<std::string> GetKeywords() { return _keywords; }

  // proprietary functions
  void AddToken(std::string token);
};

#endif /* GRAPHEDGE_H_ */
