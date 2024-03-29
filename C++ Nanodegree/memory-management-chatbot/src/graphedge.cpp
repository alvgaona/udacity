#include <iostream>

#include "../include/graphedge.h"
#include "../include/graphnode.h"

GraphEdge::GraphEdge(int id) {
  _id = id;
  _childNode = nullptr;
  _parentNode = nullptr;
  _keywords = {};
}

GraphEdge::GraphEdge(GraphEdge &source) {
  _childNode = source._childNode;
  _parentNode = source._parentNode;
  _id = source._id;
  _keywords = source._keywords;
}

GraphEdge::GraphEdge(GraphEdge &&source) noexcept {
  _childNode = source._childNode;
  _parentNode = source._parentNode;
  _id = source._id;
  _keywords = std::move(source._keywords);

  source._childNode = nullptr;
  source._parentNode = nullptr;
  source._id = 0;
}

GraphEdge& GraphEdge::operator=(GraphEdge const& source) {
  std::cout << "GraphEdge Copy Assignment Operator" << std::endl;

  if (this == &source) {
    return *this;
  }

  // shallow copy of data handles
  _childNode = source._childNode;
  _parentNode = source._parentNode;
  _id = source._id;
  _keywords = source._keywords;

  return *this;
}

GraphEdge& GraphEdge::operator=(GraphEdge &&source) noexcept {
  std::cout << "GraphEdge Move Assignment Operator" << std::endl;

  if (this == &source) {
    return *this;
  }

  // transfering data handles
  _childNode = source._childNode;
  _parentNode = source._parentNode;
  _id = source._id;
  _keywords = std::move(source._keywords);

  // releasing data handles
  source._childNode = nullptr;
  source._parentNode = nullptr;
  source._id = 0;

  return *this;
}

void GraphEdge::SetChildNode(GraphNode *childNode) { _childNode = childNode; }

void GraphEdge::SetParentNode(GraphNode *parentNode) {
  _parentNode = parentNode;
}

void GraphEdge::AddToken(std::string token) { _keywords.push_back(token); }
