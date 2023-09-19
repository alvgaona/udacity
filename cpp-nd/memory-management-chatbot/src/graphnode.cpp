#include <iostream>
#include <memory>
#include <exception>

#include "../include/graphnode.h"
#include "../include/graphedge.h"
#include "../include/chatlogic.h"

GraphNode::GraphNode(int id) { _id = id; }

GraphNode::~GraphNode() {}

GraphNode::GraphNode(GraphNode&& source) noexcept {
  _chatBot = source._chatBot;
  _childEdges = std::move(source._childEdges);
  _parentEdges = std::move(source._parentEdges);
  _id = source._id;
  _answers = std::move(source._answers);

  // release handles
  source._id = -1;
}

GraphNode& GraphNode::operator=(GraphNode&& source) noexcept {
  if (this == &source) {
    return *this;
  }

  _chatBot = source._chatBot;
  _childEdges = std::move(source._childEdges);
  _parentEdges = std::move(source._parentEdges);
  _id = source._id;
  _answers = std::move(source._answers);

  // release handles;
  source._id = -1;

  return *this;
}



void GraphNode::AddToken(const std::string& token) { _answers.push_back(token); }

void GraphNode::AddEdgeToParentNode(GraphEdge* edge) {
  _parentEdges.push_back(edge);
}

void GraphNode::AddEdgeToChildNode(std::unique_ptr<GraphEdge> edge) {
  _childEdges.push_back(std::move(edge));
}

void GraphNode::MoveChatbotHere(ChatBot &&chatbot) {
  _chatBot = std::move(chatbot);
  _chatBot.GetChatLogicHandle()->SetChatbotHandle(&_chatBot);
  _chatBot.SetCurrentNode(this);
}

void GraphNode::MoveChatbotToNewNode(GraphNode *newNode) {
  newNode->MoveChatbotHere(std::move(_chatBot));
}

GraphEdge *GraphNode::GetChildEdgeAtIndex(int index) {
  return _childEdges[index].get();
}
