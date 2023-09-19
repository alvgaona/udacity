#ifndef CHATBOT_H_
#define CHATBOT_H_

#include <wx/bitmap.h>

#include <string>

class GraphNode;  // forward declaration
class ChatLogic;  // forward declaration

class ChatBot {
 private:
  // data handles (owned)
  wxBitmap *_image;  // avatar image

  // data handles (not owned)
  GraphNode *_currentNode;
  GraphNode *_rootNode;
  ChatLogic *_chatLogic;

  // proprietary functions
  int ComputeLevenshteinDistance(std::string s1, std::string s2);

 public:
  // constructors / destructors
  ChatBot();  // constructor WITHOUT memory allocation
  ChatBot(const std::string &filename);  // constructor WITH memory allocation
  ~ChatBot();

  ChatBot(ChatBot const& source);                  // copy constructor
  ChatBot(ChatBot&& source) noexcept ;             // move constructor
  ChatBot &operator=(ChatBot const& source);       // copy assignment constructor
  ChatBot &operator=(ChatBot &&source) noexcept ;  // move assignment constructor

  ChatLogic* GetChatLogicHandle() { return _chatLogic; }

  // getters / setters
  void SetCurrentNode(GraphNode *node);
  void SetRootNode(GraphNode *rootNode) { _rootNode = rootNode; }
  void SetChatLogicHandle(ChatLogic *chatLogic) { _chatLogic = chatLogic; }
  wxBitmap *GetImageHandle() { return _image; }

  // communication
  void ReceiveMessageFromUser(const std::string &message);
};

#endif /* CHATBOT_H_ */
