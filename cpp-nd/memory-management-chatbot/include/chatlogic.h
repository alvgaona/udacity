#ifndef CHATLOGIC_H_
#define CHATLOGIC_H_

#include <string>
#include <vector>

#include "chatgui.h"

// forward declarations
class ChatBot;
class GraphEdge;
class GraphNode;

class ChatLogic {
 private:

  // data handles (owned)
  std::vector<std::unique_ptr<GraphNode>> _nodes;

  // data handles (not owned)
  GraphNode *_currentNode;
  ChatBot *_chatBot;
  ChatBotPanelDialog *_panelDialog;

  // proprietary type definitions
  typedef std::vector<std::pair<std::string, std::string>> tokenlist;

  // non-copyable class
  ChatLogic(ChatLogic const& source);
  ChatLogic& operator=(ChatLogic const& source);

  // proprietary functions
  template <typename T>
  void AddAllTokensToElement(const std::string& tokenID, tokenlist &tokens,
                             T &element);

 public:
  // constructor / destructor
  ChatLogic();
  ~ChatLogic();
  ChatLogic(ChatLogic && source) noexcept;
  ChatLogic& operator=(ChatLogic&& source) noexcept;

  // getter / setter
  void SetPanelDialogHandle(ChatBotPanelDialog *panelDialog);
  void SetChatbotHandle(ChatBot *chatbot);

  // proprietary functions
  void LoadAnswerGraphFromFile(const std::string& filename);
  void SendMessageToChatbot(const std::string& message);
  void SendMessageToUser(const std::string& message);
  wxBitmap *GetImageFromChatbot();
};

#endif /* CHATLOGIC_H_ */
