#ifndef CHATGUI_H_
#define CHATGUI_H_

#include <wx/wx.h>
#include <iostream>
#include <memory>

class ChatLogic;  // forward declaration

// middle part of the window containing the dialog between user and chatbot
class ChatBotPanelDialog : public wxScrolledWindow {
 private:
  // control elements
  wxBoxSizer *_dialogSizer;
  wxBitmap _image;

  std::unique_ptr<ChatLogic> _chatLogic;

  // non-copyable class
  ChatBotPanelDialog(ChatBotPanelDialog const& source);
  ChatBotPanelDialog& operator=(ChatBotPanelDialog const& source);

 public:
  // constructor / destructor
  ChatBotPanelDialog(wxWindow *parent, wxWindowID id);
  ~ChatBotPanelDialog();

  ChatBotPanelDialog(ChatBotPanelDialog&& source) noexcept;
  ChatBotPanelDialog& operator=(ChatBotPanelDialog&& source) noexcept;

  // getter / setter
  ChatLogic *GetChatLogicHandle() { return _chatLogic.get(); }

  // events
  void paintEvent(wxPaintEvent &evt);
  void paintNow();
  void render(wxDC &dc);

  // proprietary functions
  void AddDialogItem(const wxString& text, bool isFromUser = true);
  void PrintChatbotResponse(const std::string& response);

  DECLARE_EVENT_TABLE()
};

// dialog item shown in ChatBotPanelDialog
class ChatBotPanelDialogItem : public wxPanel {
 private:
  // control elements
  wxStaticBitmap *_chatBotImg;
  wxStaticText *_chatBotTxt;

 public:
  // constructor / destructor
  ChatBotPanelDialogItem(wxPanel *parent, const wxString &text,
                         bool isFromUser);

  ChatBotPanelDialogItem(ChatBotPanelDialogItem const& source);
  ChatBotPanelDialogItem& operator=(ChatBotPanelDialogItem const& source);
  ChatBotPanelDialogItem(ChatBotPanelDialogItem&& source) noexcept;
  ChatBotPanelDialogItem& operator=(ChatBotPanelDialogItem&& source) noexcept;
};

// frame containing all control elements
class ChatBotFrame : public wxFrame {
 private:
  // control elements
  ChatBotPanelDialog *_panelDialog;
  wxTextCtrl *_userTextCtrl;

  // events
  void OnEnter(wxCommandEvent &WXUNUSED(event));

 public:
  // constructor / desctructor
  ChatBotFrame(const wxString &title);

  ChatBotFrame(ChatBotFrame const& source);
  ChatBotFrame& operator=(ChatBotFrame const& source);
  ChatBotFrame(ChatBotFrame&& source) noexcept;
  ChatBotFrame& operator=(ChatBotFrame&& source) noexcept;
};

// control panel for background image display
class ChatBotFrameImagePanel : public wxPanel {
  // control elements
  wxBitmap _image;

 public:
  // constructor / desctructor
  ChatBotFrameImagePanel(wxFrame *parent);

  // events
  void paintEvent(wxPaintEvent &evt);
  void paintNow();
  void render(wxDC &dc);

  DECLARE_EVENT_TABLE()

  ChatBotFrameImagePanel(ChatBotFrameImagePanel const& source);
  ChatBotFrameImagePanel& operator=(ChatBotFrameImagePanel const& source);
  ChatBotFrameImagePanel(ChatBotFrameImagePanel&& source) noexcept;
  ChatBotFrameImagePanel& operator=(ChatBotFrameImagePanel&& source) noexcept;
};

// wxWidgets app that hides main()
class ChatBotApp : public wxApp {
 public:
  // events
  virtual bool OnInit();
};

#endif /* CHATGUI_H_ */
