// wxWidgets "Hello World" Program
// For compilers that support precompilation, includes "wx/wx.h".
#include <wx/wxprec.h>
#include <wx/timer.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif

class MyApp : public wxApp
{
public:
    virtual bool OnInit();
    int OnRun();
};

class MyFrame : public wxFrame
{
public:
    MyFrame();
    void OnTimer(wxTimerEvent& event);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
private:
    void OnHello(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);
    wxTimer m_timer;
    wxDECLARE_EVENT_TABLE();
};

wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
    EVT_TIMER(TIMER_ID, MyFrame::OnTimer)
wxEND_EVENT_TABLE()

enum
{
    ID_Hello = 1
};

wxIMPLEMENT_APP(MyApp);

bool MyApp::OnInit()
{   
    // char* argvForRos = new char[argc];
    // for (int i = 0; i < argc; ++i)
    //  { 
    //     argvForRos[i] = strdup(wxString(argv[i]).mb_str());
    //  }
    // ros::init(argc, _argvForRos, "myGui");
    ros::init(argc, (char **) argv, "my_other_GUI");
    MyFrame *frame = new MyFrame();
    frame->Show(true);
    return true;
}


int MyApp::OnRun()
{
    ros::NodeHandle nh;
    // cv::namedWindow("view");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw/compressed", 1, imageCallback);
    return wxApp::OnRun();
    // ros::spin();
    // cv::destroyWindow("view");
}

MyFrame::MyFrame()    : wxFrame(NULL, wxID_ANY, "Hello World"): m_timer(this, TIMER_ID)
// MyFrame::MyFrame()    : wxFrame(NULL, wxID_ANY, "Hello World")
{
    m_timer.Start(1000);
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(ID_Hello, "&Hello...\tCtrl-H",
                     "Help string shown in status bar for this menu item");
    menuFile->AppendSeparator();
    menuFile->Append(wxID_EXIT);
    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);
    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuHelp, "&Help");
    SetMenuBar( menuBar );
    CreateStatusBar();
    SetStatusText("Welcome to wxWidgets!");
    Bind(wxEVT_MENU, &MyFrame::OnHello, this, ID_Hello);
    Bind(wxEVT_MENU, &MyFrame::OnAbout, this, wxID_ABOUT);
    Bind(wxEVT_MENU, &MyFrame::OnExit, this, wxID_EXIT);
}
void MyFrame::OnTimer(wxTimerEvent& event)
{
    ros::spinonce();
}
void MyFrame::OnExit(wxCommandEvent& event)
{
    Close(true);
}
void MyFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox("This is a wxWidgets Hello World example",
                 "About Hello World", wxOK | wxICON_INFORMATION);
}
void MyFrame::OnHello(wxCommandEvent& event)
{
    wxLogMessage("Hello world from wxWidgets!");
}

void MyFrame::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
