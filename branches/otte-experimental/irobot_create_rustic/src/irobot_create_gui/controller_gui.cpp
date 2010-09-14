/*
 *  client.cpp
 *  
 *
 *  Created by Gheric Speiginer and Keenan Black 09 ???
 *
 *  Modified by Michael Otte University of Colorado at Boulder 2010 for irobot_create_rustic 
 *
 */

#include <wx/wx.h>
#include <ros/ros.h>
#include <irobot_create_rustic/irobot_create_controller.h>
#include "slider_2d.h"

#ifdef __APPLE__
#include <ApplicationServices/ApplicationServices.h>
#endif


class MyFrame : public wxFrame
	{
	public:
		MyFrame(const wxString& title);
		Slider2D *slider_2d;
		IRobotCreateController *controller;
		
		void StartROS(int argc, char** argv);
		
	private:
		//void OnMouseMotion(wxMouseEvent &event);
		void OnValueChangedEvent(wxCommandEvent &event);
		
		bool m_connected;
		
		DECLARE_EVENT_TABLE()
	};

class MyApp : public wxApp
	{
	public:
		virtual bool OnInit();
	};

BEGIN_EVENT_TABLE(MyFrame, wxFrame)
EVT_COMMAND(wxID_ANY, ValueChangedEvent, MyFrame::OnValueChangedEvent)
END_EVENT_TABLE()

MyFrame::MyFrame(const wxString& title)
: wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(250, 150))
{
	m_connected = false;
	
	
	//wxPanel *panel = new wxPanel(this,wxID_ANY);
	
	slider_2d = new Slider2D(this,wxID_ANY, wxPoint(-1, -1),
							 wxSize(250, 150));
	slider_2d->SetBackgroundColour(wxColour(0,0,0));
	slider_2d->SetSnapToCenter(true);
	slider_2d->SetMapToSquare(true);
	slider_2d->SetFocus();
	
	
	//wxPanel *panel2 = new wxPanel(panel,wxID_ANY, wxPoint(45,30),
	//							  wxSize(250, 150));
	
	//panel2->SetBackgroundColour(wxColour(255,255,0));
		
    //controller = new IRobotCreateController();
}

void 
MyFrame::StartROS(int argc, char** argv) {
	ros::init(argc, argv, "irobot_create_gui");
	controller = new IRobotCreateController();
	m_connected = true;
}

void
MyFrame::OnValueChangedEvent(wxCommandEvent &event) {
	if (!m_connected) return;
	
	double x = slider_2d->GetX();
	double y = slider_2d->GetY();	
	
	if (y > 0.0) 
	{
		controller->setSpeeds(y,-x);
	} else {
		controller->setSpeeds(y,-x);
	}
}

IMPLEMENT_APP(MyApp)

bool MyApp::OnInit()
{
#if defined(__WXMAC__)
	// Allows the program to be focussed in OS X without placing it in a bundle
	ProcessSerialNumber PSN;
	GetCurrentProcess(&PSN);
	TransformProcessType(&PSN,kProcessTransformToForegroundApplication);
#endif	
	
	//ros::init(argc, (char**)argv, "irobot_create_gui");
	sleep(2);
	
    MyFrame *frame = new MyFrame(wxT("iRobot Create"));
	frame->Centre();
    frame->Show(true);
	
	frame->StartROS(argc, (char**)argv);
	
    return true;
}
