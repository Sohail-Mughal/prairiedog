#ifndef SLIDER_2D_H
#define SLIDER_2D_H
#include <wx/wx.h>
#include <math.h>

class Slider2D : public wxPanel
	{
		DECLARE_DYNAMIC_CLASS(Slider2D)
		
	public: 
		
		Slider2D () {}
		Slider2D (wxWindow *parent, 
				  wxWindowID id = wxID_ANY,
				  const wxPoint& pos = wxDefaultPosition,
				  const wxSize& size = wxDefaultSize, 
				  long style = wxBORDER_DEFAULT) 
		: wxPanel(parent, id, pos, size, style) 
		{
			m_snap_to_center = false;
			m_map_to_square = false;
			m_center_x = size.GetWidth()/2.0;	
			m_center_y = size.GetHeight()/2.0; 
		}
		
		void OnPaint(wxPaintEvent& event);
		void OnMouseMotion(wxMouseEvent& event);
		void OnResize(wxSizeEvent& event);
		
		double GetX(); // always between -1 and 1
		double GetY(); // always between -1 and 1
		double GetMagnitude();  // circular mapping: value will be between 0 and 1
								// square mapping: value will be between 0 and sqrt(2)
		void SetPosition( double x, double y );
		bool IsSnapToCenter() { return m_snap_to_center; }
		void SetSnapToCenter(bool snap_to_center) { m_snap_to_center = snap_to_center; RefreshPosition(); }
		bool IsMapToSquare() { return m_map_to_square; } // square mapping makes it possible for X and Y 
														 // to have absolute values of 1 at the same time
		void SetMapToSquare(bool map_to_square) { m_map_to_square = map_to_square; RefreshPosition(); }
		
	private:
		
		bool m_snap_to_center;
		bool m_map_to_square; // map the unit to a unit square. This makes it possible for X
		
		double m_x;
		double m_y;
		double m_normalized_x;
		double m_normalized_y;
		double m_normalized_radius;
		double m_square_x;
		double m_square_y;
		double m_center_x;
		double m_center_y;
		
		void RefreshPosition();
		
		DECLARE_EVENT_TABLE()
	
	};

DECLARE_EVENT_TYPE( ValueChangedEvent, wxID_ANY )

#define SLIDER_2D_RADIUS 10.0
#define SLIDER_2D_BORDER 15.0

#endif