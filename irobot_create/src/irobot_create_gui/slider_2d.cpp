#include "slider_2d.h"
DEFINE_EVENT_TYPE( ValueChangedEvent )

IMPLEMENT_DYNAMIC_CLASS(Slider2D,wxPanel);

BEGIN_EVENT_TABLE(Slider2D, wxPanel)
EVT_PAINT(Slider2D::OnPaint)
EVT_MOUSE_EVENTS(Slider2D::OnMouseMotion)
EVT_SIZE(Slider2D::OnResize)
END_EVENT_TABLE()

double circle_rad;
double angle;

double 
Slider2D::GetX() { 
	if (m_map_to_square) {
		return m_square_x;
	} else return m_normalized_x*m_normalized_radius;;
}

double 
Slider2D::GetY() {
	if (m_map_to_square) {
		return -m_square_y;
	} else return -m_normalized_y*m_normalized_radius;;
}

double 
Slider2D::GetMagnitude() 
{	
	if (m_map_to_square) {
		sqrt(m_square_x*m_square_x + m_square_y*m_square_y);
	} else return m_normalized_radius;
}

void 
Slider2D::SetPosition( double x, double y ) 
{ 
	angle = atan2(y,x);
	m_normalized_x = cos(angle);
	m_normalized_y = sin(angle);
	m_normalized_radius = sqrt(x*x + y*y)/(circle_rad-SLIDER_2D_RADIUS);
	m_normalized_radius = fmin(1.0,m_normalized_radius);
	RefreshPosition(); 
}

void 
Slider2D::RefreshPosition() 
{
	m_x = m_center_x + (circle_rad-SLIDER_2D_RADIUS)*m_normalized_radius*m_normalized_x; 
	m_y = m_center_y + (circle_rad-SLIDER_2D_RADIUS)*m_normalized_radius*m_normalized_y; 

	printf("%f\n",angle);
	if (m_map_to_square) {
		if ((angle > -M_PI/4.0 && angle < 0) || // top right
			(angle < M_PI/4.0 && angle > 0)) { // bottom right
			m_square_x = m_normalized_radius;
			m_square_y = m_normalized_radius * tan(angle);
		}
		else if (angle > -3.0*M_PI/4.0 && angle < -M_PI/4.0) { // top
			m_square_x = m_normalized_radius * tan(angle-M_PI/2.0);
			m_square_y = -m_normalized_radius;
		}
		else if ((angle < -3.0*M_PI/4.0 && angle > -M_PI) || // top left
				 (angle > 3.0*M_PI/4.0 && angle < M_PI)) { // bottom left
			m_square_x = -m_normalized_radius;
			m_square_y = m_normalized_radius * tan(-angle-M_PI);
		}
		else if (angle < 3.0*M_PI/4.0 && angle > M_PI/4.0) { // top
			m_square_x = m_normalized_radius * tan(-angle-M_PI/2.0);
			m_square_y = m_normalized_radius;
		}
	}
	
	Refresh(); 
	
	// post an event saying that the value has changed
	wxCommandEvent event( ValueChangedEvent );
	wxPostEvent(this, event);
}

void 
Slider2D::OnPaint(wxPaintEvent& event)
{
	wxPaintDC dc (this);
	
	wxCoord width, height;
	GetClientSize( &width, &height );
	
	m_center_x = width/2.0;	
	m_center_y = height/2.0;
	
	printf("center %d, %d\n", m_center_x, m_center_y);
	
	circle_rad = fmin(width,height)/2.0 - SLIDER_2D_BORDER;
	
	dc.SetPen(wxPen(wxColour(80, 80, 80))); 
	dc.SetBrush(wxBrush(wxColour(100, 100, 100)));
	dc.DrawCircle(m_center_x,
				  m_center_y,
				  circle_rad);
	dc.SetBrush(wxBrush(wxColour(110, 110, 110)));
	dc.DrawCircle(m_center_x,
				  m_center_y,
				  circle_rad - SLIDER_2D_RADIUS);
	dc.SetBrush(wxBrush(wxColour(120, 120, 110)));
	dc.DrawCircle(m_center_x,
				  m_center_y,
				  m_normalized_radius*(circle_rad - SLIDER_2D_RADIUS));
	
	// Draw a cross hair
	dc.DrawLine(m_center_x-circle_rad+SLIDER_2D_RADIUS, 
				m_center_y, 
				m_center_x+circle_rad-SLIDER_2D_RADIUS, 
				m_center_y);
	dc.DrawLine(m_center_x, 
				m_center_y-circle_rad+SLIDER_2D_RADIUS, 
				m_center_x, 
				m_center_y+circle_rad-SLIDER_2D_RADIUS);
	
	// Draw triangle thingy
	wxPoint triangle[3];
	triangle[0] = wxPoint(m_center_x,m_center_y);
	double lx = SLIDER_2D_RADIUS*cos(angle + M_PI/2.0);
	double ly = SLIDER_2D_RADIUS*sin(angle + M_PI/2.0);
	lx *= 0.90;
	ly *= 0.90;
	triangle[1] = wxPoint(m_x-lx,m_y-ly);
	triangle[2] = wxPoint(m_x+lx,m_y+ly);
	dc.SetPen(wxPen(wxColour(220, 220, 220))); 
	dc.SetBrush(wxBrush(wxColour(200, 200, 200)));
	dc.DrawPolygon(3,triangle);
	
	
	// Draw the Slider
	dc.SetPen(wxPen(wxColour(80, 80, 80))); 
	dc.SetBrush(wxBrush(wxColour(255, 255, 184)));
	dc.DrawCircle(m_x,m_y,SLIDER_2D_RADIUS);
	
	
	if (m_map_to_square) {
		double x_s = m_center_x + m_square_x*(circle_rad-SLIDER_2D_RADIUS);
		double y_s = m_center_y + m_square_y*(circle_rad-SLIDER_2D_RADIUS);
		double marker_rad = SLIDER_2D_RADIUS/3.0;
		
		// Draw another triangle thingy
		triangle[0] = wxPoint(m_x,m_y);
		double lx = marker_rad*cos(angle + M_PI/2.0);
		double ly = marker_rad*sin(angle + M_PI/2.0);
		lx *= 0.90;
		ly *= 0.90;
		triangle[1] = wxPoint(x_s-lx,y_s-ly);
		triangle[2] = wxPoint(x_s+lx,y_s+ly);
		dc.SetPen(wxPen(wxColour(220, 220, 220))); 
		dc.SetBrush(wxBrush(wxColour(200, 200, 200)));
		dc.DrawPolygon(3,triangle);
		
		// Draw the marker
		dc.SetPen(wxPen(wxColour(80, 80, 80))); 
		dc.SetBrush(wxBrush(wxColour(255, 255, 184)));
		dc.DrawCircle(x_s, 
					  y_s, 
					  marker_rad);
	}
}

void 
Slider2D::OnMouseMotion(wxMouseEvent& event) 
{
	if (event.ButtonDown() || event.Dragging()) {
		wxClientDC dc(this);
		wxPoint p = event.GetLogicalPosition(dc);
		p.x = p.x - m_center_x;
		p.y = p.y - m_center_y;
		SetPosition(p.x, p.y);
		printf("%f %f\n",GetX(), GetY());
	} else {
		if (m_snap_to_center) {
			SetPosition(0.0, 0.0);
			if (m_map_to_square) {
				m_square_x = 0;
				m_square_y = 0;
			}
			printf("%f %f\n",GetX(), GetY());
		}
	}
}

void
Slider2D::OnResize(wxSizeEvent& event) 
{
	wxSize size = event.GetSize();
	m_center_x = size.GetWidth()/2.0;	
	m_center_y = size.GetHeight()/2.0;
	RefreshPosition();
}