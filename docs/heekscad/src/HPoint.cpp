// HPoint.cpp
// Copyright (c) 2009, Dan Heeks
// This program is released under the BSD license. See the file COPYING for details.

#include "stdafx.h"
#include "HPoint.h"
#include "../interface/PropertyVertex.h"
#include "Gripper.h"
#include "DigitizeMode.h"
#include "Drawing.h"

static unsigned char cross16[32] = {0x80, 0x01, 0x40, 0x02, 0x20, 0x04, 0x10, 0x08, 0x08, 0x10, 0x04, 0x20, 0x02, 0x40, 0x01, 0x80, 0x01, 0x80, 0x02, 0x40, 0x04, 0x20, 0x08, 0x10, 0x10, 0x08, 0x20, 0x04, 0x40, 0x02, 0x80, 0x01};
static unsigned char cross16_selected[32] = {0xc0, 0x03, 0xe0, 0x07, 0x70, 0x0e, 0x38, 0x1c, 0x1c, 0x38, 0x0e, 0x70, 0x07, 0xe0, 0x03, 0xc0, 0x03, 0xc0, 0x07, 0xe0, 0x0e, 0x70, 0x1c, 0x38, 0x38, 0x1c, 0x70, 0x0e, 0xe0, 0x07, 0xc0, 0x03};

HPoint::~HPoint(void)
{
}

HPoint::HPoint(const gp_Pnt &p, const HeeksColor* col)
{
	m_p = p;
	color = *col;
	m_draw_unselected=true;
}

HPoint::HPoint(const HPoint &p)
{
	operator=(p);
}

const HPoint& HPoint::operator=(const HPoint &b)
{
	HeeksObj::operator =(b);

	m_p = b.m_p;
	color = b.color;
	m_draw_unselected = b.m_draw_unselected;
	return *this;
}

const wxBitmap &HPoint::GetIcon()
{
	static wxBitmap* icon = NULL;
	if(icon == NULL)icon = new wxBitmap(wxImage(wxGetApp().GetResFolder() + _T("/icons/point.png")));
	return *icon;
}

bool HPoint::IsDifferent(HeeksObj* o)
{
	HPoint* other = (HPoint*)o;
	if(m_p.Distance(other->m_p) > wxGetApp().m_geom_tol)
		return true;

	return HeeksObj::IsDifferent(o);
}

void HPoint::glCommands(bool select, bool marked, bool no_color)
{
	if(!no_color){
		wxGetApp().glColorEnsuringContrast(color);
	}
	GLfloat save_depth_range[2];
	if(marked){
		glGetFloatv(GL_DEPTH_RANGE, save_depth_range);
		glDepthRange(0, 0);
	}
	else if(!m_draw_unselected)
	{
		glBegin(GL_POINTS);
		glVertex3d(m_p.X(), m_p.Y(), m_p.Z());
		glEnd();
		return;
	}


	glRasterPos3d(m_p.X(), m_p.Y(), m_p.Z());
	glBitmap(16, 16, 8, 8, 10.0, 0.0, marked ? cross16_selected : cross16);
	if(marked){
		glDepthRange(save_depth_range[0], save_depth_range[1]);
	}
}

void HPoint::GetBox(CBox &box)
{
	box.Insert(m_p.X(), m_p.Y(), m_p.Z());
}

HeeksObj *HPoint::MakeACopy(void)const
{
	return new HPoint(*this);
}

void HPoint::ModifyByMatrix(const double *m)
{
	gp_Trsf mat = make_matrix(m);
	m_p.Transform(mat);
}

void HPoint::GetGripperPositions(std::list<GripData> *list, bool just_for_endof)
{
	if(just_for_endof)
	{
		list->push_back(GripData((EnumGripperType)0,m_p.X(),m_p.Y(),m_p.Z(),NULL));
	}
}

static void on_set_point(const double *vt, HeeksObj* object){
	((HPoint*)object)->m_p = make_point(vt);
	wxGetApp().Repaint();
}

void HPoint::GetProperties(std::list<Property *> *list)
{
	double p[3];
	extract(m_p, p);
	list->push_back(new PropertyVertex(_("position"), p, this, on_set_point));

	HeeksObj::GetProperties(list);
}

bool HPoint::GetStartPoint(double* pos)
{
	extract(m_p, pos);
	return true;
}

bool HPoint::GetEndPoint(double* pos)
{
	extract(m_p, pos);
	return true;
}

void HPoint::WriteXML(TiXmlNode *root)
{
	TiXmlElement * element;
	element = new TiXmlElement( "Point" );
	root->LinkEndChild( element );  
	element->SetAttribute("col", color.COLORREF_color());
	element->SetDoubleAttribute("x", m_p.X());
	element->SetDoubleAttribute("y", m_p.Y());
	element->SetDoubleAttribute("z", m_p.Z());
	WriteBaseXML(element);
}


//static
HeeksObj* HPoint::ReadFromXMLElement(TiXmlElement* pElem)
{
	gp_Pnt p;
	HeeksColor c;

	// get the attributes
	for(TiXmlAttribute* a = pElem->FirstAttribute(); a; a = a->Next())
	{
		std::string name(a->Name());
		if(name == "col"){c = HeeksColor((long)(a->IntValue()));}
		else if(name == "x"){p.SetX(a->DoubleValue());}
		else if(name == "y"){p.SetY(a->DoubleValue());}
		else if(name == "z"){p.SetZ(a->DoubleValue());}
	}

	HPoint* new_object = new HPoint(p, &c);
	new_object->ReadBaseXML(pElem);

	return new_object;
}

void HPoint::Draw(wxDC& dc)
{
	wxGetApp().PlotSetColor(color);
	double s[3], e[3];
	double line_length = 1.5;
	extract(m_p, s);
	extract(m_p, e); e[0] -= line_length; e[1] -= line_length; wxGetApp().PlotLine(s, e);
	extract(m_p, e); e[0] += line_length; e[1] -= line_length; wxGetApp().PlotLine(s, e);
	extract(m_p, e); e[0] -= line_length; e[1] += line_length; wxGetApp().PlotLine(s, e);
	extract(m_p, e); e[0] += line_length; e[1] += line_length; wxGetApp().PlotLine(s, e);
}

