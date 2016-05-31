// NiceTextCtrl.cpp
// Copyright (c) 2010, Dan Heeks
// This program is released under the BSD license. See the file COPYING for details.

#include <stdafx.h>
#include "NiceTextCtrl.h"
#include "ObjList.h"

CDoubleCtrl::CDoubleCtrl(wxWindow* parent, wxWindowID id, double factor)
:wxTextCtrl(parent, id), m_factor(factor)
{
}

double CDoubleCtrl::GetValue()
{
	double value = 0.0;
	if(!wxTextCtrl::GetValue().ToDouble(&value))return 0.0;
	else return value / m_factor;
}

wxString CDoubleCtrl::DoubleToString(double value)
{
#ifdef UNICODE
	std::wostringstream _value;
#else
	std::ostringstream _value;
#endif
	_value.imbue(std::locale("C"));
	_value<<std::setprecision(10);
	_value << value * m_factor;

	return(_value.str().c_str());
}

void CDoubleCtrl::SetValue(double value)
{
	wxTextCtrl::SetValue(DoubleToString(value));
}

#ifdef HEEKSCAD
	#define VIEW_UNITS (wxGetApp().m_view_units)
#else
	#define VIEW_UNITS (heeksCAD->GetViewUnits())
#endif

CLengthCtrl::CLengthCtrl(wxWindow* parent, wxWindowID id)
:CDoubleCtrl(parent, id, 1/VIEW_UNITS)
{
}

CObjectIdsCtrl::CObjectIdsCtrl(wxWindow* parent, wxWindowID id)
:wxTextCtrl(parent, id)
{
}

void CObjectIdsCtrl::GetIDList(std::list<int> &id_list)
{
	wxString str = wxTextCtrl::GetValue();
	wxString s = _T("");
	unsigned int len = str.Len();
	for(unsigned int i = 0; i <= len; i++)
	{
		if(i == len || str[i] == _T(' '))
		{
			long id = 0;
			if(s.ToLong(&id))
			{
				if(id != 0)id_list.push_back(id);
			}
			s = _T("");
		}
		else
		{
			s.Append(str[i]);
		}
	}
}

void CObjectIdsCtrl::SetFromIDList(std::list<int> &id_list)
{
	wxString str;
	int i = 0;
	for (std::list<int>::iterator It = id_list.begin(); It != id_list.end(); It++)
	{
		int id = *It;
		if(i != 0)str.Append(_T(" "));
		str.Append(wxString::Format(_T("%d"), id));
		i++;
	}

	wxTextCtrl::SetValue(str);
}
