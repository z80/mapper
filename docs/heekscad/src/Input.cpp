// Input.cpp
// Copyright (c) 2011, Dan Heeks
// This program is released under the BSD license. See the file COPYING for details.

#include "stdafx.h"
#include "../interface/DoubleInput.h"
#include "../interface/HDialogs.h"
#include "../interface/NiceTextCtrl.h"
#include "../interface/PictureFrame.h"
#include "HeeksFrame.h"
#include "HLine.h"
#include "HILine.h"
#include "OptionsCanvas.h"

class PictureWindow;

bool HeeksCADapp::InputInt(const wxChar* prompt, const wxChar* value_name, int &value)
{
	double dvalue;
	if(!InputDouble(prompt, value_name, dvalue))return false;
	value = (int)(dvalue + 0.5);
	return true;
}

bool HeeksCADapp::InputDouble(const wxChar* prompt, const wxChar* value_name, double &value)
{
	if(m_input_uses_modal_dialog)
	{
		HDialog dlg(m_frame);
		wxBoxSizer *sizerMain = new wxBoxSizer(wxVERTICAL);
		wxStaticText *static_label = new wxStaticText(&dlg, wxID_ANY, prompt);
		sizerMain->Add( static_label, 0, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, dlg.control_border );
		CDoubleCtrl* value_control = new CDoubleCtrl(&dlg);
		value_control->SetValue(value);
		dlg.AddLabelAndControl(sizerMain, value_name, value_control);
		dlg.MakeOkAndCancel(wxHORIZONTAL).AddToSizer(sizerMain);
		dlg.SetSizer( sizerMain );
		sizerMain->SetSizeHints(&dlg);
		sizerMain->Fit(&dlg);
		value_control->SetFocus();
		if(dlg.ShowModal() == wxID_OK)
		{
			value = value_control->GetValue();
			return true;
		}
		return false;
	}
	else
	{
		CInputMode* save_mode = input_mode_object;
		CDoubleInput double_input(prompt, value_name, value);
		SetInputMode(&double_input);

		OnRun();

		SetInputMode(save_mode);

		if(CDoubleInput::m_success)value = double_input.m_value;

		return CDoubleInput::m_success;
	}
}

enum
{
    ID_RADIO1 = 100,
	ID_RADIO2,
	ID_RADIO3,
	ID_RADIO_OTHER,
	ID_POSX,
	ID_POSY,
	ID_POSZ,
	ID_BUTTON_PICK,
	ID_VECTORX,
	ID_VECTORY,
	ID_VECTORZ,
	ID_BUTTON_VECTOR_PICK,
};

class AngleAndPlaneDlg : public HDialog
{
	bool m_ignore_event_functions;

	PictureWindow *m_picture;
public:
	int m_axis_type;
	wxTextCtrl *txt_num_copies;
	wxRadioButton *rbXy;
	wxRadioButton *rbXz;
	wxRadioButton *rbYz;
	wxRadioButton *rbOther;
	CDoubleCtrl *angle_ctrl;
	CLengthCtrl *axial_shift_ctrl;
	CLengthCtrl *posx;
	CLengthCtrl *posy;
	CLengthCtrl *posz;
	CDoubleCtrl *vectorx;
	CDoubleCtrl *vectory;
	CDoubleCtrl *vectorz;

	AngleAndPlaneDlg(wxWindow *parent, double angle, const double* axis, int axis_type, const double* pos, int *number_of_copies, double* axial_shift = NULL):HDialog(parent)
	{
		m_ignore_event_functions = true;
		m_axis_type = axis_type;
		wxBoxSizer *sizerMain = new wxBoxSizer(wxVERTICAL);

		if(number_of_copies)
		{
			AddLabelAndControl(sizerMain, _("number of copies"), txt_num_copies = new wxTextCtrl(this, wxID_ANY, wxString::Format(_T("%d"), *number_of_copies)));
		}

		wxBoxSizer *sizerPosAndAxis = new wxBoxSizer(wxHORIZONTAL);
		sizerMain->Add(sizerPosAndAxis, 0, wxGROW);

		posx = NULL;
		posy = NULL;
		posz = NULL;

		if(pos)
		{
			wxBoxSizer *sizerPos = new wxBoxSizer(wxVERTICAL);
			sizerPosAndAxis->Add(sizerPos, 0, wxGROW);
			if(m_axis_type == 0)sizerPos->Add( new wxStaticText(this, wxID_ANY, _("Position to rotate about")), 0, wxEXPAND | wxALL, control_border );
			else AddLabelAndControl(sizerPos, _("Position to rotate about"), new wxButton(this, ID_BUTTON_PICK, _("Select")));
			AddLabelAndControl(sizerPos, _("X"), posx = new CLengthCtrl(this, ID_POSX));
			AddLabelAndControl(sizerPos, _("Y"), posy = new CLengthCtrl(this, ID_POSY));
			AddLabelAndControl(sizerPos, _("Z"), posz = new CLengthCtrl(this, ID_POSZ));
			if(m_axis_type == 0)sizerPos->Add( new wxButton(this, ID_BUTTON_PICK, _("Select")), 0, wxEXPAND | wxALL, control_border );

			posx->SetValue( pos[0] );
			posy->SetValue( pos[1] );
			posz->SetValue( pos[2] );
		}

		rbXy = NULL;
		rbXz = NULL;
		rbYz = NULL;
		rbOther = NULL;
		vectorx = NULL;
		vectory = NULL;
		vectorz = NULL;

		if(axis)
		{
			// add picture
			m_picture = new PictureWindow(this, wxSize(150, 100));
			wxBoxSizer *pictureSizer = new wxBoxSizer(wxHORIZONTAL);
			pictureSizer->Add(m_picture, 1, wxGROW);
			sizerMain->Add( pictureSizer, 0, wxALL, control_border );

			wxBoxSizer *sizerPlane = new wxBoxSizer(wxVERTICAL);
			pictureSizer->Add(sizerPlane, 0, wxGROW);

			if(axis_type == 0)
			{
				// not standard axis
				wxBoxSizer *sizerAxis = new wxBoxSizer(wxVERTICAL);
				sizerPosAndAxis->Add(sizerAxis, 0, wxGROW);

				sizerAxis->Add( new wxStaticText(this, wxID_ANY, _("Axis to rotate about")), 0, wxEXPAND | wxALL, control_border );
				AddLabelAndControl(sizerAxis, _("X"), vectorx = new CDoubleCtrl(this, ID_VECTORX));
				AddLabelAndControl(sizerAxis, _("Y"), vectory = new CDoubleCtrl(this, ID_VECTORY));
				AddLabelAndControl(sizerAxis, _("Z"), vectorz = new CDoubleCtrl(this, ID_VECTORZ));
				sizerAxis->Add( new wxButton(this, ID_BUTTON_VECTOR_PICK, _("Select a line")), 0, wxEXPAND | wxALL, control_border );
				vectorx->SetValue( axis[0] );
				vectory->SetValue( axis[1] );
				vectorz->SetValue( axis[2] );
			}

			rbXy = new wxRadioButton( this, ID_RADIO1, _T("XY"), wxDefaultPosition, wxDefaultSize, wxRB_GROUP );
			rbXz = new wxRadioButton( this, ID_RADIO2, _T("XZ") );
			rbYz = new wxRadioButton( this, ID_RADIO3, _T("YZ") );
			rbOther = new wxRadioButton( this, ID_RADIO_OTHER, _T("Other") );
			sizerPlane->Add( rbXy, 0, wxALL, control_border );
			sizerPlane->Add( rbXz, 0, wxALL, control_border );
			sizerPlane->Add( rbYz, 0, wxALL, control_border );
			sizerPlane->Add( rbOther, 0, wxALL, control_border );

			rbXy->SetValue( axis_type == 1 );
			rbXz->SetValue( axis_type == 2 );
			rbYz->SetValue( axis_type == 3 );
			rbOther->SetValue( axis_type == 0 );
		}

		AddLabelAndControl(sizerMain, _("angle"), angle_ctrl = new CDoubleCtrl(this));
		angle_ctrl->SetValue(angle);

		if(axial_shift)
		{
			AddLabelAndControl(sizerMain, _("axial shift"), axial_shift_ctrl = new CLengthCtrl(this));
			axial_shift_ctrl->SetValue(*axial_shift);
		}
		else
		{
			angle_ctrl = NULL;
		}

		// add OK and Cancel to right side
		MakeOkAndCancel(wxHORIZONTAL).AddToSizer(sizerMain);

		SetSizer( sizerMain );
		sizerMain->SetSizeHints(this);
		sizerMain->Fit(this);

		if(number_of_copies)txt_num_copies->SetFocus();
		else angle_ctrl->SetFocus();

		SetPicture();

		m_ignore_event_functions = false;
	}

	void SetPicture(const wxString& name)
	{
		m_picture->SetPicture(wxGetApp().GetResFolder() + _T("/bitmaps/angle/") + name + _T(".png"), wxBITMAP_TYPE_PNG);
	}

	void SetPicture()
	{
		if(rbXy && rbXy->GetValue())SetPicture(_T("xy"));
		else if(rbXz && rbXz->GetValue())SetPicture(_T("xz"));
		else if(rbYz && rbYz->GetValue())SetPicture(_T("yz"));
		else if(rbOther && rbOther->GetValue())SetPicture(_T("line"));
	}

	void OnRadioButton(wxCommandEvent& event)
	{
		if(m_ignore_event_functions)return;

		switch(event.GetId())
		{
		case ID_RADIO1:
		case ID_RADIO2:
		case ID_RADIO3:
			if(m_axis_type == 0)
			{
				EndModal(event.GetId());
				return;
			}
			break;

		case ID_RADIO_OTHER:
			if(m_axis_type != 0)
			{
				EndModal(event.GetId());
				return;
			}
			break;
		}

		SetPicture();
	}

	void OnPick(wxCommandEvent& event)
	{
		this->EndDialog(ID_BUTTON_PICK);
	}

	void OnPickVector(wxCommandEvent& event)
	{
		this->EndDialog(ID_BUTTON_VECTOR_PICK);
	}

	DECLARE_EVENT_TABLE()
};

BEGIN_EVENT_TABLE(AngleAndPlaneDlg, HDialog)
    EVT_RADIOBUTTON(ID_RADIO1,AngleAndPlaneDlg::OnRadioButton)
    EVT_RADIOBUTTON(ID_RADIO2,AngleAndPlaneDlg::OnRadioButton)
    EVT_RADIOBUTTON(ID_RADIO3,AngleAndPlaneDlg::OnRadioButton)
    EVT_RADIOBUTTON(ID_RADIO_OTHER,AngleAndPlaneDlg::OnRadioButton)
    EVT_BUTTON(ID_BUTTON_PICK,AngleAndPlaneDlg::OnPick)
    EVT_BUTTON(ID_BUTTON_VECTOR_PICK,AngleAndPlaneDlg::OnPickVector)
END_EVENT_TABLE()

bool HeeksCADapp::InputAngleWithPlane(double &angle, double *axis, double *pos, int *number_of_copies, double *axial_shift)
{
	double save_axis[3], save_pos[3];
	if(axis)memcpy(save_axis, axis, 3*sizeof(double));
	if(pos)memcpy(save_pos, pos, 3*sizeof(double));

	int axis_type = 0; // not one of the three
	if(axis){
		gp_Dir v_axis(axis[0], axis[1], axis[2]);
		gp_Dir xy_axis(0.0, 0.0, 1.0);
		gp_Dir xz_axis(0.0, -1.0, 0.0);
		gp_Dir yz_axis(1.0, 0.0, 0.0);

		if(v_axis.IsEqual(xy_axis, 0.0000001))axis_type = 1;
		else if(v_axis.IsEqual(xz_axis, 0.0000001))axis_type = 2;
		else if(v_axis.IsEqual(yz_axis, 0.0000001))axis_type = 3;
	}

	while(1)
	{
		AngleAndPlaneDlg dlg(m_frame, angle, axis, axis_type, pos, number_of_copies, axial_shift);
		int ret = dlg.ShowModal();

		if(number_of_copies)
		{
			long long_num_copies;
			if(dlg.txt_num_copies->GetValue().ToLong(&long_num_copies))
				*number_of_copies = long_num_copies;
		}
		angle = dlg.angle_ctrl->GetValue();
		if(axis)
		{
			if(dlg.rbXy->GetValue())
			{
				axis[0] = 0.0;
				axis[1] = 0.0;
				axis[2] = 1.0;
			}
			else if(dlg.rbXz->GetValue())
			{
				axis[0] = 0.0;
				axis[1] = -1.0;
				axis[2] = 0.0;
			}
			else if(dlg.rbYz->GetValue())
			{
				axis[0] = 1.0;
				axis[1] = 0.0;
				axis[2] = 0.0;
			}
			else
			{
				if (dlg.vectorx)axis[0] = dlg.vectorx->GetValue();
				if (dlg.vectory)axis[1] = dlg.vectory->GetValue();
				if (dlg.vectorz)axis[2] = dlg.vectorz->GetValue();
			}
		}

		if(pos)
		{
			pos[0] = dlg.posx->GetValue();
			pos[1] = dlg.posy->GetValue();
			pos[2] = dlg.posz->GetValue();
		}

		if(axial_shift)*axial_shift = dlg.axial_shift_ctrl->GetValue();

		if(ret == wxID_OK)
		{
			return true;
		}
		else if(ret == ID_BUTTON_PICK)
		{
			wxGetApp().PickPosition(_("Pick Position"), pos);
		}
		else if(ret == ID_RADIO1)
		{
			axis[0] = 0.0;
			axis[1] = 0.0;
			axis[2] = 1.0;
			axis_type = 1;
		}
		else if(ret == ID_RADIO2)
		{
			axis[0] = 0.0;
			axis[1] = -1.0;
			axis[2] = 0.0;
			axis_type = 2;
		}
		else if(ret == ID_RADIO3)
		{
			axis[0] = 1.0;
			axis[1] = 0.0;
			axis[2] = 0.0;
			axis_type = 3;
		}
		else if(ret == ID_RADIO_OTHER)
		{
			axis_type = 0;
		}
		else if(ret == ID_BUTTON_VECTOR_PICK)
		{
			// pick a line to use as rotation axis
			bool line_found = false;
			gp_Lin line;
			int save_filter = wxGetApp().m_marked_list->m_filter;
			wxGetApp().PickObjects(_("Pick line for rotation axis"), MARKING_FILTER_LINE | MARKING_FILTER_ILINE, true);
			wxGetApp().m_marked_list->m_filter = save_filter;
			for(std::list<HeeksObj *>::const_iterator It = wxGetApp().m_marked_list->list().begin(); It != wxGetApp().m_marked_list->list().end(); It++)
			{
				HeeksObj* object = *It;
				if(object->GetType() == LineType)
				{
					line = ((HLine*)object)->GetLine();
					line_found = true;
				}
				else if(object->GetType() == ILineType)
				{
					line = ((HILine*)object)->GetLine();
					line_found = true;
				}
			}
			if(line_found)
			{
				extract(line.Direction(), axis);
				extract(line.Location(), pos);
				axis_type = 0;
				gp_Dir xy_axis(0.0, 0.0, 1.0);
				gp_Dir xz_axis(0.0, -1.0, 0.0);
				gp_Dir yz_axis(1.0, 0.0, 0.0);
				if(line.Direction().IsEqual(xy_axis, 0.0000001))axis_type = 1;
				else if(line.Direction().IsEqual(xz_axis, 0.0000001))axis_type = 2;
				else if(line.Direction().IsEqual(yz_axis, 0.0000001))axis_type = 3;
			}
		}
		else
		{
			if(axis)memcpy(axis, save_axis, 3*sizeof(double));
			if(pos)memcpy(pos, save_pos, 3*sizeof(double));
			return false;
		}
	}
	return false;
}
enum
{
	ID_FROMX = 100,
	ID_FROMY,
	ID_FROMZ,
	ID_TOX,
	ID_TOY,
	ID_TOZ,
	ID_BUTTON_PICK_FROM,
	ID_BUTTON_PICK_TO,
};

class FromAndToDlg : public HDialog
{
	bool m_ignore_event_functions;
	int *m_number_of_copies;
	double *m_from;
	double *m_to;
public:
	wxTextCtrl *txt_num_copies;
	CLengthCtrl *fromx;
	CLengthCtrl *fromy;
	CLengthCtrl *fromz;
	CLengthCtrl *tox;
	CLengthCtrl *toy;
	CLengthCtrl *toz;

	FromAndToDlg(wxWindow *parent, double* from, double* to, int *number_of_copies):HDialog(parent)
	{
		m_ignore_event_functions = true;
		wxBoxSizer *sizerMain = new wxBoxSizer(wxVERTICAL);

		m_number_of_copies = number_of_copies;
		m_from = from;
		m_to = to;

		if(number_of_copies)
		{
			AddLabelAndControl(sizerMain, _("number of copies"), txt_num_copies = new wxTextCtrl(this, wxID_ANY, wxString::Format(_T("%d"), *number_of_copies)));
		}

		{
			wxBoxSizer *sizerPos = new wxBoxSizer(wxVERTICAL);
			sizerMain->Add(sizerPos, 0, wxGROW);
			AddLabelAndControl(sizerPos, _("Position to move from"), new wxButton(this, ID_BUTTON_PICK_FROM, _("Select")));
			AddLabelAndControl(sizerPos, _("X"), fromx = new CLengthCtrl(this, ID_FROMX));
			AddLabelAndControl(sizerPos, _("Y"), fromy = new CLengthCtrl(this, ID_FROMY));
			AddLabelAndControl(sizerPos, _("Z"), fromz = new CLengthCtrl(this, ID_FROMZ));

			fromx->SetValue( from[0] );
			fromy->SetValue( from[1] );
			fromz->SetValue( from[2] );
		}

		{
			wxBoxSizer *sizerPos = new wxBoxSizer(wxVERTICAL);
			sizerMain->Add(sizerPos, 0, wxGROW);
			AddLabelAndControl(sizerPos, _("Position to move to"), new wxButton(this, ID_BUTTON_PICK_TO, _("Select")));
			AddLabelAndControl(sizerPos, _("X"), tox = new CLengthCtrl(this, ID_TOX));
			AddLabelAndControl(sizerPos, _("Y"), toy = new CLengthCtrl(this, ID_TOY));
			AddLabelAndControl(sizerPos, _("Z"), toz = new CLengthCtrl(this, ID_TOZ));

			tox->SetValue( to[0] );
			toy->SetValue( to[1] );
			toz->SetValue( to[2] );
		}

		// add OK and Cancel to right side
		MakeOkAndCancel(wxHORIZONTAL).AddToSizer(sizerMain);

		SetSizer( sizerMain );
		sizerMain->SetSizeHints(this);
		sizerMain->Fit(this);

		if(number_of_copies)txt_num_copies->SetFocus();
		else fromx->SetFocus();

		m_ignore_event_functions = false;
	}

	void OnPickFrom(wxCommandEvent& event)
	{
		this->EndDialog(ID_BUTTON_PICK_FROM);
	}

	void OnPickTo(wxCommandEvent& event)
	{
		this->EndDialog(ID_BUTTON_PICK_TO);
	}

	void GetAllValues()
	{
		if(m_number_of_copies)
		{
			long long_num_copies;
			if(txt_num_copies->GetValue().ToLong(&long_num_copies))
				*m_number_of_copies = long_num_copies;
		}

		m_from[0] = fromx->GetValue();
		m_from[1] = fromy->GetValue();
		m_from[2] = fromz->GetValue();

		m_to[0] = tox->GetValue();
		m_to[1] = toy->GetValue();
		m_to[2] = toz->GetValue();
	}

	DECLARE_EVENT_TABLE()
};

BEGIN_EVENT_TABLE(FromAndToDlg, HDialog)
    EVT_BUTTON(ID_BUTTON_PICK_FROM,FromAndToDlg::OnPickFrom)
    EVT_BUTTON(ID_BUTTON_PICK_TO,FromAndToDlg::OnPickTo)
END_EVENT_TABLE()

bool HeeksCADapp::InputFromAndTo(double *from, double *to, int *number_of_copies)
{
	double save_from[3], save_to[3];
	int save_num_copies = 0;
	if(number_of_copies)save_num_copies = *number_of_copies;
	memcpy(save_from, from, 3*sizeof(double));
	memcpy(save_to, to, 3*sizeof(double));

	while(1)
	{
		FromAndToDlg dlg(m_frame, from, to, number_of_copies);
		int ret = dlg.ShowModal();
		
		dlg.GetAllValues();

		if(ret == wxID_OK)
		{
			return true;
		}
		else if(ret == ID_BUTTON_PICK_FROM)
		{
			wxGetApp().PickPosition(_("Pick position to move from"), from);
		}
		else if(ret == ID_BUTTON_PICK_TO)
		{
			wxGetApp().PickPosition(_("Pick position to move to"), to);
		}
		else
		{
			memcpy(from, save_from, 3*sizeof(double));
			memcpy(to, save_to, 3*sizeof(double));
			if(number_of_copies)*number_of_copies = save_num_copies;
			return false;
		}

	}
	return false;
}	

bool HeeksCADapp::InputLength(const wxChar* prompt, const wxChar* value_name, double &value)
{
	if(m_input_uses_modal_dialog)
	{
		HDialog dlg(m_frame);
		wxBoxSizer *sizerMain = new wxBoxSizer(wxVERTICAL);
		wxStaticText *static_label = new wxStaticText(&dlg, wxID_ANY, prompt);
		sizerMain->Add( static_label, 0, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, dlg.control_border );
		CLengthCtrl* value_control = new CLengthCtrl(&dlg);
		dlg.AddLabelAndControl(sizerMain, value_name, value_control);
		dlg.MakeOkAndCancel(wxHORIZONTAL).AddToSizer(sizerMain);
		dlg.SetSizer( sizerMain );
		sizerMain->SetSizeHints(&dlg);
		sizerMain->Fit(&dlg);
		value_control->SetFocus();
		if(dlg.ShowModal() == wxID_OK)
		{
			value = value_control->GetValue();
			return true;
		}
		return false;
	}
	else
	{
		CInputMode* save_mode = input_mode_object;
		CLengthInput length_input(prompt, value_name, value);
		SetInputMode(&length_input);

		OnRun();

		SetInputMode(save_mode);

		if(CLengthInput::m_success)value = length_input.m_value;

		return CLengthInput::m_success;
	}
}

class COptionsDlg : public HDialog
{
public:
	COptionsCanvas *m_options_canvas;
	bool m_ignore_event_functions;

	COptionsDlg(wxWindow *parent):HDialog(parent)
	{
		m_ignore_event_functions = true;
		wxBoxSizer *sizerMain = new wxBoxSizer(wxVERTICAL);

		m_options_canvas = new COptionsCanvas(this);
		m_options_canvas->RefreshByRemovingAndAddingAll2();
		m_options_canvas->SetSize(0, 0, 400, 300);
		sizerMain->Add( m_options_canvas, 0, wxALL, control_border );

		// add OK and Cancel to bottom side
		MakeOkAndCancel(wxHORIZONTAL).AddToSizer(sizerMain);

		SetSizer( sizerMain );
		sizerMain->SetSizeHints(this);
		sizerMain->Fit(this);

		m_options_canvas->SetFocus();

		m_ignore_event_functions = false;
	}

	DECLARE_EVENT_TABLE()
};

BEGIN_EVENT_TABLE(COptionsDlg, HDialog)
END_EVENT_TABLE()

void HeeksCADapp::ShowModalOptions()
{
	COptionsDlg dlg(m_frame);
	dlg.ShowModal();
}
