// HeeksFrame.cpp
// Copyright (c) 2009, Dan Heeks
// This program is released under the BSD license. See the file COPYING for details.

#include "stdafx.h"
#include "HeeksFrame.h"
#include "../interface/Tool.h"
#include "../interface/ToolList.h"
#include "GraphicsCanvas.h"
#include "TreeCanvas.h"
#include "ObjPropsCanvas.h"
#include "OptionsCanvas.h"
#include "InputModeCanvas.h"
#include "LineArcDrawing.h"
#include "PointDrawing.h"
#include "RegularShapesDrawing.h"
#include "DimensionDrawing.h"
#include "Shape.h"
#include "MarkedList.h"
#include "MagDragWindow.h"
#include "ViewRotating.h"
#include "ViewZooming.h"
#include "ViewPanning.h"
#include "HArc.h"
#include "RuledSurface.h"
#include "Sphere.h"
#include "Cuboid.h"
#include "Cylinder.h"
#include "Cone.h"
#include "HText.h"
#include "TransformTools.h"
#include "SelectMode.h"
#include "CoordinateSystem.h"
#include "HeeksPrintout.h"
#include "../interface/HeeksCADInterface.h"
#include "Plugins.h"
#include "HeeksConfig.h"
#include "AboutBox.h"
#include "HSpline.h"
#include "../interface/Plugin.h"
#include "Sectioning.h"
#include "MenuSeparator.h"
#include "HGear.h"
#include "HPoint.h"
#ifdef USING_RIBBON
#include "HeeksRibbon.h"
#endif

using namespace std;

BEGIN_EVENT_TABLE( CHeeksFrame, wxFrame )
EVT_CLOSE(CHeeksFrame::OnClose)
EVT_MENU( Menu_View_ResetLayout, CHeeksFrame::OnResetLayout )
EVT_MENU_RANGE(	ID_RECENT_FIRST, ID_RECENT_FIRST + MAX_RECENT_FILES, CHeeksFrame::OnRecentFile)
#ifndef USING_RIBBON
EVT_MENU( Menu_View_SetToolBarsToLeft, CHeeksFrame::OnSetToolBarsToLeft )
EVT_MENU_RANGE(ID_FIRST_EXTERNAL_BUTTON, ID_FIRST_POP_UP_MENU_TOOL + 1000, CHeeksFrame::OnExternalButton)
//wx__DECLARE_EVT2(wxEVT_COMMAND_BUTTON_CLICKED, ID_FIRST_EXTERNAL_BUTTON, ID_FIRST_POP_UP_MENU_TOOL + 1000, wxCommandEventHandler(CHeeksFrame::OnExternalButton))
EVT_UPDATE_UI_RANGE(ID_FIRST_EXTERNAL_BUTTON, ID_FIRST_POP_UP_MENU_TOOL + 1000, CHeeksFrame::OnUpdateExternalButton)
#endif
EVT_SIZE(CHeeksFrame::OnSize)
EVT_MOVE(CHeeksFrame::OnMove)
EVT_KEY_DOWN(CHeeksFrame::OnKeyDown)
EVT_KEY_UP(CHeeksFrame::OnKeyUp)
END_EVENT_TABLE()

class DnDFile : public wxFileDropTarget
{
	public:
		DnDFile(wxFrame *pOwner) { m_pOwner = pOwner; }

		virtual bool OnDropFiles(wxCoord x, wxCoord y,
				const wxArrayString& filenames);

	private:
		wxFrame *m_pOwner;
};

bool DnDFile::OnDropFiles(wxCoord, wxCoord, const wxArrayString& filenames)
{
	size_t nFiles = filenames.GetCount();
	for ( size_t n = 0; n < nFiles; n++ )
	{
		wxGetApp().OpenFile(filenames[n]);
	}

	return true;
}

CHeeksCADInterface heekscad_interface;

CHeeksFrame::CHeeksFrame( const wxString& title, const wxPoint& pos, const wxSize& size )
	: wxFrame((wxWindow *)NULL, -1, title, pos, size)
{
	wxGetApp().m_frame = this;

	HeeksConfig config;

	m_next_id_for_button = ID_FIRST_EXTERNAL_BUTTON;
	m_printout = NULL;

#ifndef USING_RIBBON
	MakeMenus();
#endif

	m_aui_manager = new wxAuiManager(this);

	int bitmap_size = ToolImage::default_bitmap_size;
	config.Read(_T("ToolImageSize"), &bitmap_size);
	ToolImage::SetBitmapSize(bitmap_size);

	m_graphics = new CGraphicsCanvas(this);

	bool perspective = false;
	config.Read(_T("Perspective"), &perspective);
	m_graphics->m_view_point.SetPerspective(perspective);

	m_tree_canvas = new CTreeCanvas(this);

	m_options = new COptionsCanvas(this);
	m_input_canvas = new CInputModeCanvas(this);

	m_properties = new CObjPropsCanvas(this);

	wxString exe_folder = wxGetApp().GetExeFolder();

	m_main_toolbar_removed = false;
	m_geometry_toolbar_removed = false;
	m_solid_toolbar_removed = false;
	m_viewing_toolbar_removed = false;

#ifdef USING_RIBBON
	m_ribbon = new HeeksRibbon(this);
	m_aui_manager->AddPane(m_ribbon, wxAuiPaneInfo().ToolbarPane().Gripper(false).Name(_T("Ribbon")).Movable(false).MinSize(wxSize(-1, 85)).Top());
#else
	AddToolBars();
#endif

	// Center
	m_aui_manager->AddPane(m_graphics, wxAuiPaneInfo().Name(_T("Graphics")).Caption(_("Graphics")).CentrePane().BestSize(wxSize(800, 600)));
	// Left-side (objects, properties, etc.)
	int m_leftpanel_last_position = 0;
	m_aui_manager->AddPane(m_tree_canvas, wxAuiPaneInfo().Name(_T("Objects")).Caption(_("Objects")).Left().Layer(1).BestSize(wxSize(300, 400)).Position(m_leftpanel_last_position++));
	m_aui_manager->AddPane(m_options, wxAuiPaneInfo().Name(_T("Options")).Caption(_("Options")).Left().Layer(1).BestSize(wxSize(300, 200)).Position(m_leftpanel_last_position++));
	m_aui_manager->AddPane(m_input_canvas, wxAuiPaneInfo().Name(_T("Input")).Caption(_("Input")).Left().Layer(1).BestSize(wxSize(300, 200)).Position(m_leftpanel_last_position++));
	m_aui_manager->AddPane(m_properties, wxAuiPaneInfo().Name(_T("Properties")).Caption(_("Properties")).Left().Layer(1).BestSize(wxSize(300, 200)).Position(m_leftpanel_last_position++));

	// add to hiding list for full screen mode
	wxGetApp().RegisterHideableWindow(m_tree_canvas);
	wxGetApp().RegisterHideableWindow(m_options);
	wxGetApp().RegisterHideableWindow(m_input_canvas);
	wxGetApp().RegisterHideableWindow(m_properties);

	// set xml reading functions
	wxGetApp().InitializeXMLFunctions();

	// load up any other dlls and call OnStartUp on each of them
	{
		std::list<PluginData> plugins;
		ReadPluginsList(plugins);
		wxString save_current_directory = ::wxGetCwd();

		for(std::list<PluginData>::iterator It = plugins.begin(); It != plugins.end(); It++)
		{
			PluginData &pd = *It;
			if(pd.enabled)
			{
				wxFileName fn(pd.path);
				fn.Normalize();
				wxString path = fn.GetPath();

				::wxSetWorkingDirectory(path);

				wxDynamicLibrary* shared_library = new wxDynamicLibrary(fn.GetFullPath(),wxDL_NOW|wxDL_GLOBAL );
				if(shared_library->IsLoaded()){
					bool success;
					void(*OnStartUp)(CHeeksCADInterface*, const wxString&) = (void (*)(CHeeksCADInterface*, const wxString&))(shared_library->GetSymbol(_T("OnStartUp"), &success));
					if(OnStartUp)
					{
						(*OnStartUp)(&heekscad_interface, path);
						wxGetApp().m_loaded_libraries.push_back(Plugin(fn.GetName(), path, shared_library));
					}
				}
				else{
					delete shared_library;
				}

				::wxSetWorkingDirectory(save_current_directory);
			}
		}

	}

	SetDropTarget(new DnDFile(this));

#ifndef USING_RIBBON
	m_menuWindow->AppendSeparator();
	m_menuWindow->Append( Menu_View_ResetLayout, _( "Reset Layout" ) );
	m_menuWindow->Append( Menu_View_SetToolBarsToLeft, _( "Set toolbars to left" ) );
#endif

	//Read layout
	wxString str;
	wxString default_perspective = wxT("default");
	config.Read(_T("AuiPerspective"), &str, default_perspective);
#ifndef USING_RIBBON   // just for now don't load perspective, while sorting out the ribbon
	if (!str.IsSameAs(default_perspective))
	{
		LoadPerspective(str);
	}
#endif

	m_aui_manager->Update();
}

CHeeksFrame::~CHeeksFrame()
{
	wxGetApp().Clear(); // delete all the objects, from the dlls

	// call the shared libraries function OnFrameDelete, so they can write profile strings while aui manager still exists
	for(std::list<Plugin>::iterator It = wxGetApp().m_loaded_libraries.begin(); It != wxGetApp().m_loaded_libraries.end(); It++){
		wxDynamicLibrary* shared_library = It->dynamic_library;
		bool success;
		void(*OnFrameDelete)() = (void(*)())(shared_library->GetSymbol(_T("OnFrameDelete"), &success));
		if(OnFrameDelete)(*OnFrameDelete)();
	}

	//Save the application layout
	wxString str = m_aui_manager->SavePerspective();
#if _DEBUG
	// save the layout string in a file
	{
#if wxUSE_UNICODE
		wofstream ofs("layout.txt");
#else
		ofstream ofs("layout.txt");
#endif
		ofs<<str.c_str();
	}
#endif

	HeeksConfig config;
	config.Write(_T("AuiPerspective"), str);
	config.Write(_T("ToolImageSize"), ToolImage::GetBitmapSize());
	config.Write(_T("Perspective"), m_graphics->m_view_point.GetPerspective());

	delete m_aui_manager;
	delete wxLog::SetActiveTarget(NULL);
	SetMenuBar(NULL);
}

void CHeeksFrame::RefreshInputCanvas()
{
	if(wxGetApp().m_frame && wxGetApp().m_frame->m_input_canvas)
		wxGetApp().m_frame->m_input_canvas->RefreshByRemovingAndAddingAll();
}

void CHeeksFrame::RefreshProperties()
{
	if(wxGetApp().m_frame && wxGetApp().m_frame->m_properties)
		wxGetApp().m_frame->m_properties->RefreshByRemovingAndAddingAll();
}

void CHeeksFrame::RefreshOptions()
{
	if(wxGetApp().m_frame && wxGetApp().m_frame->m_options)
		wxGetApp().m_frame->m_options->RefreshByRemovingAndAddingAll();
}

void CHeeksFrame::OnKeyDown(wxKeyEvent& event)
{
	wxGetApp().input_mode_object->OnKeyDown(event);
	event.Skip();
}

void CHeeksFrame::OnKeyUp(wxKeyEvent& event)
{
	wxGetApp().input_mode_object->OnKeyUp(event);
	event.Skip();
}

bool CHeeksFrame::ShowFullScreen(bool show, long style){
	static std::map< wxWindow*, bool > windows_visible;

	if(show){
		SetMenuBar(NULL);
		windows_visible.clear();
		for(std::list<wxWindow*>::iterator It = wxGetApp().m_hideable_windows.begin(); It != wxGetApp().m_hideable_windows.end(); It++)
		{
			wxWindow* w = *It;
			windows_visible.insert(std::pair< wxWindow*, bool > (w, m_aui_manager->GetPane(w).IsShown() && w->IsShown()));
			m_aui_manager->GetPane(w).Show(false);
		}
	}
	else{
		SetMenuBar(m_menuBar);
		for(std::list<wxWindow*>::iterator It = wxGetApp().m_hideable_windows.begin(); It != wxGetApp().m_hideable_windows.end(); It++)
		{
			wxWindow* w = *It;
			std::map< wxWindow*, bool >::iterator FindIt = windows_visible.find(w);
			if(FindIt != windows_visible.end()){
				bool visible = FindIt->second;
				m_aui_manager->GetPane(w).Show(visible);
			}
		}
	}

	bool res =  wxTopLevelWindow::ShowFullScreen(show, style);
	m_aui_manager->Update();
	return res;
}

void CHeeksFrame::OnClose( wxCloseEvent& event )
{
	if ( event.CanVeto() && wxGetApp().CheckForModifiedDoc() == wxCANCEL )
	{
		event.Veto();
		return;
	}

	wxGetApp().OnBeforeFrameDelete();

	event.Skip();
}

static void OnQuit( wxCommandEvent& WXUNUSED( event ) )
{	/// wxCANCEL means go on
	if(wxGetApp().CheckForModifiedDoc() == wxCANCEL)
		return;
	wxGetApp().m_frame->Close(TRUE);
}

void OnAbout( wxCommandEvent& WXUNUSED( event ) )
{
	CAboutBox dlg(wxGetApp().m_frame);
	dlg.ShowModal();
}

void OnPlugins( wxCommandEvent& WXUNUSED( event ) )
{
	CPluginsDialog dlg(wxGetApp().m_frame);
	dlg.ShowModal();
}

void OnViewObjects( wxCommandEvent& event )
{
	wxAuiPaneInfo& pane_info = wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_tree_canvas);
	if(pane_info.IsOk()){
		pane_info.Show(event.IsChecked());
		wxGetApp().m_frame->m_aui_manager->Update();
	}
}

void OnUpdateViewObjects( wxUpdateUIEvent& event )
{
	event.Check(wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_tree_canvas).IsShown());
}


void OnViewOptions( wxCommandEvent& event )
{
	wxAuiPaneInfo& pane_info = wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_options);
	if(pane_info.IsOk()){
		pane_info.Show(event.IsChecked());
		wxGetApp().m_frame->m_aui_manager->Update();
	}
}

void OnUpdateViewOptions( wxUpdateUIEvent& event )
{
	event.Check(wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_options).IsShown());
}

void OnViewInput( wxCommandEvent& event )
{
	wxAuiPaneInfo& pane_info = wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_input_canvas);
	if(pane_info.IsOk()){
		pane_info.Show(event.IsChecked());
		wxGetApp().m_frame->m_aui_manager->Update();
	}
}

void OnUpdateViewInput( wxUpdateUIEvent& event )
{
	event.Check(wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_input_canvas).IsShown());
}

void OnViewToolBar( wxCommandEvent& event )
{
	wxAuiPaneInfo& pane_info = wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_toolBar);
	if(pane_info.IsOk()){
		pane_info.Show(event.IsChecked());
		wxGetApp().m_frame->m_aui_manager->Update();
	}
}

void OnUpdateViewToolBar( wxUpdateUIEvent& event )
{
	event.Check(wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_toolBar).IsShown());
}

void OnViewGeometryBar( wxCommandEvent& event )
{
	wxAuiPaneInfo& pane_info = wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_geometryBar);
	if(pane_info.IsOk()){
		pane_info.Show(event.IsChecked());
		wxGetApp().m_frame->m_aui_manager->Update();
	}
}

void OnUpdateViewGeometryBar( wxUpdateUIEvent& event )
{
	event.Check(wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_geometryBar).IsShown());
}

void OnViewSolidBar( wxCommandEvent& event )
{
	wxAuiPaneInfo& pane_info = wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_solidBar);
	if(pane_info.IsOk()){
		pane_info.Show(event.IsChecked());
		wxGetApp().m_frame->m_aui_manager->Update();
	}
}

void OnUpdateViewSolidBar( wxUpdateUIEvent& event )
{
	event.Check(wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_solidBar).IsShown());
}

void OnViewViewingBar( wxCommandEvent& event )
{
	wxAuiPaneInfo& pane_info = wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_viewingBar);
	if(pane_info.IsOk()){
		pane_info.Show(event.IsChecked());
		wxGetApp().m_frame->m_aui_manager->Update();
	}
}

void OnUpdateViewViewingBar( wxUpdateUIEvent& event )
{
	event.Check(wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_viewingBar).IsShown());
}

void CHeeksFrame::OnResetLayout( wxCommandEvent& event )
{
	ToolImage::SetBitmapSize(ToolImage::default_bitmap_size);
	OnChangeBitmapSize();
	m_aui_manager->Update();
}

void CHeeksFrame::OnSetToolBarsToLeft( wxCommandEvent& event )
{
	OnChangeBitmapSize();
	SetToolBarsToLeft();
	m_aui_manager->Update();
}

void OnViewProperties( wxCommandEvent& event )
{
	wxAuiPaneInfo& pane_info = wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_properties);
	if(pane_info.IsOk()){
		pane_info.Show(event.IsChecked());
		wxGetApp().m_frame->m_aui_manager->Update();
	}
}

static void OnUpdateOpenRecent( wxUpdateUIEvent& event )
{
	size_t size = wxGetApp().m_frame->m_recent_files_menu->GetMenuItemCount();
	std::list<wxMenuItem*> menu_items;
	for(size_t i = 0; i< size; i++)menu_items.push_back(wxGetApp().m_frame->m_recent_files_menu->FindItemByPosition(i));
	for(std::list<wxMenuItem*>::iterator It = menu_items.begin(); It != menu_items.end(); It++)
	{
		wxMenuItem* menu_item = *It;
		wxGetApp().m_frame->m_recent_files_menu->Delete(menu_item);
	}

	int recent_id = ID_RECENT_FIRST;
	for(std::list< wxString >::iterator It = wxGetApp().m_recent_files.begin(); It != wxGetApp().m_recent_files.end() && recent_id < ID_RECENT_FIRST + MAX_RECENT_FILES; It++, recent_id++)
	{
		wxString& filepath = *It;
		wxGetApp().m_frame->m_recent_files_menu->Append(recent_id, filepath, filepath);
	}
}

static void OnUpdateViewProperties( wxUpdateUIEvent& event )
{
	event.Check(wxGetApp().m_frame->m_aui_manager->GetPane(wxGetApp().m_frame->m_properties).IsShown());
}

void OnSelectModeButton( wxCommandEvent& WXUNUSED( event ) )
{
	wxGetApp().m_marked_list->m_filter = -1;
	wxGetApp().SetInputMode((CInputMode*)(wxGetApp().m_select_mode));
}

void OnLinesButton( wxCommandEvent& WXUNUSED( event ) )
{
	line_strip.drawing_mode = LineDrawingMode;
	wxGetApp().SetInputMode(&line_strip);
}


void OnEllipseButton( wxCommandEvent& WXUNUSED( event ) )
{
	line_strip.drawing_mode = EllipseDrawingMode;
	wxGetApp().SetInputMode(&line_strip);
}
void OnPointsButton( wxCommandEvent& WXUNUSED( event ) )
{
	wxGetApp().SetInputMode(&point_drawing);
}

void OnSplinePointsButton( wxCommandEvent& WXUNUSED( event ) )
{
	if(!wxGetApp().CheckForNOrMore(wxGetApp().m_marked_list->list(), 3, PointType, 0, _("Pick three or more points to be splined"), _("Spline Through Points")))return;
	wxGetApp().StartHistory();
	std::list<gp_Pnt> points;
	for(std::list<HeeksObj*>::iterator It = wxGetApp().m_marked_list->list().begin(); It != wxGetApp().m_marked_list->list().end(); It++)
	{
		HeeksObj* object = *It;
		if(object->GetType() == PointType)
		{
			points.push_back(((HPoint*)object)->m_p);
		}
	}
	HSpline* new_object = new HSpline(points, &wxGetApp().current_color);
	wxGetApp().AddUndoably(new_object, NULL, NULL);
	wxGetApp().EndHistory();
}

void OnRectanglesButton( wxCommandEvent& WXUNUSED( event ) )
{
	regular_shapes_drawing.m_mode = RectanglesRegularShapeMode;
	wxGetApp().SetInputMode(&regular_shapes_drawing);
}

void OnObroundsButton( wxCommandEvent& WXUNUSED( event ) )
{
	regular_shapes_drawing.m_mode = ObroundRegularShapeMode;
	wxGetApp().SetInputMode(&regular_shapes_drawing);
}

void OnPolygonsButton( wxCommandEvent& WXUNUSED( event ) )
{
	regular_shapes_drawing.m_mode = PolygonsRegularShapeMode;
	wxGetApp().SetInputMode(&regular_shapes_drawing);
}

void OnTextButton( wxCommandEvent& WXUNUSED( event ) )
{
	gp_Trsf mat = wxGetApp().GetDrawMatrix(true);
	HText* new_object = new HText(mat, _T("text"), &(wxGetApp().current_color),
#ifndef WIN32
		wxGetApp().m_pVectorFont,
#endif
		0, 0);
	wxGetApp().StartHistory();
	wxGetApp().AddUndoably(new_object, NULL, NULL);
	wxGetApp().m_marked_list->Clear(true);
	wxGetApp().m_marked_list->Add(new_object, true);
	wxGetApp().SetInputMode(wxGetApp().m_select_mode);
	wxGetApp().EndHistory();
	wxGetApp().Repaint();
}

void OnDimensioningButton( wxCommandEvent& WXUNUSED( event ) )
{
	wxGetApp().SetInputMode(&dimension_drawing);
}

void OnCircles3pButton( wxCommandEvent& WXUNUSED( event ) )
{
	// See if the operator has already selected objects.  If we can find three points
	// from the selected items then we can go ahead with the circle's construction
	// without prompting for more.  If there is a different number of points found then
	// prompt the user as usual.

	std::list<HeeksObj *> selected_objects = wxGetApp().m_marked_list->list();
	std::vector<DigitizedPoint>   points;
	for (std::list<HeeksObj *>::const_iterator l_itObject = selected_objects.begin();
			l_itObject != selected_objects.end(); l_itObject++)
	{
		switch ((*l_itObject)->GetType())
		{
			case PointType:
				{
					points.push_back( DigitizedPoint( ((HPoint *)*l_itObject)->m_p, DigitizeCoordsType, *l_itObject ) );
				}
				break;
		} // End switch
	} // End if - then

	if (points.size() == 3)
	{
		gp_Circ c;
		if(DigitizedPoint::GetTangentCircle(points[0], points[1], points[2], c))
		{
			double centre[3];
			centre[0] = c.Location().Coord().X();
			centre[1] = c.Location().Coord().Y();
			centre[2] = c.Location().Coord().Z();

			wxGetApp().StartHistory();
			heekscad_interface.AddUndoably( heekscad_interface.NewCircle( centre, c.Radius() ), NULL );
			wxGetApp().EndHistory();
		}
	}
	else
	{
		line_strip.drawing_mode = CircleDrawingMode;
		line_strip.circle_mode = ThreePointsCircleMode;
		wxGetApp().SetInputMode(&line_strip);
	}
}

void OnCircles2pButton( wxCommandEvent& WXUNUSED( event ) )
{
	line_strip.drawing_mode = CircleDrawingMode;
	line_strip.circle_mode = CentreAndPointCircleMode;
	wxGetApp().SetInputMode(&line_strip);
}

void OnCirclesprButton( wxCommandEvent& WXUNUSED( event ) )
{
	line_strip.drawing_mode = CircleDrawingMode;
	line_strip.circle_mode = CentreAndRadiusCircleMode;
	wxGetApp().SetInputMode(&line_strip);
}

void OnILineButton( wxCommandEvent& WXUNUSED( event ) )
{
	line_strip.drawing_mode = ILineDrawingMode;
	wxGetApp().SetInputMode(&line_strip);
}

void OnCoordinateSystem( wxCommandEvent& WXUNUSED( event ) )
{
	gp_Trsf mat = wxGetApp().GetDrawMatrix(false);
	gp_Pnt o = gp_Pnt(0, 0, 0).Transformed(mat);
	gp_Dir x = gp_Dir(1, 0, 0).Transformed(mat);
	gp_Dir y = gp_Dir(0, 1, 0).Transformed(mat);
	CoordinateSystem* new_object = new CoordinateSystem(_("Coordinate System"), o, x, y);
	wxGetApp().m_marked_list->Clear(true);
	wxGetApp().SetInputMode(wxGetApp().m_select_mode);

	// and pick from three points
	bool result = new_object->PickFrom3Points();
	if(result)
	{
		wxGetApp().AddUndoably(new_object, NULL, NULL);
		wxGetApp().m_marked_list->Add(new_object, true);
	}
	else
	{
		delete new_object;
	}
}

void OnNewOrigin( wxCommandEvent& WXUNUSED( event ) )
{
	gp_Trsf mat = wxGetApp().GetDrawMatrix(false);
	gp_Pnt o = gp_Pnt(0, 0, 0).Transformed(mat);
	gp_Dir x = gp_Dir(1, 0, 0).Transformed(mat);
	gp_Dir y = gp_Dir(0, 1, 0).Transformed(mat);
	CoordinateSystem* new_object = new CoordinateSystem(_("Coordinate System"), o, x, y);
	wxGetApp().m_marked_list->Clear(true);
	wxGetApp().SetInputMode(wxGetApp().m_select_mode);

	// and pick from three points
	bool result = new_object->PickFrom1Point();
	if(result)
	{
		wxGetApp().AddUndoably(new_object, NULL, NULL);
		wxGetApp().m_marked_list->Add(new_object, true);
	}
	else
	{
		delete new_object;
	}
}

void OnOpenButton( wxCommandEvent& event )
{
	wxGetApp().OnOpenButton();
}

void OnImportButton(wxCommandEvent& event)
{
	HeeksConfig config;
	wxString default_directory = wxEmptyString;
	config.Read(_T("ImportDirectory"), &default_directory, _T(""));

	wxFileDialog dialog(wxGetApp().m_frame, _("Import file"), default_directory, wxEmptyString, wxGetApp().GetKnownFilesWildCardString(true, true));
	dialog.CentreOnParent();

	if (dialog.ShowModal() == wxID_OK)
	{
		if (wxGetApp().OpenFile(dialog.GetPath().c_str(), true))
		{
			wxGetApp().m_frame->m_graphics->OnMagExtents(true, true, 25);
			config.Write(_T("ImportDirectory"), dialog.GetDirectory());
		}
	}
}

void OnSaveButton( wxCommandEvent& event )
{
	wxGetApp().SaveProject();
}

void OnUpdateSave( wxUpdateUIEvent& event )
{
	event.Enable(wxGetApp().IsModified());
}

void OnSaveAsButton( wxCommandEvent& event )
{
	wxGetApp().SaveProject(true);
}

void OnResetDefaultsButton( wxCommandEvent& event )
{
	wxGetApp().RestoreDefaults();
	wxMessageBox(_("You must restart the application for the settings to be changed"));
}

void OnUndoButton( wxCommandEvent& event )
{
	wxGetApp().RollBack();
	wxGetApp().Repaint();
}

void OnRedoButton( wxCommandEvent& event )
{
	wxGetApp().RollForward();
	wxGetApp().Repaint();
}

void OnUpdateUndo( wxUpdateUIEvent& event )
{
	event.Enable(wxGetApp().CanUndo());
}

void OnUpdateRedo( wxUpdateUIEvent& event )
{
	event.Enable(wxGetApp().CanRedo());
}

void OnNewButton( wxCommandEvent& event )
{
	wxGetApp().OnNewButton();
}

void OnCutButton( wxCommandEvent& event )
{
	wxGetApp().m_marked_list->CutSelectedItems();
}

void OnUpdateCut( wxUpdateUIEvent& event )
{
	event.Enable(wxGetApp().m_marked_list->size() > 0);
}

void OnCopyButton( wxCommandEvent& event )
{
	wxGetApp().m_marked_list->CopySelectedItems();
}

void OnUpdateCopy( wxUpdateUIEvent& event )
{
	event.Enable(wxGetApp().m_marked_list->size() > 0);
}

void OnPasteButton( wxCommandEvent& event )
{
	wxGetApp().Paste(NULL, NULL);
}

void OnDeleteButton( wxCommandEvent& event )
{
	std::list<HeeksObj *> list;
	for(std::list<HeeksObj*>::iterator It = wxGetApp().m_marked_list->list().begin(); It != wxGetApp().m_marked_list->list().end(); It++)
	{
		HeeksObj* object = *It;
		if(object->CanBeRemoved())list.push_back(object);
	}
	wxGetApp().m_marked_list->Clear(true);

	if(list.size() > 0)
	{
		wxGetApp().DeleteUndoably(list);
	}
}

void OnUpdateDelete( wxUpdateUIEvent& event )
{
	event.Enable(wxGetApp().m_marked_list->size() > 0);
}

void OnUpdatePaste( wxUpdateUIEvent& event )
{
	event.Enable(wxGetApp().IsPasteReady());
}

void OnSubtractButton( wxCommandEvent& event )
{
	if(!wxGetApp().CheckForNOrMore(wxGetApp().m_marked_list->list(), 2, SolidType, FaceType, _("Pick two or more faces or solids, the first one will be cut by the others"), _("Subtract Solids")))return;
	wxGetApp().StartHistory();
	CShape::CutShapes(wxGetApp().m_marked_list->list());
	wxGetApp().EndHistory();
}

void OnFuseButton( wxCommandEvent& event )
{
	if(!wxGetApp().CheckForNOrMore(wxGetApp().m_marked_list->list(), 2, SolidType, _("Pick two or more solids to be fused together"), _("Fuse Solids")))return;
	wxGetApp().StartHistory();
	CShape::FuseShapes(wxGetApp().m_marked_list->list());
	wxGetApp().EndHistory();
}

void OnCommonButton( wxCommandEvent& event )
{
	if(!wxGetApp().CheckForNOrMore(wxGetApp().m_marked_list->list(), 2, SolidType, _("Pick two or more solids, only the shape that is contained by all of them will remain"), _("Intersection of Solids")))return;
	wxGetApp().StartHistory();
	CShape::CommonShapes(wxGetApp().m_marked_list->list());
	wxGetApp().m_marked_list->Clear(true);
	wxGetApp().EndHistory();
}

void OnFilletButton( wxCommandEvent& event )
{
	if(!wxGetApp().CheckForNOrMore(wxGetApp().m_marked_list->list(), 1, EdgeType, _("Pick one or more edges to add a fillet to"), _("Edge Fillet")))return;
	double rad = 2.0;
	HeeksConfig config;
	config.Read(_T("EdgeBlendRadius"), &rad);
	if(wxGetApp().InputLength(_("Enter Blend Radius"), _("Radius"), rad))
	{
		wxGetApp().StartHistory();
		CShape::FilletOrChamferEdges(wxGetApp().m_marked_list->list(), rad);
		config.Write(_T("EdgeBlendRadius"), rad);
		wxGetApp().m_marked_list->Clear(true);
		wxGetApp().EndHistory();
	}
}

void OnChamferButton( wxCommandEvent& event )
{
	if(!wxGetApp().CheckForNOrMore(wxGetApp().m_marked_list->list(), 1, EdgeType, _("Pick one or more edges to add a chamfer to"), _("Edge Chamfer")))return;
	double rad = 2.0;
	HeeksConfig config;
	config.Read(_T("EdgeChamferDist"), &rad);
	if(wxGetApp().InputLength(_("Enter chamfer distance"), _("Distance"), rad))
	{
		wxGetApp().StartHistory();
		CShape::FilletOrChamferEdges(wxGetApp().m_marked_list->list(), rad, true);
		config.Write(_T("EdgeChamferDist"), rad);
		wxGetApp().m_marked_list->Clear(true);
		wxGetApp().EndHistory();
	}
}

void OnSectioningButton( wxCommandEvent& event )
{
	wxGetApp().SectioningDialog();
}

void OnRuledSurfaceButton( wxCommandEvent& event )
{
	if(!wxGetApp().CheckForNOrMore(wxGetApp().m_marked_list->list(), 2, SketchType, wxString(_("Pick two or more sketches, to create a lofted solid between")) + _T("\n( ") + _( "hold down Ctrl key to select more than one solid") + _T(" )"), _("Lofted Body")))return;
	wxGetApp().StartHistory();
	PickCreateRuledSurface();
	wxGetApp().EndHistory();
}

void OnExtrudeButton( wxCommandEvent& event )
{
	if(!wxGetApp().CheckForNOrMore(wxGetApp().m_marked_list->list(), 1, SketchType, FaceType, CircleType, wxString(_("Pick one or more sketches, faces or circles, to create an extruded body from")) + _T("\n( ") + _( "hold down Ctrl key to select more than one solid") + _T(" )"), _("Extrude")))return;
	wxGetApp().StartHistory();
	PickCreateExtrusion();
	wxGetApp().EndHistory();
}

void OnRevolveButton( wxCommandEvent& event )
{
	if(!wxGetApp().CheckForNOrMore(wxGetApp().m_marked_list->list(), 1, SketchType, FaceType, CircleType, wxString(_("Pick one or more sketches, faces or circles, to create a revolved body from")) + _T("\n( ") + _( "hold down Ctrl key to select more than one solid") + _T(" )"), _("Extrude")))return;
	wxGetApp().StartHistory();
	PickCreateRevolution();
	wxGetApp().EndHistory();
}

void OnSweepButton( wxCommandEvent& event )
{
	if(!wxGetApp().CheckForNOrMore(wxGetApp().m_marked_list->list(), 1, SketchType, FaceType, CircleType, wxString(_("Pick one or more sketches, faces or circles, to sweep")) + _T("\n( ") + _( "hold down Ctrl key to select more than one solid") + _T(" )"), _("Extrude")))return;
	wxGetApp().StartHistory();
	PickCreateSweep();
	wxGetApp().EndHistory();
}

void AddObjectFromButton(HeeksObj* new_object)
{
	wxGetApp().StartHistory();
	wxGetApp().AddUndoably(new_object,NULL,NULL);
	wxGetApp().m_marked_list->Clear(true);
	wxGetApp().m_marked_list->Add(new_object, true);
	wxGetApp().SetInputMode(wxGetApp().m_select_mode);
	wxGetApp().EndHistory();
	wxGetApp().Repaint();
}

void OnGearButton( wxCommandEvent& event )
{
	HGear* new_object = new HGear();
	AddObjectFromButton(new_object);
}

void OnSphereButton( wxCommandEvent& event )
{
	gp_Trsf mat = wxGetApp().GetDrawMatrix(true);
	CSphere* new_object = new CSphere(gp_Pnt(0, 0, 0).Transformed(mat), 5, NULL, HeeksColor(240, 191, 191), 1.0f);
	AddObjectFromButton(new_object);
}

void OnCubeButton( wxCommandEvent& event )
{
	gp_Trsf mat = wxGetApp().GetDrawMatrix(false);
	CCuboid* new_object = new CCuboid(gp_Ax2(gp_Pnt(0, 0, 0).Transformed(mat), gp_Dir(0, 0, 1).Transformed(mat), gp_Dir(1, 0, 0).Transformed(mat)), 10, 10, 10, NULL, HeeksColor(191, 240, 191), 1.0f);
	AddObjectFromButton(new_object);
}

void OnCylButton( wxCommandEvent& event )
{
	gp_Trsf mat = wxGetApp().GetDrawMatrix(true);
	CCylinder* new_object = new CCylinder(gp_Ax2(gp_Pnt(0, 0, 0).Transformed(mat), gp_Dir(0, 0, 1).Transformed(mat), gp_Dir(1, 0, 0).Transformed(mat)), 5, 10, NULL, HeeksColor(191, 191, 240), 1.0f);
	AddObjectFromButton(new_object);
}

void OnConeButton( wxCommandEvent& event )
{
	gp_Trsf mat = wxGetApp().GetDrawMatrix(true);
	CCone* new_object = new CCone(gp_Ax2(gp_Pnt(0, 0, 0).Transformed(mat), gp_Dir(0, 0, 1).Transformed(mat), gp_Dir(1, 0, 0).Transformed(mat)), 10, 5, 20, NULL, HeeksColor(240, 240, 191), 1.0f);
	AddObjectFromButton(new_object);
}

void OnRedrawButton( wxCommandEvent& event )
{
	wxGetApp().RecalculateGLLists();
	wxGetApp().Repaint();
}


void OnMagButton( wxCommandEvent& event )
{
	wxGetApp().SetInputMode(wxGetApp().magnification);
}

void OnMagExtentsButton( wxCommandEvent& event )
{
	wxGetApp().m_frame->m_graphics->OnMagExtents(true, true, 6);
}

void OnMagXYButton( wxCommandEvent& event )
{
	wxGetApp().m_frame->m_graphics->OnMagXY(true);
}

void OnMagXYMButton( wxCommandEvent& event )
{
	wxGetApp().m_frame->m_graphics->OnMagXYM(true);
}

void OnMagXZButton( wxCommandEvent& event )
{
	wxGetApp().m_frame->m_graphics->OnMagXZ(true);
}

void OnMagXZMButton( wxCommandEvent& event )
{
	wxGetApp().m_frame->m_graphics->OnMagXZM(true);
}

void OnMagYZButton( wxCommandEvent& event )
{
	wxGetApp().m_frame->m_graphics->OnMagYZ(true);
}

void OnMagYZMButton( wxCommandEvent& event )
{
	wxGetApp().m_frame->m_graphics->OnMagYZM(true);
}

void OnMagXYZButton( wxCommandEvent& event )
{
	wxGetApp().m_frame->m_graphics->OnMagXYZ(true);
}

void OnMagNoRotButton( wxCommandEvent& event )
{
	wxGetApp().m_frame->m_graphics->OnMagExtents(false, true, 6);
}

void OnMagPreviousButton( wxCommandEvent& event )
{
	wxGetApp().m_frame->m_graphics->OnMagPrevious();
}

void OnViewRotateButton( wxCommandEvent& event )
{
	wxGetApp().SetInputMode(wxGetApp().viewrotating);
}

void OnViewZoomButton( wxCommandEvent& event )
{
	wxGetApp().SetInputMode(wxGetApp().viewzooming);
}

void OnViewPanButton( wxCommandEvent& event )
{
	wxGetApp().SetInputMode(wxGetApp().viewpanning);
}

void OnFullScreenButton( wxCommandEvent& event )
{
	wxGetApp().m_frame->ShowFullScreen(true);
}

void OnMoveTranslateButton( wxCommandEvent& event )
{
	TransformTools::Translate(false);
}

void OnCopyTranslateButton( wxCommandEvent& event )
{
	TransformTools::Translate(true);
}

void OnMoveRotateButton( wxCommandEvent& event )
{
	TransformTools::Rotate(false);
}

void OnCopyRotateButton( wxCommandEvent& event )
{
	TransformTools::Rotate(true);
}

void OnMoveMirrorButton( wxCommandEvent& event )
{
	TransformTools::Mirror(false);
}

void OnCopyMirrorButton( wxCommandEvent& event )
{
	TransformTools::Mirror(true);
}

void OnMoveScaleButton( wxCommandEvent& event )
{
	TransformTools::Scale(false);
}

#ifndef USING_RIBBON
void CHeeksFrame::OnExternalButton( wxCommandEvent& event )
{
	// this might be the undo button! wxGetApp().CreateUndoPoint();
	int id = event.GetId();

	std::map<int, SExternalButtonFunctions >::iterator FindIt = m_external_buttons.find(id);
	if(FindIt != m_external_buttons.end()){
		SExternalButtonFunctions& ebf = FindIt->second;
		(*(ebf.on_button))(event);
	}

	// this might be the undo button! wxGetApp().Changed();
}
#endif

void CHeeksFrame::OnRecentFile( wxCommandEvent& event )
{
	int id = event.GetId();

	int recent_id = ID_RECENT_FIRST;
	for(std::list< wxString >::iterator It = wxGetApp().m_recent_files.begin(); It != wxGetApp().m_recent_files.end() && recent_id < ID_RECENT_FIRST + MAX_RECENT_FILES; It++, recent_id++)
	{
		if(recent_id != id)continue;
		wxString& filepath = *It;

		int res = wxGetApp().CheckForModifiedDoc();
		if(res != wxCANCEL)
		{
			wxGetApp().OnBeforeNewOrOpen(true, res);
			wxGetApp().Reset();
			wxGetApp().OpenFile(filepath.c_str());
			wxGetApp().m_frame->m_graphics->OnMagExtents(true, true, 25);
			wxGetApp().OnNewOrOpen(true, res);
			wxGetApp().ClearHistory();
			wxGetApp().SetLikeNewFile();
		}
		break;
	}
}

#ifndef USING_RIBBON
void CHeeksFrame::OnUpdateExternalButton( wxUpdateUIEvent& event )
{
	int id = event.GetId();

	std::map<int, SExternalButtonFunctions >::iterator FindIt = m_external_buttons.find(id);
	if(FindIt != m_external_buttons.end()){
		SExternalButtonFunctions& ebf = FindIt->second;
		if(ebf.on_update_button)(*(ebf.on_update_button))(event);
	}
}
#endif
void CHeeksFrame::OnSize( wxSizeEvent& evt )
{
	wxSize size = evt.GetSize();
	int width = size.GetWidth();
	int height = size.GetHeight();
	HeeksConfig config;
	config.Write(_T("MainFrameWidth"), width);
	config.Write(_T("MainFrameHeight"), height);

	// call add-ins OnSize functions
	for(std::list< void(*)(wxSizeEvent&) >::iterator It = wxGetApp().m_on_graphics_size_list.begin(); It != wxGetApp().m_on_graphics_size_list.end(); It++)
	{
		void(*callback)(wxSizeEvent&) = *It;
		(*callback)(evt);
	}
}

void CHeeksFrame::OnMove( wxMoveEvent& evt )
{
	wxPoint pos = GetPosition();
	int posx = pos.x;
	int posy = pos.y;
	HeeksConfig config;
	config.Write(_T("MainFramePosX"), posx);
	config.Write(_T("MainFramePosY"), posy);
}

int CHeeksFrame::MakeNextIDForTool(void(*onButtonFunction)(wxCommandEvent&), void(*onUpdateButtonFunction)(wxUpdateUIEvent&))
{
	while(m_external_buttons.find(m_next_id_for_button) != m_external_buttons.end())
	{
		// already used
		m_next_id_for_button++;
	}

	if(m_next_id_for_button >= ID_FIRST_POP_UP_MENU_TOOL)
	{
		// too many button IDs!
		wxMessageBox(_T("too many button IDs!, see CHeeksFrame::GetNextIDForTool"));
	}

	int id_to_use = m_next_id_for_button;


	SExternalButtonFunctions ebf;
	ebf.on_button = onButtonFunction;
	ebf.on_update_button = onUpdateButtonFunction;
	m_external_buttons.insert(std::pair<int, SExternalButtonFunctions > ( id_to_use, ebf ));
	m_next_id_for_button++;
	return id_to_use;
}

void CHeeksFrame::SetToolFunctions(int Id, void(*onButtonFunction)(wxCommandEvent&), void(*onUpdateButtonFunction)(wxUpdateUIEvent&))
{
	SExternalButtonFunctions ebf;
	ebf.on_button = onButtonFunction;
	ebf.on_update_button = onUpdateButtonFunction;
	m_external_buttons.erase(Id);
	m_external_buttons.insert(std::pair<int, SExternalButtonFunctions > ( Id, ebf ));
}

int CHeeksFrame::AddMenuItem(wxMenu* menu, const wxString& text, const wxBitmap& bitmap, void(*onButtonFunction)(wxCommandEvent&), void(*onUpdateButtonFunction)(wxUpdateUIEvent&), wxMenu* submenu, bool check_item)
{
	int id_to_use = MakeNextIDForTool(onButtonFunction, onUpdateButtonFunction);

	wxMenuItem *menuItem = new wxMenuItem(menu, id_to_use, text, wxString(_T("")), check_item ? wxITEM_CHECK : wxITEM_NORMAL, submenu);
	if(!check_item)menuItem->SetBitmap(bitmap);
	menu->Append(menuItem);

	return id_to_use;
}

wxToolBarToolBase* CHeeksFrame::AddToolBarTool(wxToolBar* toolbar, const wxString& title, const wxBitmap& bitmap, const wxString& caption, void(*onButtonFunction)(wxCommandEvent&), void(*onUpdateButtonFunction)(wxUpdateUIEvent&))
{
	int id_to_use = MakeNextIDForTool(onButtonFunction, onUpdateButtonFunction);

	return toolbar->AddTool(id_to_use, title, bitmap, caption);
}

static std::map<int, Tool*> tool_map_for_OnTool;

static void OnTool(wxCommandEvent& event)
{
	std::map<int, Tool*>::iterator FindIt = tool_map_for_OnTool.find(event.GetId());
	if(FindIt != tool_map_for_OnTool.end())
	{
		Tool* tool = FindIt->second;
		tool->Run();
	}
}

void CHeeksFrame::AddToolBarTool(wxToolBar* toolbar, Tool* tool)
{
	wxBitmap* bitmap = tool->Bitmap();
	if(bitmap)
	{
		wxToolBarToolBase* button_added = wxGetApp().m_frame->AddToolBarTool(toolbar, tool->GetTitle(), *bitmap, tool->GetToolTip(), OnTool);
		if(button_added)tool_map_for_OnTool.insert( std::pair<int, Tool*> ( button_added->GetId(), tool ) );
	}
}

class CFlyOutButton;
class ToolBarPopup;

class CFlyOutButton: public wxBitmapButton
{
	wxToolBar *m_toolBar;
	wxBitmap m_current_bitmap;
	wxTimer m_timer;
	bool m_disappears_on_click;

	public:
	CFlyOutList m_flyout_list;
	ToolBarPopup* m_toolbarPopup;

	CFlyOutButton(const CFlyOutList &flyout_list,
			wxToolBar *toolbar,
			int id_to_use,
			const wxPoint& pos,
			const wxSize& size,
			bool disappears_on_click)
		:wxBitmapButton(toolbar, id_to_use, flyout_list.GetMainItem()->m_bitmap, pos, size,
#ifdef WIN32
				wxBORDER_SIMPLE
#else
				wxBORDER_NONE
#endif
				)
		,m_toolBar(toolbar)
		,m_disappears_on_click(disappears_on_click)
		,m_flyout_list(flyout_list)
		,m_toolbarPopup(NULL)
	{
	}

	~CFlyOutButton();

	void OnMouse( wxMouseEvent& event );

	void OnMenuEvent2(int id);

	void OnMenuEvent(wxCommandEvent& event)
	{
		OnMenuEvent2(event.GetId());
	}

	void OnIdle(wxIdleEvent& event);

	private:
	DECLARE_EVENT_TABLE()
};

BEGIN_EVENT_TABLE(CFlyOutButton, wxBitmapButton)
	EVT_MOUSE_EVENTS(CFlyOutButton::OnMouse)
	EVT_MENU_RANGE(ID_FIRST_POP_UP_MENU_TOOL, ID_FIRST_POP_UP_MENU_TOOL + 1000, CFlyOutButton::OnMenuEvent)
	EVT_IDLE(CFlyOutButton::OnIdle)
END_EVENT_TABLE()


class BitmapButton2:public wxBitmapButton
{
	CFlyOutButton *m_fly_out_button;

	public:
	BitmapButton2( CFlyOutButton *fly_out_button, wxWindow* parent, wxWindowID id, const wxBitmap& bitmap, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize):wxBitmapButton(parent, id, bitmap, pos, size), m_fly_out_button(fly_out_button){}
	void OnMouse( wxMouseEvent& event )
	{
		if(event.LeftUp())
		{
			m_fly_out_button->OnMenuEvent2(this->GetId());
		}
	}
	private:
	DECLARE_EVENT_TABLE()
};

BEGIN_EVENT_TABLE(BitmapButton2, wxBitmapButton)
	EVT_MOUSE_EVENTS(BitmapButton2::OnMouse)
END_EVENT_TABLE()


#define FLYOUT_PANEL_BORDER 5

class ToolBarPopup: public wxPopupTransientWindow
{
	public:
		wxWindow *m_toolBar;
		wxScrolledWindow *m_panel;

	public:
		ToolBarPopup( CFlyOutButton *flyout_button):wxPopupTransientWindow( flyout_button )
	{
		m_panel = new wxScrolledWindow( this, wxID_ANY );
		m_panel->SetBackgroundColour( *wxLIGHT_GREY );

		m_toolBar = new wxWindow(m_panel, -1, wxDefaultPosition, wxDefaultSize);

		const CFlyOutItem* main_fo = flyout_button->m_flyout_list.GetMainItem();
		std::list<CFlyOutItem*> items_to_add;
		std::list<int> ids_to_add;
		int i = 0;
		for(std::list<CFlyOutItem*>::iterator It = flyout_button->m_flyout_list.m_list.begin(); It != flyout_button->m_flyout_list.m_list.end(); It++, i++)
		{
			CFlyOutItem *fo = *It;
			int id_to_use = i+ID_FIRST_POP_UP_MENU_TOOL;
			if(fo == main_fo)
			{
				items_to_add.push_front(fo);
				ids_to_add.push_front(id_to_use);
			}
			else
			{
				items_to_add.push_back(fo);
				ids_to_add.push_back(id_to_use);
			}
		}

		std::list<CFlyOutItem*>::iterator ItemsIt = items_to_add.begin();
		std::list<int>::iterator IdIt = ids_to_add.begin();
		i = 0;
		for(; ItemsIt != items_to_add.end(); ItemsIt++, IdIt++, i++)
		{
			CFlyOutItem *fo = *ItemsIt;
			int id_to_use = *IdIt;
			BitmapButton2* button = new BitmapButton2(flyout_button, m_toolBar, id_to_use, fo->m_bitmap, wxPoint(0, i * (ToolImage::GetBitmapSize() + FLYOUT_PANEL_BORDER)),
#ifdef WIN32
				wxSize(ToolImage::GetBitmapSize() + FLYOUT_PANEL_BORDER, ToolImage::GetBitmapSize() + FLYOUT_PANEL_BORDER)
#else
				wxDefaultSize
#endif
				);
			button->SetToolTip(fo->m_tooltip);
			fo->flyout_button = flyout_button;
			fo->button = button;
		}

		m_toolBar->SetSize(wxSize(ToolImage::GetBitmapSize(), ToolImage::GetBitmapSize() * items_to_add.size()));

		wxBoxSizer *topSizer = new wxBoxSizer( wxVERTICAL );
		topSizer->Add( m_toolBar, 0, wxALL, FLYOUT_PANEL_BORDER );

		m_panel->SetAutoLayout( true );
		m_panel->SetSizer( topSizer );
		topSizer->Fit(m_panel);
		topSizer->Fit(this);
	}

	private:
		wxButton *m_button;
		wxStaticText *m_mouseText;
};

CFlyOutButton::~CFlyOutButton()
{
	delete m_toolbarPopup;
}

void CFlyOutButton::OnMouse( wxMouseEvent& event )
{
	if(event.Entering())
	{
		// delete previous popup
#ifdef WIN32
		if(m_toolbarPopup)m_toolbarPopup->Close();
#else
		if (m_toolbarPopup) delete m_toolbarPopup;
#endif

		// make a new popup toolbar
		m_toolbarPopup = new ToolBarPopup( this );
		wxWindow *btn = (wxWindow*) event.GetEventObject();
		wxPoint pos = btn->ClientToScreen( wxPoint(0,0) );
		wxSize sz = btn->GetSize();
#ifdef WIN32
		m_toolbarPopup->Move(pos.x - FLYOUT_PANEL_BORDER, pos.y - 3 - FLYOUT_PANEL_BORDER);
#else
		m_toolbarPopup->Move(pos.x - FLYOUT_PANEL_BORDER, pos.y - FLYOUT_PANEL_BORDER);
#endif
		m_toolbarPopup->Popup();
	}
}

void CFlyOutButton::OnMenuEvent2(int id)
{
	int i = 0;
	for(std::list<CFlyOutItem*>::const_iterator It = m_flyout_list.m_list.begin(); It != m_flyout_list.m_list.end(); It++, i++)
	{
		if( i+ID_FIRST_POP_UP_MENU_TOOL == id)
		{
			// hide the popup
			if(m_disappears_on_click)
			{
				delete m_toolbarPopup;
				m_toolbarPopup = NULL;
			}

			// call the OnButtonFunction
			const CFlyOutItem &fo = *(*It);

			// call the OnButtonFunction
			wxCommandEvent event;
			(*fo.m_onButtonFunction)(event);

			break;
		}
	}
}

void CFlyOutButton::OnIdle(wxIdleEvent& event)	{
	if(m_toolbarPopup)
	{
		const wxPoint & pt = ::wxGetMousePosition();
		if(!m_toolbarPopup->GetScreenRect().Contains(pt))
		{
			delete m_toolbarPopup;
			m_toolbarPopup = NULL;
		}
	}
}


static void OnEndofButton( wxCommandEvent& event )
{
	wxGetApp().digitize_end = !wxGetApp().digitize_end;
#ifndef USING_RIBBON
	wxGetApp().m_frame->m_endof_button->m_bitmap = wxBitmap(ToolImage(wxGetApp().digitize_end ? _T("endof") : _T("endofgray")));
	wxGetApp().m_frame->m_endof_button->button->SetBitmapLabel(wxBitmap(ToolImage(wxGetApp().digitize_end ? _T("endof") : _T("endofgray"))));
	wxGetApp().m_frame->m_endof_button->flyout_button->SetBitmapLabel(wxBitmap(ToolImage(wxGetApp().digitize_end ? _T("endof") : _T("endofgray"))));
#endif
}

static void OnIntersButton(wxCommandEvent& event)
{
	wxGetApp().digitize_inters = !wxGetApp().digitize_inters;
#ifndef USING_RIBBON
	wxGetApp().m_frame->m_inters_button->m_bitmap = wxBitmap(ToolImage(wxGetApp().digitize_inters ? _T("inters") : _T("intersgray")));
	wxGetApp().m_frame->m_inters_button->button->SetBitmapLabel(wxBitmap(ToolImage(wxGetApp().digitize_inters ? _T("inters") : _T("intersgray"))));
#endif
}

static void OnCentreButton(wxCommandEvent& event)
{
	wxGetApp().digitize_centre = !wxGetApp().digitize_centre;
#ifndef USING_RIBBON
	wxGetApp().m_frame->m_centre_button->m_bitmap = wxBitmap(ToolImage(wxGetApp().digitize_centre ? _T("centre") : _T("centregray")));
	wxGetApp().m_frame->m_centre_button->button->SetBitmapLabel(wxBitmap(ToolImage(wxGetApp().digitize_centre ? _T("centre") : _T("centregray"))));
#endif
}

static void OnMidpointButton(wxCommandEvent& event)
{
	wxGetApp().digitize_midpoint = !wxGetApp().digitize_midpoint;
#ifndef USING_RIBBON
	wxGetApp().m_frame->m_midpoint_button->m_bitmap = wxBitmap(ToolImage(wxGetApp().digitize_midpoint ? _T("midpoint") : _T("midpointgray")));
	wxGetApp().m_frame->m_midpoint_button->button->SetBitmapLabel(wxBitmap(ToolImage(wxGetApp().digitize_midpoint ? _T("midpoint") : _T("midpointgray"))));
#endif
}

static void OnSnapButton(wxCommandEvent& event)
{
	wxGetApp().draw_to_grid = !wxGetApp().draw_to_grid;
#ifndef USING_RIBBON
	wxGetApp().m_frame->m_snap_button->m_bitmap = wxBitmap(ToolImage(wxGetApp().draw_to_grid ? _T("snap") : _T("snapgray")));
	wxGetApp().m_frame->m_snap_button->button->SetBitmapLabel(wxBitmap(ToolImage(wxGetApp().draw_to_grid ? _T("snap") : _T("snapgray"))));
#endif
}

static void OnEditGridButton(wxCommandEvent& event)
{
	wxGetApp().InputDouble(_("Edit snap grid"), _("snap grid"), wxGetApp().digitizing_grid);
	wxGetApp().m_frame->m_graphics->Refresh();
}

void CHeeksFrame::AddToolBarFlyout(wxToolBar* toolbar, const CFlyOutList& flyout_list, bool disappears_on_click)
{
	if(flyout_list.m_list.size() == 0)return;
	int id_to_use = MakeNextIDForTool(NULL, NULL);
	wxBitmapButton* button = new CFlyOutButton(flyout_list, toolbar, id_to_use, wxDefaultPosition,
#ifdef WIN32
		wxSize(ToolImage::GetBitmapSize() + FLYOUT_PANEL_BORDER, ToolImage::GetBitmapSize())
#else
		wxDefaultSize
#endif
		,disappears_on_click);
	toolbar->AddControl(button);
}

// a class just so I can get at the protected m_tools of wxToolBar
class ToolBarForGettingToolsFrom: public wxToolBar
{
	public:
		void GetToolsIdList(std::list<int> &list)
		{
			wxToolBarToolsList::compatibility_iterator node;
			for ( node = m_tools.GetFirst(); node; node = node->GetNext() )
			{
				wxToolBarToolBase *tool = node->GetData();
				list.push_back(tool->GetId());
			}
		}
};

void CHeeksFrame::ClearToolBar(wxToolBar* m_toolBar)
{
	ToolBarForGettingToolsFrom* toolBar = (ToolBarForGettingToolsFrom*)m_toolBar;

	std::list<int> list;
	toolBar->GetToolsIdList(list);
	for(std::list<int>::iterator It = list.begin(); It != list.end(); It++)
	{
		int id = *It;
		m_external_buttons.erase(id);
		tool_map_for_OnTool.erase(id);
		if(id < m_next_id_for_button)m_next_id_for_button = id;
		m_toolBar->DeleteTool(id);
	}
}

//static
void CHeeksFrame::AddToolToListAndMenu(Tool *t, std::vector<ToolIndex> &tool_index_list, wxMenu *menu)
{
	assert(t); // NULLs are no longer allowed

	if (t->IsSeparator()) menu->AppendSeparator();
	else if (t->IsAToolList())
	{
		wxMenu *menu2 = new wxMenu;
		std::list<Tool*>& tool_list = ((ToolList*)t)->m_tool_list;
		std::list<Tool*>::iterator It;
		for (It=tool_list.begin();It!=tool_list.end();It++)
		{
			AddToolToListAndMenu(*It, tool_index_list, menu2);
		}
		wxMenuItem* menu_item = menu->Append(0, t->GetTitle(), menu2);
		wxBitmap* bitmap = t->Bitmap();
		if(bitmap)menu_item->SetBitmap(*bitmap);
	}
	else
	{
		ToolIndex ti;
		ti.m_tool = t;
		ti.m_index = tool_index_list.size();
		tool_index_list.push_back(ti);
		wxMenuItem* menu_item = menu->Append(ti.m_index+ID_FIRST_POP_UP_MENU_TOOL, t->GetTitle());
		wxBitmap* bitmap = t->Bitmap();
		if(bitmap)menu_item->SetBitmap(*bitmap);
		if(t->Disabled())menu->Enable(ti.m_index+1, false);
		if(t->Checked ())menu->Check(ti.m_index+1, true);
	}
}

void CHeeksFrame::Draw(wxDC& dc)
{
	wxGetApp().Draw(dc);
}

void OnPrint(wxCommandEvent& WXUNUSED(event))
{
	wxPrintDialogData printDialogData(* wxGetApp().m_printData);

	wxPrinter printer(& printDialogData);
	wxGetApp().m_frame->m_printout = new HeeksPrintout(_T("Heeks printout"));
	if (!printer.Print(wxGetApp().m_frame, wxGetApp().m_frame->m_printout, true /*prompt*/))
	{
		if (wxPrinter::GetLastError() == wxPRINTER_ERROR)
			wxMessageBox(_("There was a problem printing.\nPerhaps your current printer is not set correctly?"), _("Printing"), wxOK);
		else
			wxMessageBox(_("You canceled printing"), _("Printing"), wxOK);
	}
	else
	{
		(*wxGetApp().m_printData) = printer.GetPrintDialogData().GetPrintData();
	}

	delete wxGetApp().m_frame->m_printout;
	wxGetApp().m_frame->m_printout = NULL;
}

void OnPrintPreview(wxCommandEvent& WXUNUSED(event))
{
	// Pass two printout objects: for preview, and possible printing.
	wxPrintDialogData printDialogData(* wxGetApp().m_printData);
	wxPrintPreview *preview = new wxPrintPreview(new HeeksPrintout, new HeeksPrintout, & printDialogData);
	if (!preview->Ok())
	{
		delete preview;
		wxMessageBox(_("There was a problem previewing.\nPerhaps your current printer is not set correctly?"), _("Previewing"), wxOK);
		return;
	}

	wxPreviewFrame *frame = new wxPreviewFrame(preview, wxGetApp().m_frame, _("Demo Print Preview"), wxPoint(100, 100), wxSize(600, 650));
	frame->Centre(wxBOTH);
	frame->Initialize();
	frame->Show();
}

void OnPageSetup(wxCommandEvent& WXUNUSED(event))
{
	(*wxGetApp().m_pageSetupData) = *(wxGetApp().m_printData);

	wxPageSetupDialog pageSetupDialog(wxGetApp().m_frame, wxGetApp().m_pageSetupData);
	pageSetupDialog.ShowModal();

	(*wxGetApp().m_printData) = pageSetupDialog.GetPageSetupDialogData().GetPrintData();
	(*wxGetApp().m_pageSetupData) = pageSetupDialog.GetPageSetupDialogData();
}

void CHeeksFrame::OnChangeBitmapSize()
{
	if(!m_main_toolbar_removed)m_aui_manager->DetachPane(m_toolBar);
	if(!m_geometry_toolbar_removed)m_aui_manager->DetachPane(m_geometryBar);
	if(!m_solid_toolbar_removed)m_aui_manager->DetachPane(m_solidBar);
	if(!m_viewing_toolbar_removed)m_aui_manager->DetachPane(m_viewingBar);

	for(std::list< wxToolBarBase* >::iterator It = wxGetApp().m_external_toolbars.begin(); It != wxGetApp().m_external_toolbars.end(); It++)
	{
		wxToolBarBase* toolbar = *It;
		m_aui_manager->DetachPane(toolbar);
	}

	if(!m_main_toolbar_removed)wxGetApp().RemoveHideableWindow(m_toolBar);
	if(!m_geometry_toolbar_removed)wxGetApp().RemoveHideableWindow(m_geometryBar);
	if(!m_solid_toolbar_removed)wxGetApp().RemoveHideableWindow(m_solidBar);
	if(!m_viewing_toolbar_removed)wxGetApp().RemoveHideableWindow(m_viewingBar);
	for(std::list< wxToolBarBase* >::iterator It = wxGetApp().m_external_toolbars.begin(); It != wxGetApp().m_external_toolbars.end(); It++)
	{
		wxToolBarBase* toolbar = *It;
		wxGetApp().RemoveHideableWindow(toolbar);
	}

	if(!m_main_toolbar_removed)delete m_toolBar;
	if(!m_geometry_toolbar_removed)delete m_geometryBar;
	if(!m_solid_toolbar_removed)delete m_solidBar;
	if(!m_viewing_toolbar_removed)delete m_viewingBar;

	wxGetApp().m_external_toolbars.clear();

#ifndef USING_RIBBON
	AddToolBars();
#endif

	if(m_input_canvas)m_input_canvas->AddToolBar();
	RefreshInputCanvas();
	if(m_properties)m_properties->AddToolBar();
	RefreshProperties();
}

void CHeeksFrame::SetToolBarsSize()
{
	if(!m_main_toolbar_removed)m_toolBar->SetToolBitmapSize(wxSize(ToolImage::GetBitmapSize(), ToolImage::GetBitmapSize()));
	if(!m_geometry_toolbar_removed)m_geometryBar->SetToolBitmapSize(wxSize(ToolImage::GetBitmapSize(), ToolImage::GetBitmapSize()));
	if(!m_solid_toolbar_removed)m_solidBar->SetToolBitmapSize(wxSize(ToolImage::GetBitmapSize(), ToolImage::GetBitmapSize()));
	if(!m_viewing_toolbar_removed)m_viewingBar->SetToolBitmapSize(wxSize(ToolImage::GetBitmapSize(), ToolImage::GetBitmapSize()));

	for(std::list< wxToolBarBase* >::iterator It = wxGetApp().m_external_toolbars.begin(); It != wxGetApp().m_external_toolbars.end(); It++)
	{
		wxToolBarBase* toolbar = *It;
		toolbar->SetToolBitmapSize(wxSize(ToolImage::GetBitmapSize(), ToolImage::GetBitmapSize()));
	}
}

#ifndef USING_RIBBON

void CHeeksFrame::MakeMenus()
{
	// File Menu
	wxMenu *file_menu = new wxMenu;
	AddMenuItem(file_menu, _("New\tCtrl+N"), ToolImage(_T("new")), OnNewButton);
	AddMenuItem(file_menu, _("Open...\tCtrl+O"), ToolImage(_T("open")), OnOpenButton);
	m_recent_files_menu = new wxMenu;
	m_recent_files_menu->Append(-1, _T("test"));
	AddMenuItem(file_menu, _("Open Recent"), ToolImage(_T("recent")), NULL, OnUpdateOpenRecent, m_recent_files_menu);
	file_menu->AppendSeparator();
	AddMenuItem(file_menu, _("Save\tCtrl+S"), ToolImage(_T("save")), OnSaveButton, OnUpdateSave);
	AddMenuItem(file_menu, _("Save As..."), ToolImage(_T("saveas")), OnSaveAsButton);
	file_menu->AppendSeparator();
	AddMenuItem(file_menu, _("Import..."), ToolImage(_T("import")), OnImportButton);
	file_menu->AppendSeparator();
	AddMenuItem(file_menu, _("Print...\tCtrl+P"), ToolImage(_T("print")), OnPrint);
	AddMenuItem(file_menu, _("Page Setup..."), ToolImage(_T("psetup")), OnPageSetup);
	AddMenuItem(file_menu, _("Print Preview"), ToolImage(_T("ppreview")), OnPrintPreview);
	file_menu->AppendSeparator();
	AddMenuItem(file_menu, _("Plugins..."), ToolImage(_T("plugin")), OnPlugins);
	file_menu->AppendSeparator();
	//AddMenuItem(file_menu, _("Save Settings"), ToolImage(_T("save")), OnSaveSettingsButton);
	//file_menu->AppendSeparator();
	AddMenuItem(file_menu, _("Restore All Defaults"), ToolImage(_T("restore")), OnResetDefaultsButton);
	file_menu->AppendSeparator();
	AddMenuItem(file_menu, _("Exit\tCtrl+Q"), ToolImage(_T("exit")), OnQuit);

	// Edit Menu
	wxMenu *edit_menu = new wxMenu;
	AddMenuItem(edit_menu, _("Undo\tCtrl+Z"), ToolImage(_T("undo")), OnUndoButton, OnUpdateUndo);
	AddMenuItem(edit_menu, _("Redo\tCtrl+Shift+Z"), ToolImage(_T("redo")), OnRedoButton, OnUpdateRedo);
	edit_menu->AppendSeparator();
	AddMenuItem(edit_menu, _("Cut\tCtrl+X"), ToolImage(_T("cut")), OnCutButton, OnUpdateCut);
	AddMenuItem(edit_menu, _("Copy\tCtrl+C"), ToolImage(_T("copy")), OnCopyButton, OnUpdateCopy);
	AddMenuItem(edit_menu, _("Paste\tCtrl+V"), ToolImage(_T("paste")), OnPasteButton, OnUpdatePaste);
	AddMenuItem(edit_menu, _("Delete"), ToolImage(_T("delete")), OnDeleteButton, OnUpdateDelete);
	edit_menu->AppendSeparator();
	AddMenuItem(edit_menu, _("Select Mode"), ToolImage(_T("select")), OnSelectModeButton);

	// Geometry Menu
	wxMenu *geometry_menu = new wxMenu;
	AddMenuItem(geometry_menu, _("Draw a sketch"), ToolImage(_T("lines")), OnLinesButton);
	AddMenuItem(geometry_menu, _("Draw Rectangles"), ToolImage(_T("rect")), OnRectanglesButton);
	AddMenuItem(geometry_menu, _("Draw Obrounds"), ToolImage(_T("obround")), OnObroundsButton);
	AddMenuItem(geometry_menu, _("Draw Polygons"), ToolImage(_T("pentagon")), OnPolygonsButton);
	AddMenuItem(geometry_menu, _("Draw Ellipses"), ToolImage(_T("circles")), OnEllipseButton);
	AddMenuItem(geometry_menu, _("Draw Infinite Lines"), ToolImage(_T("iline")), OnILineButton);
	AddMenuItem(geometry_menu, _("Draw Points"), ToolImage(_T("point")), OnPointsButton);
	geometry_menu->AppendSeparator();
	AddMenuItem(geometry_menu, _("Spline Through Points"), ToolImage(_T("splpts")), OnSplinePointsButton);
	geometry_menu->AppendSeparator();
	AddMenuItem(geometry_menu, _("Add Text"), ToolImage(_T("text")), OnTextButton);
	AddMenuItem(geometry_menu, _("Add Dimension"), ToolImage(_T("dimension")), OnDimensioningButton);
	AddMenuItem(geometry_menu, _("Add Gear"), ToolImage(_T("gear")), OnGearButton);

	wxMenu *coordinate_menu = new wxMenu;
	//AddMenuItem(coordinate_menu, _("Add Coordinate System"), ToolImage(_T("coordsys")), coordinate_menu);

	AddMenuItem(coordinate_menu, _("Pick 3 points"), ToolImage(_T("coordsys")), OnCoordinateSystem);
	//coordinate_menu->AppendSeparator();
	AddMenuItem(coordinate_menu, _("Pick 1 point"), ToolImage(_T("coordsys")), OnNewOrigin);

	// View Menu
	wxMenu *view_menu = new wxMenu;
	AddMenuItem(view_menu, _("Previous view"), ToolImage(_T("magprev")), OnMagPreviousButton);
	view_menu->AppendSeparator();
	AddMenuItem(view_menu, _("Zoom window"), ToolImage(_T("mag")), OnMagButton);
	AddMenuItem(view_menu, _("Fit view to extents"), ToolImage(_T("magextents")), OnMagExtentsButton);
	AddMenuItem(view_menu, _("Fit view to extents, but no rotation"), ToolImage(_T("magnorot")), OnMagNoRotButton);
	AddMenuItem(view_menu, _("View XY Front"), ToolImage(_T("magxy")),       OnMagXYButton );
	AddMenuItem(view_menu, _("View XY Back"), ToolImage(_T("magxym")),       OnMagXYMButton);
	AddMenuItem(view_menu, _("View XZ Top"), ToolImage(_T("magxz")),         OnMagXZButton );
	AddMenuItem(view_menu, _("View XZ Bottom"), ToolImage(_T("magxzm")),     OnMagXZMButton);
	AddMenuItem(view_menu, _("View YZ Right"), ToolImage(_T("magyz")),       OnMagYZButton );
	AddMenuItem(view_menu, _("View YZ Left"), ToolImage(_T("magyzm")),       OnMagYZMButton);
	AddMenuItem(view_menu, _("View XZY Isometric"), ToolImage(_T("magxyz")), OnMagXYZButton);
	view_menu->AppendSeparator();
	AddMenuItem(view_menu, _("View rotate"), ToolImage(_T("viewrot")), OnViewRotateButton);
	AddMenuItem(view_menu, _("View zoom"), ToolImage(_T("zoom")), OnViewZoomButton);
	AddMenuItem(view_menu, _("View pan"), ToolImage(_T("pan")), OnViewPanButton);
	AddMenuItem(view_menu, _("Full screen"), ToolImage(_T("fullscreen")), OnFullScreenButton);
	view_menu->AppendSeparator();
	AddMenuItem(view_menu, _("Redraw"), ToolImage(_T("redraw")), OnRedrawButton);

	// Solids Menu
	wxMenu *solids_menu = new wxMenu;
	AddMenuItem(solids_menu, _("Add a sphere"), ToolImage(_T("sphere")), OnSphereButton);
	AddMenuItem(solids_menu, _("Add a cube"), ToolImage(_T("cube")), OnCubeButton);
	AddMenuItem(solids_menu, _("Add a cylinder"), ToolImage(_T("cyl")), OnCylButton);
	AddMenuItem(solids_menu, _("Add a cone"), ToolImage(_T("cone")), OnConeButton);
	solids_menu->AppendSeparator();
	AddMenuItem(solids_menu, _("Loft two sketches"), ToolImage(_T("ruled")), OnRuledSurfaceButton);
	AddMenuItem(solids_menu, _("Extrude a sketch"), ToolImage(_T("extrude")), OnExtrudeButton);
	AddMenuItem(solids_menu, _("Revolve a sketch"), ToolImage(_T("revolve")), OnRevolveButton);
	AddMenuItem(solids_menu, _("Sweep objects along a sketch"), ToolImage(_T("sweep")), OnSweepButton);
	solids_menu->AppendSeparator();
	AddMenuItem(solids_menu, _("Cut"), ToolImage(_T("subtract")), OnSubtractButton);
	AddMenuItem(solids_menu, _("Fuse"), ToolImage(_T("fuse")), OnFuseButton);
	AddMenuItem(solids_menu, _("Common"), ToolImage(_T("common")), OnCommonButton);
	solids_menu->AppendSeparator();
	AddMenuItem(solids_menu, _("Fillet"), ToolImage(_T("fillet")), OnFilletButton);
	AddMenuItem(solids_menu, _("Chamfer"), ToolImage(_T("chamfer")), OnChamferButton);
	AddMenuItem(solids_menu, _("Sectioning"), ToolImage(_T("section")), OnSectioningButton);

	// Transformations Menu
	wxMenu *transform_menu = new wxMenu;
	AddMenuItem(transform_menu, _("Move Translate"), ToolImage(_T("movet")), OnMoveTranslateButton);
	AddMenuItem(transform_menu, _("Copy Translate"), ToolImage(_T("copyt")), OnCopyTranslateButton);
	transform_menu->AppendSeparator();
	AddMenuItem(transform_menu, _("Move Rotate"), ToolImage(_T("mover")), OnMoveRotateButton);
	AddMenuItem(transform_menu, _("Copy Rotate"), ToolImage(_T("copyr")), OnCopyRotateButton);
	transform_menu->AppendSeparator();
	AddMenuItem(transform_menu, _("Move Mirror"), ToolImage(_T("movem")), OnMoveMirrorButton);
	AddMenuItem(transform_menu, _("Copy Mirror"), ToolImage(_T("copym")), OnCopyMirrorButton);
	transform_menu->AppendSeparator();
	AddMenuItem(transform_menu, _("Move Scale"), ToolImage(_T("moves")), OnMoveScaleButton);

	// Window Menu
	m_menuWindow = new wxMenu;
	m_objects_menu_id = AddMenuItem(m_menuWindow, _("Objects"), wxBitmap(), OnViewObjects, OnUpdateViewObjects, NULL, true);
	m_options_menu_id = AddMenuItem(m_menuWindow, _("Options"), wxBitmap(), OnViewOptions, OnUpdateViewOptions, NULL, true);
	m_input_menu_id = AddMenuItem(m_menuWindow, _("Input"), wxBitmap(), OnViewInput, OnUpdateViewInput, NULL, true);
	m_properties_menu_id = AddMenuItem(m_menuWindow, _("Properties"), wxBitmap(), OnViewProperties, OnUpdateViewProperties, NULL, true);
	m_menuWindow->AppendSeparator();
	m_main_toolbar_menu_id = AddMenuItem(m_menuWindow, _("Standard Tool Bar"), wxBitmap(), OnViewToolBar, OnUpdateViewToolBar, NULL, true);
	m_solids_toolbar_menu_id = AddMenuItem(m_menuWindow, _("Solids Tool Bar"), wxBitmap(), OnViewSolidBar, OnUpdateViewSolidBar, NULL, true);
	m_geometry_toolbar_menu_id = AddMenuItem(m_menuWindow, _("Geometry Tool Bar"), wxBitmap(), OnViewGeometryBar, OnUpdateViewGeometryBar, NULL, true);
	m_viewing_toolbar_menu_id = AddMenuItem(m_menuWindow, _("Viewing Tool Bar"), wxBitmap(), OnViewViewingBar, OnUpdateViewViewingBar, NULL, true);

	// Help Menu
	m_menuHelp = new wxMenu;
	AddMenuItem(m_menuHelp, _("About"), ToolImage(_T("about")), OnAbout);

	// Add them to the main menu
	m_menuBar = new wxMenuBar;
	m_menuBar->Append( file_menu, _( "&File" ) );
	m_menuBar->Append( edit_menu, _( "&Edit" ) );
	m_menuBar->Append( view_menu, _( "&View" ) );
	m_menuBar->Append( geometry_menu, _( "&Geometry" ) );
	m_menuBar->Append( solids_menu, _( "&Solid" ) );
	m_menuBar->Append( coordinate_menu, _( "&Set Origin" ) );
	m_menuBar->Append( transform_menu, _( "&Transform" ) );
	m_menuBar->Append( m_menuWindow, _( "&Window" ) );
	m_menuBar->Append( m_menuHelp, _( "&Help" ) );

	SetMenuBar( m_menuBar );
}

void CHeeksFrame::AddToolBar(wxToolBarBase* tb, const wxString& name, const wxString& caption)
{
	m_aui_manager->AddPane(tb, wxAuiPaneInfo().Name(name).Caption(caption).ToolbarPane().Top().Position(m_toolbars_last_position++));
}

void CHeeksFrame::AddToolBars()
{
	m_toolbars_last_position = 0;
	if(!m_main_toolbar_removed)m_toolBar = new wxToolBar(this, -1, wxDefaultPosition, wxDefaultSize, wxTB_NODIVIDER | wxTB_FLAT);
	if(!m_geometry_toolbar_removed)m_geometryBar = new wxToolBar(this, -1, wxDefaultPosition, wxDefaultSize, wxTB_NODIVIDER | wxTB_FLAT);
	if(!m_solid_toolbar_removed)m_solidBar = new wxToolBar(this, -1, wxDefaultPosition, wxDefaultSize, wxTB_NODIVIDER | wxTB_FLAT);
	if(!m_viewing_toolbar_removed)m_viewingBar = new wxToolBar(this, -1, wxDefaultPosition, wxDefaultSize, wxTB_NODIVIDER | wxTB_FLAT);

	SetToolBarsSize();

	if(!m_main_toolbar_removed)
	{
		if(!wxGetApp().m_no_creation_mode)AddToolBarTool(m_toolBar, _T("New"), ToolImage(_T("new")), _("New file"), OnNewButton);
		AddToolBarTool(m_toolBar, _T("Open"), ToolImage(_T("open")), _("Open file"), OnOpenButton);
		if(!wxGetApp().m_no_creation_mode)AddToolBarTool(m_toolBar, _T("Save"), ToolImage(_T("save")), _("Save file"), OnSaveButton, OnUpdateSave);
		m_toolBar->AddSeparator();
		if(!wxGetApp().m_no_creation_mode)
		{
			AddToolBarTool(m_toolBar, _T("Cut"), ToolImage(_T("cut")), _("Cut selected items to the clipboard"), OnCutButton, OnUpdateCut);
			AddToolBarTool(m_toolBar, _T("Copy"), ToolImage(_T("copy")), _("Copy selected items to the clipboard"), OnCopyButton, OnUpdateCopy);
			AddToolBarTool(m_toolBar, _T("Paste"), ToolImage(_T("paste")), _("Paste items from the clipboard"), OnPasteButton, OnUpdatePaste);
			m_toolBar->AddSeparator();
		}
		AddToolBarTool(m_toolBar, _T("Undo"), ToolImage(_T("undo")), _("Undo the previous command"), OnUndoButton, OnUpdateUndo);
		AddToolBarTool(m_toolBar, _T("Redo"), ToolImage(_T("redo")), _("Redo the next command"), OnRedoButton, OnUpdateRedo);
		m_toolBar->AddSeparator();
		AddToolBarTool(m_toolBar, _T("Select"), ToolImage(_T("select")), _("Select Mode"), OnSelectModeButton);

		m_toolBar->Realize();
		AddToolBar(m_toolBar, _T("ToolBar"), _("General Tools"));
		wxGetApp().RegisterHideableWindow(m_toolBar);
	}

	if(!m_geometry_toolbar_removed)
	{
		{
			CFlyOutList flyout_list(_T("Sketches"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Lines"), ToolImage(_T("lines")), _("Draw a sketch"), OnLinesButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Rectangles"), ToolImage(_T("rect")), _("Start drawing rectangles"), OnRectanglesButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Obrounds"), ToolImage(_T("obround")), _("Start drawing obrounds"), OnObroundsButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Polygons"), ToolImage(_T("pentagon")), _("Start drawing polygons"), OnPolygonsButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Gear"), ToolImage(_T("gear")), _("Add a gear"), OnGearButton));
			AddToolBarFlyout(m_geometryBar, flyout_list);
		}

		{
			CFlyOutList flyout_list(_T("Circles"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("circ3p"), ToolImage(_T("circ3p")), _("Draw circles through 3 points"), OnCircles3pButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("circ2p"), ToolImage(_T("circ2p")), _("Draw circles, centre and point"), OnCircles2pButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("circpr"), ToolImage(_T("circpr")), _("Draw circles, centre and radius"), OnCirclesprButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Ellipses"), ToolImage(_T("ellipse")), _("Draw ellipses"), OnEllipseButton));
			AddToolBarFlyout(m_geometryBar, flyout_list);
		}

		{
			CFlyOutList flyout_list(_T("OtherDrawing"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("ILine"), ToolImage(_T("iline")), _("Start Drawing Infinite Lines"), OnILineButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Points"), ToolImage(_T("point")), _("Start Drawing Points"), OnPointsButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Spline"), ToolImage(_T("splpts")), _("Spline Through Points"), OnSplinePointsButton));
			AddToolBarFlyout(m_geometryBar, flyout_list);
		}

		{
			CFlyOutList flyout_list(_T("Text"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Text"), ToolImage(_T("text")), _("Add a text object"), OnTextButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Dimensioning"), ToolImage(_T("dimension")), _("Add a dimension"), OnDimensioningButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("CoordinateSys"), ToolImage(_T("coordsys")), _("Add Coordinate System"), OnCoordinateSystem));
			AddToolBarFlyout(m_geometryBar, flyout_list);
		}

		{
			CFlyOutList flyout_list(_T("Transformations"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Move Translate"), ToolImage(_T("movet")), _("Translate selected items"), OnMoveTranslateButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Copy Translate"), ToolImage(_T("copyt")), _("Copy and translate selected items"), OnCopyTranslateButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Move Rotate"), ToolImage(_T("mover")), _("Rotate selected items"), OnMoveRotateButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Copy Rotate"), ToolImage(_T("copyr")), _("Copy and rotate selected items"), OnCopyRotateButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Move Mirror"), ToolImage(_T("movem")), _("Mirror selected items"), OnMoveMirrorButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Copy Mirror"), ToolImage(_T("copym")), _("Copy and mirror selected items"), OnCopyMirrorButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Move Scale"), ToolImage(_T("moves")), _("Scale selected items"), OnMoveScaleButton));
			AddToolBarFlyout(m_geometryBar, flyout_list);
		}

		{
			CFlyOutList flyout_list(_T("Digitizing"));
			flyout_list.m_list.push_back(m_endof_button = new CFlyOutItem( _T("Endof"), ToolImage(wxGetApp().digitize_end ? _T("endof") : _T("endofgray")), _("End point"), OnEndofButton));
			flyout_list.m_list.push_back(m_inters_button = new CFlyOutItem(_T("Inters"), ToolImage(wxGetApp().digitize_inters ? _T("inters") : _T("intersgray")), _("Intersection"), OnIntersButton));
			flyout_list.m_list.push_back(m_centre_button = new CFlyOutItem(_T("Centre"), ToolImage(wxGetApp().digitize_centre ? _T("centre") : _T("centregray")), _("Centre"), OnCentreButton));
			flyout_list.m_list.push_back(m_midpoint_button = new CFlyOutItem( _T("Midpoint"), ToolImage(wxGetApp().digitize_midpoint ? _T("midpoint") : _T("midpointgray")), _("Midpoint"), OnMidpointButton));
			flyout_list.m_list.push_back(m_snap_button = new CFlyOutItem(_T("Snap"), ToolImage(wxGetApp().draw_to_grid ? _T("snap") : _T("snapgray")), _("Snap"), OnSnapButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("EditGrid"), ToolImage(_T("editgrid")), _("edit grid"), OnEditGridButton));
			AddToolBarFlyout(m_geometryBar, flyout_list, false);
		}

		m_geometryBar->Realize();
		AddToolBar(m_geometryBar, _T("GeomBar"), _("Geometry Tools"));
		wxGetApp().RegisterHideableWindow(m_geometryBar);
	}

	if(!m_solid_toolbar_removed)
	{
		{
			CFlyOutList flyout_list(_T("SolidPrimitives"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("sphere"), ToolImage(_T("sphere")), _("Add a sphere"), OnSphereButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("cube"), ToolImage(_T("cube")), _("Add a cube"), OnCubeButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("cyl"), ToolImage(_T("cyl")), _("Add a cylinder"), OnCylButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("cone"), ToolImage(_T("cone")), _("Add a cone"), OnConeButton));
			AddToolBarFlyout(m_solidBar, flyout_list);
		}

		{
			CFlyOutList flyout_list(_T("SolidMake"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Ruled Surface"), ToolImage(_T("ruled")), _("Create a lofted face"), OnRuledSurfaceButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Extrude"), ToolImage(_T("extrude")), _("Extrude a wire or face"), OnExtrudeButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Revolve"), ToolImage(_T("revolve")), _("Revolve a wire or face"), OnRevolveButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Sweep"), ToolImage(_T("sweep")), _("Sweep a wire or face"), OnSweepButton));
			AddToolBarFlyout(m_solidBar, flyout_list);
		}

		{
			CFlyOutList flyout_list(_T("SolidBooleans"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Cut"), ToolImage(_T("subtract")), _("Cut one solid from another"), OnSubtractButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Fuse"), ToolImage(_T("fuse")), _("Fuse one solid to another"), OnFuseButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Common"), ToolImage(_T("common")), _("Find common solid between two solids"), OnCommonButton));
			AddToolBarFlyout(m_solidBar, flyout_list);
		}

		{
			CFlyOutList flyout_list(_T("SolidChamfers"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Fillet"), ToolImage(_T("fillet")), _("Make a fillet on selected edges"), OnFilletButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Chamfer"), ToolImage(_T("chamfer")), _("Make a chamfer on selected edges"), OnChamferButton));
			AddToolBarFlyout(m_solidBar, flyout_list);
		}

		m_solidBar->Realize();
		AddToolBar(m_solidBar, _T("SolidBar"), _("Solid Tools"));
		wxGetApp().RegisterHideableWindow(m_solidBar);
	}

	if(!m_viewing_toolbar_removed)
	{
		{
			CFlyOutList flyout_list(_T("ViewMag"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Mag Extents"), ToolImage(_T("magextents")), _("Zoom in to fit the extents of the drawing into the graphics window"), OnMagExtentsButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Mag No Rotation"), ToolImage(_T("magnorot")), _("Zoom in to fit the extents of the drawing into the graphics window, but without rotating the view"), OnMagNoRotButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("Zoom Window"), ToolImage(_T("mag")), _("Zoom in to a dragged window"), OnMagButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("View Back"), ToolImage(_T("magprev")), _("Go back to previous view"), OnMagPreviousButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("FullScreen"), ToolImage(_T("fullscreen")), _("Switch to full screen view ( press escape to return )"), OnFullScreenButton));
			AddToolBarFlyout(m_viewingBar, flyout_list);
		}

		{
			CFlyOutList flyout_list(_T("ViewSpecific"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("View XY Front"), ToolImage(_T("magxy")), _("View XY Front"), OnMagXYButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("View XY Back"), ToolImage(_T("magxym")), _("View XY Back"), OnMagXYMButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("View XZ Top"), ToolImage(_T("magxz")), _("View XZ Top"), OnMagXZButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("View XZ Bottom"), ToolImage(_T("magxzm")), _("View XZ Bottom"), OnMagXZMButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("View YZ Right"), ToolImage(_T("magyz")), _("View YZ Right"), OnMagYZButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("View YZ Left"), ToolImage(_T("magyzm")), _("View YZ Left"), OnMagYZMButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("View XY Isometric"), ToolImage(_T("magxyz")), _("View XY Isometric"), OnMagXYZButton));
			AddToolBarFlyout(m_viewingBar, flyout_list);
		}

		{
			CFlyOutList flyout_list(_T("ViewMag"));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("View Rotate"), ToolImage(_T("viewrot")), _("Enter view rotating mode"), OnViewRotateButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("View Zoom"), ToolImage(_T("zoom")), _("Drag to zoom in and out"), OnViewZoomButton));
			flyout_list.m_list.push_back(new CFlyOutItem(_T("View Pan"), ToolImage(_T("pan")), _("Drag to move view"), OnViewPanButton));
			AddToolBarFlyout(m_viewingBar, flyout_list);
		}

		AddToolBarTool(m_viewingBar, _T("Redraw"), ToolImage(_T("redraw")), _("Redraw"), OnRedrawButton);

		m_viewingBar->Realize();
		AddToolBar(m_viewingBar, _T("ViewingBar"), _("Viewing Tools"));
		wxGetApp().RegisterHideableWindow(m_viewingBar);
	}

	// Process AddToolBars() for each plugin
	for(std::list< void(*)() >::iterator It = wxGetApp().m_AddToolBars_list.begin(); It != wxGetApp().m_AddToolBars_list.end(); It++)
	{
		void(*callbackfunc)() = *It;
		(*callbackfunc)();
	}
}
#endif

void CHeeksFrame::LoadPerspective(const wxString& str)
{
	m_aui_manager->LoadPerspective(str);

	// translate the window captions
	m_aui_manager->GetPane(m_graphics).Caption(_("Graphics"));
	m_aui_manager->GetPane(m_tree_canvas).Caption(_("Objects"));
	m_aui_manager->GetPane(m_options).Caption(_("Options"));
	m_aui_manager->GetPane(m_input_canvas).Caption(_("Input"));
	m_aui_manager->GetPane(m_properties).Caption(_("Properties"));
	m_aui_manager->GetPane(m_toolBar).Caption(_("General Tools"));
	m_aui_manager->GetPane(m_geometryBar).Caption(_("Geometry Tools"));
	m_aui_manager->GetPane(m_solidBar).Caption(_("Solid Tools"));
	m_aui_manager->GetPane(m_viewingBar).Caption(_("Viewing Tools"));

#ifndef USING_RIBBON
	SetToolBarsSize();
#endif
}

void CHeeksFrame::SetToolBarsToLeft()
{
	m_aui_manager->GetPane(m_toolBar).Left();
	m_aui_manager->GetPane(m_geometryBar).Left();
	m_aui_manager->GetPane(m_solidBar).Left();
	m_aui_manager->GetPane(m_viewingBar).Left();

	for(std::list< wxToolBarBase* >::iterator It = wxGetApp().m_external_toolbars.begin(); It != wxGetApp().m_external_toolbars.end(); It++)
	{
		wxToolBarBase* toolbar = *It;
		m_aui_manager->GetPane(toolbar).Left();
	}
}

CFlyOutItem::CFlyOutItem(const wxString& title, const wxBitmap& bitmap, const wxString& tooltip, void(*onButtonFunction)(wxCommandEvent&)):m_title(title), m_bitmap(bitmap), m_tooltip(tooltip), m_onButtonFunction(onButtonFunction)
{
}

CFlyOutList::CFlyOutList(const wxString& title):CFlyOutItem(title, wxBitmap(), _T(""), NULL)
{
}

const CFlyOutItem* CFlyOutList::GetMainItem()const
{
	// get the item to show on toolbar
	return m_list.front();
}
