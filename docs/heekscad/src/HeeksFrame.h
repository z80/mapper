// HeeksFrame.h
// Copyright (c) 2009, Dan Heeks
// This program is released under the BSD license. See the file COPYING for details.

//#define USING_RIBBON

class CTreeCanvas;
class CGraphicsCanvas;
class CObjPropsCanvas;
class COptionsCanvas;
class CInputModeCanvas;
class HeeksPrintout;
class HeeksRibbon;

struct SExternalButtonFunctions{
	void (*on_button)(wxCommandEvent&);
	void (*on_update_button)(wxUpdateUIEvent&);
};

struct ToolIndex{
	Tool *m_tool;
	int m_index;
};

enum{
	ID_RECENT_FIRST = wxID_HIGHEST + 1,
	Menu_View_ResetLayout = ID_RECENT_FIRST + MAX_RECENT_FILES,
	Menu_View_SetToolBarsToLeft,
	ID_TREE_CTRL,
	ID_FIRST_EXTERNAL_BUTTON,
	ID_FIRST_POP_UP_MENU_TOOL = ID_FIRST_EXTERNAL_BUTTON + 1000,
	ID_NEXT_ID = ID_FIRST_POP_UP_MENU_TOOL + 1000
};

class CFlyOutToolBar{
public:
	wxToolBar* toolbar;
};

class BitmapButton2;
class CFlyOutButton;

class CFlyOutItem{
public:
	wxString m_title;
	wxBitmap m_bitmap;
	wxString m_tooltip;
	void(*m_onButtonFunction)(wxCommandEvent&);
	CFlyOutButton *flyout_button;
	BitmapButton2* button;

	CFlyOutItem(const wxString& title, const wxBitmap& bitmap, const wxString& tooltip, void(*onButtonFunction)(wxCommandEvent&));

	virtual bool IsAList(){return false;}
};

class CFlyOutList: public CFlyOutItem
{
public:
	std::list<CFlyOutItem*> m_list;

	CFlyOutList(const wxString& title);
	virtual ~CFlyOutList() {};

	// CFlyOutItem's virtual functions
	bool IsAList(){return true;}

	// Get the item that should appear on the main toolbar
	const CFlyOutItem* GetMainItem()const;
};

class CHeeksFrame : public wxFrame
{
private:
	int m_next_id_for_button;
	std::map<int, SExternalButtonFunctions > m_external_buttons;

public:
	CTreeCanvas *m_tree_canvas;
	CGraphicsCanvas* m_graphics;
	CObjPropsCanvas* m_properties;
	COptionsCanvas* m_options;
	CInputModeCanvas* m_input_canvas;
	wxAuiManager* m_aui_manager;
	wxToolBar *m_toolBar;
	wxToolBar *m_geometryBar;
	wxToolBar *m_solidBar;
	wxToolBar *m_viewingBar;
	wxToolBar *m_digitizingBar;
	wxToolBar *m_transformBar;
	wxMenuBar *m_menuBar;
	wxMenu* m_recent_files_menu;
	wxMenu *m_menuWindow;
	wxMenu *m_menuHelp;
	HeeksPrintout* m_printout;
	wxString m_extra_about_box_str;
#ifdef USING_RIBBON
	HeeksRibbon *m_ribbon;
#else
	CFlyOutItem* m_endof_button;
	CFlyOutItem* m_inters_button;
	CFlyOutItem* m_centre_button;
	CFlyOutItem* m_midpoint_button;
	CFlyOutItem* m_snap_button;
#endif

	bool m_main_toolbar_removed;
	bool m_geometry_toolbar_removed;
	bool m_solid_toolbar_removed;
	bool m_viewing_toolbar_removed;
	bool m_transform_toolbar_removed;
	int m_objects_menu_id;
	int m_options_menu_id;
	int m_input_menu_id;
	int m_properties_menu_id;
	int m_main_toolbar_menu_id;
	int m_solids_toolbar_menu_id;
	int m_geometry_toolbar_menu_id;
	int m_viewing_toolbar_menu_id;

	CHeeksFrame( const wxString& title, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize );
	virtual ~CHeeksFrame();

	void OnClose( wxCloseEvent& event );
	void OnResetLayout( wxCommandEvent& event );
	void OnSetToolBarsToLeft( wxCommandEvent& event );
	void OnExternalButton( wxCommandEvent& event );
	void OnRecentFile( wxCommandEvent& event );
	void OnUpdateExternalButton( wxUpdateUIEvent& event );
	void OnSize( wxSizeEvent& evt );
	void OnMove( wxMoveEvent& evt );
	void OnKeyDown(wxKeyEvent& event);
	void OnKeyUp(wxKeyEvent& event);
	wxToolBarToolBase* AddToolBarTool(wxToolBar* toolbar, const wxString& title, const wxBitmap& bitmap, const wxString& caption, void(*onButtonFunction)(wxCommandEvent&), void(*onUpdateButtonFunction)(wxUpdateUIEvent&) = NULL);
	void AddToolBarTool(wxToolBar* toolbar, Tool* tool);
	void AddToolBarFlyout(wxToolBar* toolbar, const CFlyOutList& flyout, bool disappears_on_click = true);
	int MakeNextIDForTool(void(*onButtonFunction)(wxCommandEvent&), void(*onUpdateButtonFunction)(wxUpdateUIEvent&));
	void SetToolFunctions(int Id, void(*onButtonFunction)(wxCommandEvent&), void(*onUpdateButtonFunction)(wxUpdateUIEvent&));
	void ClearToolBar(wxToolBar* m_toolBar);
	int AddMenuItem(wxMenu* menu, const wxString& text, const wxBitmap& bitmap, void(*onButtonFunction)(wxCommandEvent&), void(*onUpdateButtonFunction)(wxUpdateUIEvent&) = NULL, wxMenu* submenu = NULL, bool check_item = false);
	void Draw(wxDC& dc);
	void OnChangeBitmapSize();
#ifndef USING_RIBBON
	void MakeMenus();
	void AddToolBars();
	void AddToolBar(wxToolBarBase* tb, const wxString& name, const wxString& caption);
	int m_toolbars_last_position;
#endif
	void LoadPerspective(const wxString& str);
	void SetDefaultLayout(const wxString& str); // call this from dll's OnStartUp
	void SetToolBarsToLeft();
	void SetToolBarsSize();
	void RefreshInputCanvas();
	void RefreshProperties();
	void RefreshOptions();

	//wxTopLevelWindow's virtual functions
	bool ShowFullScreen(bool show, long style = wxFULLSCREEN_ALL);
	static void AddToolToListAndMenu(Tool *t, std::vector<ToolIndex> &tool_index_list, wxMenu *menu);
private:

	DECLARE_EVENT_TABLE()
};

void OnSelectModeButton(wxCommandEvent& event);
void OnOpenButton(wxCommandEvent& event);
void OnImportButton(wxCommandEvent& event);
void OnSaveButton(wxCommandEvent& event);
void OnUpdateSave(wxUpdateUIEvent& event);
void OnSaveAsButton(wxCommandEvent& event);
void OnUndoButton(wxCommandEvent& event);
void OnRedoButton(wxCommandEvent& event);
void OnUpdateUndo(wxUpdateUIEvent& event);
void OnUpdateRedo(wxUpdateUIEvent& event);
void OnNewButton(wxCommandEvent& event);
void OnCutButton(wxCommandEvent& event);
void OnUpdateCut(wxUpdateUIEvent& event);
void OnCopyButton(wxCommandEvent& event);
void OnUpdateCopy(wxUpdateUIEvent& event);
void OnPasteButton(wxCommandEvent& event);
void OnDeleteButton(wxCommandEvent& event);
void OnUpdateDelete(wxUpdateUIEvent& event);
void OnUpdatePaste(wxUpdateUIEvent& event);
void OnSubtractButton(wxCommandEvent& event);
void OnPrint(wxCommandEvent& event);
void OnPrintPreview(wxCommandEvent& event);
void OnPageSetup(wxCommandEvent& event);
void OnPlugins(wxCommandEvent& event);
void OnViewObjects(wxCommandEvent& event);
void OnUpdateViewObjects(wxUpdateUIEvent& event);
void OnViewOptions(wxCommandEvent& event);
void OnUpdateViewOptions(wxUpdateUIEvent& event);
void OnViewInput(wxCommandEvent& event);
void OnUpdateViewInput(wxUpdateUIEvent& event);
void OnViewToolBar(wxCommandEvent& event);
void OnUpdateViewToolBar(wxUpdateUIEvent& event);
void OnViewGeometryBar(wxCommandEvent& event);
void OnUpdateViewGeometryBar(wxUpdateUIEvent& event);
void OnViewSolidBar(wxCommandEvent& event);
void OnUpdateViewSolidBar(wxUpdateUIEvent& event);
void OnViewViewingBar(wxCommandEvent& event);
void OnUpdateViewViewingBar(wxUpdateUIEvent& event);
void OnViewProperties(wxCommandEvent& event);
void OnLinesButton(wxCommandEvent& event);
void OnEllipseButton(wxCommandEvent& event);
void OnPointsButton(wxCommandEvent& event);
void OnSplinePointsButton(wxCommandEvent& event);
void OnRectanglesButton(wxCommandEvent& event);
void OnObroundsButton(wxCommandEvent& event);
void OnPolygonsButton(wxCommandEvent& event);
void OnTextButton(wxCommandEvent& event);
void OnDimensioningButton(wxCommandEvent& event);
void OnCircles3pButton(wxCommandEvent& event);
void OnCircles2pButton(wxCommandEvent& event);
void OnCirclesprButton(wxCommandEvent& event);
void OnILineButton(wxCommandEvent& event);
void OnCoordinateSystem(wxCommandEvent& event);
void OnNewOrigin(wxCommandEvent& event);
void OnFuseButton(wxCommandEvent& event);
void OnCommonButton(wxCommandEvent& event);
void OnFilletButton(wxCommandEvent& event);
void OnChamferButton(wxCommandEvent& event);
void OnSectioningButton(wxCommandEvent& event);
void OnRuledSurfaceButton(wxCommandEvent& event);
void OnExtrudeButton(wxCommandEvent& event);
void OnRevolveButton(wxCommandEvent& event);
void OnSweepButton(wxCommandEvent& event);
void OnGearButton(wxCommandEvent& event);
void OnSphereButton(wxCommandEvent& event);
void OnCubeButton(wxCommandEvent& event);
void OnCylButton(wxCommandEvent& event);
void OnConeButton(wxCommandEvent& event);
void OnRedrawButton(wxCommandEvent& event);
void OnMagButton(wxCommandEvent& event);
void OnMagExtentsButton(wxCommandEvent& event);
void OnMagXYButton(wxCommandEvent& event);
void OnMagXYMButton(wxCommandEvent& event);
void OnMagXZButton(wxCommandEvent& event);
void OnMagXZMButton(wxCommandEvent& event);
void OnMagYZButton(wxCommandEvent& event);
void OnMagYZMButton(wxCommandEvent& event);
void OnMagXYZButton(wxCommandEvent& event);
void OnMagNoRotButton(wxCommandEvent& event);
void OnMagPreviousButton(wxCommandEvent& event);
void OnViewRotateButton(wxCommandEvent& event);
void OnViewZoomButton(wxCommandEvent& event);
void OnViewPanButton(wxCommandEvent& event);
void OnFullScreenButton(wxCommandEvent& event);
void OnMoveTranslateButton(wxCommandEvent& event);
void OnCopyTranslateButton(wxCommandEvent& event);
void OnMoveRotateButton(wxCommandEvent& event);
void OnCopyRotateButton(wxCommandEvent& event);
void OnMoveMirrorButton(wxCommandEvent& event);
void OnCopyMirrorButton(wxCommandEvent& event);
void OnMoveScaleButton(wxCommandEvent& event);
void OnAbout(wxCommandEvent& event);
