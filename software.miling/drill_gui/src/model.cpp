
#include "model.h"



// Catch mouse events
class FaceSelectorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static FaceSelectorStyle * New();

    FaceSelectorStyle()
    {
        selectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
        selectedActor  = vtkSmartPointer<vtkActor>::New();
    }

    virtual void OnLeftButtonDown()
    {
      // Get the location of the click (in window coordinates)
      int * pos = this->GetInteractor()->GetEventPosition();

      vtkSmartPointer<vtkCellPicker> picker = vtkSmartPointer<vtkCellPicker>::New();
      picker->SetTolerance(0.0005);

      // Pick from this location.
      picker->Pick(pos[0], pos[1], 0, this->GetDefaultRenderer());

      double * worldPosition = picker->GetPickPosition();
      std::cout << "Cell id is: " << picker->GetCellId() << std::endl;

      if ( picker->GetCellId() != -1 )
      {

        std::cout << "Pick position is: " << worldPosition[0] << " " << worldPosition[1]
                  << " " << worldPosition[2] << endl;

        vtkSmartPointer<vtkIdTypeArray> ids = vtkSmartPointer<vtkIdTypeArray>::New();
        ids->SetNumberOfComponents(1);
        ids->InsertNextValue(picker->GetCellId());

        vtkSmartPointer<vtkSelectionNode> selectionNode = vtkSmartPointer<vtkSelectionNode>::New();
        selectionNode->SetFieldType(vtkSelectionNode::CELL);
        selectionNode->SetContentType(vtkSelectionNode::INDICES);
        selectionNode->SetSelectionList(ids);

        vtkSmartPointer<vtkSelection> selection = vtkSmartPointer<vtkSelection>::New();
        selection->AddNode(selectionNode);

        vtkSmartPointer<vtkExtractSelection> extractSelection = vtkSmartPointer<vtkExtractSelection>::New();

        extractSelection->SetInputData(0, this->Data);
        extractSelection->SetInputData(1, selection);

        extractSelection->Update();

        // In selection
        vtkSmartPointer<vtkUnstructuredGrid> selected = vtkSmartPointer<vtkUnstructuredGrid>::New();
        selected->ShallowCopy(extractSelection->GetOutput());

        std::cout << "There are " << selected->GetNumberOfPoints()
                  << " points in the selection." << std::endl;
        std::cout << "There are " << selected->GetNumberOfCells()
                  << " cells in the selection." << std::endl;

        selectedMapper->SetInputData(selected);

        selectedActor->SetMapper(selectedMapper);
        selectedActor->GetProperty()->EdgeVisibilityOn();
        selectedActor->GetProperty()->SetEdgeColor( 1, 0, 0 );
        selectedActor->GetProperty()->SetLineWidth( 3 );

        this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(selectedActor);

        }
      // Forward events
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    vtkSmartPointer<vtkPolyData>      Data;
    vtkSmartPointer<vtkDataSetMapper> selectedMapper;
    vtkSmartPointer<vtkActor>         selectedActor;

};










class EdgeSelectorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static EdgeSelectorStyle* New();

    EdgeSelectorStyle()
    {
        selectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
        selectedActor = vtkSmartPointer<vtkActor>::New();
    }

    virtual void OnLeftButtonDown()
    {
      // Get the location of the click (in window coordinates)
      int * pos = this->GetInteractor()->GetEventPosition();

      vtkSmartPointer<vtkCellPicker> picker = vtkSmartPointer<vtkCellPicker>::New();
      picker->SetTolerance(0.0005);

      // Pick from this location.
      picker->Pick(pos[0], pos[1], 0, this->GetDefaultRenderer());

      double* worldPosition = picker->GetPickPosition();
      std::cout << "Cell id is: " << picker->GetCellId() << std::endl;

      if ( picker->GetCellId() != -1 )
      {

        std::cout << "Pick position is: " << worldPosition[0] << " " << worldPosition[1]
                  << " " << worldPosition[2] << endl;

        vtkSmartPointer<vtkIdTypeArray> ids = vtkSmartPointer<vtkIdTypeArray>::New();
        ids->SetNumberOfComponents(1);
        ids->InsertNextValue(picker->GetCellId());

        vtkSmartPointer<vtkSelectionNode> selectionNode = vtkSmartPointer<vtkSelectionNode>::New();
        selectionNode->SetFieldType( vtkSelectionNode::EDGE );
        selectionNode->SetContentType( vtkSelectionNode::INDICES );
        selectionNode->SetSelectionList(ids);

        vtkSmartPointer<vtkSelection> selection = vtkSmartPointer<vtkSelection>::New();
        selection->AddNode(selectionNode);

        vtkSmartPointer<vtkExtractSelection> extractSelection = vtkSmartPointer<vtkExtractSelection>::New();

        extractSelection->SetInputData(0, this->Data);
        extractSelection->SetInputData(1, selection);

        extractSelection->Update();

        // In selection
        vtkSmartPointer<vtkUnstructuredGrid> selected = vtkSmartPointer<vtkUnstructuredGrid>::New();
        selected->ShallowCopy(extractSelection->GetOutput());

        std::cout << "There are " << selected->GetNumberOfPoints()
                  << " points in the selection." << std::endl;
        std::cout << "There are " << selected->GetNumberOfCells()
                  << " cells in the selection." << std::endl;

        selectedMapper->SetInputData(selected);

        selectedActor->SetMapper(selectedMapper);
        selectedActor->GetProperty()->EdgeVisibilityOn();
        selectedActor->GetProperty()->SetEdgeColor( 1, 0, 0 );
        selectedActor->GetProperty()->SetLineWidth( 3 );

        this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(selectedActor);

        }
      // Forward events
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    vtkSmartPointer<vtkPolyData>      Data;
    vtkSmartPointer<vtkDataSetMapper> selectedMapper;
    vtkSmartPointer<vtkActor>         selectedActor;

};





vtkStandardNewMacro(FaceSelectorStyle);
vtkStandardNewMacro(EdgeSelectorStyle);











Model::Model( vtkRenderer * ren , vtkRenderWindowInteractor * iren )
{
    this->renderer = ren;
    this->iren     = iren;

    ptsM      = vtkPoints::New();
    polyDataM = vtkPolyData::New();
    polyDataM->Allocate();
    mapperM = vtkPolyDataMapper::New();
    mapperM->SetInputData( polyDataM );
    actorM = vtkActor::New();
    actorM->SetMapper( mapperM );
    actorM->GetProperty()->SetColor( 0.0, 0.75, 0.75 );

    ptsS      = vtkPoints::New();
    polyDataS = vtkPolyData::New();
    polyDataS->Allocate();
    mapperS = vtkPolyDataMapper::New();
    mapperS->SetInputData( polyDataS );
    actorS = vtkActor::New();
    actorS->SetMapper( mapperS );
    actorS->GetProperty()->SetColor( 0.0, 0.75, 0.0 );
}

Model::~Model()
{
}

void Model::loadModel( const std::string & fname )
{
    std::wstring   wfname( fname.begin(), fname.end() );
    ocl::STLReader reader( wfname, modelOrig );
}

void Model::loadSample( const std::string & fname )
{
    std::wstring wfname( fname.begin(), fname.end() );
    ocl::STLReader reader( wfname, sampleOrig );
}

void Model::setModeSampleFace()
{
    if ( faceSelector )
        faceSelector->Delete();

    faceSelector = FaceSelectorStyle::New();
    iren->SetInteractorStyle( faceSelector );

    faceSelector->SetDefaultRenderer( renderer );
    faceSelector->Data = polyDataS;
}

void Model::setModeSampleEdge()
{
    if ( edgeSelector )
        edgeSelector->Delete();

    edgeSelector = EdgeSelectorStyle::New();
    iren->SetInteractorStyle( edgeSelector );

    edgeSelector->SetDefaultRenderer( renderer );
    edgeSelector->Data = polyDataS;
}

void Model::setModeModelFace()
{
    if ( faceSelector )
        faceSelector->Delete();

    faceSelector = FaceSelectorStyle::New();
    iren->SetInteractorStyle( faceSelector );

    faceSelector->SetDefaultRenderer( renderer );
    faceSelector->Data = polyDataM;
}

void Model::setModeModelEdge()
{
    if ( edgeSelector )
        edgeSelector->Delete();

    edgeSelector = EdgeSelectorStyle::New();
    iren->SetInteractorStyle( edgeSelector );

    edgeSelector->SetDefaultRenderer( renderer );
    edgeSelector->Data = polyDataS;
}

void Model::dropOnFace()
{
}

void Model::alignToEdge()
{
}























