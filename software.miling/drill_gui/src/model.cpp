
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

    ~FaceSelectorStyle()
    {
        if ( selectedMapper )
            selectedMapper->Delete();
        if ( selectedActor )
            selectedActor->Delete();
    }

    virtual void OnLeftButtonDown()
    {
      // Get the location of the click (in window coordinates)
      int * pos = this->GetInteractor()->GetEventPosition();

      vtkSmartPointer<vtkCellPicker> picker = vtkSmartPointer<vtkCellPicker>::New();
      picker->SetTolerance(0.0005);

      // Pick from this location.
      picker->Pick(pos[0], pos[1], 0, this->GetDefaultRenderer());

      //double * worldPosition = picker->GetPickPosition();
      //std::cout << "Cell id is: " << picker->GetCellId() << std::endl;

      if ( picker->GetCellId() != -1 )
      {
        vtkIdType cnt, * inds;
        Data->GetCellPoints( picker->GetCellId(), cnt, inds );
        model->faceSelectedCallback( inds );
        return;

        /*
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
        */
      }
      // Forward events
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    vtkSmartPointer<vtkPolyData>      Data;
    vtkSmartPointer<vtkDataSetMapper> selectedMapper;
    vtkSmartPointer<vtkActor>         selectedActor;
    Model * model;
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

    ~EdgeSelectorStyle()
    {
        if ( selectedMapper )
            selectedMapper->Delete();
        if ( selectedActor )
            selectedActor->Delete();
    }

    virtual void OnLeftButtonDown()
    {
      // Get the location of the click (in window coordinates)
      int * pos = this->GetInteractor()->GetEventPosition();

      vtkSmartPointer<vtkCellPicker> picker = vtkSmartPointer<vtkCellPicker>::New();
      picker->SetTolerance(0.005);

      // Pick from this location.
      picker->Pick(pos[0], pos[1], 0, this->GetDefaultRenderer());

      //double* worldPosition = picker->GetPickPosition();
      //std::cout << "Cell id is: " << picker->GetCellId() << std::endl;

      if ( picker->GetCellId() != -1 )
      {
        vtkIdType cnt, * inds;
        Data->GetCellPoints( picker->GetCellId(), cnt, inds );
        model->edgeSelectedCallback( inds );
        return;

        /*
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

        std::cout << "There are " << selected->GetNumberOfElements( vtkSelectionNode::EDGE )
                  << " edges in the selection." << std::endl;

        selectedMapper->SetInputData(selected);

        selectedActor->SetMapper(selectedMapper);
        selectedActor->GetProperty()->EdgeVisibilityOn();
        selectedActor->GetProperty()->SetEdgeColor( 1, 0, 0 );
        selectedActor->GetProperty()->SetLineWidth( 3 );

        this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(selectedActor);
        */
      }
      // Forward events
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    vtkSmartPointer<vtkPolyData>      Data;
    vtkSmartPointer<vtkDataSetMapper> selectedMapper;
    vtkSmartPointer<vtkActor>         selectedActor;
    Model * model;
};





vtkStandardNewMacro(FaceSelectorStyle);
vtkStandardNewMacro(EdgeSelectorStyle);











Model::Model( vtkRenderer * ren , vtkRenderWindowInteractor * iren )
{
    this->renderer = ren;
    this->iren     = iren;
    selectionMode = IDLE_MODE;

    resetMatrices();

    mapperM = vtkPolyDataMapper::New();
    actorM = vtkActor::New();
    actorM->SetMapper( mapperM );
    actorM->GetProperty()->SetColor( 0.0, 0.75, 0.75 );
    actorM->GetProperty()->SetOpacity( 0.5 );
    actorM->GetProperty()->SetLineWidth( 5 );
    renderer->AddActor( actorM );

    mapperS = vtkPolyDataMapper::New();
    actorS = vtkActor::New();
    actorS->SetMapper( mapperS );
    actorS->GetProperty()->SetColor( 0.0, 0.75, 0.0 );
    actorS->GetProperty()->SetOpacity( 0.5 );
    actorS->GetProperty()->SetLineWidth( 5 );
    renderer->AddActor( actorS );

    mapperSel = vtkPolyDataMapper::New();
    actorSel = vtkActor::New();
    actorSel->SetMapper( mapperSel );
    actorSel->GetProperty()->SetColor( 0.85, 0.0, 0.0 );
    actorSel->GetProperty()->SetLineWidth( 9 );
    renderer->AddActor( actorSel );
}

Model::~Model()
{
}

void Model::loadModel( const std::string & fname )
{
    std::wstring   wfname( fname.begin(), fname.end() );
    ocl::STLReader reader( wfname, modelOrig );

    prepareFaces( modelOrig, ptsM, polyDataM, mapperM );
}

void Model::loadSample( const std::string & fname )
{
    std::wstring wfname( fname.begin(), fname.end() );
    ocl::STLReader reader( wfname, sampleOrig );

    prepareFaces( sampleOrig, ptsS, polyDataS, mapperS );
}

void Model::setModeSampleFace()
{
    selectionMode = FACE_SAMPLE;

    if ( faceSelector )
        faceSelector->Delete();

    prepareFaces( sampleOrig, ptsS, polyDataS, mapperS );

    faceSelector = FaceSelectorStyle::New();
    iren->SetInteractorStyle( faceSelector );

    faceSelector->SetDefaultRenderer( renderer );
    faceSelector->Data = polyDataS;
    faceSelector->model = this;
}

void Model::setModeSampleEdge()
{
    selectionMode = EDGE_SAMPLE;

    if ( edgeSelector )
        edgeSelector->Delete();

    prepareEdges( sampleOrig, ptsS, polyDataS, mapperS );

    edgeSelector = EdgeSelectorStyle::New();
    iren->SetInteractorStyle( edgeSelector );

    edgeSelector->SetDefaultRenderer( renderer );
    edgeSelector->Data = polyDataS;
    edgeSelector->model = this;
}

void Model::setModeModelFace()
{
    selectionMode = FACE_MODEL;

    if ( faceSelector )
        faceSelector->Delete();

    prepareFaces( modelOrig, ptsM, polyDataM, mapperM );

    faceSelector = FaceSelectorStyle::New();
    iren->SetInteractorStyle( faceSelector );

    faceSelector->SetDefaultRenderer( renderer );
    faceSelector->Data = polyDataM;
    faceSelector->model = this;
}

void Model::setModeModelEdge()
{
    selectionMode = EDGE_MODEL;

    if ( edgeSelector )
        edgeSelector->Delete();

    prepareEdges( sampleOrig, ptsM, polyDataM, mapperM );

    edgeSelector = EdgeSelectorStyle::New();
    iren->SetInteractorStyle( edgeSelector );

    edgeSelector->SetDefaultRenderer( renderer );
    edgeSelector->Data = polyDataM;
    edgeSelector->model = this;
}

void Model::dropOnFace()
{
}

void Model::alignToEdge()
{
}

void Model::resetMatrices()
{
    for ( int i=0; i<3; i++ )
    {
        for ( int j=0; j<4; j++ )
        {
            A[i][j] = 0.0;
            B[i][j] = 0.0;
        }
        A[i][i] = 1.0;
        B[i][i] = 1.0;
    }
}

void Model::convertPoint( double x1, double x2, double x3, double & y1, double & y2, double & y3 )
{
    double a1 = A[0][0]*x1 + A[0][1]*x2 + A[0][2]*x3 + A[0][3];
    double a2 = A[1][0]*x1 + A[1][1]*x2 + A[1][2]*x3 + A[1][3];
    double a3 = A[2][0]*x1 + A[2][1]*x2 + A[2][2]*x3 + A[2][3];
    y1 = B[0][0]*a1 + B[0][1]*a2 + B[0][2]*a3 + B[0][3];
    y2 = B[1][0]*a1 + B[1][1]*a2 + B[1][2]*a3 + B[1][3];
    y3 = B[2][0]*a1 + B[2][1]*a2 + B[2][2]*a3 + B[2][3];
}

void Model::convertPoint( double * x, double * y )
{
    convertPoint( x[0], x[1], x[2], y[0], y[1], y[2] );
}

void Model::prepareFaces( ocl::STLSurf & surf, vtkSmartPointer<vtkPoints> & pts, vtkSmartPointer<vtkPolyData> & polyData, vtkSmartPointer<vtkPolyDataMapper> & mapper )
{
    if ( pts )
        pts->Delete();
    if ( polyData )
        polyData->Delete();
    pts = vtkPoints::New();
    polyData = vtkPolyData::New();
    polyData->Allocate();

    mapper->SetInputData( polyData );

    int ind = 0;
    for ( std::list<ocl::Triangle>::iterator i=surf.tris.begin(); i!=surf.tris.end(); i++ )
    {
        ocl::Triangle & t = *i;

        for ( int j=0; j<3; j++ )
        {
            double x1 = t.p[j].x;
            double x2 = t.p[j].y;
            double x3 = t.p[j].z;
            double y1 = A[0][0]*x1 + A[0][1]*x2 + A[0][2]*x3 + A[0][3];
            double y2 = A[1][0]*x1 + A[1][1]*x2 + A[1][2]*x3 + A[1][3];
            double y3 = A[2][0]*x1 + A[2][1]*x2 + A[2][2]*x3 + A[2][3];
            double z1 = B[0][0]*y1 + B[0][1]*y2 + B[0][2]*y3 + B[0][3];
            double z2 = B[1][0]*y1 + B[1][1]*y2 + B[1][2]*y3 + B[1][3];
            double z3 = B[2][0]*y1 + B[2][1]*y2 + B[2][2]*y3 + B[2][3];
            pts->InsertNextPoint( z1, z2, z3 );
        }
        vtkIdType ids[3];
        ids[0] = ind;
        ids[1] = ind+1;
        ids[2] = ind+2;
        ind += 3;
        polyData->InsertNextCell( VTK_TRIANGLE, 3, ids );
    }
    polyData->SetPoints( pts );
    polyData->BuildCells();
    polyData->BuildLinks();
    //mapper->SetScalarRange( 0, sz-1 );
    vtkIdType cnt, * inds;
    polyData->GetCellPoints( 2, cnt, inds );


    renderer->GetRenderWindow()->Render();
}

void Model::prepareEdges( ocl::STLSurf & surf, vtkSmartPointer<vtkPoints> & pts, vtkSmartPointer<vtkPolyData> & polyData, vtkSmartPointer<vtkPolyDataMapper> & mapper )
{
    if ( pts )
        pts->Delete();
    if ( polyData )
        polyData->Delete();
    pts = vtkPoints::New();
    polyData = vtkPolyData::New();

    polyData->Allocate();
    mapper->SetInputData( polyData );

    int ind = 0;
    for ( std::list<ocl::Triangle>::iterator i=surf.tris.begin(); i!=surf.tris.end(); i++ )
    {
        ocl::Triangle & t = *i;

        for ( int j=0; j<3; j++ )
        {
            double x1 = t.p[j].x;
            double x2 = t.p[j].y;
            double x3 = t.p[j].z;
            double y1 = A[0][0]*x1 + A[0][1]*x2 + A[0][2]*x3 + A[0][3];
            double y2 = A[1][0]*x1 + A[1][1]*x2 + A[1][2]*x3 + A[1][3];
            double y3 = A[2][0]*x1 + A[2][1]*x2 + A[2][2]*x3 + A[2][3];
            double z1 = B[0][0]*y1 + B[0][1]*y2 + B[0][2]*y3 + B[0][3];
            double z2 = B[1][0]*y1 + B[1][1]*y2 + B[1][2]*y3 + B[1][3];
            double z3 = B[2][0]*y1 + B[2][1]*y2 + B[2][2]*y3 + B[2][3];
            pts->InsertNextPoint( z1, z2, z3 );
        }
        vtkIdType ids[2];
        ids[0] = ind;
        ids[1] = ind+1;
        polyData->InsertNextCell( VTK_LINE, 2, ids );
        ids[0] = ind+1;
        ids[1] = ind+2;
        polyData->InsertNextCell( VTK_LINE, 2, ids );
        ids[0] = ind+2;
        ids[1] = ind;
        polyData->InsertNextCell( VTK_LINE, 2, ids );
        ind += 3;
    }
    polyData->SetPoints( pts );
    polyData->BuildCells();
    polyData->BuildLinks();

    renderer->GetRenderWindow()->Render();
}

void Model::faceSelectedCallback( vtkIdType * inds )
{
    ocl::STLSurf & s = ( selectionMode == FACE_SAMPLE ) ? sampleOrig : modelOrig;
    int ind = 0;
    for ( std::list<ocl::Triangle>::iterator i=s.tris.begin(); i!=s.tris.end(); i++ )
    {
        ocl::Triangle & t = *i;

        // Indices go in sequential order.
        if ( ind == inds[0] )
        {
            if ( ptsSel )
                ptsSel->Delete();
            ptsSel = vtkPoints::New();
            for ( int j=0; j<3; j++ )
            {
                ocl::Point & p = t.p[j];
                double y[3];
                convertPoint( p.x, p.y, p.z, y[0], y[1], y[2] );
                ptsSel->InsertNextPoint( y[0], y[1], y[2] );
            }

            if ( polyDataSel )
                polyDataSel->Delete();
            polyDataSel = vtkPolyData::New();
            polyDataSel->Allocate();
            vtkIdType ids[2];
            ids[0] = 0;
            ids[1] = 1;
            polyDataSel->InsertNextCell( VTK_LINE, 2, ids );
            ids[0] = 1;
            ids[1] = 2;
            polyDataSel->InsertNextCell( VTK_LINE, 2, ids );
            ids[0] = 2;
            ids[1] = 0;
            polyDataSel->InsertNextCell( VTK_LINE, 2, ids );
            polyDataSel->SetPoints( ptsSel );
            mapperSel->SetInputData( polyDataSel );
            renderer->GetRenderWindow()->Render();
            break;
        }
        ind += 3;
    }
}

void Model::edgeSelectedCallback( vtkIdType * inds )
{
    const double MARGIN = 0.001;
    // There should be at least two triangles.
    ocl::Triangle ta, tb;
    ocl::Point    pa, pb; // Points on the edge.
    ocl::Point    na, nb; // Normals for faces containing this edge.
    bool bFound = false;
    ocl::STLSurf & s = ( selectionMode == EDGE_SAMPLE ) ? sampleOrig : modelOrig;

    int ind = 0;

    // First triangle is searched by index.
    for ( std::list<ocl::Triangle>::iterator i=s.tris.begin(); i!=s.tris.end(); i++ )
    {
        ocl::Triangle & t = *i;

        for ( int j=0; j<3; j++ )
        {
            int indA = ind + j;
            if ( indA == inds[0] )
            {
                int indB = (j + 1) % 3;
                pa = t.p[j];
                pb = t.p[indB];
                na = t.n;
                ta = t;
                //i++; // Increase pointer by one.
                // Second triangle is seearched by value because indices are different.
                for ( std::list<ocl::Triangle>::iterator k=s.tris.begin(); k!=s.tris.end(); k++ )
                {
                    // Don't analyze the same triangle.
                    if ( k == i )
                        continue;
                    ocl::Triangle & t = *k;
                    for ( int s=0; s<3; s++ )
                    {
                        int i1 = s;
                        ocl::Point & p1 = t.p[i1];
                        double d1 = (p1-pa).norm();
                        // Point is considered to be the same if distance is not greater then MARGIN.
                        if ( d1 <= MARGIN )
                        {
                            // Other two points are candidates.
                            int i2 = (i1+1)%3;
                            ocl::Point & p2 = t.p[i2];
                            double d2 = (p2-pb).norm();
                            if ( d2 <= MARGIN )
                            {
                                bFound = true;
                                nb = t.n;
                                tb = t;
                                break;
                            }
                            // Check the last point of triangle.
                            i2 = (i2+1)%3;
                            ocl::Point & p3 = t.p[i2];
                            d2 = (p3-pb).norm();
                            if ( d2 <= MARGIN )
                            {
                                bFound = true;
                                nb = t.n;
                                tb = t;
                                break;
                            }
                        }
                    }
                    if ( bFound )
                        break;
                }
                break;
            }

        }

        // Render both triangles and resultant normal.
        if ( bFound )
        {
            if ( ptsSel )
                ptsSel->Delete();
            ptsSel = vtkPoints::New();

            for ( int j=0; j<3; j++ )
            {
                ocl::Point & p = ta.p[j];
                double y[3];
                convertPoint( p.x, p.y, p.z, y[0], y[1], y[2] );
                ptsSel->InsertNextPoint( y[0], y[1], y[2] );
            }
            for ( int j=0; j<3; j++ )
            {
                ocl::Point & p = tb.p[j];
                double y[3];
                convertPoint( p.x, p.y, p.z, y[0], y[1], y[2] );
                ptsSel->InsertNextPoint( y[0], y[1], y[2] );
            }
            double y[3];
            convertPoint( pa.x, pa.y, pa.z, y[0], y[1], y[2] );
            //ptsSel->InsertNextPoint( y[0], y[1], y[2] );
            convertPoint( pb.x, pb.y, pb.z, y[0], y[1], y[2] );
            //ptsSel->InsertNextPoint( y[0], y[1], y[2] );
            ocl::Point n0 = (pa + pb)*0.5;
            convertPoint( n0.x, n0.y, n0.z, y[0], y[1], y[2] );
            ptsSel->InsertNextPoint( y[0], y[1], y[2] );
            ocl::Point n1 = (na + nb);
            n1.normalize();
            n1 = n0 + n1;
            convertPoint( n1.x, n1.y, n1.z, y[0], y[1], y[2] );
            ptsSel->InsertNextPoint( y[0], y[1], y[2] );

            if ( polyDataSel )
                polyDataSel->Delete();
            polyDataSel = vtkPolyData::New();
            polyDataSel->Allocate();

            vtkIdType ids[2];
            ids[0] = 0;
            ids[1] = 1;
            polyDataSel->InsertNextCell( VTK_LINE, 2, ids );
            ids[0] = 1;
            ids[1] = 2;
            polyDataSel->InsertNextCell( VTK_LINE, 2, ids );
            ids[0] = 2;
            ids[1] = 0;
            polyDataSel->InsertNextCell( VTK_LINE, 2, ids );

            ids[0] = 3;
            ids[1] = 4;
            polyDataSel->InsertNextCell( VTK_LINE, 2, ids );
            ids[0] = 4;
            ids[1] = 5;
            polyDataSel->InsertNextCell( VTK_LINE, 2, ids );
            ids[0] = 5;
            ids[1] = 3;
            polyDataSel->InsertNextCell( VTK_LINE, 2, ids );

            ids[0] = 6;
            ids[1] = 7;
            polyDataSel->InsertNextCell( VTK_LINE, 2, ids );

            polyDataSel->SetPoints( ptsSel );
            mapperSel->SetInputData( polyDataSel );
            renderer->GetRenderWindow()->Render();

            return;
        }

        ind += 3;
    }
}
























