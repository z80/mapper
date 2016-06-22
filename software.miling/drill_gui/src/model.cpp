
#include "model.h"
#include <boost/foreach.hpp>


// Catch mouse events
class FaceSelectorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static FaceSelectorStyle * New();

    FaceSelectorStyle()
    {
    }

    ~FaceSelectorStyle()
    {
        if ( picker )
            picker->Delete();
    }

    virtual void OnLeftButtonDown()
    {
      // Get the location of the click (in window coordinates)
      int * pos = this->GetInteractor()->GetEventPosition();

      if ( !picker )
        picker = vtkSmartPointer<vtkCellPicker>::New();
      picker->SetTolerance(0.0005);

      // Pick from this location.
      picker->Pick(pos[0], pos[1], 0, this->GetDefaultRenderer());

      vtkIdType cellId = picker->GetCellId();

      if ( cellId != -1 )
      {
          vtkIdType cnt, * inds;
          Data->GetCellPoints( cellId, cnt, inds );
          if ( ( model->selectionMode == Model::FACE_SAMPLE ) ||
               ( model->selectionMode == Model::FACE_MODEL ) )
              model->faceSelectedCallback( inds );
          else if ( ( model->selectionMode == Model::EDGE_SAMPLE ) ||
                    ( model->selectionMode == Model::EDGE_MODEL ) )
              model->edgeSelectedCallback( inds );
      }
      // Forward events
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    vtkSmartPointer<vtkCellPicker> picker;
    vtkSmartPointer<vtkPolyData>   Data;
    Model * model;
};

vtkStandardNewMacro(FaceSelectorStyle);











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

    faceSelector = FaceSelectorStyle::New();
}

Model::~Model()
{
    if ( faceSelector )
        faceSelector->Delete();
}

void Model::loadModel( const std::string & fname )
{
    modelOrig.bb.clear();
    modelOrig.tris.clear();

    std::wstring   wfname( fname.begin(), fname.end() );
    ocl::STLReader reader( wfname, modelOrig );

    convertSurf(  modelOrig, model );
    prepareFaces( model, ptsM, polyDataM, mapperM );
}

void Model::loadSample( const std::string & fname )
{
    sampleOrig.bb.clear();
    sampleOrig.tris.clear();

    std::wstring wfname( fname.begin(), fname.end() );
    ocl::STLReader reader( wfname, sampleOrig );

    convertSurf(  sampleOrig, sample );
    prepareFaces( sample, ptsS, polyDataS, mapperS );
}

void Model::setModeSampleFace()
{
    selectionMode = FACE_SAMPLE;

    prepareFaces( sample, ptsS, polyDataS, mapperS );

    iren->SetInteractorStyle( faceSelector );

    faceSelector->SetDefaultRenderer( renderer );
    faceSelector->Data = polyDataS;
    faceSelector->model = this;
}

void Model::setModeSampleEdge()
{
    selectionMode = EDGE_SAMPLE;

    prepareEdges( sample, ptsS, polyDataS, mapperS );

    iren->SetInteractorStyle( faceSelector );

    faceSelector->SetDefaultRenderer( renderer );
    faceSelector->Data = polyDataS;
    faceSelector->model = this;
}

void Model::setModeModelFace()
{
    selectionMode = FACE_MODEL;

    prepareFaces( model, ptsM, polyDataM, mapperM );

    iren->SetInteractorStyle( faceSelector );

    faceSelector->SetDefaultRenderer( renderer );
    faceSelector->Data = polyDataM;
    faceSelector->model = this;
}

void Model::setModeModelEdge()
{
    selectionMode = EDGE_MODEL;

    prepareEdges( model, ptsM, polyDataM, mapperM );

    iren->SetInteractorStyle( faceSelector );

    faceSelector->SetDefaultRenderer( renderer );
    faceSelector->Data = polyDataM;
    faceSelector->model = this;
}

void Model::dropOnFace()
{
    // Use selected face.
    ocl::Triangle t = selectedFace;

    // Normal to face.
    ocl::Point n = t.n;
    double nx, ny;
    double e = std::numeric_limits<double>::epsilon();
    // XY projection.
    if ( ( fabs( n.x ) > e ) ||
         ( fabs( n.y ) > e ) )
    {
        // Normalize projection and rotate to match it with Ox.
        double l = sqrt( n.x * n.x + n.y*n.y );
        nx = n.x / l;
        ny = n.y / l;
    }
    else
    {
        // Just unit matrix.
        nx = 1.0;
        ny = 0.0;
    }
    // And now rotate around Oy on minus cosine between n and (0, 0, -1).
    // Again inverted is equal to transposed.
    double c = -n.z;
    double s = sqrt( 1.0-c*c );

    // And resultant transformation is B*A*T.
    double x = t.p[0].x;
    double y = t.p[0].y;
    double z = t.p[0].z;
    //double A[4][4];
    A[0][0] = c*nx;  A[0][1] = c*ny;  A[0][2] = s;   A[0][3] = c*(-ny*y-nx*x)-s*z;
    A[1][0] = -ny;   A[1][1] = nx;    A[1][2] = 0.0; A[1][3] = ny*x-nx*y;
    A[2][0] = -nx*s; A[2][1] = -ny*s; A[2][2] = c;   A[2][3] = -c*z-s*(-ny*y-nx*x);

    // Modify data.
    convertSurf( sampleOrig, sample );
    convertSurf( modelOrig,  model );

    // Plot data.
    switch ( selectionMode )
    {
    case FACE_SAMPLE:
        setModeSampleFace();
        break;
    case FACE_MODEL:
        setModeModelFace();
        break;
    case EDGE_SAMPLE:
        setModeSampleEdge();
        break;
    case EDGE_MODEL:
        setModeModelEdge();
        break;
    }

    renderer->GetRenderWindow()->Render();
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
            pts->InsertNextPoint( x1, x2, x3 );
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
    //vtkIdType cnt, * inds;
    //polyData->GetCellPoints( 2, cnt, inds );


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
            pts->InsertNextPoint( x1, x2, x3 );
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

void Model::convertSurfs()
{
    convertSurf( sampleOrig, sample );
    convertSurf( modelOrig,  model );
}

void Model::convertSurf( const ocl::STLSurf & from, ocl::STLSurf & to )
{
    to.tris.clear();
    to.bb.clear();
    BOOST_FOREACH( const ocl::Triangle & tfrom, from.tris )
    {
        ocl::Triangle t = tfrom;
        for ( int i=0; i<3; i++ )
        {
            ocl::Point p = t.p[i];
            convertPoint( p.x, p.y, p.z, p.x, p.y, p.z );
            t.p[i] = p;
        }
        to.tris.push_back( t );
        to.bb.addTriangle( t );
    }
}

void Model::faceSelectedCallback( vtkIdType * inds )
{
    ocl::STLSurf & s     = ( selectionMode == FACE_SAMPLE ) ? sample     : model;
    ocl::STLSurf & sOrig = ( selectionMode == FACE_SAMPLE ) ? sampleOrig : modelOrig;
    int ind = 0;
    auto j = sOrig.tris.begin();
    for ( auto i=s.tris.begin(); i!=s.tris.end(); i++ )
    {
        ocl::Triangle & t = *i;

        // Indices go in sequential order.
        if ( ind == inds[0] )
        {
            // Assign selected face.
            selectedFace = *j;

            // Visualize it.
            if ( ptsSel )
                ptsSel->Delete();
            ptsSel = vtkPoints::New();
            for ( int j=0; j<3; j++ )
            {
                ocl::Point & p = t.p[j];
                ptsSel->InsertNextPoint( p.x, p.y, p.z );
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
        j++;
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
    ocl::STLSurf & s     = ( selectionMode == EDGE_SAMPLE ) ? sample     : model;
    ocl::STLSurf & sOrig = ( selectionMode == EDGE_SAMPLE ) ? sampleOrig : modelOrig;

    int ind = 0;

    // First triangle is searched by index.
    auto iOrig = sOrig.tris.begin();
    auto kOrig = sOrig.tris.begin();
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

                // Assign edge points.
                edgeA = (*iOrig).p[j];
                edgeB = (*iOrig).p[indB];
                edgeN = (*iOrig).n;

                // Second triangle is searched by value because indices are different.
                auto kOrig = sOrig.tris.begin();
                for ( std::list<ocl::Triangle>::iterator k=s.tris.begin(); k!=s.tris.end(); k++ )
                {
                    // Don't analyze the same triangle.
                    if ( k == i )
                    {
                        kOrig++;
                        continue;
                    }
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
                                edgeN += (*kOrig).n;
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
                                edgeN += (*kOrig).n;
                                break;
                            }
                        }
                    }
                    if ( bFound )
                        break;
                    kOrig++;
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
                ptsSel->InsertNextPoint( p.x, p.y, p.z );
            }
            for ( int j=0; j<3; j++ )
            {
                ocl::Point & p = tb.p[j];
                ptsSel->InsertNextPoint( p.x, p.y, p.z );
            }
            ocl::Point n0 = (pa + pb)*0.5;

            ptsSel->InsertNextPoint( n0.x, n0.y, n0.z );
            ocl::Point n1 = (na + nb);
            n1.normalize();
            n1 = n0 + n1;
            ptsSel->InsertNextPoint( n1.x, n1.y, n1.z );

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
        iOrig++;
    }
}
























