
// First include the required header files for the VTK classes we are using.
#include "vtkConeSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkCamera.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkProperty.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyleMultiTouchCamera.h"


#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkRendererCollection.h>
#include <vtkDataSetMapper.h>
#include <vtkUnstructuredGrid.h>
#include <vtkIdTypeArray.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPlaneSource.h>
#include <vtkCellPicker.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h>
#include <vtkSelectionNode.h>
#include <vtkSelection.h>
#include <vtkExtractSelection.h>
#include <vtkObjectFactory.h>



#include <string>
#include <stlsurf.hpp>
#include <stlreader.hpp>
#include <triangle.hpp>
#include <climits>

// Catch mouse events
class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
  static MouseInteractorStyle* New();

  MouseInteractorStyle()
  {
    selectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    selectedActor = vtkSmartPointer<vtkActor>::New();
  }

    virtual void OnLeftButtonDown()
    {
      // Get the location of the click (in window coordinates)
      int* pos = this->GetInteractor()->GetEventPosition();

      vtkSmartPointer<vtkCellPicker> picker =
        vtkSmartPointer<vtkCellPicker>::New();
      picker->SetTolerance(0.0005);

      // Pick from this location.
      picker->Pick(pos[0], pos[1], 0, this->GetDefaultRenderer());

      double* worldPosition = picker->GetPickPosition();
      std::cout << "Cell id is: " << picker->GetCellId() << std::endl;

      if(picker->GetCellId() != -1)
        {

        std::cout << "Pick position is: " << worldPosition[0] << " " << worldPosition[1]
                  << " " << worldPosition[2] << endl;

        vtkSmartPointer<vtkIdTypeArray> ids =
          vtkSmartPointer<vtkIdTypeArray>::New();
        ids->SetNumberOfComponents(1);
        ids->InsertNextValue(picker->GetCellId());

        vtkSmartPointer<vtkSelectionNode> selectionNode =
          vtkSmartPointer<vtkSelectionNode>::New();
        selectionNode->SetFieldType(vtkSelectionNode::CELL);
        selectionNode->SetContentType(vtkSelectionNode::INDICES);
        selectionNode->SetSelectionList(ids);

        vtkSmartPointer<vtkSelection> selection =
          vtkSmartPointer<vtkSelection>::New();
        selection->AddNode(selectionNode);

        vtkSmartPointer<vtkExtractSelection> extractSelection =
          vtkSmartPointer<vtkExtractSelection>::New();
#if VTK_MAJOR_VERSION <= 5
        extractSelection->SetInput(0, this->Data);
        extractSelection->SetInput(1, selection);
#else
        extractSelection->SetInputData(0, this->Data);
        extractSelection->SetInputData(1, selection);
#endif
        extractSelection->Update();

        // In selection
        vtkSmartPointer<vtkUnstructuredGrid> selected =
          vtkSmartPointer<vtkUnstructuredGrid>::New();
        selected->ShallowCopy(extractSelection->GetOutput());

        std::cout << "There are " << selected->GetNumberOfPoints()
                  << " points in the selection." << std::endl;
        std::cout << "There are " << selected->GetNumberOfCells()
                  << " cells in the selection." << std::endl;


#if VTK_MAJOR_VERSION <= 5
        selectedMapper->SetInputConnection(
          selected->GetProducerPort());
#else
        selectedMapper->SetInputData(selected);
#endif

        selectedActor->SetMapper(selectedMapper);
        selectedActor->GetProperty()->EdgeVisibilityOn();
        selectedActor->GetProperty()->SetEdgeColor(1,0,0);
        selectedActor->GetProperty()->SetLineWidth(3);

        this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(selectedActor);

        }
      // Forward events
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }

    vtkSmartPointer<vtkPolyData> Data;
    vtkSmartPointer<vtkDataSetMapper> selectedMapper;
    vtkSmartPointer<vtkActor> selectedActor;

};

vtkStandardNewMacro(MouseInteractorStyle);


int main()
{
    vtkRenderer *ren1= vtkRenderer::New();
    ren1->SetBackground( 0.1, 0.2, 0.4 );

    /*
    ocl::STLSurf surf;
    ocl::STLReader reader( L"./test.stl", surf );

    int sz = static_cast<int>( surf.tris.size() );
    for ( std::list<ocl::Triangle>::iterator i=surf.tris.begin(); i!=surf.tris.end(); i++ )
    {
        ocl::Triangle & t = *i;
        vtkSmartPointer<vtkPoints>   pts      = vtkPoints::New();
        vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::New();
        polyData->Allocate();
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkPolyDataMapper::New();
        mapper->SetInputData( polyData );
        vtkSmartPointer<vtkActor> actor = vtkActor::New();
        actor->SetMapper( mapper );
        actor->GetProperty()->SetColor( 0.75, 0.0, 0.0 );

        for ( int j=0; j<3; j++ )
            pts->InsertNextPoint( t.p[j].x, t.p[j].y, t.p[j].z );
        vtkIdType ids[2];
        ids[0] = 0;
        ids[1] = 1;
        polyData->InsertNextCell( VTK_LINE, 2, ids );
        ids[0] = 1;
        ids[1] = 2;
        polyData->InsertNextCell( VTK_LINE, 2, ids );
        ids[0] = 2;
        ids[1] = 0;
        polyData->InsertNextCell( VTK_LINE, 2, ids );

        polyData->SetPoints( pts );
        mapper->SetScalarRange( 0, sz-1 );

        ren1->AddActor( actor );
    }*/


    ocl::STLSurf surf2;
    ocl::STLReader reader2( L"./test.stl", surf2 );
    // Rotate using arbitrary face. For example, the very first one.
    ocl::Triangle t = *surf2.tris.begin();
    ocl::Point n = t.n;
    double nx, ny;
    // XY projection.
    if ( ( abs( n.x ) > std::numeric_limits<double>::epsilon() ) &&
         ( abs( n.y ) > std::numeric_limits<double>::epsilon() ) )
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
    double A[4][4];
    A[0][0] = c*nx;  A[0][1] = c*ny;  A[0][2] = s;   A[0][3] = c*(-ny*y-nx*x)-s*z;
    A[1][0] = -ny;   A[1][1] = nx;    A[1][2] = 0.0; A[1][3] = ny*x-nx*y;
    A[2][0] = -nx*s; A[2][1] = -ny*s; A[2][2] = c;   A[2][3] = -c*z-s*(-ny*y-nx*x);
    A[3][0] = 0.0;   A[3][1] = 0.0;   A[3][2] = 0.0; A[3][3] = 1.0;

    vtkSmartPointer<vtkPoints>   pts      = vtkPoints::New();
    vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::New();
    polyData->Allocate();
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkPolyDataMapper::New();
    mapper->SetInputData( polyData );
    vtkSmartPointer<vtkActor> actor = vtkActor::New();
    actor->SetMapper( mapper );
    actor->GetProperty()->SetColor( 0.0, 0.75, 0.0 );
    actor->GetProperty()->SetOpacity( 0.5 );
    int sz = static_cast<int>( surf2.tris.size() );
    int ind = 0;
    for ( std::list<ocl::Triangle>::iterator i=surf2.tris.begin(); i!=surf2.tris.end(); i++ )
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
            pts->InsertNextPoint( y1, y2, y3 );
        }
        vtkIdType ids[3];
        ids[0] = ind;
        ids[1] = ind+1;
        ids[2] = ind+2;
        ind += 3;
        polyData->InsertNextCell( VTK_TRIANGLE, 3, ids );
    }
    polyData->SetPoints( pts );
    mapper->SetScalarRange( 0, sz-1 );

    ren1->AddActor( actor );




    vtkRenderWindow *renWin = vtkRenderWindow::New();
    renWin->AddRenderer( ren1 );
    renWin->SetSize( 300, 300 );

    vtkSmartPointer<vtkCamera> cam = vtkCamera::New();
    cam->SetFocalPoint( 0.0, 0.0, 0.0 );
    cam->SetPosition( 150.0, 150.0, 0.0 );

    ren1->SetActiveCamera( cam );



    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
    iren->SetRenderWindow(renWin);
    //vtkInteractorStyleMultiTouchCamera *style = vtkInteractorStyleMultiTouchCamera::New();
    vtkSmartPointer<MouseInteractorStyle> style = vtkSmartPointer<MouseInteractorStyle>::New();
    iren->SetInteractorStyle(style);

    style->SetDefaultRenderer( ren1 );
    style->Data = polyData;

    //Start the event loop
    iren->Initialize();
    iren->Start();

    return 0;
}


