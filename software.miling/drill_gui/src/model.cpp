
#include "model.h"

Model::Model()
{
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
    std::wstring wfname( fname.begin(), fname.end() );
    ocl::STLReader reader( wfname, modelOrig );
}

void Model::loadSample( const std::string & fname )
{
    std::wstring wfname( fname.begin(), fname.end() );
    ocl::STLReader reader( wfname, sampleOrig );
}

void Model::setModeSampleFace()
{
}

void Model::setModeSampleEdge()
{
}

void Model::setModeModelFace()
{
}

void Model::setModeModelEdge()
{
}

void Model::dropOnFace()
{
}

void Model::alignToEdge()
{
}






