import ocl
import pyocl
import camvtk
import time
import datetime
import vtk
import math

def main():  
    print ocl.revision()
    myscreen = camvtk.VTKScreen()   
    myscreen.camera.SetPosition(-15, -8, 15)
    myscreen.camera.SetFocalPoint(0,0, 0)   
    # axis arrows
    #camvtk.drawArrows(myscreen,center=(0,0,0))

    s = ocl.SphereOCTVolume()
    s.center = ocl.Point(-2.50,-0.6,0)
    s.radius = 1.1345

    # screenshot writer
    w2if = vtk.vtkWindowToImageFilter()
    w2if.SetInput(myscreen.renWin)
    lwr = vtk.vtkPNGWriter()
    lwr.SetInput( w2if.GetOutput() )
    
    cp= ocl.Point(0,0,-16)
    #depths = [3, 4, 5, 6, 7, 8]
    max_depth = 8
    root_scale = 16
    t = ocl.Octree(root_scale, max_depth, cp)
    t.init(4)
    n = 0 # the frame number
    nmax=80
    theta=0
    dtheta=0.06
    thetalift=-0.01
    s.center =  ocl.Point( 1.3*math.cos(theta),1.3*math.sin(theta),thetalift*theta)  
    mc = ocl.MarchingCubes()
    while (n<=nmax):
        print "diff...",
        t_before = time.time() 
        t.diff_negative(s)
        t_after = time.time() 
        build_time = t_after-t_before
        print "done in ", build_time," s"
        
        if n==nmax:
            t_before = time.time() 
            print "mc()...",
            tris = mc.mc_tree(t) #.mc_triangles()
            t_after = time.time() 
            mc_time = t_after-t_before
            print "done in ", mc_time," s"
            print " mc() got ", len(tris), " triangles"
            mc_surf = camvtk.STLSurf( triangleList=tris, color=camvtk.red )
            #mc_surf.SetWireframe()
            mc_surf.SetColor(camvtk.cyan)
            print " STLSurf()...",
            myscreen.addActor( mc_surf )
            print "done."
            nodes = t.get_leaf_nodes()
            allpoints=[]
            #for no in nodes:
            #    verts = no.vertices()
            #    for v in verts:
            #        allpoints.append(v)
            #oct_points = camvtk.PointCloud( allpoints )
            print " PointCloud()...",
            #myscreen.addActor( oct_points )
            print "done."
            print " render()...",
            myscreen.render()

            print "done."

            #lwr.SetFileName("frames/mc8_frame"+ ('%06d' % n)+".png")
            #myscreen.camera.Azimuth( 2 )
            #myscreen.render()
            #w2if.Modified() 
            #lwr.Write()
                
            #mc_surf.SetWireframe()
            #print "sleep...",
            #time.sleep(1.02)
            #print "done."
                
            
            if n is not nmax:
                myscreen.removeActor( mc_surf )
                #myscreen.removeActor( oct_points )
        
        # move forward
        theta = n*dtheta
        sp1 = ocl.Point(s.center)
        s.center =  ocl.Point( 1.3*math.cos(theta),1.3*math.sin(theta),thetalift*theta)  
        sp2 = ocl.Point(s.center)
        print "line from ",sp1," to ",sp2
        if n is not nmax:
            myscreen.addActor( camvtk.Line( p1=(sp1.x,sp1.y,sp1.z),p2=(sp2.x,sp2.y,sp2.z), color=camvtk.red ) )
        print "center moved to", s.center
        n=n+1
    print "All done."
    myscreen.iren.Start() 

if __name__ == "__main__":

    main()
