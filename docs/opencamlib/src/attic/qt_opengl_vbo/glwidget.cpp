

#include <iostream>
#include <QObject>
#include <GL/glut.h>
#include "glwidget.h"

NeHeWidget::NeHeWidget( QWidget *parent, char *name ) 
  : QGLWidget(parent) {
    m_timer = new QTimer(this);
    m_timer->setInterval(10);
    connect( m_timer, SIGNAL(timeout()), this, SLOT(timeOutSlot()) );
    rtri = 0.0;
    rquad = 0.0;
    m_timer->start();

}


void NeHeWidget::initializeGL() {
    std::cout << "initializeGL()\n";
    glShadeModel(GL_SMOOTH);
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glLoadIdentity();
    // glEnable(GL_DEPTH_TEST);
    genVBO();  
}


void NeHeWidget::resizeGL( int width, int height ) {
    std::cout << "resizeGL(" << width << " , " << height << " )\n";
    if (height == 0)    {
       height = 1;
    }
    glViewport(0, 0, width, height); // Reset The Current Viewport
    glMatrixMode(GL_PROJECTION); // Select The Projection Matrix
    glLoadIdentity(); // Reset The Projection Matrix
    // Calculate The Aspect Ratio Of The Window
    gluPerspective(45.0f, (GLfloat)width / (GLfloat)height, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW); // Select The Modelview Matrix
    glLoadIdentity(); // Reset The Modelview Matrix
    return;
}



void NeHeWidget::paintGL()  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glLoadIdentity();
    glTranslatef(-1.5f,0.0f,-6.0f);
    glRotatef(rtri,0.0f,1.0f,0.0f);
    glColor3f(1.0f,0.0f,0.0f);
    glBegin(GL_TRIANGLES);
        glColor3f(1.0f,0.0f,0.0f);
        glVertex3f( 0.0f, 1.0f, 0.0f);
        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(-1.0f,-1.0f, 0.0f);
        glColor3f(0.0f,0.0f,1.0f);
        glVertex3f( 1.0f,-1.0f, 0.0f);
    glEnd();
    
    glLoadIdentity();
    glTranslatef(1.5f,0.0f,-6.0f);
    glRotatef(rquad,1.0f,0.0f,0.0f);
    glColor3f(0.5f,0.5f,1.0f);
    glBegin(GL_QUADS);
        glVertex3f(-1.0f, 1.0f, 0.0f);
        glVertex3f( 1.0f, 1.0f, 0.0f);
        glVertex3f( 1.0f,-1.0f, 0.0f);
        glVertex3f(-1.0f,-1.0f, 0.0f);
    glEnd();
    
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    paintVBO();
}
