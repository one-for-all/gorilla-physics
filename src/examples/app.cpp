#include <cstring>
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

//////////////////////////////////////////////////////////////////
void Application::initGraphics()
{
  // Set background color
  glClearColor(0.9f, 0.95f, 1.0f, 1.0f);
  
  // Enable depth comparison and depth buffer update
  glEnable(GL_DEPTH_TEST);

  // Smooth shading
  glShadeModel(GL_SMOOTH);

  setView();
}

//////////////////////////////////////////////////////////////////
void Application::setView()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // Set up field of view angle, aspect ratio, near and far clipping planes
  gluPerspective(60.0, (double)width/(double)height, 1.0, 500);
  glMatrixMode(GL_MODELVIEW);
}

//////////////////////////////////////////////////////////////////
void Application::display()
{
  glClear(GL_COLOR_BUFFER_BIT);

  glBegin(GL_LINES);
  glVertex2i(1, 1);
  glVertex2i(639, 319);
  glEnd();
}

//////////////////////////////////////////////////////////////////
const char* Application::getTitle()
{
  return "Gorilla Physics Engine Base Example";
}

//////////////////////////////////////////////////////////////////
void Application::deinit()
{
}

//////////////////////////////////////////////////////////////////
void Application::update()
{
  // Mark current window as needing to be redisplayed
  glutPostRedisplay();
}

//////////////////////////////////////////////////////////////////
void Application::key(unsigned char /*key*/)
{
}

//////////////////////////////////////////////////////////////////
void Application::resize(int width, int height)
{
  if (height <= 0) height = 1;

  this->width = width;
  this->height = height;
  glViewport(0, 0, width, height);
  setView();
}

//////////////////////////////////////////////////////////////////
void Application::mouse(int /* button */, int /* state */, 
                        int /* x */, int /* y */)
{
}

//////////////////////////////////////////////////////////////////
void Application::mouseDrag(int /* x */, int /* y */)
{
}
