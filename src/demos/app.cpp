#include <cstring>
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

void Application::initGraphics()
{
  glClearColor(0.9f, 0.95f, 1.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);

  setView();
}

void Application::setView()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (double)width/(double)height, 1.0, 500);
  glMatrixMode(GL_MODELVIEW);
}

void Application::display()
{
  glClear(GL_COLOR_BUFFER_BIT);

  glBegin(GL_LINES);
  glVertex2i(1, 1);
  glVertex2i(639, 319);
  glEnd();
}

const char* Application::getTitle()
{
  return "Gorilla Physics Engine Demo";
}

void Application::deinit()
{
}

void Application::update()
{
  glutPostRedisplay();
}

void Application::key(unsigned char /*key*/)
{
}

void Application::resize(int width, int height)
{
  if (height <= 0) height = 1;

  Application::width = width;
  Application::height = height;
  glViewport(0, 0, width, height);
  setView();
}

void Application::mouse(int /* button */, int /* state */, 
                        int /* x */, int /* y */)
{
}

void Application::mouseDrag(int /* x */, int /* y */)
{
}

void Application::renderText(float x, float y, const char *text, void *font)
{
  glDisable(GL_DEPTH_TEST);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0.0, (double)width, 0.0, (double)height, -1.0, 1.0);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  if (font == nullptr)
    font = GLUT_BITMAP_HELVETICA_10;
  
  size_t len = strlen(text);

  glRasterPos2f(x, y);
  for (const char *letter = text; letter < text+len; ++letter)
  {
    if (*letter == '\n')
    {
      y -= 12.0f;
      glRasterPos2f(x, y);
    }
    glutBitmapCharacter(font, *letter);
  }

  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

  glEnable(GL_DEPTH_TEST);
}