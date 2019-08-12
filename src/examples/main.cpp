#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

extern Application *getApplication();

Application *app;

void createWindow(const char *title)
{
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(640, 320);
  glutInitWindowPosition(0, 0);
  glutCreateWindow(title);
}

void update()
{
  TimingData::get().update();
  app->update();
}

void display()
{
  app->display();
  glFlush();
  glutSwapBuffers();
}

void mouse(int button, int state, int x, int y)
{
  app->mouse(button, state, x, y);
}

void reshape(int width, int height)
{
  app->resize(width, height);
}

void keyboard(unsigned char key, int /* x */, int /* y */)
{
  app->key(key);
}

void motion(int x, int y)
{
  app->mouseDrag(x, y);
}

int main(int argc, char** argv)
{
  glutInit(&argc, argv);
  TimingData::init();

  app = getApplication();
  createWindow(app->getTitle());

  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutDisplayFunc(display);
  glutIdleFunc(update);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);

  app->initGraphics();
  glutMainLoop();

  app->deinit();
  delete app;
  TimingData::deinit();
}