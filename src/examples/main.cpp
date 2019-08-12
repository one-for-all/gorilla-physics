#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

extern Application *getApplication();

Application *app;

/// \brief Create GLUT window
void createWindow(const char *title)
{
  // Init glut with double buffering, RGBA, and depth
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  
  // Set window size in pixels
  glutInitWindowSize(640, 320);

  // Set window position from upper-left corner
  glutInitWindowPosition(0, 0);

  // Create window with title
  glutCreateWindow(title);
}

/// \brief Process background update when idle
void update()
{
  TimingData::get().update();
  app->update();
}

/// \brief Display next frame
void display()
{
  app->display();
  glFlush();
  glutSwapBuffers();
}

/// \brief Process mouse events
void mouse(int button, int state, int x, int y)
{
  app->mouse(button, state, x, y);
}

/// \brief Resize window
void reshape(int width, int height)
{
  app->resize(width, height);
}

/// \brief Process keyboard input
void keyboard(unsigned char key, int /* x */, int /* y */)
{
  app->key(key);
}

/// \brief Process mouse drag event
void motion(int x, int y)
{
  app->mouseDrag(x, y);
}

int main(int argc, char** argv)
{
  // Init application
  glutInit(&argc, argv);
  TimingData::init();

  // Create GLUT window
  app = getApplication();
  createWindow(app->getTitle());

  // Register GLUT callbacks
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutDisplayFunc(display);
  glutIdleFunc(update);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);

  // Start application
  app->initGraphics();
  glutMainLoop();

  // Clean up resources
  app->deinit();
  delete app;
  TimingData::deinit();
}