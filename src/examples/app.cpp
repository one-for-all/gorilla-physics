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

//////////////////////////////////////////////////////////////////
MassAggregateApplication::MassAggregateApplication(std::size_t particleCount)
: world(particleCount*10)
{
  this->particleArray = new gorilla::Particle[particleCount];
  for (std::size_t i = 0; i < particleCount; ++i)
  {
    this->world.getParticles().push_back(particleArray + i);
  }

  this->groundContactGenerator.init( &(this->world.getParticles()) );
  this->world.getContactGenerators().push_back(&this->groundContactGenerator);
}

//////////////////////////////////////////////////////////////////
MassAggregateApplication::~MassAggregateApplication()
{
  delete[] this->particleArray;
}

//////////////////////////////////////////////////////////////////
void MassAggregateApplication::initGraphics()
{
  Application::initGraphics();
}

//////////////////////////////////////////////////////////////////
void MassAggregateApplication::display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  gluLookAt(0.0, 3.5, 8.0, 0.0, 3.5, 0.0, 0.0, 1.0, 0.0);

  glColor3f(0, 0, 0);

  gorilla::ParticleWorld::Particles &particles = this->world.getParticles();

  for (auto &particle : particles)
  {
    const auto &pos = particle->getPosition();
    glPushMatrix();
    glTranslatef(pos.x(), pos.y(), pos.z());
    glutSolidSphere(0.1, 20, 10);
    glPopMatrix();
  }
}

//////////////////////////////////////////////////////////////////
void MassAggregateApplication::update()
{
  this->world.startFrame();

  double duration = TimingData::get().lastFrameDuration * 0.001;
  if (duration <= 0)
    return;

  this->world.runPhysics(duration);

  Application::update();
}