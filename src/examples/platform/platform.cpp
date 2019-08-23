#include <gorilla/gorilla.h>
#include "../ogl_headers.h"
#include "../app.h"
#include "../timing.h"

#include <stdio.h>
#include <cassert>

/// \brief Mass of each particle in the platform
#define BASE_MASS 1

/// \brief Number of rods
#define ROD_COUNT 15

/// \brief platform example
class PlatformDemo : public MassAggregateApplication
{
protected:
  /// \brief Rods for the structure
  gorilla::ParticleRod* rods;

public:
  /// \brief Constructor
  PlatformDemo();

  /// \brief Destructor
  virtual ~PlatformDemo();

  /// Documentation inherited
  virtual const char* getTitle() override;

  /// Documentation inherited
  virtual void display() override;

  /// Documentation inherited
  virtual void update() override;

  /// Documentation inherited
  virtual void key(unsigned char key) override;
};

/////////////////////////////////////////////////////////////////
PlatformDemo::PlatformDemo()
: MassAggregateApplication(6), rods(nullptr)
{
  this->particleArray[0].setPosition(0, 0, 1);
  this->particleArray[1].setPosition(0, 0, -1);
  this->particleArray[2].setPosition(-3,2,1);
  this->particleArray[3].setPosition(-3,2,-1);
  this->particleArray[4].setPosition(4,2,1);
  this->particleArray[5].setPosition(4,2,-1);

  for (std::size_t i = 0; i < 6; ++i)
  {
    this->particleArray[i].setMass(BASE_MASS);
    this->particleArray[i].setVelocity(0, 0, 0);
    this->particleArray[i].setDamping(0.9);
    this->particleArray[i].setAcceleration(gorilla::GRAVITY);
    this->particleArray[i].clearAccumulator();
  }

  this->rods = new gorilla::ParticleRod[ROD_COUNT];

  rods[0].particles[0] = &particleArray[0];
  rods[0].particles[1] = &particleArray[1];
  rods[0].length = 2;
  rods[1].particles[0] = &particleArray[2];
  rods[1].particles[1] = &particleArray[3];
  rods[1].length = 2;
  rods[2].particles[0] = &particleArray[4];
  rods[2].particles[1] = &particleArray[5];
  rods[2].length = 2;

  rods[3].particles[0] = &particleArray[2];
  rods[3].particles[1] = &particleArray[4];
  rods[3].length = 7;
  rods[4].particles[0] = &particleArray[3];
  rods[4].particles[1] = &particleArray[5];
  rods[4].length = 7;

  rods[5].particles[0] = &particleArray[0];
  rods[5].particles[1] = &particleArray[2];
  rods[5].length = 3.606;
  rods[6].particles[0] = &particleArray[1];
  rods[6].particles[1] = &particleArray[3];
  rods[6].length = 3.606;

  rods[7].particles[0] = &particleArray[0];
  rods[7].particles[1] = &particleArray[4];
  rods[7].length = 4.472;
  rods[8].particles[0] = &particleArray[1];
  rods[8].particles[1] = &particleArray[5];
  rods[8].length = 4.472;

  rods[9].particles[0] = &particleArray[0];
  rods[9].particles[1] = &particleArray[3];
  rods[9].length = 4.123;
  rods[10].particles[0] = &particleArray[2];
  rods[10].particles[1] = &particleArray[5];
  rods[10].length = 7.28;
  rods[11].particles[0] = &particleArray[4];
  rods[11].particles[1] = &particleArray[1];
  rods[11].length = 4.899;
  rods[12].particles[0] = &particleArray[1];
  rods[12].particles[1] = &particleArray[2];
  rods[12].length = 4.123;
  rods[13].particles[0] = &particleArray[3];
  rods[13].particles[1] = &particleArray[4];
  rods[13].length = 7.28;
  rods[14].particles[0] = &particleArray[5];
  rods[14].particles[1] = &particleArray[0];
  rods[14].length = 4.899;

  for (std::size_t i = 0; i < ROD_COUNT; ++i)
    this->world.getContactGenerators().push_back(&rods[i]);
}

/////////////////////////////////////////////////////////////////
PlatformDemo::~PlatformDemo()
{
  if (rods != nullptr)
    delete[] rods;
}

/////////////////////////////////////////////////////////////////
void PlatformDemo::display()
{
  MassAggregateApplication::display();

  glBegin(GL_LINES);
  glColor3f(0, 0, 1);
  for (std::size_t i = 0; i < ROD_COUNT; ++i)
  {
    gorilla::Particle **particles = this->rods[i].particles;
    const auto &pos0 = particles[0]->getPosition();
    const auto &pos1 = particles[1]->getPosition();
    glVertex3f(pos0.x(), pos0.y(), pos0.z());
    glVertex3f(pos1.x(), pos1.y(), pos1.z());
  }
  glEnd();
}

/////////////////////////////////////////////////////////////////
void PlatformDemo::update()
{
  MassAggregateApplication::update();
}

/////////////////////////////////////////////////////////////////
const char* PlatformDemo::getTitle()
{
  return "Gorilla > Platform Demo";
}

/////////////////////////////////////////////////////////////////
void PlatformDemo::key(unsigned char key)
{
  MassAggregateApplication::key(key);
}

/////////////////////////////////////////////////////////////////
Application* getApplication()
{
  return new PlatformDemo();
}