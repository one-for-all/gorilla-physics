#include <gorilla/gorilla.h>
#include "../ogl_headers.h"
#include "../app.h"
#include "../timing.h"

#include <stdio.h>

static gorilla::Random grandom;

/// \brief Firework particle
class Firework : public gorilla::Particle
{
public:
  /// \brief Type of firework
  unsigned type;

  /// \brief Gradually decreasing
  /// Explode when age passes zero
  gorilla::real age;

  /// \brief Simulate firework for duration time
  /// \returns True if firework should be removed
  bool update(gorilla::real duration)
  {
    this->integrate(duration);
    this->age -= duration;
    return (age < 0) || (position.y() < 0);
  }
};

/// \brief Firework rules that specify firework characteristics
struct FireworkRule
{
  /// \brief Type of firework
  unsigned type;

  /// \brief Min age for explosion
  gorilla::real minAge;

  /// \brief Max age for explosion
  gorilla::real maxAge;

  /// \brief Min initial relative velocity to parent
  gorilla::Vector3 minVelocity;

  /// \brief max initial relative velocity to parent
  gorilla::Vector3 maxVelocity;

  /// \brief Damping for firework particle
  gorilla::real damping;

  /// \brief The new firework type to create when current explodes
  struct Payload
  {
    /// \brief Type of new firework
    unsigned type;

    /// \brief Number of new firework particles
    unsigned count;

    /// \brief Set payload characteristics
    void set(unsigned type, unsigned count)
    {
      this->type = type;
      this->count = count;
    }
  };

  /// \brief Number of payloads
  unsigned payloadCount;

  /// \brief Set of payloads
  Payload *payloads;

  /// \brief Constructor
  FireworkRule() : payloadCount(0), payloads(nullptr)
  {
  }

  /// \brief Initialize payloads
  void init(unsigned payloadCount)
  {
    this->payloadCount = payloadCount;
    payloads = new Payload[payloadCount];
  }

  /// \brief Destructor
  ~FireworkRule()
  {
    if (payloads != nullptr)
    {
      delete[] payloads;
      payloads = nullptr;
    }
  }

  /// \brief Set firework characteristics
  void setParameters(unsigned type, gorilla::real minAge, gorilla::real maxAge,
                     const gorilla::Vector3 &minVelocity, const gorilla::Vector3 &maxVelocity,
                     gorilla::real damping)
  {
    this->type = type;
    this->minAge = minAge;
    this->maxAge = maxAge;
    this->minVelocity = minVelocity;
    this->maxVelocity = maxVelocity;
    this->damping = damping;
  }

  /// \brief Create firework of this rule
  void create(Firework *firework, const Firework *parent = nullptr) const
  {
    assert(firework != nullptr);
    firework->type = this->type;
    firework->age = grandom.randomReal(this->minAge, this->maxAge);

    gorilla::Vector3 vel;
    if (parent)
    {
      firework->setPosition(parent->getPosition());
      vel += parent->getVelocity();
    }
    else
    {
      gorilla::Vector3 start;
      int x = (int)grandom.randomInt(3) - 1;
      start.x() = 5.0 * gorilla::real(x);
      firework->setPosition(start);
    }

    vel += grandom.randomVector(minVelocity, maxVelocity);
    firework->setVelocity(vel);

    firework->setMass(1);
    firework->setDamping(this->damping);
    firework->setAcceleration(gorilla::GRAVITY);
  }
};

/// \brief Firework application
class FireworksDemo : public Application
{
private:
  /// \brief Max number of fireworks that can exist
  const static unsigned maxFireworks = 1024;

  /// \brief Hold the firework particles
  Firework fireworks[maxFireworks];

  /// \brief Index of next firework slot to use
  unsigned nextFireworkIndex;

  /// \brief Number of rules
  const static unsigned ruleCount = 9;

  /// \brief Hold all the rules
  FireworkRule rules[ruleCount];

  /// \brief Dispatch a firework particle
  void create(unsigned type, const Firework *parent = nullptr);

  /// \brief Dispatch a number of firework particles
  void create(unsigned type, unsigned number, const Firework *parent);

  /// \brief Initialize firework rules
  void initFireworkRules();

public:
  FireworksDemo();
  ~FireworksDemo();

  // Documentation inherited
  virtual void initGraphics() override;

  // Documentation inherited
  virtual const char *getTitle() override;

  // Documentation inherited
  virtual void update();

  // Documentation inherited
  virtual void display();

  // Documentation inherited
  virtual void key(unsigned char key);
};

///////////////////////////////////////////////////////////////////////////
FireworksDemo::FireworksDemo() : nextFireworkIndex(0)
{
  for (Firework *firework = this->fireworks;
       firework < this->fireworks + this->maxFireworks; ++firework)
  {
    firework->type = 0;
  }

  initFireworkRules();
}

///////////////////////////////////////////////////////////////////////////
FireworksDemo::~FireworksDemo()
{
}

///////////////////////////////////////////////////////////////////////////
void FireworksDemo::initFireworkRules()
{
  rules[0].init(2);
  rules[0].setParameters(1, 0.5f, 1.4f,
                         gorilla::Vector3(-5, 25, -5),
                         gorilla::Vector3(5, 28, 5),
                         0.1);
  rules[0].payloads[0].set(3, 5);
  rules[0].payloads[1].set(5, 5);

  rules[1].init(1);
  rules[1].setParameters(
      2,                            // type
      0.5f, 1.0f,                   // age range
      gorilla::Vector3(-5, 10, -5), // min velocity
      gorilla::Vector3(5, 20, 5),   // max velocity
      0.8                           // damping
  );
  rules[1].payloads[0].set(4, 2);

  rules[2].init(0);
  rules[2].setParameters(
      3,                            // type
      0.5f, 1.5f,                   // age range
      gorilla::Vector3(-5, -5, -5), // min velocity
      gorilla::Vector3(5, 5, 5),    // max velocity
      0.1                           // damping
  );

  rules[3].init(0);
  rules[3].setParameters(
      4,                            // type
      0.25f, 0.5f,                  // age range
      gorilla::Vector3(-20, 5, -5), // min velocity
      gorilla::Vector3(20, 5, 5),   // max velocity
      0.2                           // damping
  );

  rules[4].init(1);
  rules[4].setParameters(
      5,                            // type
      0.5f, 1.0f,                   // age range
      gorilla::Vector3(-20, 2, -5), // min velocity
      gorilla::Vector3(20, 18, 5),  // max velocity
      0.01                          // damping
  );
  rules[4].payloads[0].set(3, 5);

  rules[5].init(0);
  rules[5].setParameters(
      6,                           // type
      3, 5,                        // age range
      gorilla::Vector3(-5, 5, -5), // min velocity
      gorilla::Vector3(5, 10, 5),  // max velocity
      0.95                         // damping
  );

  rules[6].init(1);
  rules[6].setParameters(
      7,                            // type
      4, 5,                         // age range
      gorilla::Vector3(-5, 50, -5), // min velocity
      gorilla::Vector3(5, 60, 5),   // max velocity
      0.01                          // damping
  );
  rules[6].payloads[0].set(8, 10);

  rules[7].init(0);
  rules[7].setParameters(
      8,                            // type
      0.25f, 0.5f,                  // age range
      gorilla::Vector3(-1, -1, -1), // min velocity
      gorilla::Vector3(1, 1, 1),    // max velocity
      0.01                          // damping
  );

  rules[8].init(0);
  rules[8].setParameters(
      9,                             // type
      3, 5,                          // age range
      gorilla::Vector3(-15, 10, -5), // min velocity
      gorilla::Vector3(15, 15, 5),   // max velocity
      0.95                           // damping
  );
}

///////////////////////////////////////////////////////////////////////////
void FireworksDemo::initGraphics()
{
  Application::initGraphics();
  glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
}

///////////////////////////////////////////////////////////////////////////
const char *FireworksDemo::getTitle()
{
  return "Gorilla Physics Engine Example > Fireworks";
}

///////////////////////////////////////////////////////////////////////////
void FireworksDemo::create(unsigned type, const Firework *parent)
{
  FireworkRule *rule = this->rules + (type - 1);
  rule->create(this->fireworks + this->nextFireworkIndex, parent);
  this->nextFireworkIndex = (this->nextFireworkIndex + 1) % this->maxFireworks;
}

///////////////////////////////////////////////////////////////////////////
void FireworksDemo::create(unsigned type, unsigned number,
                           const Firework *parent)
{
  for (unsigned i = 0; i < number; ++i)
    this->create(type, parent);
}

///////////////////////////////////////////////////////////////////////////
void FireworksDemo::update()
{
  // Find the duration of the last frame in seconds
  gorilla::real duration = TimingData::get().lastFrameDuration * 0.001;
  if (duration <= 0.0)
    return;

  for (Firework *firework = this->fireworks;
       firework < this->fireworks + this->maxFireworks;
       ++firework)
  {
    // Check if we need to process this firework.
    if (firework->type > 0)
    {
      // Does it need removing?
      if (firework->update(duration))
      {
        // Find the appropriate rule
        FireworkRule *rule = this->rules + (firework->type - 1);

        // Delete the current firework (this doesn't affect its
        // position and velocity for passing to the create function,
        // just whether or not it is processed for rendering or
        // physics.
        firework->type = 0;

        // Add the payloads
        for (unsigned i = 0; i < rule->payloadCount; i++)
        {
          FireworkRule::Payload *payload = rule->payloads + i;
          this->create(payload->type, payload->count, firework);
        }
      }
    }
  }

  Application::update();
}

///////////////////////////////////////////////////////////////////////////
void FireworksDemo::display()
{
  const static gorilla::real size = 0.1;

  // Clear the viewport and set the camera direction
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  gluLookAt(0.0, 4.0, 10.0, 0.0, 4.0, 0.0, 0.0, 1.0, 0.0);

  // Render each firework in turn
  glBegin(GL_QUADS);
  for (Firework *firework = this->fireworks;
       firework < this->fireworks + this->maxFireworks;
       firework++)
  {
    // Check if we need to process this firework.
    if (firework->type > 0)
    {
      switch (firework->type)
      {
      case 1:
        glColor3f(1, 0, 0);
        break;
      case 2:
        glColor3f(1, 0.5f, 0);
        break;
      case 3:
        glColor3f(1, 1, 0);
        break;
      case 4:
        glColor3f(0, 1, 0);
        break;
      case 5:
        glColor3f(0, 1, 1);
        break;
      case 6:
        glColor3f(0.4f, 0.4f, 1);
        break;
      case 7:
        glColor3f(1, 0, 1);
        break;
      case 8:
        glColor3f(1, 1, 1);
        break;
      case 9:
        glColor3f(1, 0.5f, 0.5f);
        break;
      };

      // Draw four corners
      const gorilla::Vector3 &pos = firework->getPosition();
      glVertex3f(pos.x() - size, pos.y() - size, pos.z());
      glVertex3f(pos.x() + size, pos.y() - size, pos.z());
      glVertex3f(pos.x() + size, pos.y() + size, pos.z());
      glVertex3f(pos.x() - size, pos.y() + size, pos.z());

      // Render the firework's reflection
      glVertex3f(pos.x() - size, -pos.y() - size, pos.z());
      glVertex3f(pos.x() + size, -pos.y() - size, pos.z());
      glVertex3f(pos.x() + size, -pos.y() + size, pos.z());
      glVertex3f(pos.x() - size, -pos.y() + size, pos.z());
    }
  }
  glEnd();
}

///////////////////////////////////////////////////////////////////////////
void FireworksDemo::key(unsigned char key)
{
  switch (key)
  {
  case '1': create(1, 1, nullptr); break;
  case '2': create(2, 1, nullptr); break;
  case '3': create(3, 1, nullptr); break;
  case '4': create(4, 1, nullptr); break;
  case '5': create(5, 1, nullptr); break;
  case '6': create(6, 1, nullptr); break;
  case '7': create(7, 1, nullptr); break;
  case '8': create(8, 1, nullptr); break;
  case '9': create(9, 1, nullptr); break;
  }
}

/// \brief Called by the common application framework to create an application
/// object (with new) and return a pointer.
Application* getApplication()
{
  return new FireworksDemo();
}
