#include <gorilla/gorilla.h>
#include "../ogl_headers.h"
#include "../app.h"
#include "../timing.h"

#include <stdio.h>
#include <cassert>

#include <iostream>

/// \brief Flight simulation application
class FlightSimDemo : public Application
{
  /// \brief Aerodynamic control force at left wing
  private: gorilla::AeroControl leftWing;

  /// \brief Aerodynamic control force at right wing
  private: gorilla::AeroControl rightWing;

  /// \brief Aerodynamic control force at rudder
  private: gorilla::AeroControl rudder;

  /// \brief Aerodynamic force at tail
  private: gorilla::Aero tail;

  /// \brief Left wing control value
  private: gorilla::real leftWingControl;

  /// \brief Right wing control value
  private: gorilla::real rightWingControl;

  /// \brief Rudder control value
  private: gorilla::real rudderControl;

  /// \brief Aircraft body
  private: gorilla::RigidBody aircraft;

  /// \brief Registry of forces in the application
  private: gorilla::ForceRegistry registry;

  /// \brief Wind speed
  private: gorilla::Vector3 windSpeed;

  /// \brief Reset body to initial state
  private: void resetPlane();

  /// \brief Constructor
  public: FlightSimDemo();

  /// \brief Destructor
  public: ~FlightSimDemo();

  // Documentation inherited
  public: virtual const char* getTitle();

  // Documentation inherited
  public: virtual void display();

  // Documentation inherited
  public: virtual void update();

  // Documentation inherited
  public: virtual void key(unsigned char key);
};

//////////////////////////////////////////////////////////////////////
FlightSimDemo::FlightSimDemo() :
Application(),
leftWing((gorilla::Matrix3() << 0, 0, 0,
                               -1, -0.5f, 0,
                               0, 0, 0).finished(),
          (gorilla::Matrix3() << 0,0,0,
                                 -0.995f,-0.5f,0,
                                 0,0,0).finished(),
          (gorilla::Matrix3() << 0,0,0,
                                 -1.005f,-0.5f,0,
                                 0,0,0).finished(),
          gorilla::Vector3(-1.0f, 0.0f, -2.0f),
          &windSpeed),
rightWing((gorilla::Matrix3() << 0,0,0,
                                 -1,-0.5f,0,
                                 0,0,0).finished(),
          (gorilla::Matrix3() << 0,0,0,
                                 -0.995f,-0.5f,0,
                                 0,0,0).finished(),
          (gorilla::Matrix3() << 0,0,0,
                                 -1.005f,-0.5f,0,
                                 0,0,0).finished(),
          gorilla::Vector3(-1.0f, 0.0f, 2.0f),
          &windSpeed),
rudder((gorilla::Matrix3() << 0,0,0,
                              0,0,0,
                              0,0,0).finished(),
        (gorilla::Matrix3() << 0,0,0,
                               0,0,0,
                               0.01f,0,0).finished(),
        (gorilla::Matrix3() << 0,0,0,
                               0,0,0,
                               -0.01f,0,0).finished(),
          gorilla::Vector3(2.0f, 0.5f, 0),
          &windSpeed),
tail((gorilla::Matrix3() << 0, 0, 0,
                            -1, -0.5, 0,
                            0, 0, -0.1).finished(),
      gorilla::Vector3(2, 0, 0),
      &windSpeed),
leftWingControl(0), rightWingControl(0), rudderControl(0),
windSpeed(0, 0, 0)
{
  this->resetPlane();

  this->aircraft.setMass(2.5);
  gorilla::Matrix3 inertiaTensor = gorilla::CuboidInertiaTensor(
                                    gorilla::Vector3(4, 2, 2), 1);
  this->aircraft.setInertiaTensor(inertiaTensor);

  this->aircraft.setDamping(0.8, 0.8);

  this->aircraft.setAcceleration(gorilla::GRAVITY);
  this->aircraft.calculateDerivedData();

  this->registry.add(&(this->aircraft), &(this->leftWing));
  this->registry.add(&(this->aircraft), &(this->rightWing));
  this->registry.add(&(this->aircraft), &(this->rudder));
  this->registry.add(&(this->aircraft), &(this->tail));
}

//////////////////////////////////////////////////////////////////////
FlightSimDemo::~FlightSimDemo()
{
}

//////////////////////////////////////////////////////////////////////
void FlightSimDemo::resetPlane()
{
  this->aircraft.setPosition(0, 0, 0);
  this->aircraft.setOrientation(1, 0, 0, 0);

  this->aircraft.setVelocity(0, 0, 0);
  this->aircraft.setAngularVelocity(0, 0, 0);
}

//////////////////////////////////////////////////////////////////////
static void drawAircraft()
{
    // Fuselage
    glPushMatrix();
    glTranslatef(-0.5f, 0, 0);
    glScalef(2.0f, 0.8f, 1.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Rear Fuselage
    glPushMatrix();
    glTranslatef(1.0f, 0.15f, 0);
    glScalef(2.75f, 0.5f, 0.5f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Wings
    glPushMatrix();
    glTranslatef(0, 0.3f, 0);
    glScalef(0.8f, 0.1f, 6.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Rudder
    glPushMatrix();
    glTranslatef(2.0f, 0.775f, 0);
    glScalef(0.75f, 1.15f, 0.1f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Tail-plane
    glPushMatrix();
    glTranslatef(1.9f, 0, 0);
    glScalef(0.85f, 0.1f, 2.0f);
    glutSolidCube(1.0f);
    glPopMatrix();
}

//////////////////////////////////////////////////////////////////////
/// \brief Fill GL array which is colum major indexing
static void fillGLArray(const gorilla::Matrix4 &mat, GLfloat gl_array[16])
{
  // Row index
  for (int i = 0; i < 4; ++i)
  {
    // Column index
    for (int j = 0; j < 4; ++j)
    {
      gl_array[i+j*4] = mat(i, j);
    }
  }
}

//////////////////////////////////////////////////////////////////////
void FlightSimDemo::display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  // Set looking pose
  const auto &pos = this->aircraft.getPosition();
  gorilla::Vector3 offset(4+this->aircraft.getVelocity().norm(), 0, 0);
  offset = this->aircraft.getTransform().block<3, 3>(0, 0) * offset;
  gluLookAt(pos.x()+offset.x(), pos.y()+5.0f, pos.z()+offset.z(),
            pos.x(), pos.y(), pos.z(),
            0.0, 1.0, 0.0);

  // Create ground plane
  glColor3f(0.6f,0.6f,0.6f);
  int bx = int(pos.x());
  int bz = int(pos.z());
  glBegin(GL_QUADS);
  for (int x = -20; x <= 20; x++) for (int z = -20; z <= 20; z++)
  {
      glVertex3f(bx+x-0.1f, 0, bz+z-0.1f);
      glVertex3f(bx+x-0.1f, 0, bz+z+0.1f);
      glVertex3f(bx+x+0.1f, 0, bz+z+0.1f);
      glVertex3f(bx+x+0.1f, 0, bz+z-0.1f);
  }
  glEnd();

  // Set the transform matrix for the aircraft
  gorilla::Matrix4 transform = this->aircraft.getTransform();
  GLfloat gl_transform[16];
  fillGLArray(transform, gl_transform);
  glPushMatrix();
  glMultMatrixf(gl_transform);

  // Draw the aircraft
  glColor3f(0,0,0);
  drawAircraft();
  glPopMatrix();

  glColor3f(0.8f, 0.8f, 0.8f);
  glPushMatrix();
  glTranslatef(0, -1.0f - pos.y(), 0);
  glScalef(1.0f, 0.001f, 1.0f);
  glMultMatrixf(gl_transform);
  drawAircraft();
  glPopMatrix();

  char buffer[256];
  sprintf(
      buffer,
      "Altitude: %.1f | Speed %.1f",
      this->aircraft.getPosition().y(),
      this->aircraft.getVelocity().norm()
      );
  glColor3f(0,0,0);
  renderText(10.0f, 24.0f, buffer);

  sprintf(
        buffer,
        "Left Wing: %.1f | Right Wing: %.1f | Rudder %.1f",
        leftWingControl, rightWingControl, rudderControl
        );
    renderText(10.0f, 10.0f, buffer);
}

//////////////////////////////////////////////////////////////////////
void FlightSimDemo::update()
{
  const auto duration = TimingData::get().lastFrameDuration * 0.001;
  if (duration <= 0)
    return;

  this->aircraft.clearAccumulators();

  gorilla::Vector3 propulsion(-10, 0, 0);
  propulsion = this->aircraft.getTransform().block<3, 3>(0, 0) * propulsion;
  this->aircraft.addForce(propulsion);

  this->registry.updateForces(duration);

  this->aircraft.integrate(duration);

  auto pos = this->aircraft.getPosition();
  if (pos.y() < 0)
  {
    pos.y() = 0;
    this->aircraft.setPosition(pos);

    if (this->aircraft.getVelocity().y() < -10)
      this->resetPlane();
  }

  Application::update();
}

//////////////////////////////////////////////////////////////////////
const char* FlightSimDemo::getTitle()
{
  return "Gorilla > Flight Simulation";
}

//////////////////////////////////////////////////////////////////////
void FlightSimDemo::key(unsigned char key)
{
  switch(key)
    {
    case 'q': case 'Q':
        rudderControl += 0.1f;
        break;

    case 'e': case 'E':
        rudderControl -= 0.1f;
        break;

    case 'w': case 'W':
        leftWingControl -= 0.1f;
        rightWingControl -= 0.1f;
        break;

    case 's': case 'S':
        leftWingControl += 0.1f;
        rightWingControl += 0.1f;
        break;

    case 'd': case 'D':
        leftWingControl -= 0.1f;
        rightWingControl += 0.1f;
        break;

    case 'a': case 'A':
        leftWingControl += 0.1f;
        rightWingControl -= 0.1f;
        break;

    case 'x': case 'X':
        leftWingControl = 0.0f;
        rightWingControl = 0.0f;
        rudderControl = 0.0f;
        break;

    case 'r': case 'R':
        resetPlane();
        break;

    default:
        Application::key(key);
    }

    // Make sure the controls are in range
    if (leftWingControl < -1.0f) leftWingControl = -1.0f;
    else if (leftWingControl > 1.0f) leftWingControl = 1.0f;
    if (rightWingControl < -1.0f) rightWingControl = -1.0f;
    else if (rightWingControl > 1.0f) rightWingControl = 1.0f;
    if (rudderControl < -1.0f) rudderControl = -1.0f;
    else if (rudderControl > 1.0f) rudderControl = 1.0f;

    // Update the control surfaces
    leftWing.setControl(leftWingControl);
    rightWing.setControl(rightWingControl);
    rudder.setControl(rudderControl);
}

//////////////////////////////////////////////////////////////////////
Application* getApplication()
{
  return new FlightSimDemo();
}