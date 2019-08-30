#include <cstdlib>

#include <gorilla/gorilla.h>

/// \brief Base class for example applications
class Application
{
protected:
  /// \brief Window height in pixels
  int height;

  /// \brief Window width in pixels
  int width;

public:
  /// \brief Get application title
  virtual const char* getTitle();

  /// \brief Initialize graphics
  virtual void initGraphics();

  /// \brief Set graphics view
  virtual void setView();

  /// \brief Clean up resources
  virtual void deinit();

  /// \brief Display frame
  virtual void display();

  /// \brief Update application state
  virtual void update();

  /// \brief Process keyboard input
  virtual void key(unsigned char key);

  /// \brief Process window resizing event
  virtual void resize(int width, int height);

  /// \brief Process mouse event
  virtual void mouse(int button, int state, int x, int y);

  /// \brief Process mouse drag event
  virtual void mouseDrag(int x, int y);

  /// \brief Destructor
  virtual ~Application() = default;

  /// \brief Render text on screen
  void renderText(float x, float y, const char *text, void* font=NULL);
};


/// \brief Base class for mass-aggregate applications
class MassAggregateApplication : public Application
{
protected:
  /// \brief Application physics world
  gorilla::ParticleWorld world;

  /// \brief Particles in the world
  gorilla::Particle *particleArray;

  /// \brief Generator for ground contacts
  gorilla::GroundContactsGenerator groundContactGenerator;
public:
  /// \brief Constructor
  MassAggregateApplication(std::size_t ParticleCount);

  /// \brief Destructor
  virtual ~MassAggregateApplication();

  /// Documentation inherited
  virtual void update();

  /// Documentation inherited
  virtual void initGraphics();

  /// Documentation inherited
  virtual void display();
};