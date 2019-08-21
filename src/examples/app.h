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
};


/// \brief Base class for mass-aggregate applications
class MassAggregateApplication : public Application
{
};