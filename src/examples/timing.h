#ifndef GORILLA_EXAMPLE_TIMING_H
#define GORILLA_EXAMPLE_TIMING_H

#include <cstdlib>
using namespace std;

/// \brief Hold timing data
struct TimingData
{
public:
  /// \brief Current render frame number
  size_t frameNumber;

  /// \brief Timestamp of end of last frame in milliseconds
  unsigned lastFrameTimestamp;

  /// \brief Duration of last frame in milliseconds
  unsigned lastFrameDuration;

  /// \brief Clockstamp of end of last frame
  unsigned long lastFrameClockstamp;

  /// \brief Duration of last frame in clock ticks
  unsigned long lastFrameClockTicks;

  /// \brief Whether rendering is paused
  bool isPaused;

  /// \brief Running average of frame duration
  double averageFrameDuration;

  /// \brief Inverse of average frame duration
  /// giving frames per second
  double fps;

  /// \brief Get global timing data
  static TimingData& get();

  /// \brief Update timing system
  /// should be called once per frame
  static void update();

  /// \brief Initialize timing system
  static void init();

  /// \brief Clean up timing system
  static void deinit();

  /// \brief Get current timing in milliseconds
  static unsigned getTime();

  /// \brief Get clock ticks
  static unsigned long getClock();

private:
  // Constructors are set to be private to prevent instances from being created
  TimingData() {}
  TimingData(const TimingData &) {}
  TimingData& operator=(const TimingData &);
};

#endif // GORILLA_EXAMPLE_TIMING_H
