#ifndef GORILLA_DEMO_TIMING_H
#define GORILLA_DEMO_TIMING_H

struct TimingData
{
  unsigned frameNumber;
  unsigned lastFrameTimestamp;
  unsigned lastFrameDuration;
  unsigned long lastFrameClockstamp;
  unsigned long lastFrameClockTicks;
  bool isPaused;
  double averageFrameDuration;
  float fps;

  static TimingData& get();
  static void update();
  static void init();
  static void deinit();
  static unsigned getTime();
  static unsigned long getClock();

private:
  // Private to prevent instances from being created
  TimingData() {}
  TimingData(const TimingData &) {}
  TimingData& operator=(const TimingData &);
};

#endif // GORILLA_DEMO_TIMING_H