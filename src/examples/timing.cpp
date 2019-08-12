#include "timing.h"

// Hold internal timing data for the performance counter
static bool qpcFlag;

#if (__APPLE__ || __unix)
  #define TIMING_UNIX 1
  
  #include <stdlib.h>
  #include <sys/time.h>

  typedef unsigned long long LONGLONG;
#else
  #define TIMING_WINDOWS 1

  #include <windows.h>
  #include <mmsystem.h>

  static double qpcFrequency;
#endif 

unsigned systemTime()
{
#if TIMING_UNIX
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec * 1000 + tv.tv_usec/1000;
#else
  if(qpcFlag)
  {
    static LONGLONG qpcMillisPerTick;
    QueryPerformanceCounter((LARGE_INTEGER*)&qpcMillisPerTick);
    return (unsigned)(qpcMillisPerTick * qpcFrequency);
  }
  else
  {
    return unsigned(timeGetTime());
  }
#endif

}

unsigned TimingData::getTime()
{
  return systemTime();
}

#if TIMING_WINDOWS
unsigned long systemClock()
{
  __asm {
    rdtsc;
  }
}
#endif

unsigned long TimingData::getClock()
{
#if TIMING_UNIX
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec * 1000 + tv.tv_usec/1000;
#else
  return systemClock();
#endif
}

void initTime()
{
#if TIMING_UNIX
  qpcFlag = false;
#else
  LONGLONG time;
  qpcFlag = (QueryPerformanceFrequency((LARGE_INTEGER*)&time) > 0);
  if (qpcFlag)
    qpcFrequency = 1000.0 / time;
#endif
}

static TimingData *timingData = nullptr;

TimingData& TimingData::get()
{
  return (TimingData&)*timingData;
}

void TimingData::update()
{
  if (!timingData) return;

  if (!timingData->isPaused)
    timingData->frameNumber++;

  unsigned thisTime = systemTime();
  timingData->lastFrameDuration = thisTime - timingData->lastFrameTimestamp;
  timingData->lastFrameTimestamp = thisTime;

  unsigned long thisClock = getClock();
  timingData->lastFrameClockTicks = thisClock - timingData->lastFrameClockstamp;
  timingData->lastFrameClockstamp = thisClock;

  if (timingData->frameNumber > 1)
  {
    if (timingData->averageFrameDuration <= 0)
    {
      timingData->averageFrameDuration = (double)timingData->lastFrameDuration;
    }
    else
    {
      timingData->averageFrameDuration *= 0.99;
      timingData->averageFrameDuration += 
        0.01 * (double)timingData->lastFrameDuration;
      
      timingData->fps = (float)(1000.0/timingData->averageFrameDuration);
    }
  }
}

void TimingData::init()
{
  initTime();
  if (!timingData) 
    timingData = new TimingData();

  timingData->frameNumber = 0;
  
  timingData->lastFrameTimestamp = systemTime();
  timingData->lastFrameDuration = 0;
  
  timingData->lastFrameClockstamp = getClock();
  timingData->lastFrameClockTicks = 0;

  timingData->isPaused = false;
  
  timingData->averageFrameDuration = 0;
  timingData->fps = 0;      
}

void TimingData::deinit()
{
  delete timingData;
  timingData = nullptr;
}