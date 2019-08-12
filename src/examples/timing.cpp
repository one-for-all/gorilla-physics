#include "timing.h"

/// \brief Whether holding internal timing data for the performance counter
static bool qpcFlag;

/// \brief Shared global timing data
static TimingData *timingData = nullptr;

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

/// \brief Get system time
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

//////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////
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

/// \brief Initialize performance frequency if available
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

///////////////////////////////////////////////////////////////////
TimingData& TimingData::get()
{
  return (TimingData&)*timingData;
}

///////////////////////////////////////////////////////////////////
void TimingData::update()
{
  if (!timingData) return;

  if (!timingData->isPaused)
    timingData->frameNumber++;

  // Get time
  unsigned thisTime = systemTime();
  timingData->lastFrameDuration = thisTime - timingData->lastFrameTimestamp;
  timingData->lastFrameTimestamp = thisTime;

  unsigned long thisClock = getClock();
  timingData->lastFrameClockTicks = thisClock - timingData->lastFrameClockstamp;
  timingData->lastFrameClockstamp = thisClock;

  // Update average frame duration
  if (timingData->frameNumber > 1)
  {
    if (timingData->averageFrameDuration <= 0)
    {
      timingData->averageFrameDuration =  
        static_cast<double>(timingData->lastFrameDuration);
    }
    else
    {
      timingData->averageFrameDuration *= 0.99;
      timingData->averageFrameDuration += 
        0.01 * static_cast<double>(timingData->lastFrameDuration);
      
      timingData->fps = 1000.0/timingData->averageFrameDuration;
    }
  }
}

/////////////////////////////////////////////////////////////////////
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

/////////////////////////////////////////////////////////////////////
void TimingData::deinit()
{
  delete timingData;
  timingData = nullptr;
}