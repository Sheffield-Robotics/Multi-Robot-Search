#include "utilities/timing.h"
#include "utilities/misc.h"

Timing::Timing(const char* name)
{
  _name = name;
  _useLogger = false;
  _stats = new Statistics<double>(name);
  start();
}

Timing::~Timing()
{
   delete _stats;
}

bool Timing::_timingEnabled = true;

void Timing::setGlobalTiming(bool enabled)
{
   _timingEnabled = enabled;
}

void Timing::start()
{
   _startTime = getCurrentTime();
   _ended = false;
}

void Timing::end()
{
   if(!_ended)
      _endTime = getCurrentTime();
   _ended = true;
}

/**
 * This function also calls end().
 */
double Timing::diff()
{
   end();
   return (_endTime - _startTime);
}

/**
 * This function also calls end().
 * \param [in] alwaysPrint if true, overrides global setting and prints anyways
 */
void Timing::printInfo(bool alwaysPrint)
{
   if(!alwaysPrint) {
      if(!_timingEnabled)
         return;
   }

   //char buf[4096];
   //snprintf(buf, 4095, "%s took: %.1fms.\n", _name, diff() * 1000.0);
   if (_useLogger)
   {
      //M_INFO2("%s took: %.1fms.\n", _name, diff() * 1000.0);
   }
   else {
      printf("\033[33m");
      printf("%s took: %.1fms.\n", _name, diff() * 1000.0);
      printf("\033[0m");
   }
}

void Timing::printStats(int numSamples, bool alwaysPrint)
{
   if(!alwaysPrint) {
      if(!_timingEnabled)
         return;
   }

   if (_stats->getNumMeasurements() >= numSamples) {
      printf("\033[34m\n");
      _stats->print(1000.0); // in ms
      printf("\033[0m");
      _stats->reset();
   }
}


void Timing::addToStats()
{
   _stats->addMeasurement(diff());
   start();
}

