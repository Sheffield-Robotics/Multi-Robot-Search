#ifndef TIMING_H
#define TIMING_H

#include "timeutil.h"
#include "statistics.h"

/// A Simple class for timing purposes.
/**
 * Common Usage:
 * Timing t("Nasty Function");
 * nastyFunction();
 * t.printInfo();
 *
 * Will rlogDebug: "Nasty Function took 123ms."
 *
 * The timing can be (re-)started and ended by calling the respective functions.
 * diff() and printInfo() will end the timing implicitly.
 */
class Timing
{
   public:
      Timing(const char* name = "Timing");
      ~Timing();

      /// Resets start time to current time.
      void start();
      /// Stops the timing until start is called again.
      void end();

      /// Secs since construction.
      double diff();

      /// Does rlogInfo time since construction.
      void printInfo(bool alwaysPrint = false);

      /// Print statistics after collecting `numSamples`
      void printStats(int numSamples=1, bool alwaysPrint = false);

      /// Set, if timing info will be printed out for all Timing instances.
      static void setGlobalTiming(bool enabled);

      /// Use Logger or just colored output
      void useLogger(bool on) {_useLogger=on;}

      /// Add current time to statistics and restart
      void addToStats();

   private:
      double _startTime;
      double _endTime;
      const char* _name;
      bool _ended;
      bool _useLogger;

      Statistics<double> * _stats;

      static bool _timingEnabled;
};

#endif

