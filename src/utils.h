#pragma once
#include <sys/time.h>
#include <sys/resource.h>
#include <glog/logging.h>
#include <atomic>
#include <chrono>
#include <sstream>

#ifndef FB_LOG_EVERY_MS
/**
 * Issues a LOG(severity) no more often than every
 * milliseconds. Example:
 *
 * FB_LOG_EVERY_MS(INFO, 10000) << "At least ten seconds passed"
 *   " since you last saw this.";
 *
 * The implementation uses for statements to introduce variables in
 * a nice way that doesn't mess surrounding statements.  It is thread
 * safe.  Non-positive intervals will always log.
 */
#define FB_LOG_EVERY_MS(severity, milli_interval)                            \
  for (decltype(milli_interval) FB_LEM_once = 1,                             \
                                FB_LEM_interval = (milli_interval);          \
       FB_LEM_once; )                                                        \
    for (::std::chrono::milliseconds::rep FB_LEM_prev, FB_LEM_now =          \
             FB_LEM_interval <= 0 ? 0 :                                      \
             ::std::chrono::duration_cast< ::std::chrono::milliseconds>(     \
                 ::std::chrono::system_clock::now().time_since_epoch()       \
                 ).count();                                                  \
         FB_LEM_once; )                                                      \
      for (static ::std::atomic< ::std::chrono::milliseconds::rep>           \
               FB_LEM_hist; FB_LEM_once; FB_LEM_once = 0)                    \
        if (FB_LEM_interval > 0 &&                                           \
            (FB_LEM_now - (FB_LEM_prev =                                     \
                           FB_LEM_hist.load(std::memory_order_acquire)) <    \
                                                          FB_LEM_interval || \
             !FB_LEM_hist.compare_exchange_strong(FB_LEM_prev,FB_LEM_now))) {\
        } else                                                               \
          LOG(severity)

#endif

inline unsigned int countWordsInString(std::string const& str)
{
  std::stringstream stream(str);
  return std::distance(std::istream_iterator<std::string>(stream), std::istream_iterator<std::string>());
}

inline long getMemoryUsage() {
  struct rusage usage;
  if (0 == getrusage(RUSAGE_SELF, &usage))
    return usage.ru_maxrss;  // bytes
  else
    return 0;
}
