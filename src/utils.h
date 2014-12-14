#pragma once
#include <sys/time.h>
#include <sys/resource.h>
#include <sstream>

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
