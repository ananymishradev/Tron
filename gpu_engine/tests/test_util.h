#pragma once

#include <cmath>
#include <cstdio>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, \
                   #condition);                                               \
      return 1;                                                                \
    }                                                                          \
  } while (false)

#define CHECK_NEAR(actual, expected, tolerance)                                \
  do {                                                                         \
    const auto check_actual = (actual);                                        \
    const auto check_expected = (expected);                                    \
    const auto check_tolerance = (tolerance);                                  \
    if (std::fabs(check_actual - check_expected) > check_tolerance) {          \
      std::fprintf(stderr,                                                     \
                   "CHECK_NEAR failed at %s:%d: %s ~= %s (actual=%g, "        \
                   "expected=%g, tolerance=%g)\n",                            \
                   __FILE__, __LINE__, #actual, #expected,                   \
                   static_cast<double>(check_actual),                        \
                   static_cast<double>(check_expected),                       \
                   static_cast<double>(check_tolerance));                     \
      return 1;                                                                \
    }                                                                          \
  } while (false)
