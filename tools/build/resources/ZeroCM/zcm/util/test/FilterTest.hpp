#pragma once

#include <iostream>

#include "cxxtest/TestSuite.h"

#include "Filter.hpp"

class FilterTest : public CxxTest::TestSuite
{
  public:
    void setUp() override {}
    void tearDown() override {}

    void testEndResult()
    {
        zcm::Filter f(1, 0.8);
        for (double i = 0; i < 2000; i++) f(10, 0.01);
        TS_ASSERT_DELTA(f[zcm::Filter::FilterMode::LOW_PASS],  10, 1e-5);
        TS_ASSERT_DELTA(f[zcm::Filter::FilterMode::BAND_PASS],  0, 1e-5);
        TS_ASSERT_DELTA(f[zcm::Filter::FilterMode::HIGH_PASS],  0, 1e-5);
    }

    void testConvergenceTimeEst()
    {
        TS_ASSERT_DELTA(zcm::Filter::convergenceTimeToNatFreq(10, 0.8), 0.24848, 1e-5);
    }
};
