#pragma once

#include "UnitTest.h"

class OnboardAddressTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _testDefaultStoredAndEnvironment();
    void _testSetHostTrimsAndResets();
};
