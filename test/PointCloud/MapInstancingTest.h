#pragma once

#include "UnitTest.h"

class MapInstancingTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _testInitialState();
    void _testSetCloudBuffer();
    void _testFollowsReceiver();
    void _testPointSizeMarksDirty();
};
