#pragma once

#include "UnitTest.h"

class PointCloudInstancingTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _testInitialState();
    void _testToScenePosition();
    void _testSetFrameBuffer();
    void _testHeightColors();
    void _testPointSizeMarksDirty();
    void _testFrozenAndStamp();
    void _testPersistenceKeepsPointsInTheWorld();
    void _testPersistenceDropsOldPointsAndMergesCells();
    void _testPersistenceNeedsPoseAndSwitchesOff();
};
