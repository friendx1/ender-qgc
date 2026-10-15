#pragma once

#include "UnitTest.h"

class MapReceiverTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _testInitialState();
    void _testHostFromEnvironment();
    void _testPoseThenMapCutsToRadius();
    void _testMapBeforePoseWaitsForPose();
    void _testPoseMovesModelNotMap();
    void _testFrozenKeepsDisplayedState();
    void _testReceivesOverLocalConnection();
};
