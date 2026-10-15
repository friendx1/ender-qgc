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
    void _testPoseMovesVehicleNotMap();
    void _testScanPlacementUsesPoseAtScanStamp();
    void _testWantMapSendsControlAndDropsMaps();
    void _testTrajectoryOnlyOnWish();
    void _testFrozenKeepsDisplayedState();
    void _testReceivesOverLocalConnection();
};
