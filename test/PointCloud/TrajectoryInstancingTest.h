#pragma once

#include "UnitTest.h"

class TrajectoryInstancingTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _testInitialState();
    void _testResampleSpacingAndEnds();
    void _testResampleCapsTheCount();
    void _testSetLinesBuffer();
    void _testFollowsReceiverAndKind();
    void _testPointSizeAndColorMarkDirty();
};
