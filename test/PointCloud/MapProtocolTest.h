#pragma once

#include "UnitTest.h"

class MapProtocolTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _testPoseRoundTrip();
    void _testPoseRejectsBadInput();
    void _testMapUnpacksBirksPacket();
    void _testMapRejectsGarbage();
    void _testStreamSplitAtAnyByte();
    void _testStreamRecoversAfterGarbage();
    void _testStreamSkipsHugeLengthAndUnknownVersion();
    void _testSceneTransformMatchesBodyFrame();
};
