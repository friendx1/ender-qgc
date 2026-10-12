#pragma once

#include "UnitTest.h"

class EpclFrameAssemblerTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _testSingleChunkFrame();
    void _testMultiChunkFrame();
    void _testChunksOutOfOrder();
    void _testIntensity();
    void _testEmptyFrame();
    void _testIncompleteFrameDroppedByNewerFrame();
    void _testLateChunkOfOldFrameIgnored();
    void _testBadMagicAndVersion();
    void _testTruncatedDatagram();
    void _testInconsistentChunks();
    void _testSequenceWrapAround();
    void _testPointCountMismatch();
};
