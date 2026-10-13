#pragma once

#include "UnitTest.h"

class GoalProtocolTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _testEncodeGoalMatchesPythonBytes();
    void _testDecodeAck();
    void _testDecodeRejectedAck();
    void _testBadAcksAreIgnored();
};
