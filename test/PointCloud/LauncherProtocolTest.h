#pragma once

#include "UnitTest.h"

class LauncherProtocolTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _testEncodeStatusRequest();
    void _testEncodeStepRequestCarriesTheStep();
    void _testDecodeReply();
    void _testBadRepliesAreIgnored();
};
