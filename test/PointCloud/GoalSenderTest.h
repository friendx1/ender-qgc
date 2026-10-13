#pragma once

#include "UnitTest.h"

class GoalSenderTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _testSendAndAccept();
    void _testRejectedGoal();
    void _testNoAnswerRetriesThenTimesOut();
    void _testAckOfOtherGoalIsIgnored();
    void _testInvalidHost();
};
