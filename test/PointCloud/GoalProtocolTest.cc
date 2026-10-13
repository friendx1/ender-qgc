#include "GoalProtocolTest.h"

#include "GoalProtocol.h"

// The reference bytes come from ender_lidar_bridge/goal_protocol.py, so both sides must agree on the format

void GoalProtocolTest::_testEncodeGoalMatchesPythonBytes()
{
    GoalRequest goal;
    goal.sequence = 5;
    goal.scanStampMs = 123456;
    goal.x = 3.5f;
    goal.y = -2.25f;

    const QByteArray bytes = GoalProtocol::encodeGoal(goal);

    QCOMPARE(bytes.size(), GoalProtocol::kGoalSize);
    QCOMPARE(bytes.toHex(), QByteArray("45474f4c01010500000040e2010000006040000010c0"));
}

void GoalProtocolTest::_testDecodeAck()
{
    const auto ack = GoalProtocol::decodeAck(QByteArray::fromHex("45474f410100090000000000c03f000080c0"));

    QVERIFY(ack.has_value());
    QCOMPARE(ack->status, quint8(GoalProtocol::Ok));
    QCOMPARE(ack->sequence, quint32(9));
    QCOMPARE(ack->worldX, 1.5f);
    QCOMPARE(ack->worldY, -4.0f);
}

void GoalProtocolTest::_testDecodeRejectedAck()
{
    const auto ack = GoalProtocol::decodeAck(QByteArray::fromHex("45474f410102070000000000000000000000"));

    QVERIFY(ack.has_value());
    QCOMPARE(ack->status, quint8(GoalProtocol::TooFar));
    QCOMPARE(ack->sequence, quint32(7));
}

void GoalProtocolTest::_testBadAcksAreIgnored()
{
    const QByteArray good = QByteArray::fromHex("45474f410100090000000000c03f000080c0");
    QVERIFY(GoalProtocol::decodeAck(good).has_value());

    QVERIFY(!GoalProtocol::decodeAck(QByteArray()).has_value());
    QVERIFY(!GoalProtocol::decodeAck(good.left(good.size() - 1)).has_value());
    QVERIFY(!GoalProtocol::decodeAck(good + QByteArray(1, '\0')).has_value());

    QByteArray badMagic = good;
    badMagic[0] = 'X';
    QVERIFY(!GoalProtocol::decodeAck(badMagic).has_value());

    QByteArray badVersion = good;
    badVersion[4] = 2;
    QVERIFY(!GoalProtocol::decodeAck(badVersion).has_value());

    // A goal is not an acknowledgement
    GoalRequest goal;
    QVERIFY(!GoalProtocol::decodeAck(GoalProtocol::encodeGoal(goal)).has_value());
}
