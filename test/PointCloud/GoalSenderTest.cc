#include "GoalSenderTest.h"

#include <QtCore/QtEndian>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkDatagram>
#include <QtNetwork/QUdpSocket>
#include <QtTest/QSignalSpy>
#include <bit>

#include "GoalProtocol.h"
#include "GoalSender.h"

namespace {

/// Acts as the onboard computer: a socket on localhost that the sender talks to
class FakeDrone
{
public:
    FakeDrone() { ok = socket.bind(QHostAddress::LocalHost, 0); }

    quint16 port() const { return socket.localPort(); }

    QNetworkDatagram waitForGoal()
    {
        if (!socket.hasPendingDatagrams()) {
            socket.waitForReadyRead(2000);
        }
        return socket.receiveDatagram();
    }

    void answer(const QNetworkDatagram& goal, quint8 status, quint32 sequence, float worldX, float worldY)
    {
        QByteArray ack;
        ack.append("EGOA", 4);
        ack.append(char(GoalProtocol::kVersion));
        ack.append(char(status));
        appendLe(ack, sequence);
        appendLe(ack, std::bit_cast<quint32>(worldX));
        appendLe(ack, std::bit_cast<quint32>(worldY));
        socket.writeDatagram(ack, goal.senderAddress(), goal.senderPort());
    }

    static quint32 sequenceOf(const QNetworkDatagram& goal)
    {
        return qFromLittleEndian<quint32>(reinterpret_cast<const uchar*>(goal.data().constData() + 6));
    }

    QUdpSocket socket;
    bool ok = false;

private:
    static void appendLe(QByteArray& out, quint32 value)
    {
        const quint32 le = qToLittleEndian(value);
        out.append(reinterpret_cast<const char*>(&le), sizeof(le));
    }
};

}  // namespace

void GoalSenderTest::_testSendAndAccept()
{
    FakeDrone drone;
    QVERIFY(drone.ok);
    GoalSender sender;
    sender.setHost(QStringLiteral("127.0.0.1"));
    sender.setPort(drone.port());
    QSignalSpy result(&sender, &GoalSender::goalResult);

    sender.sendGoal(2.5, -1.0, 4242);
    QVERIFY(sender.pending());

    const QNetworkDatagram goal = drone.waitForGoal();
    QCOMPARE(goal.data().size(), GoalProtocol::kGoalSize);
    QCOMPARE(goal.data().left(6).toHex(), QByteArray("45474f4c0101"));  // magic, version, kind
    drone.answer(goal, GoalProtocol::Ok, FakeDrone::sequenceOf(goal), 10.0f, 7.0f);

    QVERIFY(result.wait(2000));
    QCOMPARE(result.first().at(0).toBool(), true);
    QVERIFY(!sender.pending());
    QVERIFY(sender.resultAccepted());
}

void GoalSenderTest::_testRejectedGoal()
{
    FakeDrone drone;
    QVERIFY(drone.ok);
    GoalSender sender;
    sender.setHost(QStringLiteral("127.0.0.1"));
    sender.setPort(drone.port());
    QSignalSpy result(&sender, &GoalSender::goalResult);

    sender.sendGoal(30.0, 30.0, 1);
    const QNetworkDatagram goal = drone.waitForGoal();
    drone.answer(goal, GoalProtocol::TooFar, FakeDrone::sequenceOf(goal), 0.0f, 0.0f);

    QVERIFY(result.wait(2000));
    QCOMPARE(result.first().at(0).toBool(), false);
    QVERIFY(!sender.resultAccepted());
    QVERIFY(!sender.resultText().isEmpty());
}

void GoalSenderTest::_testNoAnswerRetriesThenTimesOut()
{
    FakeDrone drone;
    QVERIFY(drone.ok);
    GoalSender sender;
    sender.setHost(QStringLiteral("127.0.0.1"));
    sender.setPort(drone.port());
    QSignalSpy result(&sender, &GoalSender::goalResult);

    sender.sendGoal(1.0, 1.0, 1);

    const int timeoutMs = GoalSender::kRetryIntervalMs * (GoalSender::kMaxAttempts + 2);
    QVERIFY(result.wait(timeoutMs));
    QCOMPARE(result.first().at(0).toBool(), false);
    QVERIFY(!sender.pending());

    // Every attempt carried the same goal sequence, so the onboard computer sets the goal only once
    QList<QByteArray> received;
    while (drone.socket.hasPendingDatagrams()) {
        received.append(drone.socket.receiveDatagram().data());
    }
    QCOMPARE(received.size(), GoalSender::kMaxAttempts);
    for (const QByteArray& datagram : std::as_const(received)) {
        QCOMPARE(datagram, received.first());
    }
}

void GoalSenderTest::_testAckOfOtherGoalIsIgnored()
{
    FakeDrone drone;
    QVERIFY(drone.ok);
    GoalSender sender;
    sender.setHost(QStringLiteral("127.0.0.1"));
    sender.setPort(drone.port());
    QSignalSpy result(&sender, &GoalSender::goalResult);

    sender.sendGoal(1.0, 1.0, 1);
    const QNetworkDatagram goal = drone.waitForGoal();
    const quint32 sequence = FakeDrone::sequenceOf(goal);

    drone.answer(goal, GoalProtocol::Ok, sequence + 1, 0.0f, 0.0f);  // answer to some other goal
    QVERIFY(!result.wait(150));
    QVERIFY(sender.pending());

    drone.answer(goal, GoalProtocol::Ok, sequence, 0.0f, 0.0f);
    QVERIFY(result.wait(2000));
    QCOMPARE(result.first().at(0).toBool(), true);
}

void GoalSenderTest::_testInvalidHost()
{
    GoalSender sender;
    sender.setHost(QStringLiteral("not an address"));
    QSignalSpy result(&sender, &GoalSender::goalResult);

    sender.sendGoal(1.0, 1.0, 1);

    QCOMPARE(result.count(), 1);
    QCOMPARE(result.first().at(0).toBool(), false);
    QVERIFY(!sender.pending());
}
