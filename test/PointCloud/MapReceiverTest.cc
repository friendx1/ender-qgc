#include "MapReceiverTest.h"

#include <QtNetwork/QHostAddress>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtTest/QSignalSpy>

#include "MapReceiver.h"
#include "MapTestData.h"

using namespace MapTestData;

void MapReceiverTest::_testInitialState()
{
    MapReceiver receiver;

    QVERIFY(!receiver.active());
    QVERIFY(!receiver.connected());
    QVERIFY(!receiver.receiving());
    QVERIFY(!receiver.poseValid());
    QCOMPARE(receiver.pointCount(), 0);
    QCOMPARE(receiver.port(), MapReceiver::kDefaultPort);
    QCOMPARE(receiver.modelPosition(), QVector3D());
}

void MapReceiverTest::_testHostFromEnvironment()
{
    const QByteArray previous = qgetenv("ENDER_GOAL_HOST");
    qunsetenv("ENDER_GOAL_HOST");
    QCOMPARE(MapReceiver().host(), QString::fromLatin1(MapReceiver::kDefaultHost));

    qputenv("ENDER_GOAL_HOST", "10.1.2.3");
    QCOMPARE(MapReceiver().host(), QStringLiteral("10.1.2.3"));

    if (previous.isNull()) {
        qunsetenv("ENDER_GOAL_HOST");
    } else {
        qputenv("ENDER_GOAL_HOST", previous);
    }
}

void MapReceiverTest::_testPoseThenMapCutsToRadius()
{
    MapReceiver receiver;
    QSignalSpy mapSpy(&receiver, &MapReceiver::mapChanged);

    receiver.setRadius(20.0f);
    // The map holds points at x = 0, 5 and 30 meters
    receiver.handleBytes(poseMessage(0, 0, 0) + mapMessage());

    QCOMPARE(mapSpy.count(), 1);
    QCOMPARE(receiver.pointCount(), 2);  // the point at 30 m is outside the radius
    QVERIFY(receiver.receiving());
    QVERIFY(receiver.poseValid());

    // A larger radius keeps all three points of the next map
    receiver.setRadius(50.0f);
    receiver.handleBytes(mapMessage());
    QCOMPARE(receiver.pointCount(), 3);
}

void MapReceiverTest::_testMapBeforePoseWaitsForPose()
{
    MapReceiver receiver;

    receiver.handleBytes(mapMessage());
    QCOMPARE(receiver.pointCount(), 0);  // cannot be cut and placed without a pose
    QVERIFY(!receiver.poseValid());

    receiver.handleBytes(poseMessage(0, 0, 0));
    QCOMPARE(receiver.pointCount(), 2);
    QVERIFY(receiver.poseValid());
}

void MapReceiverTest::_testPoseMovesModelNotMap()
{
    MapReceiver receiver;
    receiver.handleBytes(poseMessage(0, 0, 0) + mapMessage());
    QSignalSpy mapSpy(&receiver, &MapReceiver::mapChanged);
    QSignalSpy poseSpy(&receiver, &MapReceiver::poseChanged);

    const QVector3D before = receiver.modelPosition();
    receiver.handleBytes(poseMessage(2, 0, 0));

    QCOMPARE(mapSpy.count(), 0);  // flying does not rebuild the map
    QVERIFY(poseSpy.count() >= 1);
    QVERIFY(receiver.modelPosition() != before);
    QCOMPARE(receiver.pointCount(), 2);
}

void MapReceiverTest::_testFrozenKeepsDisplayedState()
{
    MapReceiver receiver;
    receiver.handleBytes(poseMessage(0, 0, 0) + mapMessage());
    const QVector3D shown = receiver.modelPosition();

    receiver.setFrozen(true);
    receiver.setRadius(50.0f);
    receiver.handleBytes(poseMessage(3, 0, 0) + mapMessage());
    QCOMPARE(receiver.modelPosition(), shown);
    QCOMPARE(receiver.pointCount(), 2);  // still the old map

    receiver.setFrozen(false);
    QVERIFY(receiver.modelPosition() != shown);
    QCOMPARE(receiver.pointCount(), 3);
}

void MapReceiverTest::_testReceivesOverLocalConnection()
{
    QTcpServer server;
    QVERIFY(server.listen(QHostAddress::LocalHost, 0));

    MapReceiver receiver;
    receiver.setHost(QStringLiteral("127.0.0.1"));
    receiver.setPort(server.serverPort());
    QSignalSpy connectedSpy(&receiver, &MapReceiver::connectedChanged);
    receiver.setActive(true);

    QVERIFY(server.waitForNewConnection(3000));
    QTcpSocket* peer = server.nextPendingConnection();
    QVERIFY(peer);
    QTRY_VERIFY_WITH_TIMEOUT(receiver.connected(), 3000);

    // The vehicle sends in pieces, as TCP may deliver it
    const QByteArray stream = poseMessage(0, 0, 0) + mapMessage();
    peer->write(stream.left(20));
    peer->flush();
    peer->write(stream.mid(20));
    peer->flush();

    QTRY_COMPARE_WITH_TIMEOUT(receiver.pointCount(), 2, 3000);

    // Switching off closes the connection, which is what stops the work on the vehicle
    receiver.setActive(false);
    QTRY_VERIFY_WITH_TIMEOUT(peer->state() == QAbstractSocket::UnconnectedState, 3000);
    QVERIFY(!receiver.connected());
    QCOMPARE(receiver.pointCount(), 0);
}
