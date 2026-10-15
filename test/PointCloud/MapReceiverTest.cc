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
    QCOMPARE(receiver.vehiclePosition(), QVector3D());
    QCOMPARE(receiver.scanPosition(), QVector3D());
    QVERIFY(receiver.wantMap());
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

void MapReceiverTest::_testPoseMovesVehicleNotMap()
{
    MapReceiver receiver;
    receiver.handleBytes(poseMessage(0, 0, 0) + mapMessage());
    QSignalSpy mapSpy(&receiver, &MapReceiver::mapChanged);
    QSignalSpy poseSpy(&receiver, &MapReceiver::poseChanged);

    const QVector3D before = receiver.vehiclePosition();
    receiver.handleBytes(poseMessage(2, 0, 0));

    QCOMPARE(mapSpy.count(), 0);  // flying does not rebuild the map
    QVERIFY(poseSpy.count() >= 1);
    QVERIFY(receiver.vehiclePosition() != before);
    // 2 m ahead of the origin is 200 cm along -z in the scene
    QCOMPARE_FUZZY(receiver.vehiclePosition().z(), -200.0f, 0.01f);
    QCOMPARE(receiver.pointCount(), 2);
}

void MapReceiverTest::_testScanPlacementUsesPoseAtScanStamp()
{
    MapReceiver receiver;
    QSignalSpy placementSpy(&receiver, &MapReceiver::scanPlacementChanged);

    // Poses at 1000 ms (x = 0) and 1100 ms (x = 1)
    receiver.handleBytes(poseMessage(0, 0, 0, 0, 0, 0, 1, 1000) + poseMessage(1, 0, 0, 0, 0, 0, 1, 1100));

    // The scan was taken in the middle of the two poses
    receiver.setScanStamp(1050);
    QCOMPARE_FUZZY(receiver.scanPosition().z(), -50.0f, 0.01f);
    QCOMPARE_FUZZY(receiver.vehiclePosition().z(), -100.0f, 0.01f);  // the latest pose is the later one

    // A scan newer than the newest pose uses the newest pose, until the matching pose arrives
    receiver.setScanStamp(1150);
    QCOMPARE_FUZZY(receiver.scanPosition().z(), -100.0f, 0.01f);
    receiver.handleBytes(poseMessage(3, 0, 0, 0, 0, 0, 1, 1200));
    QCOMPARE_FUZZY(receiver.scanPosition().z(), -200.0f, 0.01f);

    // A stamp the history knows nothing about leaves the scan where it is
    const QVector3D kept = receiver.scanPosition();
    receiver.setScanStamp(99999);
    QCOMPARE(receiver.scanPosition(), kept);
    QVERIFY(placementSpy.count() >= 3);

    // While frozen the placement does not move, afterwards it catches up
    receiver.setScanStamp(1100);
    receiver.setFrozen(true);
    receiver.setScanStamp(1200);
    QCOMPARE_FUZZY(receiver.scanPosition().z(), -100.0f, 0.01f);
    receiver.setFrozen(false);
    QCOMPARE_FUZZY(receiver.scanPosition().z(), -300.0f, 0.01f);
}

void MapReceiverTest::_testFrozenKeepsDisplayedState()
{
    MapReceiver receiver;
    receiver.handleBytes(poseMessage(0, 0, 0) + mapMessage());
    const QVector3D shown = receiver.vehiclePosition();

    receiver.setFrozen(true);
    receiver.setRadius(50.0f);
    receiver.handleBytes(poseMessage(3, 0, 0) + mapMessage());
    QCOMPARE(receiver.vehiclePosition(), shown);
    QCOMPARE(receiver.pointCount(), 2);  // still the old map

    receiver.setFrozen(false);
    QVERIFY(receiver.vehiclePosition() != shown);
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

void MapReceiverTest::_testTrajectoryOnlyOnWish()
{
    const QList<QList<QVector3D>> exp = {{QVector3D(0, 0, 0), QVector3D(1, 0, 0), QVector3D(2, 0, 0)}};
    const QList<QList<QVector3D>> backup = {{QVector3D(0, 1, 0), QVector3D(0, 2, 0)}};

    QTcpServer server;
    QVERIFY(server.listen(QHostAddress::LocalHost, 0));

    MapReceiver receiver;
    receiver.setHost(QStringLiteral("127.0.0.1"));
    receiver.setPort(server.serverPort());
    receiver.setWantMap(false);
    receiver.setActive(true);
    QSignalSpy changed(&receiver, &MapReceiver::trajectoryChanged);

    QVERIFY(server.waitForNewConnection(3000));
    QTcpSocket* peer = server.nextPendingConnection();
    QVERIFY(peer);
    QTRY_VERIFY_WITH_TIMEOUT(receiver.connected(), 3000);
    QTRY_VERIFY_WITH_TIMEOUT(peer->bytesAvailable() >= EmapProtocol::kHeaderSize + 1, 3000);
    QCOMPARE(peer->readAll(), EmapProtocol::encodeControl(false, false));

    // Not wanted: a trajectory that is on its way is dropped, the pose after it still arrives
    peer->write(trajectoryMessage(EmapProtocol::kKindExp, exp) + poseMessage(0, 0, 0));
    peer->flush();
    QTRY_VERIFY_WITH_TIMEOUT(receiver.poseValid(), 3000);
    QVERIFY(receiver.trajectory(EmapProtocol::kKindExp).isEmpty());
    QCOMPARE(changed.count(), 0);

    // Asking for it is passed on together with the wish for the map
    receiver.setWantTrajectory(true);
    QTRY_VERIFY_WITH_TIMEOUT(peer->bytesAvailable() >= EmapProtocol::kHeaderSize + 1, 3000);
    QCOMPARE(peer->readAll(), EmapProtocol::encodeControl(false, true));

    // Both kinds are kept apart, the newest state of a kind replaces the previous one
    peer->write(trajectoryMessage(EmapProtocol::kKindExp, exp) + trajectoryMessage(EmapProtocol::kKindBackup, backup));
    peer->flush();
    QTRY_COMPARE_WITH_TIMEOUT(changed.count(), 2, 3000);
    QCOMPARE(receiver.trajectory(EmapProtocol::kKindExp), exp);
    QCOMPARE(receiver.trajectory(EmapProtocol::kKindBackup), backup);
    peer->write(trajectoryMessage(EmapProtocol::kKindExp, {}));
    peer->flush();
    QTRY_VERIFY_WITH_TIMEOUT(receiver.trajectory(EmapProtocol::kKindExp).isEmpty(), 3000);
    QCOMPARE(receiver.trajectory(EmapProtocol::kKindBackup), backup);

    // An unknown kind is ignored, and an unknown kind index has no lines
    QVERIFY(receiver.trajectory(9).isEmpty());

    // Not wanting it any more empties the display
    receiver.setWantTrajectory(false);
    QVERIFY(receiver.trajectory(EmapProtocol::kKindBackup).isEmpty());

    // The same when the connection goes away
    receiver.setWantTrajectory(true);
    peer->write(trajectoryMessage(EmapProtocol::kKindBackup, backup));
    peer->flush();
    QTRY_VERIFY_WITH_TIMEOUT(!receiver.trajectory(EmapProtocol::kKindBackup).isEmpty(), 3000);
    receiver.setActive(false);
    QVERIFY(receiver.trajectory(EmapProtocol::kKindBackup).isEmpty());
}

void MapReceiverTest::_testWantMapSendsControlAndDropsMaps()
{
    QTcpServer server;
    QVERIFY(server.listen(QHostAddress::LocalHost, 0));

    MapReceiver receiver;
    receiver.setHost(QStringLiteral("127.0.0.1"));
    receiver.setPort(server.serverPort());
    receiver.setWantMap(false);  // the free view wants the pose only
    receiver.setActive(true);

    QVERIFY(server.waitForNewConnection(3000));
    QTcpSocket* peer = server.nextPendingConnection();
    QVERIFY(peer);
    QTRY_VERIFY_WITH_TIMEOUT(receiver.connected(), 3000);

    // The first thing the vehicle hears is that no map is wanted
    QTRY_VERIFY_WITH_TIMEOUT(peer->bytesAvailable() >= EmapProtocol::kHeaderSize + 1, 3000);
    QCOMPARE(peer->readAll(), EmapProtocol::encodeControl(false));

    // A map that is already on its way is dropped, the pose is used
    peer->write(poseMessage(0, 0, 0) + mapMessage());
    peer->flush();
    QTRY_VERIFY_WITH_TIMEOUT(receiver.poseValid(), 3000);
    QCOMPARE(receiver.pointCount(), 0);
    QVERIFY(!receiver.receiving());

    // Asking for the map is passed on
    receiver.setWantMap(true);
    QTRY_VERIFY_WITH_TIMEOUT(peer->bytesAvailable() >= EmapProtocol::kHeaderSize + 1, 3000);
    QCOMPARE(peer->readAll(), EmapProtocol::encodeControl(true));
    peer->write(mapMessage());
    peer->flush();
    QTRY_COMPARE_WITH_TIMEOUT(receiver.pointCount(), 2, 3000);

    // Not wanting it any more empties the display
    receiver.setWantMap(false);
    QCOMPARE(receiver.pointCount(), 0);
    QVERIFY(!receiver.receiving());
}
