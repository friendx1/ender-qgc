#include "MapProtocolTest.h"

#include <QtCore/QRandomGenerator>
#include <cmath>
#include <limits>

#include "MapProtocol.h"
#include "MapTestData.h"
#include "PointCloudInstancing.h"

using namespace MapTestData;

void MapProtocolTest::_testPoseRoundTrip()
{
    const auto pose = EmapProtocol::parsePose(posePayload(4000000123u, 1.5, -2.25, 0.75, 0.0f, 0.0f, 2.0f, 2.0f));
    QVERIFY(pose.has_value());
    QCOMPARE(pose->stampMs, quint32(4000000123u));
    QCOMPARE(pose->position, QVector3D(1.5f, -2.25f, 0.75f));
    // The quaternion is normalized
    QCOMPARE_FUZZY(pose->orientation.lengthSquared(), 1.0f, 1e-5f);
    QCOMPARE_FUZZY(pose->orientation.z(), 0.70710678f, 1e-5f);
}

void MapProtocolTest::_testPoseRejectsBadInput()
{
    QVERIFY(!EmapProtocol::parsePose(QByteArray(10, '\0')));
    QVERIFY(!EmapProtocol::parsePose(QByteArray(EmapProtocol::kPoseSize + 1, '\0')));

    const double nan = std::numeric_limits<double>::quiet_NaN();
    QVERIFY(!EmapProtocol::parsePose(posePayload(0, nan, 0, 0, 0, 0, 0, 1)));
    QVERIFY(!EmapProtocol::parsePose(posePayload(0, 0, 0, 0, 0, 0, 0, std::numeric_limits<float>::infinity())));
    QVERIFY(!EmapProtocol::parsePose(posePayload(0, 0, 0, 0, 0, 0, 0, 0)));  // no rotation at all
}

void MapProtocolTest::_testMapUnpacksBirksPacket()
{
    const auto cloud = EmapProtocol::parseMap(goldenMapPacket());
    QVERIFY(cloud.has_value());
    QCOMPARE(cloud->frameId, QStringLiteral("world"));
    QCOMPARE(cloud->stampMs, quint32(12345));
    QCOMPARE(cloud->points.size(), qsizetype(3));
    // Centimeter resolution relative to the anchor
    QCOMPARE_FUZZY(cloud->points[0].x(), 0.0f, 0.006f);
    QCOMPARE_FUZZY(cloud->points[1].x(), 5.0f, 0.006f);
    QCOMPARE_FUZZY(cloud->points[2].x(), 30.0f, 0.006f);
    QCOMPARE_FUZZY(cloud->points[2].y(), 0.0f, 0.006f);
}

void MapProtocolTest::_testMapRejectsGarbage()
{
    QVERIFY(!EmapProtocol::parseMap(QByteArray()));
    QVERIFY(!EmapProtocol::parseMap(QByteArray("not zstd at all")));
    QVERIFY(!EmapProtocol::parseMap(shortPacket()));         // unpacks, but has no header
    QVERIFY(!EmapProtocol::parseMap(inconsistentPacket()));  // header claims more points than it carries
}

void MapProtocolTest::_testStreamSplitAtAnyByte()
{
    const QByteArray stream = poseMessage(1, 2, 3) + mapMessage() + poseMessage(4, 5, 6);

    for (int chunk : {1, 3, 7, 12, 13, 1000}) {
        EmapStreamDecoder decoder;
        QList<EmapProtocol::Message> got;
        for (qsizetype i = 0; i < stream.size(); i += chunk) {
            got.append(decoder.feed(stream.mid(i, chunk)));
        }
        QCOMPARE(got.size(), qsizetype(3));
        QCOMPARE(got[0].type, EmapProtocol::kTypePose);
        QCOMPARE(got[1].type, EmapProtocol::kTypeMap);
        QCOMPARE(got[1].payload, goldenMapPacket());
        QCOMPARE(got[2].type, EmapProtocol::kTypePose);
        QCOMPARE(decoder.badBytes(), quint64(0));
    }
}

void MapProtocolTest::_testStreamRecoversAfterGarbage()
{
    EmapStreamDecoder decoder;
    const auto got = decoder.feed(QByteArray("\x01\x02garbage-without-header") + poseMessage(1, 2, 3));
    QCOMPARE(got.size(), qsizetype(1));
    QCOMPARE(got[0].type, EmapProtocol::kTypePose);
    QVERIFY(decoder.badBytes() > 0);
}

void MapProtocolTest::_testStreamSkipsHugeLengthAndUnknownVersion()
{
    EmapStreamDecoder decoder;

    // A length field that would make the decoder wait for megabytes
    QByteArray bad = message(EmapProtocol::kTypeMap, QByteArray(2, 'z'), 1, EmapProtocol::kMaxPayloadBytes + 1);
    auto got = decoder.feed(bad + poseMessage(1, 2, 3));
    QCOMPARE(got.size(), qsizetype(1));
    QCOMPARE(got[0].type, EmapProtocol::kTypePose);

    decoder.reset();
    got = decoder.feed(message(EmapProtocol::kTypeMap, QByteArray(2, 'z'), 9) + poseMessage(1, 2, 3));
    QCOMPARE(got.size(), qsizetype(1));
    QCOMPARE(got[0].type, EmapProtocol::kTypePose);
}

void MapProtocolTest::_testVehiclePlacementMatchesBodyFrame()
{
    // For any pose and any point in the vehicle body frame (x forward, y left, z up): a node with the vehicle
    // placement that carries the point (in scene units) must show it where the world point sits in the world scene.
    QRandomGenerator rng(7);
    for (int i = 0; i < 50; ++i) {
        EmapPose pose;
        pose.position = QVector3D(rng.bounded(40.0) - 20.0, rng.bounded(40.0) - 20.0, rng.bounded(6.0));
        pose.orientation = QQuaternion::fromAxisAndAngle(
            QVector3D(rng.bounded(2.0) - 1.0, rng.bounded(2.0) - 1.0, rng.bounded(2.0) - 1.0).normalized(),
            static_cast<float>(rng.bounded(360.0)));
        const QVector3D body(rng.bounded(60.0) - 30.0, rng.bounded(60.0) - 30.0, rng.bounded(6.0));

        QVector3D position;
        QQuaternion rotation;
        EmapProtocol::sceneVehiclePlacement(pose, &position, &rotation);

        const QVector3D world = pose.position + pose.orientation.rotatedVector(body);
        const QVector3D expected = PointCloudInstancing::toScenePosition(world);
        const QVector3D actual = rotation.rotatedVector(PointCloudInstancing::toScenePosition(body)) + position;

        QCOMPARE_FUZZY(actual.x(), expected.x(), 0.05f);
        QCOMPARE_FUZZY(actual.y(), expected.y(), 0.05f);
        QCOMPARE_FUZZY(actual.z(), expected.z(), 0.05f);
    }

    // A vehicle that turned 90 degrees to the left flies along the world y axis: what is straight ahead of it
    // (body x) ends up on the left of the world scene (scene -x)
    EmapPose turned;
    turned.orientation = QQuaternion::fromAxisAndAngle(0, 0, 1, 90);
    QVector3D position;
    QQuaternion rotation;
    EmapProtocol::sceneVehiclePlacement(turned, &position, &rotation);
    const QVector3D ahead =
        rotation.rotatedVector(PointCloudInstancing::toScenePosition(QVector3D(1, 0, 0))) + position;
    QCOMPARE_FUZZY(ahead.x(), -100.0f, 0.05f);
    QCOMPARE_FUZZY(ahead.z(), 0.0f, 0.05f);
}

void MapProtocolTest::_testControlMessageCarriesTrajectoryWish()
{
    const quint8 both = EmapProtocol::kFlagWantMap | EmapProtocol::kFlagWantTrajectory;
    QCOMPARE(EmapProtocol::encodeControl(true, true), message(EmapProtocol::kTypeControl, QByteArray(1, char(both))));
    QCOMPARE(EmapProtocol::encodeControl(false, true),
             message(EmapProtocol::kTypeControl, QByteArray(1, char(EmapProtocol::kFlagWantTrajectory))));
    // As before when the trajectory is not asked for
    QCOMPARE(EmapProtocol::encodeControl(true), EmapProtocol::encodeControl(true, false));
}

void MapProtocolTest::_testTrajectoryParsing()
{
    const QList<QList<QVector3D>> lines = {
        {QVector3D(0, 0, 0), QVector3D(1.5f, -2.0f, 0.25f)},
        {QVector3D(3, 3, 3), QVector3D(4, 4, 4), QVector3D(5, 5, 5)},
    };
    const auto got = EmapProtocol::parseTrajectory(trajectoryPayload(EmapProtocol::kKindBackup, lines));
    QVERIFY(got.has_value());
    QCOMPARE(got->kind, EmapProtocol::kKindBackup);
    QCOMPARE(got->lines, lines);

    // Nothing to draw is a valid state: the planner has no trajectory
    const auto empty = EmapProtocol::parseTrajectory(trajectoryPayload(EmapProtocol::kKindExp, {}));
    QVERIFY(empty.has_value());
    QVERIFY(empty->lines.isEmpty());

    // Through the stream decoder as it comes from the socket
    EmapStreamDecoder decoder;
    const auto messages = decoder.feed(trajectoryMessage(EmapProtocol::kKindExp, lines));
    QCOMPARE(messages.size(), qsizetype(1));
    QCOMPARE(messages[0].type, EmapProtocol::kTypeTrajectory);
}

void MapProtocolTest::_testTrajectoryRejectsBadInput()
{
    const QList<QList<QVector3D>> lines = {{QVector3D(0, 0, 0), QVector3D(1, 0, 0)}};
    const QByteArray good = trajectoryPayload(EmapProtocol::kKindExp, lines);
    QVERIFY(EmapProtocol::parseTrajectory(good).has_value());

    QVERIFY(!EmapProtocol::parseTrajectory(QByteArray()).has_value());
    QVERIFY(!EmapProtocol::parseTrajectory(good.left(good.size() - 1)).has_value());     // cut short
    QVERIFY(!EmapProtocol::parseTrajectory(good + QByteArray(1, char(0))).has_value());  // bytes left over
    QVERIFY(!EmapProtocol::parseTrajectory(trajectoryPayload(7, lines)).has_value());    // unknown kind

    const float nan = std::numeric_limits<float>::quiet_NaN();
    QVERIFY(!EmapProtocol::parseTrajectory(
                 trajectoryPayload(EmapProtocol::kKindExp, {{QVector3D(0, 0, 0), QVector3D(nan, 0, 0)}}))
                 .has_value());

    // A line count that promises more than there is
    QByteArray lying = good;
    lying[2] = char(5);
    QVERIFY(!EmapProtocol::parseTrajectory(lying).has_value());
}

void MapProtocolTest::_testControlMessage()
{
    const QByteArray on = EmapProtocol::encodeControl(true);
    const QByteArray off = EmapProtocol::encodeControl(false);
    QCOMPARE(on.size(), EmapProtocol::kHeaderSize + 1);
    QCOMPARE(on, message(EmapProtocol::kTypeControl, QByteArray(1, char(EmapProtocol::kFlagWantMap))));
    QCOMPARE(off, message(EmapProtocol::kTypeControl, QByteArray(1, char(0))));

    EmapStreamDecoder decoder;
    const auto got = decoder.feed(on + off);
    QCOMPARE(got.size(), qsizetype(2));
    QCOMPARE(got[0].type, EmapProtocol::kTypeControl);
    QCOMPARE(got[0].payload.size(), qsizetype(1));
}

namespace {
EmapPose poseAt(quint32 stampMs, float x, float yawDegrees = 0.0f)
{
    EmapPose pose;
    pose.stampMs = stampMs;
    pose.position = QVector3D(x, 0, 0);
    pose.orientation = QQuaternion::fromAxisAndAngle(0, 0, 1, yawDegrees);
    return pose;
}
}  // namespace

void MapProtocolTest::_testPoseHistoryInterpolates()
{
    EmapPoseHistory history;
    QVERIFY(!history.at(1000).has_value());

    history.add(poseAt(1000, 0.0f, 0.0f));
    history.add(poseAt(1100, 2.0f, 90.0f));
    history.add(poseAt(1200, 2.0f, 90.0f));

    const auto mid = history.at(1050);
    QVERIFY(mid.has_value());
    QCOMPARE_FUZZY(mid->position.x(), 1.0f, 0.001f);
    // Half of the 90 degrees turn
    const QVector3D forward = mid->orientation.rotatedVector(QVector3D(1, 0, 0));
    QCOMPARE_FUZZY(forward.x(), std::cos(qDegreesToRadians(45.0f)), 0.001f);
    QCOMPARE_FUZZY(forward.y(), std::sin(qDegreesToRadians(45.0f)), 0.001f);

    // Exactly on a stored pose
    QCOMPARE_FUZZY(history.at(1100)->position.x(), 2.0f, 0.001f);
    QCOMPARE(history.at(1050)->stampMs, quint32(1050));
}

void MapProtocolTest::_testPoseHistoryLimits()
{
    EmapPoseHistory history;
    history.add(poseAt(1000, 0.0f));
    history.add(poseAt(1100, 1.0f));

    // A little before the oldest and after the newest pose the nearest one is used, far outside there is no answer
    QCOMPARE_FUZZY(history.at(900)->position.x(), 0.0f, 0.001f);
    QCOMPARE_FUZZY(history.at(1400)->position.x(), 1.0f, 0.001f);
    QVERIFY(!history.at(1100 + EmapPoseHistory::kToleranceMs + 1).has_value());
    QVERIFY(!history.at(1000 - EmapPoseHistory::kToleranceMs - 1).has_value());

    // The same stamp replaces, an older one is ignored
    history.add(poseAt(1100, 5.0f));
    QCOMPARE(history.size(), 2);
    QCOMPARE_FUZZY(history.at(1100)->position.x(), 5.0f, 0.001f);
    history.add(poseAt(1050, 9.0f));
    QCOMPARE(history.size(), 2);

    // Only the last kKeepMs are kept
    for (quint32 t = 1200; t <= 9000; t += 100) {
        history.add(poseAt(t, 1.0f));
    }
    QVERIFY(!history.at(2000).has_value());
    QVERIFY(history.at(8000).has_value());

    // A jump back by more than that means the clock restarted
    history.add(poseAt(50, 7.0f));
    QCOMPARE(history.size(), 1);
    QCOMPARE_FUZZY(history.at(50)->position.x(), 7.0f, 0.001f);

    // The 32 bit stamp wraps around
    EmapPoseHistory wrapping;
    wrapping.add(poseAt(0xFFFFFF9Cu, 0.0f));  // 100 ms before the wrap
    wrapping.add(poseAt(100, 2.0f));          // 100 ms after
    QCOMPARE(wrapping.size(), 2);
    const auto atWrap = wrapping.at(0);
    QVERIFY(atWrap.has_value());
    QCOMPARE_FUZZY(atWrap->position.x(), 1.0f, 0.001f);
}
