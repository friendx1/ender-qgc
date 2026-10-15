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

void MapProtocolTest::_testSceneTransformMatchesBodyFrame()
{
    // For any pose and any world point: drawing the world point with the node transform must land where the
    // point sits in the vehicle body frame (x forward, y left, z up), converted to scene coordinates.
    QRandomGenerator rng(7);
    for (int i = 0; i < 50; ++i) {
        EmapPose pose;
        pose.position = QVector3D(rng.bounded(40.0) - 20.0, rng.bounded(40.0) - 20.0, rng.bounded(6.0));
        pose.orientation = QQuaternion::fromAxisAndAngle(
            QVector3D(rng.bounded(2.0) - 1.0, rng.bounded(2.0) - 1.0, rng.bounded(2.0) - 1.0).normalized(),
            static_cast<float>(rng.bounded(360.0)));
        const QVector3D world(rng.bounded(60.0) - 30.0, rng.bounded(60.0) - 30.0, rng.bounded(6.0));

        QVector3D position;
        QQuaternion rotation;
        EmapProtocol::sceneTransform(pose, &position, &rotation);

        const QVector3D body = pose.orientation.inverted().rotatedVector(world - pose.position);
        const QVector3D expected = PointCloudInstancing::toScenePosition(body);
        const QVector3D actual = rotation.rotatedVector(PointCloudInstancing::toScenePosition(world)) + position;

        QCOMPARE_FUZZY(actual.x(), expected.x(), 0.05f);
        QCOMPARE_FUZZY(actual.y(), expected.y(), 0.05f);
        QCOMPARE_FUZZY(actual.z(), expected.z(), 0.05f);
    }

    // A vehicle that turned 90 degrees to the left sees the world x axis on its right
    EmapPose turned;
    turned.orientation = QQuaternion::fromAxisAndAngle(0, 0, 1, 90);
    QVector3D position;
    QQuaternion rotation;
    EmapProtocol::sceneTransform(turned, &position, &rotation);
    const QVector3D scene =
        rotation.rotatedVector(PointCloudInstancing::toScenePosition(QVector3D(1, 0, 0))) + position;
    QCOMPARE_FUZZY(scene.x(), 100.0f, 0.05f);  // right on the screen
    QCOMPARE_FUZZY(scene.z(), 0.0f, 0.05f);
}
