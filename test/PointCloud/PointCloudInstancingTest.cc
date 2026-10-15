#include "PointCloudInstancingTest.h"

#include <QtTest/QSignalSpy>

#include "MapReceiver.h"
#include "MapTestData.h"
#include "PointCloudInstancing.h"

static const int kEntrySize = static_cast<int>(sizeof(QQuick3DInstancing::InstanceTableEntry));

void PointCloudInstancingTest::_testInitialState()
{
    PointCloudInstancing inst;

    QCOMPARE(inst.count(), 0);
    int instanceCount = -1;
    QVERIFY(inst.getInstanceBuffer(&instanceCount).isEmpty());
    QCOMPARE(instanceCount, 0);
}

void PointCloudInstancingTest::_testToScenePosition()
{
    // 1 m forward, 2 m left, 3 m up in the body frame
    const QVector3D scene = PointCloudInstancing::toScenePosition(QVector3D(1, 2, 3));

    // Scene: x right (-left), y up, forward = -z, in centimeters
    QCOMPARE_FUZZY(scene.x(), -200.0f, 1e-3f);
    QCOMPARE_FUZZY(scene.y(), 300.0f, 1e-3f);
    QCOMPARE_FUZZY(scene.z(), -100.0f, 1e-3f);
}

void PointCloudInstancingTest::_testSetFrameBuffer()
{
    PointCloudInstancing inst;
    QSignalSpy spy(&inst, &PointCloudInstancing::countChanged);
    QVERIFY(spy.isValid());

    EpclFrame frame;
    frame.points = {QVector3D(1, 0, 0), QVector3D(0, 1, 0), QVector3D(0, 0, 1)};
    inst.setFrame(frame);

    QCOMPARE(spy.count(), 1);
    QCOMPARE(inst.count(), 3);
    int instanceCount = -1;
    const QByteArray buf = inst.getInstanceBuffer(&instanceCount);
    QCOMPARE(instanceCount, 3);
    QCOMPARE(buf.size(), qsizetype(3 * kEntrySize));
    QVERIFY(!inst._dirty);
}

void PointCloudInstancingTest::_testHeightColors()
{
    PointCloudInstancing inst;
    inst.setHeightMin(0.0f);
    inst.setHeightMax(2.0f);

    const QColor low = inst._colorForHeight(-5.0f);
    const QColor high = inst._colorForHeight(10.0f);
    const QColor mid = inst._colorForHeight(1.0f);

    // Clamped to the ends of the ramp: blue below, red above
    QCOMPARE(low.hsvHueF() > 0.6, true);
    QCOMPARE(high.hsvHueF() < 0.05, true);
    QVERIFY(low != high);
    QVERIFY(mid != low && mid != high);
}

void PointCloudInstancingTest::_testPointSizeMarksDirty()
{
    PointCloudInstancing inst;
    EpclFrame frame;
    frame.points = {QVector3D(1, 1, 1)};
    inst.setFrame(frame);
    inst.getInstanceBuffer(nullptr);
    QVERIFY(!inst._dirty);

    inst.setPointSize(8.0f);
    QVERIFY(inst._dirty);

    // Invalid sizes are rejected
    inst.getInstanceBuffer(nullptr);
    inst.setPointSize(-1.0f);
    QVERIFY(!inst._dirty);
    QCOMPARE_FUZZY(inst.pointSize(), 8.0f, 1e-4f);
}

void PointCloudInstancingTest::_testFrozenAndStamp()
{
    PointCloudInstancing inst;
    QSignalSpy frozenSpy(&inst, &PointCloudInstancing::frozenChanged);
    QSignalSpy stampSpy(&inst, &PointCloudInstancing::stampMsChanged);

    QVERIFY(!inst.frozen());
    QCOMPARE(inst.stampMs(), quint32(0));

    EpclFrame frame;
    frame.stampMs = 4242;
    frame.points = {QVector3D(1, 0, 0)};
    inst.setFrame(frame);
    QCOMPARE(inst.stampMs(), quint32(4242));
    QCOMPARE(stampSpy.count(), 1);

    inst.setFrozen(true);
    inst.setFrozen(true);  // no change, no signal
    QVERIFY(inst.frozen());
    QCOMPARE(frozenSpy.count(), 1);

    inst.setFrozen(false);
    QVERIFY(!inst.frozen());
    QCOMPARE(frozenSpy.count(), 2);
}

namespace {
EpclFrame frameAt(quint32 stampMs, const QList<QVector3D>& points)
{
    EpclFrame frame;
    frame.stampMs = stampMs;
    frame.points = points;
    return frame;
}

/// Scene position (centimeters) of the instance with the given index in the instance buffer
QVector3D instancePosition(const QByteArray& buffer, int index)
{
    const auto* entries = reinterpret_cast<const QQuick3DInstancing::InstanceTableEntry*>(buffer.constData());
    return entries[index].getPosition();
}
}  // namespace

void PointCloudInstancingTest::_testPersistenceKeepsPointsInTheWorld()
{
    using namespace MapTestData;

    MapReceiver poses;
    // The vehicle is at x = 0 m at 1000 ms and at x = 1 m at 1200 ms, not turned
    poses.handleBytes(poseMessage(0, 0, 0, 0, 0, 0, 1, 1000) + poseMessage(1, 0, 0, 0, 0, 0, 1, 1200));

    PointCloudInstancing inst;
    inst.setPoseSource(&poses);
    QSignalSpy worldSpy(&inst, &PointCloudInstancing::worldModeChanged);
    QVERIFY(!inst.worldMode());

    // The same wall point, 2 m ahead of the vehicle in both scans
    inst.setFrame(frameAt(1000, {QVector3D(2, 0, 0)}));
    QVERIFY(inst.worldMode());
    QCOMPARE(worldSpy.count(), 1);
    inst.setFrame(frameAt(1200, {QVector3D(2, 0, 0)}));

    // Two points in the world: x = 2 m and x = 3 m, not one point that follows the vehicle
    QCOMPARE(inst.count(), 2);
    int instanceCount = -1;
    const QByteArray buffer = inst.getInstanceBuffer(&instanceCount);
    QCOMPARE(instanceCount, 2);
    QCOMPARE(buffer.size(), qsizetype(2 * kEntrySize));

    QList<float> forward;  // scene -z is forward, in centimeters
    for (int i = 0; i < 2; ++i) {
        forward.append(-instancePosition(buffer, i).z());
    }
    std::sort(forward.begin(), forward.end());
    QCOMPARE_FUZZY(forward[0], 200.0f, 0.01f);
    QCOMPARE_FUZZY(forward[1], 300.0f, 0.01f);
}

void PointCloudInstancingTest::_testPersistenceDropsOldPointsAndMergesCells()
{
    using namespace MapTestData;

    MapReceiver poses;
    PointCloudInstancing inst;
    inst.setPoseSource(&poses);
    inst.setPersistenceMs(5000);

    // As on the vehicle the pose of a scan arrives together with it. The vehicle stays at the origin.
    const auto scan = [&](quint32 stampMs, const QList<QVector3D>& points) {
        poses.handleBytes(poseMessage(0, 0, 0, 0, 0, 0, 1, stampMs));
        inst.setFrame(frameAt(stampMs, points));
    };

    // Points less than one cell (10 cm) apart share a cell
    scan(1000, {QVector3D(2.02f, 0, 0), QVector3D(2.07f, 0.02f, 0), QVector3D(0, 3, 0)});
    QCOMPARE(inst.count(), 2);

    // Seeing the same point again only refreshes it
    scan(3000, {QVector3D(2.02f, 0, 0)});
    QCOMPARE(inst.count(), 2);

    // 7000 ms: the point at (0, 3) was last seen at 1000 ms, 6000 ms ago, and drops out; (2, 0) was seen at 3000 ms
    scan(7000, {QVector3D(5, 0, 0)});
    QCOMPARE(inst.count(), 2);  // (2, 0) from 3000 ms and (5, 0)

    // 9000 ms: (2, 0) is now 6000 ms old as well
    scan(9000, {QVector3D(5, 0, 0)});
    QCOMPARE(inst.count(), 1);
}

void PointCloudInstancingTest::_testPersistenceNeedsPoseAndSwitchesOff()
{
    using namespace MapTestData;

    PointCloudInstancing inst;
    MapReceiver poses;
    inst.setPoseSource(&poses);

    // No pose at all: the latest scan in the body frame, as without persistence
    inst.setFrame(frameAt(1000, {QVector3D(1, 0, 0), QVector3D(0, 1, 0)}));
    QVERIFY(!inst.worldMode());
    QCOMPARE(inst.count(), 2);

    // A pose that does not cover the stamp of the scan does not help either
    poses.handleBytes(poseMessage(0, 0, 0, 0, 0, 0, 1, 50000));
    inst.setFrame(frameAt(1000, {QVector3D(1, 0, 0), QVector3D(0, 1, 0), QVector3D(0, 0, 1)}));
    QVERIFY(!inst.worldMode());
    QCOMPARE(inst.count(), 3);

    // Now it does
    inst.setFrame(frameAt(50000, {QVector3D(1, 0, 0)}));
    QVERIFY(inst.worldMode());
    QCOMPARE(inst.count(), 1);

    // Persistence 0 goes back to the latest scan alone
    inst.setPersistenceMs(0);
    QVERIFY(!inst.worldMode());
    inst.setFrame(frameAt(50000, {QVector3D(1, 0, 0), QVector3D(0, 1, 0)}));
    QVERIFY(!inst.worldMode());
    QCOMPARE(inst.count(), 2);
}
