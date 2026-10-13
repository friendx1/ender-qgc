#include "PointCloudInstancingTest.h"

#include <QtTest/QSignalSpy>

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
