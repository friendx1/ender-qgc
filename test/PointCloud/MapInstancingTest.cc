#include "MapInstancingTest.h"

#include <QtTest/QSignalSpy>

#include "MapInstancing.h"
#include "MapReceiver.h"
#include "MapTestData.h"

static const int kEntrySize = static_cast<int>(sizeof(QQuick3DInstancing::InstanceTableEntry));

void MapInstancingTest::_testInitialState()
{
    MapInstancing inst;

    QCOMPARE(inst.count(), 0);
    int instanceCount = -1;
    QVERIFY(inst.getInstanceBuffer(&instanceCount).isEmpty());
    QCOMPARE(instanceCount, 0);
}

void MapInstancingTest::_testSetCloudBuffer()
{
    MapInstancing inst;
    QSignalSpy spy(&inst, &MapInstancing::countChanged);

    EmapCloud cloud;
    cloud.points = {QVector3D(1, 0, 0), QVector3D(0, 1, 0), QVector3D(0, 0, 1)};
    inst.setCloud(cloud);

    QCOMPARE(spy.count(), 1);
    QCOMPARE(inst.count(), 3);
    int instanceCount = -1;
    const QByteArray buf = inst.getInstanceBuffer(&instanceCount);
    QCOMPARE(instanceCount, 3);
    QCOMPARE(buf.size(), qsizetype(3 * kEntrySize));
    QVERIFY(!inst._dirty);

    // A flat map (all points at one height) must not divide by zero
    EmapCloud flat;
    flat.points = {QVector3D(1, 0, 2), QVector3D(2, 0, 2)};
    inst.setCloud(flat);
    QCOMPARE(inst.getInstanceBuffer(&instanceCount).size(), qsizetype(2 * kEntrySize));
}

void MapInstancingTest::_testFollowsReceiver()
{
    MapReceiver receiver;
    MapInstancing inst;
    inst.setReceiver(&receiver);

    receiver.handleBytes(MapTestData::poseMessage(0, 0, 0) + MapTestData::mapMessage());

    QCOMPARE(inst.count(), 2);
}

void MapInstancingTest::_testPointSizeMarksDirty()
{
    MapInstancing inst;
    EmapCloud cloud;
    cloud.points = {QVector3D(1, 1, 1)};
    inst.setCloud(cloud);
    int instanceCount = 0;
    inst.getInstanceBuffer(&instanceCount);
    QVERIFY(!inst._dirty);

    inst.setPointSize(20.0f);
    QVERIFY(inst._dirty);

    inst.getInstanceBuffer(&instanceCount);
    inst.setPointSize(20.0f);  // unchanged
    QVERIFY(!inst._dirty);
}
