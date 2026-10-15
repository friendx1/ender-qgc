#include "TrajectoryInstancingTest.h"

#include <QtTest/QSignalSpy>

#include "MapReceiver.h"
#include "MapTestData.h"
#include "TrajectoryInstancing.h"

static const int kEntrySize = static_cast<int>(sizeof(QQuick3DInstancing::InstanceTableEntry));

void TrajectoryInstancingTest::_testInitialState()
{
    TrajectoryInstancing inst;

    QCOMPARE(inst.count(), 0);
    int instanceCount = -1;
    QVERIFY(inst.getInstanceBuffer(&instanceCount).isEmpty());
    QCOMPARE(instanceCount, 0);
}

void TrajectoryInstancingTest::_testResampleSpacingAndEnds()
{
    // 1 m straight, then a corner and 0.5 m up: with 0.25 m spacing 4 + 2 steps and the end point
    const QList<QList<QVector3D>> lines = {{QVector3D(0, 0, 0), QVector3D(1, 0, 0), QVector3D(1, 0, 0.5f)}};
    const QList<QVector3D> points = TrajectoryInstancing::resample(lines, 0.25f, 1000);

    QCOMPARE(points.size(), qsizetype(7));
    QCOMPARE(points.first(), QVector3D(0, 0, 0));
    QCOMPARE(points[4], QVector3D(1, 0, 0));  // the corner is kept
    QCOMPARE(points.last(), QVector3D(1, 0, 0.5f));
    for (qsizetype i = 1; i < points.size(); ++i) {
        QVERIFY(points[i].distanceToPoint(points[i - 1]) <= 0.25f + 1e-4f);
    }

    // Short segments still give their start point, an empty line gives nothing, several lines are not joined
    const QList<QList<QVector3D>> two = {{QVector3D(0, 0, 0), QVector3D(0.01f, 0, 0)},
                                         {QVector3D(5, 0, 0), QVector3D(5.01f, 0, 0)}};
    QCOMPARE(TrajectoryInstancing::resample(two, 0.25f, 1000).size(), qsizetype(4));
    QVERIFY(TrajectoryInstancing::resample({}, 0.25f, 1000).isEmpty());
    QVERIFY(TrajectoryInstancing::resample({{}}, 0.25f, 1000).isEmpty());
}

void TrajectoryInstancingTest::_testResampleCapsTheCount()
{
    // 100 m of line with 1 cm spacing would be 10000 points
    const QList<QList<QVector3D>> lines = {{QVector3D(0, 0, 0), QVector3D(100, 0, 0)}};
    const QList<QVector3D> points = TrajectoryInstancing::resample(lines, 0.01f, 500);
    QVERIFY(points.size() <= 502);
    QVERIFY(points.size() >= 400);
    QCOMPARE(points.last(), QVector3D(100, 0, 0));
}

void TrajectoryInstancingTest::_testSetLinesBuffer()
{
    TrajectoryInstancing inst;
    QSignalSpy spy(&inst, &TrajectoryInstancing::countChanged);

    inst.setLines({{QVector3D(0, 0, 0), QVector3D(0.08f, 0, 0)}});
    QCOMPARE(spy.count(), 1);
    QCOMPARE(inst.count(), 3);  // two steps of 4 cm and the end
    int instanceCount = -1;
    const QByteArray buffer = inst.getInstanceBuffer(&instanceCount);
    QCOMPARE(instanceCount, 3);
    QCOMPARE(buffer.size(), qsizetype(3 * kEntrySize));
    QVERIFY(!inst._dirty);

    // The last cube is at the end of the line: 8 cm forward is scene -z in centimeters
    const auto* entries = reinterpret_cast<const QQuick3DInstancing::InstanceTableEntry*>(buffer.constData());
    QCOMPARE_FUZZY(entries[2].getPosition().z(), -8.0f, 1e-3f);

    inst.setLines({});
    QCOMPARE(inst.count(), 0);
    QVERIFY(inst.getInstanceBuffer(&instanceCount).isEmpty());
    QCOMPARE(instanceCount, 0);
}

void TrajectoryInstancingTest::_testFollowsReceiverAndKind()
{
    using namespace MapTestData;

    MapReceiver receiver;
    receiver.setWantTrajectory(true);
    TrajectoryInstancing exp;
    TrajectoryInstancing backup;
    backup.setKind(EmapProtocol::kKindBackup);
    exp.setReceiver(&receiver);
    backup.setReceiver(&receiver);

    receiver.handleBytes(trajectoryMessage(EmapProtocol::kKindExp, {{QVector3D(0, 0, 0), QVector3D(1, 0, 0)}}));
    QCOMPARE(exp.count(), 26);  // 25 steps of 4 cm and the end
    QCOMPARE(backup.count(), 0);

    receiver.handleBytes(trajectoryMessage(EmapProtocol::kKindBackup, {{QVector3D(0, 0, 0), QVector3D(0, 0.04f, 0)}}));
    QCOMPARE(exp.count(), 26);
    QCOMPARE(backup.count(), 2);

    // Gone with the wish; a kind that does not exist draws nothing
    receiver.setWantTrajectory(false);
    QCOMPARE(exp.count(), 0);
    QCOMPARE(backup.count(), 0);
    backup.setKind(5);
    QCOMPARE(backup.count(), 0);

    // A receiver that is set later shows what it already has
    MapReceiver other;
    other.setWantTrajectory(true);
    other.handleBytes(trajectoryMessage(EmapProtocol::kKindExp, {{QVector3D(0, 0, 0), QVector3D(0.04f, 0, 0)}}));
    TrajectoryInstancing late;
    late.setReceiver(&other);
    QCOMPARE(late.count(), 2);
}

void TrajectoryInstancingTest::_testPointSizeAndColorMarkDirty()
{
    TrajectoryInstancing inst;
    inst.setLines({{QVector3D(0, 0, 0), QVector3D(1, 0, 0)}});
    inst.getInstanceBuffer(nullptr);
    QVERIFY(!inst._dirty);

    inst.setPointSize(9.0f);
    QVERIFY(inst._dirty);
    inst.getInstanceBuffer(nullptr);

    inst.setColor(QColor(Qt::red));
    QVERIFY(inst._dirty);
    inst.getInstanceBuffer(nullptr);

    // Invalid sizes are rejected
    inst.setPointSize(-1.0f);
    QVERIFY(!inst._dirty);
    QCOMPARE_FUZZY(inst.pointSize(), 9.0f, 1e-4f);
}
