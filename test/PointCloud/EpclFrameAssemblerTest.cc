#include "EpclFrameAssemblerTest.h"

#include <QtCore/QtEndian>

#include "EpclFrameAssembler.h"

namespace {

struct TestPoint
{
    qint16 x, y, z;
    quint8 intensity;
};

template <typename T>
void append(QByteArray& out, T value)
{
    T le = qToLittleEndian(value);
    out.append(reinterpret_cast<const char*>(&le), sizeof(le));
}

/// Builds one EPCL v1 datagram carrying points [first, first + count) of a scan
QByteArray makeDatagram(quint32 seq, quint32 stampMs, const QList<TestPoint>& all, quint16 chunkIndex,
                        quint16 chunkCount, int first, int count, bool withIntensity = false,
                        quint32 totalOverride = 0xFFFFFFFF)
{
    QByteArray out;
    out.append("EPCL", 4);
    out.append(static_cast<char>(1));
    out.append(static_cast<char>(withIntensity ? 1 : 0));
    append<quint32>(out, seq);
    append<quint32>(out, stampMs);
    append<quint32>(out, totalOverride == 0xFFFFFFFF ? static_cast<quint32>(all.size()) : totalOverride);
    append<quint16>(out, chunkIndex);
    append<quint16>(out, chunkCount);
    append<quint16>(out, static_cast<quint16>(count));
    for (int i = first; i < first + count; ++i) {
        append<qint16>(out, all[i].x);
        append<qint16>(out, all[i].y);
        append<qint16>(out, all[i].z);
        if (withIntensity) {
            out.append(static_cast<char>(all[i].intensity));
        }
    }
    return out;
}

QList<TestPoint> makePoints(int n)
{
    QList<TestPoint> pts;
    for (int i = 0; i < n; ++i) {
        pts.append(
            {static_cast<qint16>(i), static_cast<qint16>(-i), static_cast<qint16>(i * 2), static_cast<quint8>(i)});
    }
    return pts;
}

}  // namespace

void EpclFrameAssemblerTest::_testSingleChunkFrame()
{
    EpclFrameAssembler assembler;
    const QList<TestPoint> pts = {{100, -50, 25, 0}, {-327, 0, 32767, 0}};

    const auto frame = assembler.feed(makeDatagram(7, 1234, pts, 0, 1, 0, 2));
    QVERIFY(frame.has_value());
    QCOMPARE(frame->sequence, 7u);
    QCOMPARE(frame->stampMs, 1234u);
    QCOMPARE(frame->points.size(), qsizetype(2));
    QCOMPARE_FUZZY(frame->points[0].x(), 1.0f, 1e-4f);
    QCOMPARE_FUZZY(frame->points[0].y(), -0.5f, 1e-4f);
    QCOMPARE_FUZZY(frame->points[0].z(), 0.25f, 1e-4f);
    QCOMPARE_FUZZY(frame->points[1].x(), -3.27f, 1e-4f);
    QCOMPARE_FUZZY(frame->points[1].z(), 327.67f, 1e-3f);
    QVERIFY(frame->intensity.isEmpty());
    QCOMPARE(assembler.completedFrames(), quint64(1));
}

void EpclFrameAssemblerTest::_testMultiChunkFrame()
{
    EpclFrameAssembler assembler;
    const QList<TestPoint> pts = makePoints(10);

    QVERIFY(!assembler.feed(makeDatagram(1, 0, pts, 0, 3, 0, 4)));
    QVERIFY(!assembler.feed(makeDatagram(1, 0, pts, 1, 3, 4, 4)));
    const auto frame = assembler.feed(makeDatagram(1, 0, pts, 2, 3, 8, 2));

    QVERIFY(frame.has_value());
    QCOMPARE(frame->points.size(), qsizetype(10));
    for (int i = 0; i < 10; ++i) {
        QCOMPARE_FUZZY(frame->points[i].x(), i * 0.01f, 1e-4f);
        QCOMPARE_FUZZY(frame->points[i].z(), i * 0.02f, 1e-4f);
    }
}

void EpclFrameAssemblerTest::_testChunksOutOfOrder()
{
    EpclFrameAssembler assembler;
    const QList<TestPoint> pts = makePoints(6);

    QVERIFY(!assembler.feed(makeDatagram(1, 0, pts, 2, 3, 4, 2)));
    QVERIFY(!assembler.feed(makeDatagram(1, 0, pts, 0, 3, 0, 2)));
    const auto frame = assembler.feed(makeDatagram(1, 0, pts, 1, 3, 2, 2));

    QVERIFY(frame.has_value());
    QCOMPARE(frame->points.size(), qsizetype(6));
    QCOMPARE_FUZZY(frame->points[0].x(), 0.0f, 1e-4f);
    QCOMPARE_FUZZY(frame->points[5].x(), 0.05f, 1e-4f);
}

void EpclFrameAssemblerTest::_testIntensity()
{
    EpclFrameAssembler assembler;
    const QList<TestPoint> pts = makePoints(3);

    const auto frame = assembler.feed(makeDatagram(1, 0, pts, 0, 1, 0, 3, true));
    QVERIFY(frame.has_value());
    QCOMPARE(frame->intensity.size(), qsizetype(3));
    QCOMPARE(frame->intensity[0], quint8(0));
    QCOMPARE(frame->intensity[2], quint8(2));
}

void EpclFrameAssemblerTest::_testEmptyFrame()
{
    EpclFrameAssembler assembler;

    const auto frame = assembler.feed(makeDatagram(1, 0, {}, 0, 1, 0, 0));
    QVERIFY(frame.has_value());
    QVERIFY(frame->points.isEmpty());
}

void EpclFrameAssemblerTest::_testIncompleteFrameDroppedByNewerFrame()
{
    EpclFrameAssembler assembler;
    const QList<TestPoint> pts = makePoints(4);

    // Frame 1 never completes
    QVERIFY(!assembler.feed(makeDatagram(1, 0, pts, 0, 2, 0, 2)));
    // Frame 2 starts and completes
    const auto frame = assembler.feed(makeDatagram(2, 0, pts, 0, 1, 0, 4));

    QVERIFY(frame.has_value());
    QCOMPARE(frame->sequence, 2u);
    QCOMPARE(assembler.droppedFrames(), quint64(1));
    QCOMPARE(assembler.completedFrames(), quint64(1));
}

void EpclFrameAssemblerTest::_testLateChunkOfOldFrameIgnored()
{
    EpclFrameAssembler assembler;
    const QList<TestPoint> pts = makePoints(4);

    QVERIFY(!assembler.feed(makeDatagram(1, 0, pts, 0, 2, 0, 2)));
    QVERIFY(assembler.feed(makeDatagram(2, 0, pts, 0, 1, 0, 4)).has_value());
    // The missing half of frame 1 arrives after frame 2 was shown
    QVERIFY(!assembler.feed(makeDatagram(1, 0, pts, 1, 2, 2, 2)));
    QCOMPARE(assembler.completedFrames(), quint64(1));
    QCOMPARE(assembler.badPackets(), quint64(0));
}

void EpclFrameAssemblerTest::_testBadMagicAndVersion()
{
    EpclFrameAssembler assembler;
    const QList<TestPoint> pts = makePoints(1);

    QByteArray badMagic = makeDatagram(1, 0, pts, 0, 1, 0, 1);
    badMagic[0] = 'X';
    QVERIFY(!assembler.feed(badMagic));

    QByteArray badVersion = makeDatagram(1, 0, pts, 0, 1, 0, 1);
    badVersion[4] = 2;
    QVERIFY(!assembler.feed(badVersion));

    QCOMPARE(assembler.badPackets(), quint64(2));
    QCOMPARE(assembler.completedFrames(), quint64(0));
}

void EpclFrameAssemblerTest::_testTruncatedDatagram()
{
    EpclFrameAssembler assembler;
    const QList<TestPoint> pts = makePoints(3);

    QVERIFY(!assembler.feed(QByteArray("EPCL")));
    QByteArray cut = makeDatagram(1, 0, pts, 0, 1, 0, 3);
    cut.chop(2);
    QVERIFY(!assembler.feed(cut));
    QCOMPARE(assembler.badPackets(), quint64(2));
}

void EpclFrameAssemblerTest::_testInconsistentChunks()
{
    EpclFrameAssembler assembler;
    const QList<TestPoint> pts = makePoints(4);

    QVERIFY(!assembler.feed(makeDatagram(1, 0, pts, 0, 2, 0, 2)));
    // Same frame announces a different chunk count
    QVERIFY(!assembler.feed(makeDatagram(1, 0, pts, 1, 3, 2, 2)));
    QCOMPARE(assembler.badPackets(), quint64(1));
}

void EpclFrameAssemblerTest::_testSequenceWrapAround()
{
    EpclFrameAssembler assembler;
    const QList<TestPoint> pts = makePoints(2);

    QVERIFY(assembler.feed(makeDatagram(0xFFFFFFFFu, 0, pts, 0, 1, 0, 2)).has_value());
    const auto next = assembler.feed(makeDatagram(0u, 0, pts, 0, 1, 0, 2));
    QVERIFY(next.has_value());
    QCOMPARE(next->sequence, 0u);
    QCOMPARE(assembler.completedFrames(), quint64(2));
}

void EpclFrameAssemblerTest::_testPointCountMismatch()
{
    EpclFrameAssembler assembler;
    const QList<TestPoint> pts = makePoints(3);

    // Header claims 5 points, the chunk only carries 3
    QVERIFY(!assembler.feed(makeDatagram(1, 0, pts, 0, 1, 0, 3, false, 5)));
    QCOMPARE(assembler.completedFrames(), quint64(0));
    QCOMPARE(assembler.badPackets(), quint64(1));
}
