#include "EpclFrameAssembler.h"

#include <QtCore/QtEndian>

#include "QGCLoggingCategory.h"

QGC_LOGGING_CATEGORY(EpclFrameAssemblerLog, "PointCloud.EpclFrameAssembler")

namespace {

constexpr char kMagic[4] = {'E', 'P', 'C', 'L'};
constexpr float kMetersPerCentimeter = 0.01f;

int bytesPerPoint(bool withIntensity)
{
    return withIntensity ? 7 : 6;
}

}  // namespace

bool EpclFrameAssembler::_isOlder(quint32 a, quint32 b)
{
    // Sequence numbers wrap at 32 bit
    return a != b && static_cast<quint32>(b - a) < 0x80000000u;
}

void EpclFrameAssembler::reset()
{
    _partial.clear();
    _hasNewest = false;
    _newest = 0;
}

std::optional<EpclFrame> EpclFrameAssembler::feed(const QByteArray& datagram)
{
    if (datagram.size() < kHeaderSize) {
        _badPackets++;
        return std::nullopt;
    }

    const auto* raw = reinterpret_cast<const uchar*>(datagram.constData());
    if (memcmp(raw, kMagic, sizeof(kMagic)) != 0) {
        _badPackets++;
        return std::nullopt;
    }

    const quint8 version = raw[4];
    const quint8 flags = raw[5];
    const quint32 sequence = qFromLittleEndian<quint32>(raw + 6);
    const quint32 stampMs = qFromLittleEndian<quint32>(raw + 10);
    const quint32 totalPoints = qFromLittleEndian<quint32>(raw + 14);
    const quint16 chunkIndex = qFromLittleEndian<quint16>(raw + 18);
    const quint16 chunkCount = qFromLittleEndian<quint16>(raw + 20);
    const quint16 pointsHere = qFromLittleEndian<quint16>(raw + 22);

    const int perPoint = bytesPerPoint(flags & kFlagIntensity);
    if (version != kVersion || chunkCount == 0 || chunkIndex >= chunkCount || totalPoints > kMaxPointsPerFrame ||
        datagram.size() != kHeaderSize + pointsHere * perPoint) {
        _badPackets++;
        return std::nullopt;
    }

    if (_hasNewest && _isOlder(sequence, _newest)) {
        // Late datagram of a scan we already gave up on
        return std::nullopt;
    }

    if (!_hasNewest || _isOlder(_newest, sequence)) {
        _droppedFrames += _partial.size();
        _partial.clear();
        _newest = sequence;
        _hasNewest = true;
    }

    Partial& partial = _partial[sequence];
    if (partial.chunks.isEmpty()) {
        partial.stampMs = stampMs;
        partial.totalPoints = totalPoints;
        partial.chunkCount = chunkCount;
        partial.flags = flags;
    } else if (partial.chunkCount != chunkCount || partial.flags != flags || partial.totalPoints != totalPoints) {
        // Chunks of one scan must agree on the layout
        _badPackets++;
        return std::nullopt;
    }

    partial.chunks.insert(chunkIndex, datagram.mid(kHeaderSize));
    if (partial.chunks.size() < partial.chunkCount) {
        return std::nullopt;
    }

    const std::optional<EpclFrame> frame = _assemble(sequence, partial);
    _partial.remove(sequence);
    if (!frame) {
        _badPackets++;
        return std::nullopt;
    }
    _completedFrames++;
    return frame;
}

std::optional<EpclFrame> EpclFrameAssembler::_assemble(quint32 sequence, const Partial& partial)
{
    const bool withIntensity = partial.flags & kFlagIntensity;
    const int perPoint = bytesPerPoint(withIntensity);

    EpclFrame frame;
    frame.sequence = sequence;
    frame.stampMs = partial.stampMs;
    frame.points.reserve(static_cast<qsizetype>(partial.totalPoints));
    if (withIntensity) {
        frame.intensity.reserve(static_cast<qsizetype>(partial.totalPoints));
    }

    for (quint16 index = 0; index < partial.chunkCount; ++index) {
        const auto it = partial.chunks.constFind(index);
        if (it == partial.chunks.constEnd()) {
            qCWarning(EpclFrameAssemblerLog) << "Chunk" << index << "missing in complete frame" << sequence;
            return std::nullopt;
        }

        const auto* data = reinterpret_cast<const uchar*>(it->constData());
        const int pointCount = static_cast<int>(it->size() / perPoint);
        for (int i = 0; i < pointCount; ++i) {
            const uchar* p = data + i * perPoint;
            const qint16 x = qFromLittleEndian<qint16>(p);
            const qint16 y = qFromLittleEndian<qint16>(p + 2);
            const qint16 z = qFromLittleEndian<qint16>(p + 4);
            frame.points.append(
                QVector3D(x * kMetersPerCentimeter, y * kMetersPerCentimeter, z * kMetersPerCentimeter));
            if (withIntensity) {
                frame.intensity.append(p[6]);
            }
        }
    }

    if (static_cast<quint32>(frame.points.size()) != partial.totalPoints) {
        qCWarning(EpclFrameAssemblerLog) << "Frame" << sequence << "has" << frame.points.size()
                                         << "points, header announced" << partial.totalPoints;
        return std::nullopt;
    }
    return frame;
}
