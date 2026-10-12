#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QHash>
#include <QtCore/QList>
#include <QtGui/QVector3D>
#include <optional>

/// One complete LiDAR scan received over EPCL v1.
/// Points are in meters in the sensor body frame (x forward, y left, z up).
struct EpclFrame
{
    quint32 sequence = 0;
    quint32 stampMs = 0;
    QList<QVector3D> points;
    QList<quint8> intensity;  ///< Empty when the sender did not include intensity
};

/// Reassembles EPCL v1 datagrams into complete scans.
///
/// Wire format (little endian), 24 byte header followed by the points:
///   4s magic "EPCL", u8 version (1), u8 flags (bit 0: intensity byte per point),
///   u32 frame_seq, u32 stamp_ms, u32 total_points, u16 chunk_index, u16 chunk_count, u16 points_here
/// A point is x, y, z as int16 centimeters, optionally followed by u8 intensity.
///
/// An incomplete scan is dropped as soon as a newer scan starts, because a partial cloud
/// would be misleading. Pure logic, no sockets, so it can be unit tested without a network.
class EpclFrameAssembler
{
public:
    static constexpr int kHeaderSize = 24;
    static constexpr quint8 kVersion = 1;
    static constexpr quint8 kFlagIntensity = 0x01;
    static constexpr quint32 kMaxPointsPerFrame = 200000;

    /// Feeds one datagram. Returns the finished scan when this datagram completed it.
    std::optional<EpclFrame> feed(const QByteArray& datagram);

    void reset();

    quint64 completedFrames() const { return _completedFrames; }

    quint64 droppedFrames() const { return _droppedFrames; }

    quint64 badPackets() const { return _badPackets; }

private:
    struct Partial
    {
        quint32 stampMs = 0;
        quint32 totalPoints = 0;
        quint16 chunkCount = 0;
        quint8 flags = 0;
        QHash<quint16, QByteArray> chunks;
    };

    static bool _isOlder(quint32 a, quint32 b);
    std::optional<EpclFrame> _assemble(quint32 sequence, const Partial& partial);

    QHash<quint32, Partial> _partial;
    bool _hasNewest = false;
    quint32 _newest = 0;
    quint64 _completedFrames = 0;
    quint64 _droppedFrames = 0;
    quint64 _badPackets = 0;
};
