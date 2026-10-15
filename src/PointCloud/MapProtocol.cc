#include "MapProtocol.h"

#include <QtCore/QtEndian>
#include <bit>
#include <cmath>
#include <cstring>

#include "PointCloudInstancing.h"
#include "QGCCompression.h"
#include "QGCLoggingCategory.h"

QGC_LOGGING_CATEGORY(MapProtocolLog, "PointCloud.MapProtocol")

namespace {

constexpr char kMagic[] = {'E', 'M', 'A', 'P'};
constexpr char kZstdMagic[] = {'\x28', '\xb5', '\x2f', '\xfd'};
constexpr int kMagicSize = 4;
constexpr int kCloudHeaderSize = 38;  // u32 u32 f64 f64 f64 u32 u16, no padding
constexpr float kCentimeterToMeter = 0.01f;

template <typename T>
T readLe(const char* p)
{
    T value;
    std::memcpy(&value, p, sizeof(T));
    return qFromLittleEndian(value);
}

double readDoubleLe(const char* p)
{
    return std::bit_cast<double>(readLe<quint64>(p));
}

float readFloatLe(const char* p)
{
    return std::bit_cast<float>(readLe<quint32>(p));
}

}  // namespace

std::optional<EmapPose> EmapProtocol::parsePose(const QByteArray& payload)
{
    if (payload.size() != kPoseSize) {
        return std::nullopt;
    }
    const char* p = payload.constData();

    EmapPose pose;
    pose.stampMs = readLe<quint32>(p);
    const double x = readDoubleLe(p + 4);
    const double y = readDoubleLe(p + 12);
    const double z = readDoubleLe(p + 20);
    const float qx = readFloatLe(p + 28);
    const float qy = readFloatLe(p + 32);
    const float qz = readFloatLe(p + 36);
    const float qw = readFloatLe(p + 40);

    const bool finite = std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(qx) &&
                        std::isfinite(qy) && std::isfinite(qz) && std::isfinite(qw);
    pose.orientation = QQuaternion(qw, qx, qy, qz);
    if (!finite || pose.orientation.lengthSquared() < 1e-6f) {
        return std::nullopt;
    }
    pose.orientation.normalize();
    pose.position = QVector3D(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    return pose;
}

std::optional<EmapCloud> EmapProtocol::parseMap(const QByteArray& payload)
{
    if (!payload.startsWith(QByteArrayView(kZstdMagic, kMagicSize))) {
        qCDebug(MapProtocolLog) << "Map packet is not a zstd frame";
        return std::nullopt;
    }
    const QByteArray raw = QGCCompression::decompressData(payload, QGCCompression::Format::ZSTD, kMaxUnpackedBytes);
    if (raw.size() < kCloudHeaderSize) {
        qCDebug(MapProtocolLog) << "Map packet does not unpack to a header";
        return std::nullopt;
    }

    const char* p = raw.constData();
    const quint32 sec = readLe<quint32>(p);
    const quint32 nsec = readLe<quint32>(p + 4);
    const double anchorX = readDoubleLe(p + 8);
    const double anchorY = readDoubleLe(p + 16);
    const double anchorZ = readDoubleLe(p + 24);
    const quint32 count = readLe<quint32>(p + 32);
    const quint16 idLength = readLe<quint16>(p + 36);

    if (!std::isfinite(anchorX) || !std::isfinite(anchorY) || !std::isfinite(anchorZ) || count > kMaxPoints) {
        return std::nullopt;
    }
    const qint64 expected = qint64(kCloudHeaderSize) + idLength + qint64(count) * 6;
    if (expected != raw.size()) {
        qCDebug(MapProtocolLog) << "Map packet length" << raw.size() << "does not match" << count << "points";
        return std::nullopt;
    }

    EmapCloud cloud;
    cloud.stampMs = static_cast<quint32>(sec * 1000ull + nsec / 1000000u);
    cloud.frameId = QString::fromLatin1(p + kCloudHeaderSize, idLength);
    cloud.points.reserve(static_cast<qsizetype>(count));
    const char* data = p + kCloudHeaderSize + idLength;
    for (quint32 i = 0; i < count; ++i, data += 6) {
        const double x = anchorX + readLe<qint16>(data) * double(kCentimeterToMeter);
        const double y = anchorY + readLe<qint16>(data + 2) * double(kCentimeterToMeter);
        const double z = anchorZ + readLe<qint16>(data + 4) * double(kCentimeterToMeter);
        cloud.points.append(QVector3D(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
    }
    return cloud;
}

QQuaternion EmapProtocol::sceneRotation(const QQuaternion& vehicleOrientation)
{
    // The body to scene mapping S (x forward, y left, z up -> x right, y up, forward = -z) is a rotation.
    // A rotation with scalar w and vector (x, y, z) in the body frame is (w, S * (x, y, z)) in the scene,
    // with S * (x, y, z) = (-y, z, -x). World to body is the inverse, so the vector part changes sign.
    const QQuaternion q = vehicleOrientation.normalized();
    return QQuaternion(q.scalar(), q.y(), -q.z(), q.x());
}

void EmapProtocol::sceneVehiclePlacement(const EmapPose& pose, QVector3D* position, QQuaternion* rotation)
{
    // sceneRotation turns world into body, the node needs the way back
    *rotation = sceneRotation(pose.orientation).conjugated();
    *position = PointCloudInstancing::toScenePosition(pose.position);
}

std::optional<EmapTrajectory> EmapProtocol::parseTrajectory(const QByteArray& payload)
{
    constexpr int kHeadSize = 4;  // u8 kind, u8 reserved, u16 line_count
    if (payload.size() < kHeadSize) {
        return std::nullopt;
    }
    const char* p = payload.constData();
    const char* const end = p + payload.size();

    EmapTrajectory trajectory;
    trajectory.kind = static_cast<quint8>(p[0]);
    if (trajectory.kind >= kKindCount) {
        return std::nullopt;
    }
    const quint16 lineCount = readLe<quint16>(p + 2);
    p += kHeadSize;

    int totalPoints = 0;
    trajectory.lines.reserve(lineCount);
    for (quint16 i = 0; i < lineCount; ++i) {
        if (end - p < 2) {
            return std::nullopt;
        }
        const quint16 count = readLe<quint16>(p);
        p += 2;
        if (end - p < qint64(count) * 12) {
            return std::nullopt;
        }
        totalPoints += count;
        QList<QVector3D> line;
        line.reserve(count);
        for (quint16 j = 0; j < count; ++j, p += 12) {
            const float x = readFloatLe(p);
            const float y = readFloatLe(p + 4);
            const float z = readFloatLe(p + 8);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                return std::nullopt;
            }
            line.append(QVector3D(x, y, z));
        }
        if (totalPoints <= kMaxTrajectoryPoints) {
            trajectory.lines.append(std::move(line));
        }
    }
    if (p != end) {
        return std::nullopt;
    }
    return trajectory;
}

QByteArray EmapProtocol::encodeControl(bool wantMap, bool wantTrajectory)
{
    QByteArray out;
    out.append("EMAP", 4);
    out.append(static_cast<char>(kVersion));
    out.append(static_cast<char>(kTypeControl));
    out.append(char(0));
    out.append(char(0));
    out.append(char(1));  // payload length, little endian
    out.append(char(0));
    out.append(char(0));
    out.append(char(0));
    out.append(static_cast<char>((wantMap ? kFlagWantMap : 0) | (wantTrajectory ? kFlagWantTrajectory : 0)));
    return out;
}

namespace {
/// Milliseconds from b to a, correct across the wrap of the 32 bit stamp
qint32 diffMs(quint32 a, quint32 b)
{
    return static_cast<qint32>(a - b);
}
}  // namespace

void EmapPoseHistory::add(const EmapPose& pose)
{
    if (!_poses.isEmpty()) {
        const qint32 step = diffMs(pose.stampMs, _poses.last().stampMs);
        if (step == 0) {
            _poses.last() = pose;
            return;
        }
        if (step < 0) {
            if (step >= -kKeepMs) {
                return;      // out of order, the newer pose is already stored
            }
            _poses.clear();  // the clock of the vehicle restarted
        }
    }
    _poses.append(pose);
    while (diffMs(_poses.last().stampMs, _poses.first().stampMs) > kKeepMs) {
        _poses.removeFirst();
    }
}

std::optional<EmapPose> EmapPoseHistory::at(quint32 stampMs) const
{
    if (_poses.isEmpty()) {
        return std::nullopt;
    }
    const EmapPose& newest = _poses.last();
    if (diffMs(stampMs, newest.stampMs) >= 0) {
        return diffMs(stampMs, newest.stampMs) <= kToleranceMs ? std::optional<EmapPose>(newest) : std::nullopt;
    }
    const EmapPose& oldest = _poses.first();
    if (diffMs(stampMs, oldest.stampMs) <= 0) {
        return diffMs(oldest.stampMs, stampMs) <= kToleranceMs ? std::optional<EmapPose>(oldest) : std::nullopt;
    }

    // Strictly inside: find the pose before and the one after
    qsizetype after = _poses.size() - 1;
    while (after > 0 && diffMs(_poses[after - 1].stampMs, stampMs) > 0) {
        --after;
    }
    const EmapPose& b = _poses[after];
    const EmapPose& a = _poses[after - 1];
    const qint32 span = diffMs(b.stampMs, a.stampMs);
    const float t = span > 0 ? static_cast<float>(diffMs(stampMs, a.stampMs)) / static_cast<float>(span) : 0.0f;

    EmapPose out;
    out.stampMs = stampMs;
    out.position = a.position + (b.position - a.position) * t;
    out.orientation = QQuaternion::slerp(a.orientation.normalized(), b.orientation.normalized(), t);
    return out;
}

QList<EmapProtocol::Message> EmapStreamDecoder::feed(const QByteArray& bytes)
{
    _buffer.append(bytes);

    QList<EmapProtocol::Message> messages;
    while (_buffer.size() >= EmapProtocol::kHeaderSize) {
        const char* p = _buffer.constData();
        const quint8 version = static_cast<quint8>(p[4]);
        const quint8 type = static_cast<quint8>(p[5]);
        const quint32 length = readLe<quint32>(p + 8);
        if (std::memcmp(p, kMagic, kMagicSize) != 0 || version != EmapProtocol::kVersion ||
            length > EmapProtocol::kMaxPayloadBytes) {
            _skipToNextMagic();
            continue;
        }
        const qsizetype end = EmapProtocol::kHeaderSize + static_cast<qsizetype>(length);
        if (_buffer.size() < end) {
            break;
        }
        messages.append({type, _buffer.mid(EmapProtocol::kHeaderSize, length)});
        _buffer.remove(0, end);
    }
    return messages;
}

void EmapStreamDecoder::reset()
{
    _buffer.clear();
}

void EmapStreamDecoder::_skipToNextMagic()
{
    qsizetype drop = _buffer.indexOf(QByteArray(kMagic, kMagicSize), 1);
    if (drop < 0) {
        // Keep the tail, it may be the start of a header that is still arriving
        drop = qMax<qsizetype>(1, _buffer.size() - (kMagicSize - 1));
    }
    _badBytes += static_cast<quint64>(drop);
    _buffer.remove(0, drop);
}
