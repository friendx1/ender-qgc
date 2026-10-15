#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QtEndian>
#include <cstring>

#include "MapProtocol.h"

/// Helpers shared by the obstacle map tests
namespace MapTestData {

/// zstd packet as Birk's pointcloud_compressor_node makes it (made with
/// ender_lidar_bridge.compressed_cloud.pack_cloud): stamp 12 s + 345000000 ns, frame "world", points (0,0,0) (5,0,0)
/// (30,0,0) in meters, anchor at their mean
inline QByteArray goldenMapPacket()
{
    static const unsigned char kBytes[] = {
        0x28, 0xb5, 0x2f, 0xfd, 0x20, 0x3d, 0x75, 0x01, 0x00, 0x14, 0x02, 0x0c, 0x00, 0x00,
        0x00, 0x40, 0x48, 0x90, 0x14, 0x55, 0x27, 0x40, 0x00, 0x03, 0x00, 0x00, 0x00, 0x05,
        0x00, 0x77, 0x6f, 0x72, 0x6c, 0x64, 0x71, 0xfb, 0x65, 0xfd, 0x29, 0x07, 0x00, 0x00,
        0x00, 0x00, 0x04, 0x00, 0x20, 0x30, 0x9f, 0xc1, 0x07, 0x3d, 0x5a, 0x70, 0x04,
    };
    return QByteArray(reinterpret_cast<const char*>(kBytes), sizeof(kBytes));
}

/// zstd frame that unpacks fine but is far too short to be a map packet ("hello")
inline QByteArray shortPacket()
{
    static const unsigned char kBytes[] = {0x28, 0xb5, 0x2f, 0xfd, 0x20, 0x05, 0x29,
                                           0x00, 0x00, 0x68, 0x65, 0x6c, 0x6c, 0x6f};
    return QByteArray(reinterpret_cast<const char*>(kBytes), sizeof(kBytes));
}

/// zstd frame with a valid header that claims 3 points but carries only 2
inline QByteArray inconsistentPacket()
{
    static const unsigned char kBytes[] = {
        0x28, 0xb5, 0x2f, 0xfd, 0x20, 0x37, 0xb5, 0x00, 0x00, 0x70, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00,
        0x05, 0x00, 0x77, 0x6f, 0x72, 0x6c, 0x64, 0x00, 0x02, 0x00, 0xc0, 0x48, 0x22, 0xc0, 0x02,
    };
    return QByteArray(reinterpret_cast<const char*>(kBytes), sizeof(kBytes));
}

template <typename T>
inline void append(QByteArray& out, T value)
{
    T le = qToLittleEndian(value);
    out.append(reinterpret_cast<const char*>(&le), sizeof(le));
}

inline QByteArray message(quint8 type, const QByteArray& payload, quint8 version = 1,
                          quint32 lengthOverride = 0xFFFFFFFF)
{
    QByteArray out;
    out.append("EMAP", 4);
    out.append(static_cast<char>(version));
    out.append(static_cast<char>(type));
    append<quint16>(out, 0);
    append<quint32>(out, lengthOverride == 0xFFFFFFFF ? static_cast<quint32>(payload.size()) : lengthOverride);
    out.append(payload);
    return out;
}

inline QByteArray posePayload(quint32 stampMs, double x, double y, double z, float qx, float qy, float qz, float qw)
{
    QByteArray out;
    append<quint32>(out, stampMs);
    for (double v : {x, y, z}) {
        quint64 bits;
        std::memcpy(&bits, &v, sizeof(bits));
        append<quint64>(out, bits);
    }
    for (float v : {qx, qy, qz, qw}) {
        quint32 bits;
        std::memcpy(&bits, &v, sizeof(bits));
        append<quint32>(out, bits);
    }
    return out;
}

inline QByteArray poseMessage(double x, double y, double z, float qx = 0, float qy = 0, float qz = 0, float qw = 1,
                              quint32 stampMs = 1000)
{
    return message(EmapProtocol::kTypePose, posePayload(stampMs, x, y, z, qx, qy, qz, qw));
}

inline QByteArray mapMessage()
{
    return message(EmapProtocol::kTypeMap, goldenMapPacket());
}

}  // namespace MapTestData
