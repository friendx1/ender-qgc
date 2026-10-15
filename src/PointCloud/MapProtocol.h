#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QList>
#include <QtCore/QString>
#include <QtGui/QQuaternion>
#include <QtGui/QVector3D>
#include <optional>

/// Pose of the vehicle in the planner world frame (camera_init), as sent by the onboard computer
struct EmapPose
{
    quint32 stampMs = 0;
    QVector3D position;  ///< meters
    QQuaternion orientation;
};

/// One obstacle map (SUPER inflate map), decoded. Points are in the planner world frame in meters.
struct EmapCloud
{
    quint32 stampMs = 0;
    QString frameId;
    QList<QVector3D> points;
};

/// Wire format of the obstacle map stream (EMAP v1) over TCP, little endian.
/// The same format is implemented in ender/ros2/ender_lidar_bridge/ender_lidar_bridge/map_protocol.py.
///
/// Every message starts with a 12 byte header:
///   4s magic "EMAP", u8 version (1), u8 type (1 = map, 2 = pose), u16 reserved, u32 payload_len
///
/// Type 1, map: the payload is the zstd packet that the onboard pointcloud_compressor_node publishes.
/// Unpacked it is: u32 stamp_sec, u32 stamp_nanosec, f64 anchor x, y, z, u32 num_points, u16 frame_id_len,
/// the frame id, then num_points x (int16 x, y, z in centimeters relative to the anchor).
///
/// Type 2, pose (44 bytes): u32 stamp_ms, f64 x, y, z, f32 qx, qy, qz, qw
///
/// Pure logic, no sockets, so it can be unit tested without a network.
class EmapProtocol
{
public:
    static constexpr int kHeaderSize = 12;
    static constexpr int kPoseSize = 44;
    static constexpr quint8 kVersion = 1;
    static constexpr quint8 kTypeMap = 1;
    static constexpr quint8 kTypePose = 2;
    static constexpr quint32 kMaxPayloadBytes = 4 * 1024 * 1024;
    static constexpr qint64 kMaxUnpackedBytes = 8 * 1024 * 1024;
    static constexpr quint32 kMaxPoints = 500000;

    struct Message
    {
        quint8 type = 0;
        QByteArray payload;
    };

    /// Pose payload to pose. Nothing for a payload of the wrong length or with a non-finite number.
    static std::optional<EmapPose> parsePose(const QByteArray& payload);

    /// Map payload (zstd packet) to points. Nothing for anything that does not unpack to a consistent packet.
    static std::optional<EmapCloud> parseMap(const QByteArray& payload);

    /// Rotation that turns scene vectors of the world frame into scene vectors of the vehicle body frame.
    /// Used with a position, see sceneTransform.
    static QQuaternion sceneRotation(const QQuaternion& vehicleOrientation);

    /// Position and rotation of a Quick 3D node that carries points given in world scene coordinates
    /// (PointCloudInstancing::toScenePosition of the world point) so that they appear in the vehicle body frame.
    static void sceneTransform(const EmapPose& pose, QVector3D* position, QQuaternion* rotation);
};

/// Cuts the TCP byte stream back into messages. Bytes that do not start a valid header are skipped
/// up to the next "EMAP", so the stream recovers after an error.
class EmapStreamDecoder
{
public:
    /// Adds received bytes and returns the messages that are now complete
    QList<EmapProtocol::Message> feed(const QByteArray& bytes);

    void reset();

    /// Bytes thrown away while looking for the next header
    quint64 badBytes() const { return _badBytes; }

private:
    void _skipToNextMagic();

    QByteArray _buffer;
    quint64 _badBytes = 0;
};
