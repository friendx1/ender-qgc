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

/// A planned trajectory of the vehicle (SUPER), as lines of points in the planner world frame in meters.
/// An empty trajectory means that the planner has none right now.
struct EmapTrajectory
{
    quint8 kind = 0;  ///< EmapProtocol::kKindExp or kKindBackup
    QList<QList<QVector3D>> lines;
};

/// Wire format of the obstacle map stream (EMAP v1) over TCP, little endian.
/// The same format is implemented in ender/ros2/ender_lidar_bridge/ender_lidar_bridge/map_protocol.py.
///
/// Every message starts with a 12 byte header:
///   4s magic "EMAP", u8 version (1), u8 type (1 = map, 2 = pose, 3 = control, 4 = trajectory), u16 reserved,
///   u32 payload_len
///
/// Type 1, map: the payload is the zstd packet that the onboard pointcloud_compressor_node publishes.
/// Unpacked it is: u32 stamp_sec, u32 stamp_nanosec, f64 anchor x, y, z, u32 num_points, u16 frame_id_len,
/// the frame id, then num_points x (int16 x, y, z in centimeters relative to the anchor).
///
/// Type 2, pose (44 bytes): u32 stamp_ms, f64 x, y, z, f32 qx, qy, qz, qw
///
/// Type 3, control (1 byte, ground station to vehicle): flags, bit 0 = the map is wanted, bit 1 = the planned
/// trajectories are wanted. The pose always flows while connected, map and trajectories only after the ground
/// station asked for them.
///
/// Type 4, trajectory (vehicle to ground station): the whole current state of one trajectory, it replaces the
/// previous one: u8 kind (0 = SUPER exp_traj, 1 = backup_traj), u8 reserved, u16 line_count, then per line
/// u16 point_count and point_count x (f32 x, y, z in meters).
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
    static constexpr quint8 kTypeControl = 3;
    static constexpr quint8 kTypeTrajectory = 4;
    static constexpr quint8 kFlagWantMap = 0x01;
    static constexpr quint8 kFlagWantTrajectory = 0x02;
    static constexpr quint8 kKindExp = 0;
    static constexpr quint8 kKindBackup = 1;
    static constexpr int kKindCount = 2;
    static constexpr int kMaxTrajectoryPoints = 20000;  ///< in total, more is cut (the vehicle sends far fewer)
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

    /// Trajectory payload to trajectory. Nothing for anything that is cut short, has bytes left over, an unknown
    /// kind or a non-finite number.
    static std::optional<EmapTrajectory> parseTrajectory(const QByteArray& payload);

    /// Control message (complete, with header) that tells the vehicle whether the map and the trajectories
    /// are wanted
    static QByteArray encodeControl(bool wantMap, bool wantTrajectory = false);

    /// Rotation that turns scene vectors of the world frame into scene vectors of the vehicle body frame
    static QQuaternion sceneRotation(const QQuaternion& vehicleOrientation);

    /// Position (scene centimeters) and rotation of a Quick 3D node that carries the vehicle: everything
    /// drawn inside that node in the vehicle body frame (PointCloudInstancing::toScenePosition) lands at the
    /// right place in the world scene, where the obstacle map is drawn without any transform.
    static void sceneVehiclePlacement(const EmapPose& pose, QVector3D* position, QQuaternion* rotation);
};

/// The last few seconds of vehicle poses. Answers "where was the vehicle when this scan was taken": scan and pose
/// stamps come from the same LiDAR clock, the pose stream is only ten times per second, so the answer is
/// interpolated between the two poses around the stamp.
///
/// Stamps are milliseconds that may wrap around, all comparisons are made on differences.
class EmapPoseHistory
{
public:
    static constexpr int kKeepMs = 5000;
    static constexpr int kToleranceMs = 500;  ///< how far a stamp may lie outside the stored poses

    /// Adds a pose. A pose with the stamp of the newest one replaces it, a slightly older one is ignored,
    /// a jump back by more than kKeepMs means the vehicle clock restarted and clears the history.
    void add(const EmapPose& pose);

    /// Pose at the stamp, interpolated. Within kToleranceMs outside the stored range the nearest end is used,
    /// farther out there is no answer.
    std::optional<EmapPose> at(quint32 stampMs) const;

    void clear() { _poses.clear(); }

    int size() const { return static_cast<int>(_poses.size()); }

private:
    QList<EmapPose> _poses;  ///< ascending by stamp
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
