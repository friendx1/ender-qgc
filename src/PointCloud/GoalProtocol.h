#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QtGlobal>
#include <optional>

/// A flight goal sent from the ground station to the onboard computer.
/// The point is in the vehicle body frame in meters (x forward, y left), as seen in the point cloud.
struct GoalRequest
{
    quint32 sequence = 0;
    quint32 scanStampMs = 0;  ///< stampMs of the scan the point was picked in
    float x = 0.0f;
    float y = 0.0f;
};

/// The onboard computer's answer to a goal
struct GoalAck
{
    quint8 status = 0;    ///< One of GoalProtocol::Status
    quint32 sequence = 0;
    float worldX = 0.0f;  ///< Goal in the planner world frame (meters), 0 when rejected
    float worldY = 0.0f;
};

/// Wire format of the flight goal uplink (EGOL v1 / EGOA v1), little endian.
/// The same format is implemented in ender/ros2/ender_lidar_bridge/ender_lidar_bridge/goal_protocol.py.
///
/// Goal, ground station to vehicle (22 bytes):
///   4s magic "EGOL", u8 version (1), u8 kind (1 = set goal), u32 goal_seq, u32 scan_stamp_ms, f32 x, f32 y
/// Acknowledgement, vehicle to ground station (18 bytes):
///   4s magic "EGOA", u8 version (1), u8 status, u32 goal_seq, f32 world_x, f32 world_y
/// Pure logic, no sockets, so it can be unit tested without a network.
class GoalProtocol
{
public:
    static constexpr int kGoalSize = 22;
    static constexpr int kAckSize = 18;
    static constexpr quint8 kVersion = 1;
    static constexpr quint8 kKindSetGoal = 1;

    enum Status : quint8
    {
        Ok = 0,
        Stale = 1,     ///< The scan is too old
        TooFar = 2,    ///< The goal is farther away than allowed
        NoPose = 3,    ///< The vehicle has no matching pose
        BadValue = 4,  ///< A number was not finite
    };

    static QByteArray encodeGoal(const GoalRequest& goal);

    /// Returns nothing for any datagram that is not a valid acknowledgement
    static std::optional<GoalAck> decodeAck(const QByteArray& datagram);
};
