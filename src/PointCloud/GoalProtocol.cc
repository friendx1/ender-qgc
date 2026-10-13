#include "GoalProtocol.h"

#include <QtCore/QtEndian>
#include <bit>
#include <cstring>

namespace {

template <typename T>
void append(QByteArray& out, T value)
{
    const T le = qToLittleEndian(value);
    out.append(reinterpret_cast<const char*>(&le), sizeof(le));
}

void appendFloat(QByteArray& out, float value)
{
    append<quint32>(out, std::bit_cast<quint32>(value));
}

template <typename T>
T read(const QByteArray& data, qsizetype offset)
{
    return qFromLittleEndian<T>(reinterpret_cast<const uchar*>(data.constData() + offset));
}

float readFloat(const QByteArray& data, qsizetype offset)
{
    return std::bit_cast<float>(read<quint32>(data, offset));
}

}  // namespace

QByteArray GoalProtocol::encodeGoal(const GoalRequest& goal)
{
    QByteArray out;
    out.reserve(kGoalSize);
    out.append("EGOL", 4);
    append<quint8>(out, kVersion);
    append<quint8>(out, kKindSetGoal);
    append<quint32>(out, goal.sequence);
    append<quint32>(out, goal.scanStampMs);
    appendFloat(out, goal.x);
    appendFloat(out, goal.y);
    return out;
}

std::optional<GoalAck> GoalProtocol::decodeAck(const QByteArray& datagram)
{
    if (datagram.size() != kAckSize || std::memcmp(datagram.constData(), "EGOA", 4) != 0) {
        return std::nullopt;
    }
    if (read<quint8>(datagram, 4) != kVersion) {
        return std::nullopt;
    }

    GoalAck ack;
    ack.status = read<quint8>(datagram, 5);
    ack.sequence = read<quint32>(datagram, 6);
    ack.worldX = readFloat(datagram, 10);
    ack.worldY = readFloat(datagram, 14);
    return ack;
}
