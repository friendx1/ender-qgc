#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QList>
#include <QtCore/QString>
#include <QtCore/QtGlobal>
#include <optional>

/// One step of the start sequence on the onboard computer, as the launcher reports it
struct LauncherStepInfo
{
    QString name;
    QString state;   ///< pending, running, ok or failed
    QString detail;  ///< Short error text or note, may be empty
    bool canStop = false;
    bool confirmRestart = false;  ///< Starting it again ends other processes too, the app asks first
};

/// The launcher's answer: overall state and the steps
struct LauncherReply
{
    quint32 sequence = 0;
    QString profile;
    QString state;    ///< stopped, partial, running, starting, stopping or error
    QString message;  ///< Short hint for the person, may be empty
    QList<LauncherStepInfo> steps;
};

/// Wire format of the launcher service (ELCH v1): one JSON object per UDP datagram, port 19903.
/// The service is ender/launcher/ender_launcher.py, its README describes the protocol.
///
/// Request: {"v":1,"seq":7,"cmd":"start"|"stop"|"status"|"step"|"stop_step","step":2,"token":"..."}
/// Reply:   {"v":1,"seq":7,"profile":"sim","state":"partial","message":"","steps":[{"name":..,"state":..,"detail":..,"can_stop":..,"confirm":..}]}
/// Pure logic, no sockets, so it can be unit tested without a network.
class LauncherProtocol
{
public:
    static constexpr int kVersion = 1;
    static constexpr quint16 kDefaultPort = 19903;

    /// stepIndex is only written for the commands "step" and "stop_step"
    static QByteArray encodeRequest(const QString& command, quint32 sequence, int stepIndex = -1);

    /// Returns nothing for any datagram that is not a valid reply (wrong version, no sequence, an error answer)
    static std::optional<LauncherReply> decodeReply(const QByteArray& datagram);
};
