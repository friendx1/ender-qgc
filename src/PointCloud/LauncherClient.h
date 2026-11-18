#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtCore/QVariantList>
#include <QtNetwork/QUdpSocket>
#include <QtQmlIntegration/QtQmlIntegration>

#include "LauncherProtocol.h"

/// Starts and stops the drone software through the launcher service on the onboard computer (ELCH v1 over UDP).
///
/// While active it asks for the state every two seconds. A command is repeated until the launcher answers,
/// because UDP may lose it. The launcher itself never runs anything but the fixed steps of its profile.
class LauncherClient : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(QString host READ host WRITE setHost NOTIFY hostChanged)
    Q_PROPERTY(bool active READ active WRITE setActive NOTIFY activeChanged)
    Q_PROPERTY(bool reachable READ reachable NOTIFY stateChanged)
    Q_PROPERTY(QString state READ state NOTIFY stateChanged)
    Q_PROPERTY(bool busy READ busy NOTIFY stateChanged)
    Q_PROPERTY(QString message READ message NOTIFY stateChanged)
    Q_PROPERTY(QVariantList steps READ steps NOTIFY stateChanged)

public:
    static constexpr int kPollIntervalMs = 2000;
    static constexpr int kRetryIntervalMs = 400;
    static constexpr int kMaxAttempts = 5;
    static constexpr int kSilenceMs = 5000;  ///< No answer for this long: the launcher counts as unreachable

    explicit LauncherClient(QObject* parent = nullptr);

    QString host() const { return _host; }

    void setHost(const QString& host);

    bool active() const { return _active; }

    /// Polling only runs while this is set, so the view asks nothing while the panel is closed
    void setActive(bool active);

    bool reachable() const { return _reachable; }

    /// stopped, partial, running, starting, stopping or error. Empty until the first answer.
    QString state() const { return _reply.state; }

    /// The launcher is starting or stopping, new commands would be refused
    bool busy() const { return _reply.state == QLatin1String("starting") || _reply.state == QLatin1String("stopping"); }

    /// Hint from the launcher or about the connection, may be empty
    QString message() const { return _message; }

    /// One map per step: name, state, detail, canStop
    QVariantList steps() const;

    /// Runs all steps that are not done yet, one after another
    Q_INVOKABLE void startAll();

    /// Runs only this step (a step number as the launcher lists them), also when it is done already
    Q_INVOKABLE void startStep(int index);

    /// Stops only this step, if the launcher knows how (canStop of the step)
    Q_INVOKABLE void stopStep(int index);

    /// Stops everything, one process after another
    Q_INVOKABLE void stopAll();

signals:
    void hostChanged();
    void activeChanged();
    void stateChanged();

private slots:
    void _readPendingDatagrams();
    void _poll();
    void _retry();
    void _checkSilence();

private:
    bool _ensureSocket();
    void _sendCommand(const QString& command, int stepIndex = -1);
    bool _send(const QByteArray& datagram);
    void _setReachable(bool reachable);

    QUdpSocket* _socket = nullptr;
    QTimer _pollTimer;
    QTimer _retryTimer;
    QTimer _silenceTimer;
    QString _host;
    bool _active = false;
    bool _reachable = false;
    QString _message;
    LauncherReply _reply;

    quint32 _nextSequence = 0;
    quint32 _commandSequence = 0;
    bool _commandPending = false;
    QByteArray _commandDatagram;
    int _attempts = 0;
    qint64 _lastReplyMs = 0;
};
