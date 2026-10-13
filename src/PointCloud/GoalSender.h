#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtNetwork/QUdpSocket>
#include <QtQmlIntegration/QtQmlIntegration>

#include "GoalProtocol.h"

/// Sends a flight goal to the onboard computer (EGOL v1 over UDP) and waits for its acknowledgement.
///
/// The goal is repeated until it is acknowledged, because UDP may lose it. The onboard computer
/// ignores a repeated goal sequence, so repeating does not set the goal twice.
class GoalSender : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(QString host READ host WRITE setHost NOTIFY hostChanged)
    Q_PROPERTY(quint16 port READ port WRITE setPort NOTIFY portChanged)
    Q_PROPERTY(bool pending READ pending NOTIFY pendingChanged)
    Q_PROPERTY(bool resultAccepted READ resultAccepted NOTIFY resultChanged)
    Q_PROPERTY(QString resultText READ resultText NOTIFY resultChanged)

public:
    static constexpr quint16 kDefaultPort = 19901;
    static constexpr const char* kDefaultHost = "192.168.144.50";
    static constexpr int kRetryIntervalMs = 400;
    static constexpr int kMaxAttempts = 5;

    explicit GoalSender(QObject* parent = nullptr);

    /// IP address of the onboard computer. Taken from the environment variable ENDER_GOAL_HOST if set.
    QString host() const { return _host; }

    void setHost(const QString& host);

    quint16 port() const { return _port; }

    void setPort(quint16 port);

    bool pending() const { return _pending; }

    bool resultAccepted() const { return _resultAccepted; }

    QString resultText() const { return _resultText; }

    /// Sends the point (meters, vehicle body frame: x forward, y left) picked in the scan with this stamp
    Q_INVOKABLE void sendGoal(double x, double y, quint32 scanStampMs);

signals:
    void hostChanged();
    void portChanged();
    void pendingChanged();
    void resultChanged();

    /// Emitted once per goal, when the vehicle answered or when no answer came
    void goalResult(bool accepted, const QString& text);

private slots:
    void _readPendingDatagrams();
    void _retry();

private:
    bool _ensureSocket();
    bool _sendDatagram();
    void _finish(bool accepted, const QString& text);
    static QString _statusText(quint8 status);

    QUdpSocket* _socket = nullptr;
    QTimer _retryTimer;
    QString _host;
    quint16 _port = kDefaultPort;

    GoalRequest _request;
    QByteArray _datagram;
    quint32 _nextSequence = 0;
    int _attempts = 0;

    bool _pending = false;
    bool _resultAccepted = false;
    QString _resultText;
};
