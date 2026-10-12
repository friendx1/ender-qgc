#pragma once

#include <QtCore/QElapsedTimer>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtNetwork/QUdpSocket>
#include <QtQmlIntegration/QtQmlIntegration>

#include "EpclFrameAssembler.h"

/// Receives the LiDAR point cloud sent by the onboard computer (EPCL v1 over UDP)
/// and keeps the most recent complete scan.
class PointCloudReceiver : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(bool active READ active WRITE setActive NOTIFY activeChanged)
    Q_PROPERTY(quint16 port READ port WRITE setPort NOTIFY portChanged)
    Q_PROPERTY(bool listening READ listening NOTIFY listeningChanged)
    Q_PROPERTY(bool receiving READ receiving NOTIFY receivingChanged)
    Q_PROPERTY(int pointCount READ pointCount NOTIFY frameReceived)
    Q_PROPERTY(int frameRate READ frameRate NOTIFY frameRateChanged)
    Q_PROPERTY(QString errorString READ errorString NOTIFY errorStringChanged)

public:
    static constexpr quint16 kDefaultPort = 19900;
    static constexpr int kReceivingTimeoutMs = 2000;

    explicit PointCloudReceiver(QObject* parent = nullptr);
    ~PointCloudReceiver() override;

    bool active() const { return _active; }

    void setActive(bool active);

    quint16 port() const { return _port; }

    void setPort(quint16 port);

    bool listening() const { return _listening; }

    bool receiving() const { return _receiving; }

    int pointCount() const { return static_cast<int>(_latest.points.size()); }

    int frameRate() const { return _frameRate; }

    QString errorString() const { return _errorString; }

    /// Most recent complete scan, valid until the next frameReceived()
    const EpclFrame& latestFrame() const { return _latest; }

signals:
    void activeChanged();
    void portChanged();
    void listeningChanged();
    void receivingChanged();
    void frameRateChanged();
    void errorStringChanged();
    void frameReceived();

private slots:
    void _readPendingDatagrams();
    void _updateStatus();

private:
    void _start();
    void _stop();
    void _handleDatagram(const QByteArray& datagram);
    void _setListening(bool listening);
    void _setReceiving(bool receiving);
    void _setErrorString(const QString& errorString);

    QUdpSocket* _socket = nullptr;
    QTimer _statusTimer;
    QElapsedTimer _sinceLastFrame;
    EpclFrameAssembler _assembler;
    EpclFrame _latest;

    bool _active = false;
    quint16 _port = kDefaultPort;
    bool _listening = false;
    bool _receiving = false;
    int _frameRate = 0;
    int _framesSinceStatus = 0;
    QString _errorString;
};
