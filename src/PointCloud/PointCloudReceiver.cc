#include "PointCloudReceiver.h"

#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkDatagram>

#include "QGCLoggingCategory.h"

QGC_LOGGING_CATEGORY(PointCloudReceiverLog, "PointCloud.PointCloudReceiver")

namespace {
constexpr int kStatusIntervalMs = 500;
}

PointCloudReceiver::PointCloudReceiver(QObject* parent) : QObject(parent)
{
    _statusTimer.setInterval(kStatusIntervalMs);
    connect(&_statusTimer, &QTimer::timeout, this, &PointCloudReceiver::_updateStatus);
}

PointCloudReceiver::~PointCloudReceiver()
{
    _stop();
}

void PointCloudReceiver::setActive(bool active)
{
    if (_active == active) {
        return;
    }
    _active = active;
    if (_active) {
        _start();
    } else {
        _stop();
    }
    emit activeChanged();
}

void PointCloudReceiver::setPort(quint16 port)
{
    if (_port == port) {
        return;
    }
    _port = port;
    emit portChanged();
    if (_active) {
        _stop();
        _start();
    }
}

void PointCloudReceiver::_start()
{
    if (_socket) {
        return;
    }

    _socket = new QUdpSocket(this);
    if (!_socket->bind(QHostAddress::AnyIPv4, _port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
        qCWarning(PointCloudReceiverLog) << "Cannot listen on UDP port" << _port << ":" << _socket->errorString();
        _setErrorString(_socket->errorString());
        _socket->deleteLater();
        _socket = nullptr;
        return;
    }

    // A scan arrives as a burst of datagrams, the default buffer can overflow before we read them
    _socket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, kReceiveBufferBytes);

    qCDebug(PointCloudReceiverLog) << "Listening for point cloud on UDP port" << _port;
    connect(_socket, &QUdpSocket::readyRead, this, &PointCloudReceiver::_readPendingDatagrams);
    _assembler.reset();
    _setErrorString(QString());
    _setListening(true);
    _statusTimer.start();
}

void PointCloudReceiver::_stop()
{
    _statusTimer.stop();
    if (_socket) {
        _socket->close();
        _socket->deleteLater();
        _socket = nullptr;
    }
    _setListening(false);
    _setReceiving(false);
    if (_frameRate != 0) {
        _frameRate = 0;
        emit frameRateChanged();
    }
    _framesSinceStatus = 0;
}

void PointCloudReceiver::_readPendingDatagrams()
{
    while (_socket && _socket->hasPendingDatagrams()) {
        const QNetworkDatagram datagram = _socket->receiveDatagram();
        _handleDatagram(datagram.data());
    }
}

void PointCloudReceiver::_handleDatagram(const QByteArray& datagram)
{
    const std::optional<EpclFrame> frame = _assembler.feed(datagram);
    if (!frame) {
        return;
    }

    _latest = *frame;
    _framesSinceStatus++;
    _sinceLastFrame.start();
    _setReceiving(true);
    emit frameReceived();
}

void PointCloudReceiver::_updateStatus()
{
    const int rate = _framesSinceStatus * 1000 / kStatusIntervalMs;
    _framesSinceStatus = 0;
    if (rate != _frameRate) {
        _frameRate = rate;
        emit frameRateChanged();
    }

    if (_receiving && (!_sinceLastFrame.isValid() || _sinceLastFrame.hasExpired(kReceivingTimeoutMs))) {
        _setReceiving(false);
    }
}

void PointCloudReceiver::_setListening(bool listening)
{
    if (_listening == listening) {
        return;
    }
    _listening = listening;
    emit listeningChanged();
}

void PointCloudReceiver::_setReceiving(bool receiving)
{
    if (_receiving == receiving) {
        return;
    }
    _receiving = receiving;
    emit receivingChanged();
}

void PointCloudReceiver::_setErrorString(const QString& errorString)
{
    if (_errorString == errorString) {
        return;
    }
    _errorString = errorString;
    emit errorStringChanged();
}
