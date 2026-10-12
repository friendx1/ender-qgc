#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QPointer>
#include <QtGui/QColor>
#include <QtGui/QVector3D>
#include <QtQmlIntegration/QtQmlIntegration>
#include <QtQuick3D/QQuick3DInstancing>

#include "EpclFrameAssembler.h"
#include "PointCloudReceiver.h"

/// Draws the latest LiDAR scan as one small cube per point, colored by height.
///
/// Scene units are centimeters. The sensor body frame (x forward, y left, z up) is mapped to the
/// Qt Quick 3D frame (x right, y up, forward = -z), so the vehicle flies toward -z on screen.
class PointCloudInstancing : public QQuick3DInstancing
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(PointCloudReceiver* receiver READ receiver WRITE setReceiver NOTIFY receiverChanged)
    Q_PROPERTY(int count READ count NOTIFY countChanged)
    Q_PROPERTY(float pointSize READ pointSize WRITE setPointSize NOTIFY pointSizeChanged)
    Q_PROPERTY(float heightMin READ heightMin WRITE setHeightMin NOTIFY heightRangeChanged)
    Q_PROPERTY(float heightMax READ heightMax WRITE setHeightMax NOTIFY heightRangeChanged)

    friend class PointCloudInstancingTest;

public:
    explicit PointCloudInstancing(QQuick3DObject* parent = nullptr);

    PointCloudReceiver* receiver() const { return _receiver; }

    void setReceiver(PointCloudReceiver* receiver);

    int count() const { return static_cast<int>(_frame.points.size()); }

    /// Edge length of each point cube in centimeters
    float pointSize() const { return _pointSize; }

    void setPointSize(float pointSize);

    /// Heights in meters that map to the cold and the warm end of the color ramp
    float heightMin() const { return _heightMin; }

    void setHeightMin(float heightMin);

    float heightMax() const { return _heightMax; }

    void setHeightMax(float heightMax);

    /// Replaces the displayed scan
    void setFrame(const EpclFrame& frame);

    /// Body frame (meters) to scene position (centimeters)
    static QVector3D toScenePosition(const QVector3D& bodyMeters);

signals:
    void receiverChanged();
    void countChanged();
    void pointSizeChanged();
    void heightRangeChanged();

protected:
    QByteArray getInstanceBuffer(int* instanceCount) override;

private:
    void _onFrameReceived();
    QColor _colorForHeight(float heightMeters) const;

    QPointer<PointCloudReceiver> _receiver;
    EpclFrame _frame;
    QByteArray _instanceData;
    bool _dirty = false;
    float _pointSize = 4.0f;
    float _heightMin = -1.5f;
    float _heightMax = 2.5f;
};
