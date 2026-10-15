#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QPointer>
#include <QtGui/QColor>
#include <QtQmlIntegration/QtQmlIntegration>
#include <QtQuick3D/QQuick3DInstancing>

#include "MapReceiver.h"

/// Draws the obstacle map as one small cube per point, in warm colors by height so that it differs from the
/// scan, which uses a blue to red ramp.
///
/// The cubes sit at the world points (scene units are centimeters, see PointCloudInstancing::toScenePosition).
/// The model that carries this instancing takes MapReceiver::modelPosition and modelRotation, which move
/// the unchanged buffer into the vehicle frame. A new buffer is built only when a new map arrives.
class MapInstancing : public QQuick3DInstancing
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(MapReceiver* receiver READ receiver WRITE setReceiver NOTIFY receiverChanged)
    Q_PROPERTY(int count READ count NOTIFY countChanged)
    Q_PROPERTY(float pointSize READ pointSize WRITE setPointSize NOTIFY pointSizeChanged)

    friend class MapInstancingTest;

public:
    explicit MapInstancing(QQuick3DObject* parent = nullptr);

    MapReceiver* receiver() const { return _receiver; }

    void setReceiver(MapReceiver* receiver);

    int count() const { return static_cast<int>(_cloud.points.size()); }

    /// Edge length of each point cube in centimeters
    float pointSize() const { return _pointSize; }

    void setPointSize(float pointSize);

    /// Replaces the displayed map
    void setCloud(const EmapCloud& cloud);

signals:
    void receiverChanged();
    void countChanged();
    void pointSizeChanged();

protected:
    QByteArray getInstanceBuffer(int* instanceCount) override;

private:
    void _onMapChanged();
    static QColor _colorForHeight(float t);

    QPointer<MapReceiver> _receiver;
    EmapCloud _cloud;
    QByteArray _instanceData;
    float _heightMin = 0.0f;
    float _heightMax = 0.0f;
    bool _dirty = false;
    float _pointSize = 10.0f;
};
