#include "MapInstancing.h"

#include <algorithm>

#include "PointCloudInstancing.h"

namespace {
constexpr float kCubeEdgeUnits = 100.0f;  // Edge length of the built-in #Cube primitive
}

MapInstancing::MapInstancing(QQuick3DObject* parent) : QQuick3DInstancing(parent) {}

void MapInstancing::setReceiver(MapReceiver* receiver)
{
    if (_receiver == receiver) {
        return;
    }
    if (_receiver) {
        disconnect(_receiver, nullptr, this, nullptr);
    }
    _receiver = receiver;
    if (_receiver) {
        connect(_receiver, &MapReceiver::mapChanged, this, &MapInstancing::_onMapChanged);
        setCloud(_receiver->map());
    }
    emit receiverChanged();
}

void MapInstancing::setPointSize(float pointSize)
{
    if (qFuzzyCompare(_pointSize, pointSize) || pointSize <= 0.0f) {
        return;
    }
    _pointSize = pointSize;
    _dirty = true;
    markDirty();
    emit pointSizeChanged();
}

void MapInstancing::setCloud(const EmapCloud& cloud)
{
    _cloud = cloud;
    _heightMin = 0.0f;
    _heightMax = 0.0f;
    if (!_cloud.points.isEmpty()) {
        const auto [low, high] =
            std::minmax_element(_cloud.points.cbegin(), _cloud.points.cend(),
                                [](const QVector3D& a, const QVector3D& b) { return a.z() < b.z(); });
        _heightMin = low->z();
        _heightMax = high->z();
    }
    _dirty = true;
    markDirty();
    emit countChanged();
}

void MapInstancing::_onMapChanged()
{
    if (_receiver) {
        setCloud(_receiver->map());
    }
}

QColor MapInstancing::_colorForHeight(float t)
{
    // Dark orange at the bottom to bright yellow at the top
    return QColor::fromHsvF(0.03f + 0.10f * t, 0.9f, 0.65f + 0.35f * t);
}

QByteArray MapInstancing::getInstanceBuffer(int* instanceCount)
{
    if (_dirty) {
        _instanceData.clear();
        _instanceData.reserve(static_cast<qsizetype>(_cloud.points.size() * sizeof(InstanceTableEntry)));

        const float cubeScale = _pointSize / kCubeEdgeUnits;
        const QVector3D scale(cubeScale, cubeScale, cubeScale);
        const float range = _heightMax - _heightMin;
        for (const QVector3D& point : std::as_const(_cloud.points)) {
            const float t = range > 0.0f ? (point.z() - _heightMin) / range : 0.5f;
            const InstanceTableEntry entry = calculateTableEntry(PointCloudInstancing::toScenePosition(point), scale,
                                                                 QVector3D(0, 0, 0), _colorForHeight(t));
            _instanceData.append(reinterpret_cast<const char*>(&entry), sizeof(entry));
        }
        _dirty = false;
    }

    if (instanceCount) {
        *instanceCount = static_cast<int>(_cloud.points.size());
    }
    return _instanceData;
}
