#include "PointCloudInstancing.h"

#include <QtMath>

#include "PointCloudReceiver.h"
#include "QGCLoggingCategory.h"

QGC_LOGGING_CATEGORY(PointCloudInstancingLog, "PointCloud.PointCloudInstancing")

namespace {

constexpr float kCentimetersPerMeter = 100.0f;
constexpr float kCubeEdgeUnits = 100.0f;  // Edge length of the built-in #Cube primitive

}  // namespace

PointCloudInstancing::PointCloudInstancing(QQuick3DObject* parent) : QQuick3DInstancing(parent) {}

void PointCloudInstancing::setReceiver(PointCloudReceiver* receiver)
{
    if (_receiver == receiver) {
        return;
    }
    if (_receiver) {
        disconnect(_receiver, nullptr, this, nullptr);
    }
    _receiver = receiver;
    if (_receiver) {
        connect(_receiver, &PointCloudReceiver::frameReceived, this, &PointCloudInstancing::_onFrameReceived);
    }
    emit receiverChanged();
}

void PointCloudInstancing::setPointSize(float pointSize)
{
    if (qFuzzyCompare(_pointSize, pointSize) || pointSize <= 0.0f) {
        return;
    }
    _pointSize = pointSize;
    _dirty = true;
    markDirty();
    emit pointSizeChanged();
}

void PointCloudInstancing::setHeightMin(float heightMin)
{
    if (qFuzzyCompare(_heightMin, heightMin)) {
        return;
    }
    _heightMin = heightMin;
    _dirty = true;
    markDirty();
    emit heightRangeChanged();
}

void PointCloudInstancing::setHeightMax(float heightMax)
{
    if (qFuzzyCompare(_heightMax, heightMax)) {
        return;
    }
    _heightMax = heightMax;
    _dirty = true;
    markDirty();
    emit heightRangeChanged();
}

void PointCloudInstancing::setFrame(const EpclFrame& frame)
{
    _frame = frame;
    _dirty = true;
    markDirty();
    emit countChanged();
}

void PointCloudInstancing::_onFrameReceived()
{
    if (_receiver) {
        setFrame(_receiver->latestFrame());
    }
}

QVector3D PointCloudInstancing::toScenePosition(const QVector3D& bodyMeters)
{
    // Body: x forward, y left, z up. Scene: x right, y up, forward is -z.
    return QVector3D(-bodyMeters.y(), bodyMeters.z(), -bodyMeters.x()) * kCentimetersPerMeter;
}

QColor PointCloudInstancing::_colorForHeight(float heightMeters) const
{
    const float range = _heightMax - _heightMin;
    const float t = range > 0.0f ? qBound(0.0f, (heightMeters - _heightMin) / range, 1.0f) : 0.5f;
    // Blue for low points, through green and yellow, to red for high points
    return QColor::fromHsvF(0.66f * (1.0f - t), 0.9f, 1.0f);
}

QByteArray PointCloudInstancing::getInstanceBuffer(int* instanceCount)
{
    if (_dirty) {
        _instanceData.clear();
        _instanceData.reserve(static_cast<qsizetype>(_frame.points.size() * sizeof(InstanceTableEntry)));

        const float cubeScale = _pointSize / kCubeEdgeUnits;
        const QVector3D scale(cubeScale, cubeScale, cubeScale);
        for (const QVector3D& point : std::as_const(_frame.points)) {
            const InstanceTableEntry entry =
                calculateTableEntry(toScenePosition(point), scale, QVector3D(0, 0, 0), _colorForHeight(point.z()));
            _instanceData.append(reinterpret_cast<const char*>(&entry), sizeof(entry));
        }
        _dirty = false;
    }

    if (instanceCount) {
        *instanceCount = static_cast<int>(_frame.points.size());
    }
    return _instanceData;
}
