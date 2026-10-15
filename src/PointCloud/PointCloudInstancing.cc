#include "PointCloudInstancing.h"

#include <QtMath>

#include "PointCloudReceiver.h"
#include "QGCLoggingCategory.h"

QGC_LOGGING_CATEGORY(PointCloudInstancingLog, "PointCloud.PointCloudInstancing")

namespace {

constexpr float kCentimetersPerMeter = 100.0f;
constexpr float kCubeEdgeUnits = 100.0f;  // Edge length of the built-in #Cube primitive
constexpr float kVoxelSizeMeters = 0.1f;

/// Milliseconds from b to a, correct across the wrap of the 32 bit stamp
qint32 diffMs(quint32 a, quint32 b)
{
    return static_cast<qint32>(a - b);
}

quint64 voxelKey(const QVector3D& meters)
{
    const auto cell = [](float v) { return static_cast<quint64>(static_cast<qint64>(std::floor(v / kVoxelSizeMeters)) & 0x1FFFFF); };
    return (cell(meters.x()) << 42) | (cell(meters.y()) << 21) | cell(meters.z());
}

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

void PointCloudInstancing::setPersistenceMs(int persistenceMs)
{
    persistenceMs = qMax(0, persistenceMs);
    if (_persistenceMs == persistenceMs) {
        return;
    }
    _persistenceMs = persistenceMs;
    if (_persistenceMs == 0) {
        _clearVoxels();
    }
    emit persistenceMsChanged();
}

void PointCloudInstancing::setPoseSource(MapReceiver* poseSource)
{
    if (_poseSource == poseSource) {
        return;
    }
    if (_poseSource) {
        disconnect(_poseSource, nullptr, this, nullptr);
    }
    _poseSource = poseSource;
    if (_poseSource) {
        connect(_poseSource, &MapReceiver::poseChanged, this, &PointCloudInstancing::_onPoseChanged);
    } else {
        _clearVoxels();
    }
    emit poseSourceChanged();
}

void PointCloudInstancing::_onPoseChanged()
{
    // Without a pose the scans cannot be kept in the world
    if (!_voxels.isEmpty() && (!_poseSource || !_poseSource->poseValid())) {
        _clearVoxels();
    }
}

void PointCloudInstancing::_clearVoxels()
{
    if (_voxels.isEmpty()) {
        return;
    }
    _voxels.clear();
    _dirty = true;
    markDirty();
    emit worldModeChanged();
    emit countChanged();
}

void PointCloudInstancing::_accumulate(const EpclFrame& frame)
{
    const bool wasWorld = !_voxels.isEmpty();

    if (_persistenceMs > 0 && _poseSource && _poseSource->poseValid()) {
        // A scan whose moment is not covered by the poses is left out, the next one may be
        if (const std::optional<EmapPose> pose = _poseSource->poseAt(frame.stampMs)) {
            if (!_voxels.isEmpty() && diffMs(_newestStampMs, frame.stampMs) > _persistenceMs) {
                _voxels.clear();  // the clock of the vehicle restarted
            }
            _newestStampMs = frame.stampMs;

            const QQuaternion orientation = pose->orientation.normalized();
            for (const QVector3D& point : frame.points) {
                const QVector3D world = pose->position + orientation.rotatedVector(point);
                _voxels.insert(voxelKey(world), Voxel{world, point.z(), frame.stampMs});
            }
            for (auto it = _voxels.begin(); it != _voxels.end();) {
                it = diffMs(frame.stampMs, it->stampMs) > _persistenceMs ? _voxels.erase(it) : std::next(it);
            }
        }
    } else {
        _voxels.clear();
    }

    if (wasWorld != !_voxels.isEmpty()) {
        emit worldModeChanged();
    }
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
    _accumulate(frame);
    _dirty = true;
    markDirty();
    emit countChanged();
    emit stampMsChanged();
}

void PointCloudInstancing::setFrozen(bool frozen)
{
    if (_frozen == frozen) {
        return;
    }
    _frozen = frozen;
    emit frozenChanged();
    if (!_frozen && _receiver) {
        setFrame(_receiver->latestFrame());
    }
}

void PointCloudInstancing::_onFrameReceived()
{
    if (_receiver && !_frozen) {
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
        if (!_voxels.isEmpty()) {
            _instanceData.reserve(static_cast<qsizetype>(_voxels.size() * sizeof(InstanceTableEntry)));
            for (const Voxel& voxel : std::as_const(_voxels)) {
                const InstanceTableEntry entry = calculateTableEntry(toScenePosition(voxel.world), scale,
                                                                     QVector3D(0, 0, 0),
                                                                     _colorForHeight(voxel.colorHeight));
                _instanceData.append(reinterpret_cast<const char*>(&entry), sizeof(entry));
            }
        } else {
            for (const QVector3D& point : std::as_const(_frame.points)) {
                const InstanceTableEntry entry =
                    calculateTableEntry(toScenePosition(point), scale, QVector3D(0, 0, 0), _colorForHeight(point.z()));
                _instanceData.append(reinterpret_cast<const char*>(&entry), sizeof(entry));
            }
        }
        _dirty = false;
    }

    if (instanceCount) {
        *instanceCount = count();
    }
    return _instanceData;
}
