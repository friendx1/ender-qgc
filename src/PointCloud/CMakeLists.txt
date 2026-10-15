# ============================================================================
# PointCloud Module
# Live LiDAR point cloud from the onboard computer (EPCL v1 over UDP),
# obstacle map (EMAP v1 over TCP) and flight goals (EGOL v1 over UDP)
# ============================================================================

# The unit tests are compiled into the main target and include the headers below
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# ----------------------------------------------------------------------------
# QML Module
# QML_ELEMENT classes live here so MOC + qmltyperegistrar run on this small target
# ----------------------------------------------------------------------------
qt_add_library(PointCloudModule STATIC)

qt_add_qml_module(PointCloudModule
    URI QGroundControl.PointCloud
    VERSION 1.0
    RESOURCE_PREFIX /qml
    SOURCES
        EpclFrameAssembler.cc
        EpclFrameAssembler.h
        GoalProtocol.cc
        GoalProtocol.h
        GoalSender.cc
        GoalSender.h
        MapInstancing.cc
        MapInstancing.h
        MapProtocol.cc
        MapProtocol.h
        MapReceiver.cc
        MapReceiver.h
        PointCloudInstancing.cc
        PointCloudInstancing.h
        PointCloudReceiver.cc
        PointCloudReceiver.h
    QML_FILES
        PointCloudQml/PointCloudView.qml
    IMPORTS
        QtQuick
        QtQuick3D
)

target_link_libraries(PointCloudModule
    PRIVATE
        QGCCompression
        QGCLogging
    PUBLIC
        Qt6::Core
        Qt6::Gui
        Qt6::Network
        Qt6::Qml
        Qt6::QmlIntegration
        Qt6::Quick3D
)

target_include_directories(PointCloudModule PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
