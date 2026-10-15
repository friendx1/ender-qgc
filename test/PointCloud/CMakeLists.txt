# ============================================================================
# PointCloud Unit Tests
# EPCL v1 reassembly, point cloud scene mapping, the flight goal uplink and the obstacle map (EMAP v1)
# ============================================================================

target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        EpclFrameAssemblerTest.cc
        EpclFrameAssemblerTest.h
        GoalProtocolTest.cc
        GoalProtocolTest.h
        GoalSenderTest.cc
        GoalSenderTest.h
        MapInstancingTest.cc
        MapInstancingTest.h
        MapProtocolTest.cc
        MapProtocolTest.h
        MapReceiverTest.cc
        MapReceiverTest.h
        MapTestData.h
        OnboardAddressTest.cc
        OnboardAddressTest.h
        PointCloudInstancingTest.cc
        PointCloudInstancingTest.h
        TrajectoryInstancingTest.cc
        TrajectoryInstancingTest.h
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_qgc_test(EpclFrameAssemblerTest LABELS Unit PointCloud)
add_qgc_test(GoalProtocolTest LABELS Unit PointCloud)
add_qgc_test(GoalSenderTest LABELS Unit PointCloud)
add_qgc_test(MapInstancingTest LABELS Unit PointCloud)
add_qgc_test(MapProtocolTest LABELS Unit PointCloud)
add_qgc_test(MapReceiverTest LABELS Unit PointCloud)
add_qgc_test(OnboardAddressTest LABELS Unit PointCloud)
add_qgc_test(PointCloudInstancingTest LABELS Unit PointCloud)
add_qgc_test(TrajectoryInstancingTest LABELS Unit PointCloud)
