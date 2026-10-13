# ============================================================================
# PointCloud Unit Tests
# EPCL v1 reassembly, point cloud scene mapping and the flight goal uplink
# ============================================================================

target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        EpclFrameAssemblerTest.cc
        EpclFrameAssemblerTest.h
        GoalProtocolTest.cc
        GoalProtocolTest.h
        GoalSenderTest.cc
        GoalSenderTest.h
        PointCloudInstancingTest.cc
        PointCloudInstancingTest.h
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_qgc_test(EpclFrameAssemblerTest LABELS Unit PointCloud)
add_qgc_test(GoalProtocolTest LABELS Unit PointCloud)
add_qgc_test(GoalSenderTest LABELS Unit PointCloud)
add_qgc_test(PointCloudInstancingTest LABELS Unit PointCloud)
