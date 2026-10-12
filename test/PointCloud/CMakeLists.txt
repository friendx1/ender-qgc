# ============================================================================
# PointCloud Unit Tests
# EPCL v1 reassembly and point cloud scene mapping
# ============================================================================

target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
        EpclFrameAssemblerTest.cc
        EpclFrameAssemblerTest.h
        PointCloudInstancingTest.cc
        PointCloudInstancingTest.h
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_qgc_test(EpclFrameAssemblerTest LABELS Unit PointCloud)
add_qgc_test(PointCloudInstancingTest LABELS Unit PointCloud)
