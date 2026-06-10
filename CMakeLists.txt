cmake_minimum_required(VERSION 3.21)
project(bodytracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BODYTRACKER_BUILD_APP "Build the full native tracker application and its runtime dependencies." ON)
option(BODYTRACKER_BUILD_FULL_TESTS "Build tests that require native runtime dependency packages such as OpenCV or nlohmann_json." OFF)
option(BODYTRACKER_REGISTER_PYTHON_TESTS "Register Python/static CTest checks." ON)
option(BODYTRACKER_COPY_DIRECTML_ONNXRUNTIME "On Windows, copy DirectML-capable ONNX Runtime binaries for inference.device=directml." ON)
set(BODYTRACKER_ONNXRUNTIME_DIRECTML_VERSION "1.24.4" CACHE STRING "Microsoft.ML.OnnxRuntime.DirectML NuGet package version copied next to bodytracker.exe.")

if (BODYTRACKER_BUILD_APP)

add_executable(bodytracker
    src/main.cpp
    src/core/timing.cpp
    src/core/config.cpp
    src/core/logging.cpp
    src/capture/capture_health.cpp
    src/capture/camera_device.cpp
    src/capture/network_camera_source.cpp
    src/capture/frame_pairer.cpp
    src/calibration/calibration_io.cpp
    src/calibration/intrinsic_calibrator.cpp
    src/calibration/stereo_calibrator.cpp
    src/calibration/floor_calibrator.cpp
    src/calibration/body_calibrator.cpp
    src/inference/rtmpose_session.cpp
    src/inference/rtmpose_model_contract.cpp
    src/inference/rtmpose_decode.cpp
    src/tracking/roi_tracker.cpp
    src/tracking/reliability.cpp
    src/tracking/identity_assignment.cpp
    src/tracking/epipolar_geometry.cpp
    src/tracking/geometry_cache.cpp
    src/tracking/triangulation.cpp
    src/tracking/body_model.cpp
    src/tracking/body_state.cpp
    src/tracking/body_solver.cpp
    src/tracking/monocular_projection.cpp
    src/monocular_depth/hmd_depth_scale.cpp
    src/tracking/joint_limits.cpp
    src/tracking/posture_mode.cpp
    src/tracking/root_support.cpp
    src/tracking/foot_frame.cpp
    src/tracking/foot_support.cpp
    src/tracking/support_queries.cpp
    src/tracking/contact_constraints.cpp
    src/tracking/motion_consistency_filter.cpp
    src/tracking/tracker_ekf.cpp
    src/tracking/temporal_update.cpp
    src/tracking/tracker_synthesis.cpp
    src/tracking/tracking_pipeline.cpp
    src/tracking/steamvr_alignment.cpp
    src/tracking/steamvr_alignment_manager.cpp
    src/tracking/anchor_space_mapper.cpp
    src/tracking/room_depth_map.cpp
    src/io/osc_sender.cpp
    src/io/steamvr_provider.cpp
    src/io/steamvr_tracker_bridge.cpp
    src/io/hmd_provider.cpp
    src/debug/overlay_draw.cpp
    src/debug/world_debug.cpp
    src/debug/replay_log.cpp
    src/debug/replay_player.cpp
    src/ui/desktop_ui.cpp
)

find_package(nlohmann_json CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui calib3d)
find_package(onnxruntime CONFIG REQUIRED)
find_package(unofficial-webview2 CONFIG REQUIRED)
find_path(OPENVR_INCLUDE_DIR openvr.h)
find_library(OPENVR_LIBRARY NAMES openvr_api)

set(BODYTRACKER_DIRECTML_RUNTIME_DLLS)
if (WIN32 AND BODYTRACKER_COPY_DIRECTML_ONNXRUNTIME)
    set(BODYTRACKER_DIRECTML_PACKAGE_ROOT "${CMAKE_BINARY_DIR}/_deps/onnxruntime-directml-${BODYTRACKER_ONNXRUNTIME_DIRECTML_VERSION}")
    set(BODYTRACKER_DIRECTML_PACKAGE_ZIP "${BODYTRACKER_DIRECTML_PACKAGE_ROOT}/Microsoft.ML.OnnxRuntime.DirectML.${BODYTRACKER_ONNXRUNTIME_DIRECTML_VERSION}.zip")
    set(BODYTRACKER_DIRECTML_NATIVE_DIR "${BODYTRACKER_DIRECTML_PACKAGE_ROOT}/package/runtimes/win-x64/native")
    if (NOT EXISTS "${BODYTRACKER_DIRECTML_NATIVE_DIR}/onnxruntime.dll")
        file(MAKE_DIRECTORY "${BODYTRACKER_DIRECTML_PACKAGE_ROOT}")
        file(DOWNLOAD
            "https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime.DirectML/${BODYTRACKER_ONNXRUNTIME_DIRECTML_VERSION}"
            "${BODYTRACKER_DIRECTML_PACKAGE_ZIP}"
            STATUS BODYTRACKER_DIRECTML_DOWNLOAD_STATUS
            SHOW_PROGRESS
        )
        list(GET BODYTRACKER_DIRECTML_DOWNLOAD_STATUS 0 BODYTRACKER_DIRECTML_DOWNLOAD_CODE)
        if (BODYTRACKER_DIRECTML_DOWNLOAD_CODE EQUAL 0)
            file(ARCHIVE_EXTRACT
                INPUT "${BODYTRACKER_DIRECTML_PACKAGE_ZIP}"
                DESTINATION "${BODYTRACKER_DIRECTML_PACKAGE_ROOT}/package"
            )
        else()
            list(GET BODYTRACKER_DIRECTML_DOWNLOAD_STATUS 1 BODYTRACKER_DIRECTML_DOWNLOAD_MESSAGE)
            message(WARNING "DirectML ONNX Runtime package download failed: ${BODYTRACKER_DIRECTML_DOWNLOAD_MESSAGE}")
        endif()
    endif()
    if (EXISTS "${BODYTRACKER_DIRECTML_NATIVE_DIR}/onnxruntime.dll")
        list(APPEND BODYTRACKER_DIRECTML_RUNTIME_DLLS
            "${BODYTRACKER_DIRECTML_NATIVE_DIR}/onnxruntime.dll"
            "${BODYTRACKER_DIRECTML_NATIVE_DIR}/onnxruntime_providers_shared.dll"
        )
        message(STATUS "DirectML ONNX Runtime runtime DLLs will be copied from ${BODYTRACKER_DIRECTML_NATIVE_DIR}")
    else()
        message(WARNING "DirectML ONNX Runtime runtime DLLs unavailable; inference.device=directml will fall back to CPU")
    endif()
endif()

target_include_directories(bodytracker PRIVATE
    src
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(bodytracker PRIVATE
    nlohmann_json::nlohmann_json
    ${OpenCV_LIBS}
    onnxruntime::onnxruntime
    unofficial::webview2::webview2
)
if (OPENVR_INCLUDE_DIR AND OPENVR_LIBRARY)
    target_include_directories(bodytracker PRIVATE ${OPENVR_INCLUDE_DIR})
    target_link_libraries(bodytracker PRIVATE ${OPENVR_LIBRARY})
    target_compile_definitions(bodytracker PRIVATE BODYTRACKER_HAS_OPENVR=1)
endif()

if (WIN32 AND OPENVR_INCLUDE_DIR)
    add_library(driver_bodytracker SHARED
        src/steamvr_driver/driver_bodytracker.cpp
    )
    target_include_directories(driver_bodytracker PRIVATE ${OPENVR_INCLUDE_DIR})
    target_compile_features(driver_bodytracker PRIVATE cxx_std_20)
    target_link_libraries(driver_bodytracker PRIVATE ws2_32)
    target_compile_definitions(driver_bodytracker PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
    if (MSVC)
        target_compile_options(driver_bodytracker PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(driver_bodytracker PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    set_target_properties(driver_bodytracker PROPERTIES
        OUTPUT_NAME "driver_bodytracker"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/steamvr_driver/bin/win64"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/steamvr_driver/bin/win64"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/steamvr_driver/bin/win64"
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/steamvr_driver/bin/win64"
    )
    add_custom_command(TARGET driver_bodytracker POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/steamvr_driver"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/steamvr_driver/driver.vrdrivermanifest"
            "${CMAKE_BINARY_DIR}/steamvr_driver/driver.vrdrivermanifest"
    )
endif()

if (WIN32)
    target_link_libraries(bodytracker PRIVATE ws2_32 user32 ole32 shell32)
    add_custom_command(TARGET bodytracker POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:bodytracker> $<TARGET_RUNTIME_DLLS:bodytracker>
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/src/ui/app $<TARGET_FILE_DIR:bodytracker>/ui
        COMMAND_EXPAND_LISTS
    )
    if (BODYTRACKER_DIRECTML_RUNTIME_DLLS)
        add_custom_command(TARGET bodytracker POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${BODYTRACKER_DIRECTML_RUNTIME_DLLS} $<TARGET_FILE_DIR:bodytracker>
            COMMAND_EXPAND_LISTS
        )
    endif()
endif()

if (MSVC)
    target_compile_options(bodytracker PRIVATE /W4 /permissive- /EHsc)
    target_compile_definitions(bodytracker PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
else()
    target_compile_options(bodytracker PRIVATE -Wall -Wextra -Wpedantic)
endif()

endif()

include(CTest)

if (BUILD_TESTING AND BODYTRACKER_BUILD_FULL_TESTS AND NOT BODYTRACKER_BUILD_APP)
    find_package(nlohmann_json CONFIG REQUIRED)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui calib3d)
endif()

if (BUILD_TESTING)

add_executable(bodytracker_tests
    tests/osc_packet_test.cpp
    src/io/osc_sender.cpp
    src/tracking/tracker_synthesis.cpp
    src/tracking/body_model.cpp
    src/tracking/body_state.cpp
    src/tracking/support_queries.cpp
    src/tracking/epipolar_geometry.cpp
    src/tracking/triangulation.cpp
)

target_include_directories(bodytracker_tests PRIVATE src)

target_compile_features(bodytracker_tests PRIVATE cxx_std_20)

if (WIN32)
    target_link_libraries(bodytracker_tests PRIVATE ws2_32)
endif()

if (MSVC)
    target_compile_options(bodytracker_tests PRIVATE /W4 /permissive- /EHsc)
    target_compile_definitions(bodytracker_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
else()
    target_compile_options(bodytracker_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME bodytracker_osc_packet_test COMMAND bodytracker_tests)

# Depth-provenance falsification harness.
# Tests: decode-not-dead-code, stage-continuity, z-preservation,
#        no-fake-world-mix, rejected-candidate-provenance, OpenVR-safe-init.
if (OpenCV_FOUND AND TARGET onnxruntime::onnxruntime)
add_executable(depth_provenance_tests
    tests/depth_provenance_test.cpp
    src/inference/rtmpose_decode.cpp
    src/inference/rtmpose_model_contract.cpp
    src/inference/rtmpose_session.cpp
    src/io/osc_sender.cpp
    src/io/steamvr_provider.cpp
    src/tracking/body_state.cpp
    src/tracking/tracker_synthesis.cpp
    src/tracking/support_queries.cpp
    src/tracking/body_model.cpp
    src/tracking/epipolar_geometry.cpp
    src/tracking/triangulation.cpp
    src/core/logging.cpp
    src/core/timing.cpp
)
target_include_directories(depth_provenance_tests PRIVATE src tests ${OpenCV_INCLUDE_DIRS})
target_compile_features(depth_provenance_tests PRIVATE cxx_std_20)
target_link_libraries(depth_provenance_tests PRIVATE ${OpenCV_LIBS} onnxruntime::onnxruntime)
if (WIN32)
    target_link_libraries(depth_provenance_tests PRIVATE ws2_32)
    target_compile_options(depth_provenance_tests PRIVATE /W4 /permissive- /EHsc)
    target_compile_definitions(depth_provenance_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
else()
    target_compile_options(depth_provenance_tests PRIVATE -Wall -Wextra)
endif()
add_test(NAME depth_provenance_test COMMAND depth_provenance_tests)
set_tests_properties(depth_provenance_test PROPERTIES LABELS "depth_provenance;z_continuity")
else()
    message(STATUS "Skipping depth_provenance_tests because OpenCV/ONNX Runtime targets are not configured")
endif()


add_executable(osc_encoding_performance_tests
    tests/osc_encoding_performance_test.cpp
    src/io/osc_sender.cpp
)

target_include_directories(osc_encoding_performance_tests PRIVATE src tests)

target_compile_features(osc_encoding_performance_tests PRIVATE cxx_std_20)

if (WIN32)
    target_link_libraries(osc_encoding_performance_tests PRIVATE ws2_32)
endif()

if (MSVC)
    target_compile_options(osc_encoding_performance_tests PRIVATE /W4 /permissive- /EHsc)
    target_compile_definitions(osc_encoding_performance_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
else()
    target_compile_options(osc_encoding_performance_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME osc_packet_encoding_budget_test COMMAND osc_encoding_performance_tests)
set_tests_properties(osc_packet_encoding_budget_test PROPERTIES LABELS "performance;semantic")

add_executable(triangulation_confidence_tests
    tests/triangulation_confidence_test.cpp
    src/tracking/epipolar_geometry.cpp
    src/tracking/triangulation.cpp
)

target_include_directories(triangulation_confidence_tests PRIVATE src tests)

target_compile_features(triangulation_confidence_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(triangulation_confidence_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(triangulation_confidence_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME triangulation_confidence_test COMMAND triangulation_confidence_tests)

add_executable(epipolar_geometry_tests
    tests/epipolar_geometry_test.cpp
    src/tracking/epipolar_geometry.cpp
)

target_include_directories(epipolar_geometry_tests PRIVATE src tests)

target_compile_features(epipolar_geometry_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(epipolar_geometry_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(epipolar_geometry_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME epipolar_geometry_test COMMAND epipolar_geometry_tests)

add_executable(measurement_weighting_tests
    tests/measurement_weighting_test.cpp
)

target_include_directories(measurement_weighting_tests PRIVATE src tests)

target_compile_features(measurement_weighting_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(measurement_weighting_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(measurement_weighting_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME measurement_weighting_test COMMAND measurement_weighting_tests)

add_executable(tracker_synthesis_tests
    tests/tracker_synthesis_test.cpp
    src/tracking/tracker_synthesis.cpp
    src/tracking/body_model.cpp
    src/tracking/body_state.cpp
    src/tracking/support_queries.cpp
)

target_include_directories(tracker_synthesis_tests PRIVATE src tests)

target_compile_features(tracker_synthesis_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(tracker_synthesis_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(tracker_synthesis_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME tracker_synthesis_test COMMAND tracker_synthesis_tests)

add_executable(tracker_synthesis_confidence_tests
    tests/tracker_synthesis_confidence_test.cpp
    src/tracking/tracker_synthesis.cpp
    src/tracking/body_model.cpp
    src/tracking/body_state.cpp
    src/tracking/support_queries.cpp
)

target_include_directories(tracker_synthesis_confidence_tests PRIVATE src tests)

target_compile_features(tracker_synthesis_confidence_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(tracker_synthesis_confidence_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(tracker_synthesis_confidence_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME tracker_synthesis_confidence_test COMMAND tracker_synthesis_confidence_tests)

add_executable(reliability_confidence_tests
    tests/reliability_confidence_test.cpp
    src/tracking/reliability.cpp
)

target_include_directories(reliability_confidence_tests PRIVATE tests/stubs src tests)

target_compile_features(reliability_confidence_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(reliability_confidence_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(reliability_confidence_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME reliability_confidence_test COMMAND reliability_confidence_tests)

add_executable(reliability_fuzz_tests
    tests/reliability_fuzz_test.cpp
    src/tracking/reliability.cpp
)

target_include_directories(reliability_fuzz_tests PRIVATE tests/stubs src tests)

target_compile_features(reliability_fuzz_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(reliability_fuzz_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(reliability_fuzz_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME reliability_property_fuzz_test COMMAND reliability_fuzz_tests)
set_tests_properties(reliability_property_fuzz_test PROPERTIES LABELS "fuzz;semantic")

add_executable(body_state_tests
    tests/body_state_test.cpp
    src/tracking/body_state.cpp
    src/tracking/tracker_synthesis.cpp
    src/tracking/body_model.cpp
    src/tracking/support_queries.cpp
)

target_include_directories(body_state_tests PRIVATE src tests)

target_compile_features(body_state_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(body_state_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(body_state_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME unified_body_state_test COMMAND body_state_tests)

add_executable(steamvr_alignment_tests
    tests/steamvr_alignment_test.cpp
    src/tracking/steamvr_alignment.cpp
    src/tracking/steamvr_alignment_manager.cpp
    src/tracking/body_state.cpp
    src/tracking/tracker_synthesis.cpp
    src/tracking/body_model.cpp
    src/tracking/support_queries.cpp
    src/io/osc_sender.cpp
    src/io/steamvr_provider.cpp
)

target_include_directories(steamvr_alignment_tests PRIVATE src tests)

target_compile_features(steamvr_alignment_tests PRIVATE cxx_std_20)

if (WIN32)
    target_link_libraries(steamvr_alignment_tests PRIVATE ws2_32)
endif()

if (MSVC)
    target_compile_options(steamvr_alignment_tests PRIVATE /W4 /permissive- /EHsc)
    target_compile_definitions(steamvr_alignment_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
else()
    target_compile_options(steamvr_alignment_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME steamvr_controller_alignment_test COMMAND steamvr_alignment_tests)

add_executable(rtmpose_contract_tests
    tests/rtmpose_contract_test.cpp
    src/inference/rtmpose_model_contract.cpp
    src/tracking/foot_frame.cpp
)

target_include_directories(rtmpose_contract_tests PRIVATE src tests)

target_compile_features(rtmpose_contract_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(rtmpose_contract_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(rtmpose_contract_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME rtmpose_halpe26_model_contract_test COMMAND rtmpose_contract_tests)

add_executable(model_asset_tests
    tests/model_asset_test.cpp
)

target_include_directories(model_asset_tests PRIVATE tests)

target_compile_features(model_asset_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(model_asset_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(model_asset_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME provided_cocktail_primary_and_rtmw3d_postprocess_asset_test COMMAND model_asset_tests)
set_tests_properties(provided_cocktail_primary_and_rtmw3d_postprocess_asset_test PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

set(BODYTRACKER_RTMPOSE_ONNX ${CMAKE_SOURCE_DIR}/models/rtmw-dw-x-l-cocktail14-384x288.onnx)
set(BODYTRACKER_DEPTH_POSTPROCESS_ONNX ${CMAKE_SOURCE_DIR}/models/rtmw3d-x-cocktail14-384x288.onnx)
if (BODYTRACKER_REGISTER_PYTHON_TESTS)
find_package(Python3 COMPONENTS Interpreter QUIET)
if (Python3_Interpreter_FOUND)
    if (EXISTS ${BODYTRACKER_RTMPOSE_ONNX})
        add_test(
            NAME rtmw_dw_x_l_onnx_static_metadata_test
            COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/tools/inspect_onnx_contract.py
                ${BODYTRACKER_RTMPOSE_ONNX}
        )
    else()
        message(STATUS "RTMW-DW-X-L ONNX asset not found; skipping rtmw_dw_x_l_onnx_static_metadata_test for source-only package")
    endif()
    if (EXISTS ${BODYTRACKER_DEPTH_POSTPROCESS_ONNX})
        add_test(
            NAME rtmw3d_x_depth_postprocess_onnx_static_metadata_test
            COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/tools/inspect_onnx_contract.py
                ${BODYTRACKER_DEPTH_POSTPROCESS_ONNX}
        )
    else()
        message(STATUS "RTMW3D depth postprocess ONNX asset not found; skipping rtmw3d_x_depth_postprocess_onnx_static_metadata_test for source-only package")
    endif()
    add_test(
        NAME monocular_real_footage_validation_test
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/monocular_real_footage_validation_test.py
    )
    add_test(
        NAME live_bringup_readiness_test
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/live_bringup_readiness_test.py
    )
    add_test(
        NAME live_preflight_doctor_test
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/live_preflight_doctor_test.py
    )
    add_test(
        NAME runtime_control_wiring_test
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/runtime_control_wiring_test.py
    )
    add_test(
        NAME synthetic_stereo_diagnostic_test
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/synthetic_stereo_diagnostic_test.py
    )
    add_test(
        NAME synthetic_stereo_scoring_test
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/synthetic_stereo_scoring_test.py
    )
endif()
endif()

if (BODYTRACKER_BUILD_FULL_TESTS)
add_executable(frame_pairer_tests
    tests/frame_pairer_test.cpp
    src/capture/frame_pairer.cpp
    src/core/timing.cpp
)

target_include_directories(frame_pairer_tests PRIVATE
    src
    ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(frame_pairer_tests PRIVATE cxx_std_20)

target_link_libraries(frame_pairer_tests PRIVATE ${OpenCV_LIBS})

if (MSVC)
    target_compile_options(frame_pairer_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(frame_pairer_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME frame_pairer_duplicate_rejection_test COMMAND frame_pairer_tests)
endif()

if (BODYTRACKER_BUILD_APP OR BODYTRACKER_BUILD_FULL_TESTS)
add_executable(network_camera_lifecycle_tests
    tests/network_camera_lifecycle_test.cpp
    src/capture/camera_device.cpp
    src/capture/network_camera_source.cpp
    src/capture/capture_health.cpp
    src/core/logging.cpp
    src/core/timing.cpp
)

target_include_directories(network_camera_lifecycle_tests PRIVATE
    src
    tests
    ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(network_camera_lifecycle_tests PRIVATE cxx_std_20)

target_link_libraries(network_camera_lifecycle_tests PRIVATE ${OpenCV_LIBS})

if (WIN32)
    target_link_libraries(network_camera_lifecycle_tests PRIVATE ws2_32)
endif()

if (MSVC)
    target_compile_options(network_camera_lifecycle_tests PRIVATE /W4 /permissive- /EHsc)
    target_compile_definitions(network_camera_lifecycle_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
else()
    target_compile_options(network_camera_lifecycle_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME network_camera_stop_preserves_latest_frame_test COMMAND network_camera_lifecycle_tests)
endif()

if (BODYTRACKER_BUILD_FULL_TESTS)
add_executable(network_camera_source_lifecycle_tests
    tests/network_camera_source_lifecycle_test.cpp
    src/capture/network_camera_source.cpp
    src/core/logging.cpp
    src/core/timing.cpp
)

target_include_directories(network_camera_source_lifecycle_tests PRIVATE
    src
    tests
    ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(network_camera_source_lifecycle_tests PRIVATE cxx_std_20)

target_link_libraries(network_camera_source_lifecycle_tests PRIVATE ${OpenCV_LIBS})

if (WIN32)
    target_link_libraries(network_camera_source_lifecycle_tests PRIVATE ws2_32)
endif()

if (MSVC)
    target_compile_options(network_camera_source_lifecycle_tests PRIVATE /W4 /permissive- /EHsc)
    target_compile_definitions(network_camera_source_lifecycle_tests PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
else()
    target_compile_options(network_camera_source_lifecycle_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME network_camera_fake_source_lifecycle_test COMMAND network_camera_source_lifecycle_tests)
set_tests_properties(network_camera_fake_source_lifecycle_test PROPERTIES LABELS "semantic;network_fake")
endif()

if (BODYTRACKER_BUILD_APP OR BODYTRACKER_BUILD_FULL_TESTS)
add_executable(floor_geometry_calibrator_behavior_tests
    tests/floor_geometry_calibrator_behavior_test.cpp
    src/calibration/floor_calibrator.cpp
)

target_include_directories(floor_geometry_calibrator_behavior_tests PRIVATE src tests)

target_compile_features(floor_geometry_calibrator_behavior_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(floor_geometry_calibrator_behavior_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(floor_geometry_calibrator_behavior_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME floor_geometry_calibrator_behavior_test COMMAND floor_geometry_calibrator_behavior_tests)

endif()

add_executable(floor_plane_tests
    tests/floor_plane_test.cpp
)

target_include_directories(floor_plane_tests PRIVATE src)

target_compile_features(floor_plane_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(floor_plane_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(floor_plane_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME floor_plane_helper_test COMMAND floor_plane_tests)

add_executable(foot_support_tests
    tests/foot_support_test.cpp
    src/tracking/foot_support.cpp
    src/tracking/contact_constraints.cpp
    src/tracking/support_queries.cpp
)

target_include_directories(foot_support_tests PRIVATE src)

target_compile_features(foot_support_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(foot_support_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(foot_support_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME foot_support_floor_plane_test COMMAND foot_support_tests)


add_executable(root_support_tests
    tests/root_support_test.cpp
    src/tracking/root_support.cpp
    src/tracking/body_model.cpp
    src/tracking/support_queries.cpp
)

target_include_directories(root_support_tests PRIVATE src)

target_compile_features(root_support_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(root_support_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(root_support_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME root_support_false_kneeling_contact_test COMMAND root_support_tests)

add_executable(body_model_seed_tests
    tests/body_model_seed_test.cpp
    src/tracking/body_model.cpp
)

target_include_directories(body_model_seed_tests PRIVATE src)

target_compile_features(body_model_seed_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(body_model_seed_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(body_model_seed_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME body_model_seed_weighting_test COMMAND body_model_seed_tests)

add_executable(monocular_tracking_tests
    tests/monocular_tracking_test.cpp
    src/tracking/monocular_projection.cpp
    src/monocular_depth/hmd_depth_scale.cpp
)

target_include_directories(monocular_tracking_tests PRIVATE src tests)

target_compile_features(monocular_tracking_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(monocular_tracking_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(monocular_tracking_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME monocular_single_camera_projection_test COMMAND monocular_tracking_tests)

add_executable(tracker_ekf_tests
    tests/tracker_ekf_test.cpp
    src/tracking/tracker_ekf.cpp
    src/tracking/support_queries.cpp
    src/tracking/contact_constraints.cpp
)

target_include_directories(tracker_ekf_tests PRIVATE src)

target_compile_features(tracker_ekf_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(tracker_ekf_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(tracker_ekf_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME tracker_ekf_smoothing_test COMMAND tracker_ekf_tests)

add_executable(temporal_update_tests
    tests/temporal_update_test.cpp
    src/tracking/temporal_update.cpp
    src/tracking/support_queries.cpp
    src/tracking/contact_constraints.cpp
)

target_include_directories(temporal_update_tests PRIVATE src)

target_compile_features(temporal_update_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(temporal_update_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(temporal_update_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME temporal_update_foot_pinning_test COMMAND temporal_update_tests)

if (BODYTRACKER_BUILD_FULL_TESTS)
add_executable(hmd_provider_tests
    tests/hmd_provider_test.cpp
    src/io/hmd_provider.cpp
)

target_include_directories(hmd_provider_tests PRIVATE src)

target_compile_features(hmd_provider_tests PRIVATE cxx_std_20)

target_link_libraries(hmd_provider_tests PRIVATE nlohmann_json::nlohmann_json)

if (MSVC)
    target_compile_options(hmd_provider_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(hmd_provider_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME hmd_provider_json_file_test COMMAND hmd_provider_tests)
endif()

add_executable(motion_consistency_filter_tests
    tests/motion_consistency_filter_test.cpp
    src/tracking/motion_consistency_filter.cpp
    src/tracking/support_queries.cpp
    src/tracking/contact_constraints.cpp
)

target_include_directories(motion_consistency_filter_tests PRIVATE src)

target_compile_features(motion_consistency_filter_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(motion_consistency_filter_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(motion_consistency_filter_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME motion_consistency_filter_test COMMAND motion_consistency_filter_tests)

add_executable(griddy_motion_regression_tests
    tests/griddy_motion_regression_test.cpp
    src/tracking/motion_consistency_filter.cpp
    src/tracking/temporal_update.cpp
    src/tracking/support_queries.cpp
    src/tracking/contact_constraints.cpp
)

target_include_directories(griddy_motion_regression_tests PRIVATE src)

target_compile_features(griddy_motion_regression_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(griddy_motion_regression_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(griddy_motion_regression_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME griddy_motion_regression_test COMMAND griddy_motion_regression_tests)

add_executable(anchor_space_mapper_tests
    tests/anchor_space_mapper_test.cpp
    src/tracking/anchor_space_mapper.cpp
    src/tracking/monocular_projection.cpp
    src/monocular_depth/hmd_depth_scale.cpp
)

target_include_directories(anchor_space_mapper_tests PRIVATE src tests)

target_compile_features(anchor_space_mapper_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(anchor_space_mapper_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(anchor_space_mapper_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME anchor_space_mapper_projection_correction_test COMMAND anchor_space_mapper_tests)

add_executable(config_tests
    tests/config_test.cpp
    src/core/config.cpp
)

target_include_directories(config_tests PRIVATE src)

target_compile_features(config_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(config_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(config_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME config_alias_schema_validation_test COMMAND config_tests)

add_executable(profiler_budget_tests
    tests/profiler_budget_test.cpp
)

target_include_directories(profiler_budget_tests PRIVATE src tests)

target_compile_features(profiler_budget_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(profiler_budget_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(profiler_budget_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME profiler_budget_test COMMAND profiler_budget_tests)
set_tests_properties(profiler_budget_test PROPERTIES LABELS "performance;semantic")

if (BODYTRACKER_REGISTER_PYTHON_TESTS)
find_package(Python3 COMPONENTS Interpreter QUIET)
if (Python3_Interpreter_FOUND)
    add_test(
        NAME performance_budget_tool_test
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/performance_budget_tool_test.py
    )
    set_tests_properties(performance_budget_tool_test PROPERTIES LABELS "performance;qa")
endif()
endif()

if (BODYTRACKER_BUILD_FULL_TESTS)
add_executable(tracking_pipeline_fallback_tests
    tests/tracking_pipeline_fallback_test.cpp
    src/core/timing.cpp
    src/tracking/body_model.cpp
    src/tracking/body_solver.cpp
    src/tracking/monocular_projection.cpp
    src/monocular_depth/hmd_depth_scale.cpp
    src/tracking/joint_limits.cpp
    src/tracking/posture_mode.cpp
    src/tracking/root_support.cpp
    src/tracking/foot_frame.cpp
    src/tracking/foot_support.cpp
    src/tracking/support_queries.cpp
    src/tracking/contact_constraints.cpp
    src/tracking/motion_consistency_filter.cpp
    src/tracking/tracker_ekf.cpp
    src/tracking/temporal_update.cpp
    src/tracking/tracker_synthesis.cpp
    src/tracking/tracking_pipeline.cpp
    src/tracking/identity_assignment.cpp
    src/tracking/epipolar_geometry.cpp
    src/tracking/triangulation.cpp
    src/tracking/body_state.cpp
    src/calibration/body_calibrator.cpp
    src/calibration/calibration_io.cpp
)

target_include_directories(tracking_pipeline_fallback_tests PRIVATE
    src
    ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(tracking_pipeline_fallback_tests PRIVATE cxx_std_20)

target_link_libraries(tracking_pipeline_fallback_tests PRIVATE nlohmann_json::nlohmann_json ${OpenCV_LIBS})

if (MSVC)
    target_compile_options(tracking_pipeline_fallback_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(tracking_pipeline_fallback_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME tracking_pipeline_no_hmd_fallback_test COMMAND tracking_pipeline_fallback_tests)
endif()

add_executable(airborne_pipeline_regression_tests
    tests/airborne_pipeline_regression_test.cpp
    src/tracking/motion_consistency_filter.cpp
    src/tracking/temporal_update.cpp
    src/tracking/tracker_ekf.cpp
    src/tracking/tracker_synthesis.cpp
    src/tracking/body_model.cpp
    src/tracking/support_queries.cpp
    src/tracking/contact_constraints.cpp
)

target_include_directories(airborne_pipeline_regression_tests PRIVATE src)

target_compile_features(airborne_pipeline_regression_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(airborne_pipeline_regression_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(airborne_pipeline_regression_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME airborne_pipeline_regression_test COMMAND airborne_pipeline_regression_tests)

add_executable(contact_constraint_cleanup_tests
    tests/contact_constraint_cleanup_test.cpp
    src/tracking/motion_consistency_filter.cpp
    src/tracking/support_queries.cpp
    src/tracking/contact_constraints.cpp
)

target_include_directories(contact_constraint_cleanup_tests PRIVATE src)

target_compile_features(contact_constraint_cleanup_tests PRIVATE cxx_std_20)

if (MSVC)
    target_compile_options(contact_constraint_cleanup_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(contact_constraint_cleanup_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME contact_constraint_cleanup_test COMMAND contact_constraint_cleanup_tests)

if (BODYTRACKER_BUILD_FULL_TESTS)
add_executable(replay_log_export_tests
    tests/replay_log_export_test.cpp
    src/debug/replay_log.cpp
    src/tracking/contact_constraints.cpp
    src/tracking/support_queries.cpp
)

target_include_directories(replay_log_export_tests PRIVATE
    tests/stubs
    src
    tests
)

target_compile_features(replay_log_export_tests PRIVATE cxx_std_20)

target_link_libraries(replay_log_export_tests PRIVATE nlohmann_json::nlohmann_json)

if (MSVC)
    target_compile_options(replay_log_export_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(replay_log_export_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME replay_log_telemetry_export_test COMMAND replay_log_export_tests)
endif()

if (BODYTRACKER_BUILD_FULL_TESTS)
add_executable(replay_player_contract_tests
    tests/replay_player_contract_test.cpp
    src/debug/replay_player.cpp
)

target_include_directories(replay_player_contract_tests PRIVATE src tests ${OpenCV_INCLUDE_DIRS})

target_compile_features(replay_player_contract_tests PRIVATE cxx_std_20)

target_link_libraries(replay_player_contract_tests PRIVATE nlohmann_json::nlohmann_json ${OpenCV_LIBS})

if (MSVC)
    target_compile_options(replay_player_contract_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(replay_player_contract_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME replay_player_runtime_contract_test COMMAND replay_player_contract_tests)
set_tests_properties(replay_player_runtime_contract_test PROPERTIES LABELS "semantic;replay")
endif()

if (BODYTRACKER_BUILD_FULL_TESTS)
add_executable(identity_assignment_tests
    tests/identity_assignment_test.cpp
    src/tracking/identity_assignment.cpp
    src/tracking/epipolar_geometry.cpp
    src/tracking/geometry_cache.cpp
    src/tracking/triangulation.cpp
)

target_include_directories(identity_assignment_tests PRIVATE src tests ${OpenCV_INCLUDE_DIRS})

target_compile_features(identity_assignment_tests PRIVATE cxx_std_20)

target_link_libraries(identity_assignment_tests PRIVATE ${OpenCV_LIBS})

if (MSVC)
    target_compile_options(identity_assignment_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(identity_assignment_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME identity_assignment_test COMMAND identity_assignment_tests)
endif()


if (BODYTRACKER_BUILD_FULL_TESTS)

# Phase 11: Geometry cache tests
add_executable(geometry_cache_tests
    tests/geometry_cache_test.cpp
    src/tracking/geometry_cache.cpp
    src/tracking/epipolar_geometry.cpp
    src/tracking/triangulation.cpp
)

target_include_directories(geometry_cache_tests PRIVATE src tests ${OpenCV_INCLUDE_DIRS})

target_compile_features(geometry_cache_tests PRIVATE cxx_std_20)

target_link_libraries(geometry_cache_tests PRIVATE ${OpenCV_LIBS})

if (MSVC)
    target_compile_options(geometry_cache_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(geometry_cache_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME geometry_cache_test COMMAND geometry_cache_tests)
set_tests_properties(geometry_cache_test PROPERTIES LABELS "phase11;geometry_cache")

# Phase 12: Adversarial identity arbitration tests
add_executable(identity_adversarial_tests
    tests/identity_adversarial_test.cpp
    src/tracking/identity_assignment.cpp
    src/tracking/epipolar_geometry.cpp
    src/tracking/triangulation.cpp
)

target_include_directories(identity_adversarial_tests PRIVATE src tests ${OpenCV_INCLUDE_DIRS})

target_compile_features(identity_adversarial_tests PRIVATE cxx_std_20)

target_link_libraries(identity_adversarial_tests PRIVATE ${OpenCV_LIBS})

if (MSVC)
    target_compile_options(identity_adversarial_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(identity_adversarial_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME identity_adversarial_test COMMAND identity_adversarial_tests)
set_tests_properties(identity_adversarial_test PROPERTIES LABELS "phase12;identity_arbitration;adversarial")

endif()

endif()

