# Select architecture-specific 3rd_party directory
# Sets THIRD_PARTY_ARCH_DIR based on CMAKE_SYSTEM_PROCESSOR.
# When CROSS_COMPILE_AARCH64=ON (set by aarch64 cross-compile build),
# forces selection of aarch64 libraries regardless of host CMAKE_SYSTEM_PROCESSOR.
if(CROSS_COMPILE_AARCH64)
    set(THIRD_PARTY_ARCH_DIR "${THIRD_PARTY_DIR}/aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(THIRD_PARTY_ARCH_DIR "${THIRD_PARTY_DIR}/aarch64")
else()
    set(THIRD_PARTY_ARCH_DIR "${THIRD_PARTY_DIR}/x86_64")
endif()

if(NOT EXISTS "${THIRD_PARTY_ARCH_DIR}")
    message(FATAL_ERROR "3rd_party directory for ${CMAKE_SYSTEM_PROCESSOR} not found: ${THIRD_PARTY_ARCH_DIR}")
endif()

message(STATUS "Using 3rd_party libraries for: ${CMAKE_SYSTEM_PROCESSOR} -> ${THIRD_PARTY_ARCH_DIR}")