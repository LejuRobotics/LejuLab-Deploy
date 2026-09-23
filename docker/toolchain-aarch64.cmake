# RK3588 (aarch64) 交叉编译 toolchain file
# 在交叉编译 Docker 镜像内使用：
#   cmake -B build_cross -DCMAKE_TOOLCHAIN_FILE=/opt/toolchain-aarch64.cmake ...
#
# 宿主用原生 x86 的 aarch64-linux-gnu-gcc-11（与板子 gcc 11.4.0 一致），
# 配 /opt/rk3588-sysroot（从真实板子提取）。

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 交叉编译器（固定 gcc-11，匹配仓库内 CycloneDDS/iceoryx 的 GCC11 LTO 字节码）
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc-11)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++-11)

# 从板子提取的 sysroot
set(_SYSROOT /opt/rk3588-sysroot)
set(CMAKE_SYSROOT ${_SYSROOT})
set(CMAKE_FIND_ROOT_PATH ${_SYSROOT} ${_SYSROOT}/usr ${_SYSROOT}/usr/local ${_SYSROOT}/opt/drake)

# 在 sysroot 内查找库/头/包，但程序（编译器等）用宿主的
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# PACKAGE 用 BOTH：既在 sysroot 找系统包，也能命中仓库内 3rd_party/aarch64 的
# CycloneDDS/iceoryx cmake config（经 CMAKE_PREFIX_PATH 注入，不在 sysroot 下）。
# 这些 config 用绝对仓库路径指向 aarch64 预编译库，链接仍是正确架构。
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# 链接期让 ld 能在 sysroot 内解析次级依赖（间接 .so）
set(_RPATH_LINK "${_SYSROOT}/usr/lib/aarch64-linux-gnu:${_SYSROOT}/lib/aarch64-linux-gnu:${_SYSROOT}/usr/local/lib:${_SYSROOT}/opt/drake/lib")
set(CMAKE_EXE_LINKER_FLAGS_INIT    "-Wl,-rpath-link,${_RPATH_LINK}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-Wl,-rpath-link,${_RPATH_LINK}")

# pkg-config 指向 sysroot
set(ENV{PKG_CONFIG_DIR} "")
set(ENV{PKG_CONFIG_SYSROOT_DIR} ${_SYSROOT})
set(ENV{PKG_CONFIG_LIBDIR} "${_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${_SYSROOT}/usr/lib/pkgconfig:${_SYSROOT}/usr/share/pkgconfig:${_SYSROOT}/usr/local/lib/pkgconfig")

# Drake：sysroot 内扁平化路径。toolchain 在根 CMakeLists 之前加载，
# 因此该目标架构路径优先于任何宿主机标准前缀。
if(NOT DRAKE_PREFIX)
    set(DRAKE_PREFIX ${_SYSROOT}/opt/drake CACHE PATH "Drake install prefix (sysroot)")
endif()
