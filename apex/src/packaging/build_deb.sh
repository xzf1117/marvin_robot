#!/usr/bin/env bash
set -euo pipefail

# ── 版本号（去掉 v 前缀） ──────────────────────────────────────────────────
# git describe 必须在 repo 根（src/）执行
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VERSION=$(git -C "${SCRIPT_DIR}" describe --tags --abbrev=0 | sed 's/^v//')
PKG_NAME="kernelmind-apex_${VERSION}_arm64"
INSTALL_PREFIX="opt/kernelmind/apex"

echo ">>> 打包版本：${VERSION}"

# ── 工作目录（脚本上层，即 workspace 根） ──────────────────────────────────
WORKSPACE_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
cd "${WORKSPACE_DIR}"

# ── 创建 deb 目录树 ──────────────────────────────────────────────────────────
BUILD_DIR="${WORKSPACE_DIR}/${PKG_NAME}"
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}/DEBIAN"
mkdir -p "${BUILD_DIR}/${INSTALL_PREFIX}"

# ── 复制 install/ ─────────────────────────────────────────────────────────────
cp -r "${WORKSPACE_DIR}/install" "${BUILD_DIR}/${INSTALL_PREFIX}/install"

# ── 复制 vendored runtime assets ─────────────────────────────────────────────
if [ -d "${REPO_DIR}/external" ]; then
    cp -r "${REPO_DIR}/external" "${BUILD_DIR}/${INSTALL_PREFIX}/external"
fi

# ── 复制并修改 bringup 脚本 ───────────────────────────────────────────────────
for SCRIPT in bringup_RM.sh bringup_RM_glove.sh; do
    if [ -f "${REPO_DIR}/${SCRIPT}" ]; then
        sed 's|WS=.*|WS="/opt/kernelmind/apex"|g' \
            "${REPO_DIR}/${SCRIPT}" \
            > "${BUILD_DIR}/${INSTALL_PREFIX}/${SCRIPT}"
        chmod 755 "${BUILD_DIR}/${INSTALL_PREFIX}/${SCRIPT}"
    fi
done

# ── 生成 DEBIAN/control ───────────────────────────────────────────────────────
cat > "${BUILD_DIR}/DEBIAN/control" <<EOF
Package: kernelmind-apex
Version: ${VERSION}
Architecture: arm64
Maintainer: KernelMind <support@kernelmind.com>
Depends: ros-humble-ros-base
Description: KernelMind Apex — ROS 2 Humble robot software stack
 Compiled ROS 2 workspace for NVIDIA Jetson (ARM64).
 Includes marvin_msgs, marvin_description, marvin_ros2_control_wb,
 marvin_teleop, gmsl_quadcam, dm_gripper_py, bag_playback_nodes_py,
 bag_recorder_nodes_py, node_manager, robot_setting.
EOF

# ── 复制 maintainer 脚本 ──────────────────────────────────────────────────────
cp "${SCRIPT_DIR}/postinst" "${BUILD_DIR}/DEBIAN/postinst"
cp "${SCRIPT_DIR}/prerm"    "${BUILD_DIR}/DEBIAN/prerm"
chmod 755 "${BUILD_DIR}/DEBIAN/postinst" "${BUILD_DIR}/DEBIAN/prerm"

# ── 构建 .deb ─────────────────────────────────────────────────────────────────
DEB_FILE="${WORKSPACE_DIR}/${PKG_NAME}.deb"
dpkg-deb --build --root-owner-group "${BUILD_DIR}" "${DEB_FILE}"

echo ">>> 生成完毕：${DEB_FILE}"
