#!/bin/bash
# vi: ts=2:sw=2:et:ai

set -x

declare -a common

TARGET=${TARGET:-galaxys4g}
BLDJOBS=${BLDJOBS:-"$(cat /proc/cpuinfo | grep ^process | wc -l)"}
ARCH=${ARCH:-arm}
HOSTCC=${HOSTCC:-"ccache gcc"}
HOSTCXX=${HOSTCXX:-"ccache g++"}
CROSS_COMPILE=${CROSS_COMPILE:-"ccache ${HOME}/arm-2009q3/bin/arm-none-eabi-"}
DEF_INITRAMFS_PATH=${DEF_INITRAMFS_PATH:-"${HOME}/android_initramfs_samsung_galaxys4g"}
OUTPUT="$(pwd)/out/${TARGET}"
CCACHE_DIR="$(pwd)/out/.ccache"
common+=( "-j1" )
common+=( "O=${OUTPUT}" )
common+=( "ARCH=${ARCH}" )
common+=( "HOSTCC=${HOSTCC}" )
common+=( "HOSTCXX=${HOSTCXX}" )
common+=( "CROSS_COMPILE=${CROSS_COMPILE}" )

[ -d "${OUTPUT}" ] || mkdir -p "${OUTPUT}"
[ -d "${CCACHE_DIR}" ] || mkdir -p "${CCACHE_DIR}"

ccache -M 3G

# Prep initramfs
if [ -d /tmp/initramfs ]; then
  cp -rf /tmp/initramfs/* ${DEF_INITRAMFS_PATH}/
  rm -rf /tmp/initramfs
fi
cp -rf ${DEF_INITRAMFS_PATH} /tmp/initramfs
find /tmp/initramfs -name '\.git' -o -name '\.gitignore' | xargs rm -rf

# make kernel
make "${common[@]}" distclean
make "${common[@]}" ${TARGET}_defconfig
make "${common[@]}" zImage modules

# rebuild the kernel with freshly made modules
cp $(find ${OUTPUT} -name '*.ko') /tmp/initramfs/lib/modules/
cp $(find ${OUTPUT} -name '*.ko') "${DEF_INITRAMFS_PATH}/lib/modules/"
rm ${OUTPUT}/usr/*.o ${OUTPUT}/usr/initramfs_data.cpio.lzma
make "${common[@]}" zImage
