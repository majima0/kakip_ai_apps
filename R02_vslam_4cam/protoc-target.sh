#!/bin/bash

#SYSROOT=`cd $(dirname ${0}) && pwd`/sysroots/arm64-ubuntu
SYSROOT=/opt/sysroots/arm64-ubuntu
/usr/bin/qemu-aarch64-static -L ${SYSROOT} ${SYSROOT}/usr/bin/protoc $@