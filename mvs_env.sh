#!/bin/bash
# Load MVS SDK environment variables and library path
# Usage: source mvs_env.sh

export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_SDK_VERSION=
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG
export LD_LIBRARY_PATH=/opt/MVS/lib/aarch64:$LD_LIBRARY_PATH

echo "[mvs_env] MVS environment loaded."
