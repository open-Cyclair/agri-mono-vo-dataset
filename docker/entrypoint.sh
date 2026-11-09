#!/bin/bash
# Entry point of the GS-AgriField tools image.
set -e
source /opt/ros/humble/setup.bash

TOOLS=/opt/gs-agrifield/scripts

case "${1:-}" in
    download)
        shift
        exec python3 "$TOOLS/download_marv.py" "$@"
        ;;
    convert)
        shift
        exec python3 "$TOOLS/mono_camera_kitti_converter.py" "$@"
        ;;
    -h | --help | "")
        cat <<'USAGE'
GS-AgriField tools

  download <dataset_hash> --output-dir /data [--chunks ...]   download a bag from MARV
  convert --bag /data/<bag> --out /data/<out> --seq <name>    convert a bag to KITTI-like

Add --help after a command for its options.
See https://github.com/open-Cyclair/agri-mono-vo-dataset
USAGE
        ;;
    *)
        exec "$@"
        ;;
esac
