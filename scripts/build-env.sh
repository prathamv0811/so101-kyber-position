#!/usr/bin/env bash
# Source from project root in Bash or Zsh. macOS uses native framework linking.
POSITION_ROOT="$(pwd)"
if [ ! -f "$POSITION_ROOT/SDK.lock.json" ]; then
    echo "Run from the so101-kyber-position project root" >&2
    return 1
fi
if [ "$(uname -s)" = Linux ]; then
    POSITION_TRIPLET="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
    export PKG_CONFIG_PATH="$POSITION_ROOT/.cache/sysroot/usr/lib/$POSITION_TRIPLET/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
    export LIBRARY_PATH="$POSITION_ROOT/.cache/sysroot/usr/lib/$POSITION_TRIPLET${LIBRARY_PATH:+:$LIBRARY_PATH}"
    export LD_LIBRARY_PATH="$POSITION_ROOT/.cache/sysroot/usr/lib/$POSITION_TRIPLET${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export GST_PLUGIN_PATH_1_0="$POSITION_ROOT/.cache/gstreamer${GST_PLUGIN_PATH_1_0:+:$GST_PLUGIN_PATH_1_0}"
fi
