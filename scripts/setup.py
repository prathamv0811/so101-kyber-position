"""Reproduce the pinned SDK build on Ubuntu or macOS."""
import json
import os
import platform
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run(args, **kwargs):
    subprocess.run(args, cwd=ROOT, check=True, **kwargs)


def linux_build_environment():
    # Prepare a private GStreamer H.264 parser without requiring sudo.
    # Kyber native action transport needs no Linux desktop/input libraries.
    debs, prefix = ROOT / ".cache/debs", ROOT / ".cache/sysroot"
    debs.mkdir(parents=True, exist_ok=True)
    prefix.mkdir(parents=True, exist_ok=True)
    subprocess.run(["apt-get", "download", "gstreamer1.0-plugins-bad", "libgstreamer-plugins-bad1.0-0"], cwd=debs, check=True)
    for deb in debs.glob("*.deb"):
        run(["dpkg-deb", "-x", str(deb), str(prefix)])
    for pc in prefix.rglob("*.pc"):
        pc.write_text(pc.read_text().replace("prefix=/usr", "prefix=" + str(prefix / "usr")))
    triplet = subprocess.check_output(["dpkg-architecture", "-qDEB_HOST_MULTIARCH"], text=True).strip()
    lib = prefix / "usr/lib" / triplet
    plugins = ROOT / ".cache/gstreamer"
    plugins.mkdir(exist_ok=True)
    parser = lib / "gstreamer-1.0/libgstvideoparsersbad.so"
    link = plugins / parser.name
    if not link.exists():
        link.symlink_to(parser)
    for link in lib.glob("*.so"):
        if link.is_symlink() and not link.exists():
            runtime = Path("/lib") / triplet / link.readlink().name
            if runtime.exists():
                link.unlink()
                link.symlink_to(runtime)
    env = dict(os.environ)
    env["PKG_CONFIG_PATH"] = str(lib / "pkgconfig") + ":" + env.get("PKG_CONFIG_PATH", "")
    env["LIBRARY_PATH"] = str(lib) + ":" + env.get("LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = str(lib) + ":" + env.get("LD_LIBRARY_PATH", "")
    return env


def main():
    host = platform.system()
    if host not in ("Linux", "Darwin"):
        raise RuntimeError("Setup supports Ubuntu Linux and macOS")
    lock = json.loads((ROOT / "SDK.lock.json").read_text())
    sdk = ROOT / "vendor/kysdk"
    if not sdk.exists():
        run(["git", "clone", "--no-checkout", "--depth", "1", lock["sdk"]["url"], str(sdk)])
        run(["git", "-C", str(sdk), "fetch", "--depth", "1", "origin", lock["sdk"]["rev"]])
        run(["git", "-C", str(sdk), "checkout", "--detach", lock["sdk"]["rev"]])
    actual = subprocess.check_output(["git", "-C", str(sdk), "rev-parse", "HEAD"], text=True).strip()
    if actual != lock["sdk"]["rev"]:
        raise RuntimeError("Existing SDK checkout differs from SDK.lock.json; preserve/review it manually")
    for name, expected in lock["components"].items():
        component = sdk / name
        if (component / ".git").exists():
            actual = subprocess.check_output(["git", "-C", str(component), "rev-parse", "HEAD"], text=True).strip()
            if actual != expected or subprocess.check_output(["git", "-C", str(component), "status", "--porcelain"], text=True):
                raise RuntimeError(f"Preserve/review existing changes in SDK component {name}")
    run(["git", "-C", str(sdk), "submodule", "update", "--init", "--depth", "1", *lock["components"]])
    for name, expected in lock["components"].items():
        actual = subprocess.check_output(["git", "-C", str(sdk / name), "rev-parse", "HEAD"], text=True).strip()
        if actual != expected:
            raise RuntimeError(f"SDK component {name} doesn't match the lock")
    env = linux_build_environment() if host == "Linux" else dict(os.environ)
    run(["uv", "sync", "--locked", "--python", "3.12"])
    run(["cargo", "build", "--locked"], env=env)
    print(f"Built {host}/{platform.machine()} position controller against Kyber {lock['release']}")


if __name__ == "__main__":
    main()
