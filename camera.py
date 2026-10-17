"""GStreamer camera producer. Use /usr/bin/python3 for Ubuntu's GI bindings."""
import argparse
import struct
import sys
import time


def main():
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", help="Stable /dev/v4l/by-id path; omitted for a test signal")
    parser.add_argument("--frames", type=int, default=0, help="0 streams until stopped")
    args = parser.parse_args()
    source = "videotestsrc is-live=true"
    if args.device:
        # Encode the property with GStreamer's own quoting rules via JSON string syntax.
        import json
        source = "v4l2src device=" + json.dumps(args.device)
    pipeline = Gst.parse_launch(source + " ! videoconvert ! videoscale ! videorate ! "
        "video/x-raw,width=640,height=480,framerate=30/1 ! "
        "queue max-size-buffers=1 leaky=downstream ! jpegenc quality=75 ! "
        "appsink name=frames sync=false max-buffers=1 drop=true")
    sink = pipeline.get_by_name("frames")
    bus = pipeline.get_bus()
    pipeline.set_state(Gst.State.PLAYING)
    count = 0
    try:
        while args.frames == 0 or count < args.frames:
            sample = sink.emit("try-pull-sample", Gst.SECOND)
            if sample is None:
                error = bus.pop_filtered(Gst.MessageType.ERROR)
                if error:
                    raise RuntimeError(str(error.parse_error()))
                raise RuntimeError("GStreamer camera produced no frames for 1 second")
            buffer = sample.get_buffer()
            ok, mapped = buffer.map(Gst.MapFlags.READ)
            if not ok:
                raise RuntimeError("Cannot map camera frame")
            try:
                data = bytes(mapped.data)
                # Derive capture age from the GStreamer running clock and buffer PTS.
                running = pipeline.get_clock().get_time() - pipeline.get_base_time()
                if buffer.pts == Gst.CLOCK_TIME_NONE:
                    raise RuntimeError("Camera frame has no capture timestamp")
                age = max(0, running - buffer.pts)
                if age > 250_000_000:
                    continue
                sys.stdout.buffer.write(struct.pack("!IQ", len(data), time.time_ns() - age))
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
                count += 1
            finally:
                buffer.unmap(mapped)
    finally:
        pipeline.set_state(Gst.State.NULL)
    print(f"GStreamer produced {count} JPEG frames", file=sys.stderr)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
