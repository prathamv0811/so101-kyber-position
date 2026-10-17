"""Count actual decoded frames for the hardware-free KyProto H.264 test."""
import sys
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

Gst.init(None)
pipeline = Gst.parse_launch("filesrc name=source ! h264parse ! avdec_h264 max-threads=1 ! fakesink name=sink signal-handoffs=true")
pipeline.get_by_name("source").set_property("location", sys.argv[1])
count = [0]
pipeline.get_by_name("sink").connect("handoff", lambda *_: count.__setitem__(0, count[0] + 1))
pipeline.set_state(Gst.State.PLAYING)
try:
    message = pipeline.get_bus().timed_pop_filtered(4 * Gst.SECOND, Gst.MessageType.ERROR | Gst.MessageType.EOS)
    if message is None or message.type == Gst.MessageType.ERROR:
        raise RuntimeError("Video decode failed" if message is None else str(message.parse_error()))
    print(count[0])
finally:
    pipeline.set_state(Gst.State.NULL)
