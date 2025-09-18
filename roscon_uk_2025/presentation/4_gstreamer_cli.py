#!/usr/bin/env python3
import typer
import numpy as np
import cppyy
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
Gst.init(None)

def run_gstreamer(device: str = "/dev/video0", 
    width: int = 640, 
    height: int = 480, 
    fps: int = 30, 
    viz: bool = True):
    """
    Run a gstreamer pipeline to invert the colors of a video stream.
    Args:
        device: The device to use for the video stream.
        width: The width of the video stream.
        height: The height of the video stream.
        fps: The frames per second of the video stream.
    """
    # JIT a small C++ kernel (in-place fast path is trivial to swap for blur/Sobel/IPP, etc.)
    print("Compiling C++ kernel")
    cppyy.cppdef(r"""
    #include <cstdint>
    extern "C" void invert_rgb_addr(uint64_t src_addr, uint64_t dst_addr, int nbytes) {
        const uint8_t* src = reinterpret_cast<const uint8_t*>(src_addr);
        uint8_t* dst = reinterpret_cast<uint8_t*>(dst_addr);
        for (int i = 0; i < nbytes; ++i) dst[i] = 255 - src[i];
    }
    """)
    print("Compiled C++ kernel")
    invert = cppyy.gbl.invert_rgb_addr

    W, H, FPS = width, height, fps
    CAPS = f"video/x-raw,format=RGB,width={W},height={H},framerate={FPS}/1"
    FRAME_DURATION = Gst.util_uint64_scale_int(1, Gst.SECOND, FPS)

    cap = Gst.parse_launch(
        f"v4l2src device={device} ! videoconvert ! {CAPS} "
        f"! appsink name=sink emit-signals=true max-buffers=1 drop=true"
    )
    sink = cap.get_by_name("sink")

    if viz:
        disp = Gst.parse_launch(
            f"appsrc name=src is-live=true do-timestamp=true caps={CAPS} "
            f"! videoconvert ! autovideosink sync=false"
        )
    else:
        disp = Gst.parse_launch(
            f"appsrc name=src is-live=true do-timestamp=true caps={CAPS} "
            f"! shmsink socket-path=/tmp/gst_shm wait-for-connection=false sync=false"
            )
    src = disp.get_by_name("src")
    src.set_property("format", Gst.Format.TIME)

    out = np.empty(width*height*3, dtype=np.uint8)

    def on_sample(sink):
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()
        ok, m = buf.map(Gst.MapFlags.READ)
        if not ok: return Gst.FlowReturn.OK

        try:
            size = m.size
            # Create a zero-copy NumPy view of the mapped bytes, and pass its
            # raw pointer (ctypes.data) directly to the C++ kernel. This avoids
            # Python loops and extra copies on the input side.
            src_arr = np.frombuffer(m.data, dtype=np.uint8, count=size)
            # out = np.empty(size, dtype=np.uint8)
            invert(src_arr.ctypes.data, out.ctypes.data, size)
        finally:
            buf.unmap(m)

        out_buf = Gst.Buffer.new_wrapped(out.tobytes())
        # Provide explicit PTS/DTS and duration in TIME format to keep downstream
        # elements happy and avoid segment format assertions.
        if not hasattr(on_sample, "ts"): on_sample.ts = 0
        out_buf.pts = on_sample.ts
        out_buf.dts = on_sample.ts
        out_buf.duration = FRAME_DURATION
        on_sample.ts += FRAME_DURATION
        src.emit("push-buffer", out_buf)
        return Gst.FlowReturn.OK

    sink.connect("new-sample", on_sample)
    cap.set_state(Gst.State.PLAYING)
    disp.set_state(Gst.State.PLAYING)

    loop = GLib.MainLoop()
    print("Running gstreamer pipeline")
    if not viz:
        print("No viz enabled, viz with:")
        print(f"gst-launch-1.0 -v shmsrc socket-path=/tmp/gst_shm do-timestamp=true ! capsfilter caps=\"{CAPS}\" ! videoconvert ! autovideosink sync=false")
    try: loop.run()
    except KeyboardInterrupt: pass
    finally:
        cap.set_state(Gst.State.NULL)
        disp.set_state(Gst.State.NULL)

if __name__ == "__main__":
    typer.run(run_gstreamer)