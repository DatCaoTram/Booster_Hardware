"""Manual control panel for an Inspire RH56 hand: a slider per finger, gestures, live feedback.

  python -m inspire_hand_ros2.hand_gui --sim                 # simulated hand, no hardware
  python -m inspire_hand_ros2.hand_gui --serial COM5         # USB-RS485 adapter, no ROS needed
  ros2 run inspire_hand_ros2 hand_gui --ns /left_hand         # through the ROS2 driver or bridge

Only the slider you move is sent (other fingers hold), so touching one finger never
makes the rest jump.
"""
import argparse
import sys
import threading
import tkinter as tk
from tkinter import ttk

from . import rh56
from .sim import SimulatedHand

H = rh56.HOLD
LABELS = ("Little", "Ring", "Middle", "Index", "Thumb bend", "Thumb rotate")
# (delay_ms, targets) steps. Fingers close first, then the thumb rotates over them,
# like the gestures in Booster's SDK example.
PRESETS = {
    "Open": [(0, [1000] * 6)],
    "Fist": [(0, [0, 0, 0, 0, 0, H]), (300, [H, H, H, H, H, 0])],
    "Point": [(0, [0, 0, 0, 1000, 0, H]), (300, [H, H, H, H, H, 0])],
    "Scissors": [(0, [0, 0, 1000, 1000, 0, H]), (300, [H, H, H, H, H, 0])],
    "Thumbs up": [(0, [0, 0, 0, 0, 1000, 1000])],
}


class DirectBackend:
    """Talks to the hand (serial port or simulator) from the GUI thread."""

    def __init__(self, hand, label):
        self.hand, self.label = hand, label

    def send_angles(self, angles):
        self.hand.set_angles(angles)

    def send_speed(self, speed):
        self.hand.set_speeds(speed)

    def send_force(self, force):
        self.hand.set_forces(force)

    def clear_errors(self):
        self.hand.clear_errors()
        return "errors cleared"

    def read_settings(self):
        return self.hand.read_shorts(rh56.SPEED_SET)[0], self.hand.read_shorts(rh56.FORCE_SET)[0]

    def poll(self):
        fb = self.hand.read_feedback()
        return self.hand.read_angles(), fb["force"], rh56.summarize(fb["status"], fb["error"])[1]

    def close(self):
        self.hand.close()


class RosBackend:
    """Publishes to the driver's command topics and listens to its feedback."""

    def __init__(self, ns, ros_args):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Int16MultiArray
        from std_srvs.srv import Trigger

        self.rclpy, self.Msg, self.Trigger = rclpy, Int16MultiArray, Trigger
        self.ns = "/" + ns.strip("/")
        self.label = "ROS2 " + self.ns
        self.angle = self.force = None
        self.message = "waiting for diagnostics"

        rclpy.init(args=[sys.argv[0]] + ros_args)
        node = self.node = rclpy.create_node("hand_gui")
        self.pubs = {k: node.create_publisher(Int16MultiArray, "%s/cmd_%s" % (self.ns, k), 10)
                     for k in ("angle", "speed", "force")}
        node.create_subscription(Int16MultiArray, self.ns + "/angle_raw",
                                 lambda m: setattr(self, "angle", list(m.data)), 10)
        node.create_subscription(Int16MultiArray, self.ns + "/force_raw",
                                 lambda m: setattr(self, "force", list(m.data)), 10)
        node.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)
        self.clear_client = node.create_client(Trigger, self.ns + "/clear_error")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _on_diag(self, msg):
        for status in msg.status:
            if status.name.startswith(self.ns + "/"):
                self.message = status.message

    def _publish(self, key, values):
        self.pubs[key].publish(self.Msg(data=[int(v) for v in values]))

    def send_angles(self, angles):
        self._publish("angle", angles)

    def send_speed(self, speed):
        self._publish("speed", speed)

    def send_force(self, force):
        self._publish("force", force)

    def clear_errors(self):
        if not self.clear_client.service_is_ready():
            return self.ns + "/clear_error not available"
        self.clear_client.call_async(self.Trigger.Request())
        return "clear requested"

    def read_settings(self):
        return None  # the driver's speed/force parameters stay until a slider is moved

    def poll(self):
        if self.angle is None:
            raise RuntimeError("no feedback on %s/angle_raw (commands are still sent)" % self.ns)
        return self.angle, self.force or [0] * 6, self.message

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class HandPanel:
    PERIOD_MS = 50

    def __init__(self, root, backend):
        self.root, self.backend = root, backend
        self.targets = [tk.IntVar(value=1000) for _ in range(6)]
        self.sent = [None] * 6        # last target sent (or synced) per DOF
        self.dirty = set()            # DOFs whose slider moved since the last send
        self.actual = None
        self.speed, self.force = tk.IntVar(value=1000), tk.IntVar(value=500)
        try:  # start the sliders at the hand's own settings; nothing is sent until one moves
            speed, force = backend.read_settings()
            self.speed.set(speed)
            self.force.set(force)
        except Exception:  # ROS backend (None) or a silent hand: keep the defaults
            pass
        self.sent_settings = (self.speed.get(), self.force.get())
        self.status = tk.StringVar(value="connecting...")

        root.title("RH56 hand - " + backend.label)
        root.protocol("WM_DELETE_WINDOW", self.close)
        frame = ttk.Frame(root, padding=10)
        frame.grid(sticky="nsew")

        buttons = ttk.Frame(frame)
        buttons.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 8))
        for name in PRESETS:
            ttk.Button(buttons, text=name, command=lambda n=name: self.preset(n)).pack(side="left", padx=2)
        ttk.Separator(buttons, orient="vertical").pack(side="left", fill="y", padx=6)
        ttk.Button(buttons, text="Stop", command=self.stop).pack(side="left", padx=2)
        ttk.Button(buttons, text="Clear errors", command=self.clear_errors).pack(side="left", padx=2)

        for col, text in enumerate(("", "Target  (0 = closed, 1000 = open)", "Actual", "Force (g)")):
            ttk.Label(frame, text=text, foreground="#555").grid(row=1, column=col, sticky="w", padx=4)
        self.bars, self.actual_labels, self.force_labels = [], [], []
        for i, label in enumerate(LABELS):
            row = i + 2
            ttk.Label(frame, text=label, width=12).grid(row=row, column=0, sticky="w")
            tk.Scale(frame, from_=0, to=1000, orient="horizontal", length=380, variable=self.targets[i],
                     command=lambda _v, i=i: self._on_slider(i)).grid(row=row, column=1, padx=4)
            cell = ttk.Frame(frame)
            cell.grid(row=row, column=2, padx=4)
            bar = ttk.Progressbar(cell, maximum=1000, length=140)
            bar.pack(side="left")
            value = ttk.Label(cell, text="-", width=5, anchor="e")
            value.pack(side="left")
            force = ttk.Label(frame, text="-", width=6, anchor="e")
            force.grid(row=row, column=3, padx=4)
            self.bars.append(bar)
            self.actual_labels.append(value)
            self.force_labels.append(force)

        settings = ttk.Frame(frame)
        settings.grid(row=8, column=0, columnspan=4, sticky="w", pady=(10, 0))
        for text, var, top in (("Speed", self.speed, 1000), ("Force limit (g)", self.force, 1000)):
            ttk.Label(settings, text=text).pack(side="left", padx=(0, 4))
            tk.Scale(settings, from_=0, to=top, orient="horizontal", length=220, variable=var,
                     resolution=10).pack(side="left", padx=(0, 16))

        ttk.Label(frame, textvariable=self.status, wraplength=700).grid(
            row=9, column=0, columnspan=4, sticky="w", pady=(10, 0))
        root.after(0, self._loop)

    # -- user actions -------------------------------------------------------------

    def _on_slider(self, i):
        if self.targets[i].get() != self.sent[i]:
            self.dirty.add(i)

    def _apply(self, angles):
        for i, a in enumerate(angles):
            if a != H:
                self.targets[i].set(a)
                self.dirty.add(i)

    def preset(self, name):
        for delay, angles in PRESETS[name]:
            self.root.after(delay, self._apply, angles)

    def stop(self):
        if self.actual is not None:
            self._apply(self.actual)

    def clear_errors(self):
        try:
            self.status.set(self.backend.clear_errors())
        except Exception as e:
            self.status.set("clear failed: %s" % e)

    def close(self):
        try:
            self.backend.close()
        finally:
            self.root.destroy()

    # -- I/O loop -----------------------------------------------------------------

    def _loop(self):
        try:
            settings = (self.speed.get(), self.force.get())
            if settings != self.sent_settings:
                self.backend.send_speed([settings[0]] * 6)
                self.backend.send_force([settings[1]] * 6)
                self.sent_settings = settings
            if self.dirty:
                cmd = [self.targets[i].get() if i in self.dirty else H for i in range(6)]
                self.backend.send_angles(cmd)
                for i in self.dirty:
                    self.sent[i] = cmd[i]
                self.dirty.clear()
            angle, force, message = self.backend.poll()
        except Exception as e:
            self.status.set("! %s" % e)
        else:
            if self.actual is None:  # first feedback: start the sliders where the hand is
                for i, a in enumerate(angle):
                    self.sent[i] = a
                    self.targets[i].set(a)
            self.actual = angle
            for i in range(6):
                self.bars[i]["value"] = angle[i]
                self.actual_labels[i]["text"] = str(angle[i])
                self.force_labels[i]["text"] = str(force[i])
            self.status.set(message)
        self.root.after(self.PERIOD_MS, self._loop)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Manual control panel for an Inspire RH56 hand.")
    source = ap.add_mutually_exclusive_group()
    source.add_argument("--serial", metavar="PORT", help="talk to the hand directly, e.g. COM5 or /dev/ttyUSB0")
    source.add_argument("--sim", action="store_true", help="use a simulated hand")
    ap.add_argument("--id", type=int, default=1, help="hand ID (default 1)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--ns", default="/left_hand", help="ROS2 namespace of the driver (default /left_hand)")
    args, ros_args = ap.parse_known_args(argv)

    if args.sim:
        backend = DirectBackend(rh56.RH56(SimulatedHand(args.id), args.id), "simulator")
    elif args.serial:
        backend = DirectBackend(rh56.RH56.open(args.serial, args.id, args.baud),
                                "%s, id %d" % (args.serial, args.id))
    else:
        backend = RosBackend(args.ns, ros_args)
    if sys.platform == "win32":  # sharp text on scaled displays
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    root = tk.Tk()
    HandPanel(root, backend)
    root.mainloop()


if __name__ == "__main__":
    main()
