from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
import types
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import tkinter as tk
from tkinter import messagebox, ttk

import can
import canopen

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor_controller import PPConfig, ProfilePositionController, SyncProducerHelper

log = logging.getLogger(__name__)
# 设置logging级别和格式
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

@dataclass(slots=True)
class BusConfig:
    bustype: str
    channel: str
    bitrate: int


class BaseMotorView:
    def __init__(self, master: tk.Misc, node_id: int) -> None:
        self.node_id = node_id
        self.controller: Optional[ProfilePositionController] = None
        self.frame = ttk.LabelFrame(master, text=f"Node 0x{self.node_id:02X}")
        self.position_var = tk.StringVar(master, value="--")
        self.status_var = tk.StringVar(master, value="Disconnected")
        self.connect_button: ttk.Button

    def is_connected(self) -> bool:
        return self.controller is not None

    def set_controls_state(self, enabled: bool) -> None:
        raise NotImplementedError

    def on_connected(self, controller: ProfilePositionController, initial_position: float) -> None:
        self.controller = controller
        self.set_controls_state(True)
        self.update_position_display(initial_position)
        timestamp = time.strftime("%H:%M:%S")
        self.status_var.set(f"Connected {timestamp}")

    def on_disconnected(self) -> None:
        self.controller = None
        self.set_controls_state(False)
        self.update_position_display(None)
        self.status_var.set("Disconnected")

    def update_position_display(self, value: Optional[float]) -> None:
        if value is None:
            self.position_var.set("--")
        else:
            self.position_var.set(f"{value:.3f} deg")

    def on_shutdown(self) -> None:
        """Hook for extra cleanup during app shutdown."""
        pass


class PPMotorView(BaseMotorView):
    def __init__(
        self,
        master: tk.Misc,
        node_id: int,
        slider_limits: Tuple[float, float],
        slider_resolution: float,
        jog_step_default: float,
        jog_repeat_delay_ms: int,
        jog_repeat_interval_ms: int,
    ) -> None:
        super().__init__(master, node_id)
        self.slider_limits = slider_limits
        self.slider_resolution = slider_resolution
        self.last_commanded: Optional[float] = None
        self.slider_var = tk.DoubleVar(master, value=0.0)
        self.slider_readout_var = tk.StringVar(master, value="--")
        self.manual_target_var = tk.StringVar(master, value="")
        self.jog_step_var = tk.StringVar(master, value=f"{jog_step_default}")

        self._build(jog_repeat_delay_ms, jog_repeat_interval_ms)
        self.set_controls_state(False)

    def _build(self, jog_repeat_delay_ms: int, jog_repeat_interval_ms: int) -> None:
        self.frame.columnconfigure(3, weight=1)

        self.connect_button = ttk.Button(self.frame, text="Connect")
        self.connect_button.grid(row=0, column=0, sticky="ew", padx=(6, 6), pady=(6, 2))

        ttk.Label(self.frame, text="Actual position:").grid(
            row=0, column=1, sticky="w", padx=(0, 4), pady=(6, 2)
        )
        ttk.Label(self.frame, textvariable=self.position_var, width=12).grid(
            row=0, column=2, sticky="w", padx=(0, 6), pady=(6, 2)
        )
        ttk.Label(self.frame, textvariable=self.status_var, width=28).grid(
            row=0, column=3, sticky="w", padx=(0, 6), pady=(6, 2)
        )

        ttk.Label(self.frame, text="Manual target (deg):").grid(
            row=1, column=0, columnspan=2, sticky="w", padx=6, pady=2
        )
        self.manual_entry = ttk.Entry(
            self.frame,
            textvariable=self.manual_target_var,
            width=12,
        )
        self.manual_entry.grid(row=1, column=2, sticky="ew", padx=(0, 6), pady=2)
        self.manual_button = ttk.Button(self.frame, text="Move")
        self.manual_button.grid(row=1, column=3, sticky="ew", padx=6, pady=2)

        self.scale = tk.Scale(
            self.frame,
            from_=self.slider_limits[0],
            to=self.slider_limits[1],
            orient="horizontal",
            variable=self.slider_var,
            resolution=self.slider_resolution,
            showvalue=False,
            length=340,
        )
        self.scale.grid(row=2, column=0, columnspan=4, sticky="ew", padx=6, pady=(6, 2))

        ttk.Label(self.frame, textvariable=self.slider_readout_var, width=12).grid(
            row=3, column=0, columnspan=2, sticky="w", padx=6, pady=2
        )
        self.slider_button = ttk.Button(self.frame, text="Move to slider")
        self.slider_button.grid(row=3, column=3, sticky="ew", padx=6, pady=2)

        ttk.Label(self.frame, text="Jog step (deg):").grid(
            row=4, column=0, sticky="w", padx=6, pady=(6, 6)
        )
        self.jog_step_entry = ttk.Entry(
            self.frame,
            textvariable=self.jog_step_var,
            width=8,
        )
        self.jog_step_entry.grid(row=4, column=1, sticky="w", padx=(0, 6), pady=(6, 6))

        self.jog_minus = tk.Button(
            self.frame,
            text="Jog -",
            repeatdelay=jog_repeat_delay_ms,
            repeatinterval=jog_repeat_interval_ms,
        )
        self.jog_minus.grid(row=4, column=2, sticky="ew", padx=(0, 6), pady=(6, 6))

        self.jog_plus = tk.Button(
            self.frame,
            text="Jog +",
            repeatdelay=jog_repeat_delay_ms,
            repeatinterval=jog_repeat_interval_ms,
        )
        self.jog_plus.grid(row=4, column=3, sticky="ew", padx=6, pady=(6, 6))

        self._control_widgets: List[tk.Widget] = [
            self.manual_entry,
            self.manual_button,
            self.scale,
            self.slider_button,
            self.jog_step_entry,
            self.jog_minus,
            self.jog_plus,
        ]

    def set_controls_state(self, enabled: bool) -> None:
        state = tk.NORMAL if enabled else tk.DISABLED
        for widget in self._control_widgets:
            widget.configure(state=state)
        if not enabled:
            self.slider_readout_var.set("--")
            self.manual_target_var.set("")
            self.last_commanded = None

    def clamp_to_slider(self, angle: float) -> float:
        low, high = self.slider_limits
        return max(low, min(high, angle))

    def on_connected(self, controller: ProfilePositionController, initial_position: float) -> None:
        super().on_connected(controller, initial_position)
        clamped = self.clamp_to_slider(initial_position)
        self.last_commanded = initial_position
        self.slider_var.set(clamped)
        self.slider_readout_var.set(f"{initial_position:.3f} deg")
        self.manual_target_var.set(f"{initial_position:.3f}")

    def on_disconnected(self) -> None:
        super().on_disconnected()
        self.slider_var.set(0.0)


class PTMotorView(BaseMotorView):
    def __init__(self, master: tk.Misc, node_id: int) -> None:
        super().__init__(master, node_id)
        self.torque_vars = [tk.StringVar(master, value="500"), tk.StringVar(master, value="-500")]
        self.torque_buttons: List[tk.Button] = []
        self.torque_entries: List[ttk.Entry] = []
        self.active_output: Optional[int] = None
        self.last_commanded: Optional[int] = None

        self._build()
        self.set_controls_state(False)

    def _build(self) -> None:
        self.frame.columnconfigure(3, weight=1)

        self.connect_button = ttk.Button(self.frame, text="Connect")
        self.connect_button.grid(row=0, column=0, sticky="ew", padx=(6, 6), pady=(6, 2))

        ttk.Label(self.frame, text="Actual position:").grid(
            row=0, column=1, sticky="w", padx=(0, 4), pady=(6, 2)
        )
        ttk.Label(self.frame, textvariable=self.position_var, width=12).grid(
            row=0, column=2, sticky="w", padx=(0, 6), pady=(6, 2)
        )
        ttk.Label(self.frame, textvariable=self.status_var, width=28).grid(
            row=0, column=3, sticky="w", padx=(0, 6), pady=(6, 2)
        )

        for idx, label_text in enumerate(("Torque A (mA):", "Torque B (mA):")):
            ttk.Label(self.frame, text=label_text).grid(
                row=idx + 1,
                column=0,
                sticky="w",
                padx=6,
                pady=(4 if idx == 0 else 2, 2),
            )
            entry = ttk.Entry(self.frame, textvariable=self.torque_vars[idx], width=10)
            entry.grid(row=idx + 1, column=1, sticky="w", padx=(0, 6), pady=(4 if idx == 0 else 2, 2))
            self.torque_entries.append(entry)

            button = tk.Button(self.frame, text=f"Hold {chr(ord('A') + idx)}")
            button.grid(row=idx + 1, column=2, columnspan=2, sticky="ew", padx=6, pady=(4 if idx == 0 else 2, 2))
            self.torque_buttons.append(button)

    def set_controls_state(self, enabled: bool) -> None:
        state = tk.NORMAL if enabled else tk.DISABLED
        for entry in self.torque_entries:
            entry.configure(state=state)
        for button in self.torque_buttons:
            button.configure(state=state)
        if not enabled:
            self.active_output = None
            self.last_commanded = None

    def on_connected(self, controller: ProfilePositionController, initial_position: float) -> None:
        super().on_connected(controller, initial_position)
        self.status_var.set(f"{self.status_var.get()} (idle)")

    def on_disconnected(self) -> None:
        super().on_disconnected()
        self.active_output = None

    def read_torque_value(self, index: int) -> Tuple[Optional[int], Optional[str]]:
        raw = self.torque_vars[index].get().strip()
        if not raw:
            return None, "Torque value cannot be empty"
        try:
            torque = int(raw)
        except ValueError:
            return None, f"'{raw}' is not a valid integer torque"
        if torque < -1000 or torque > 1000:
            return None, "Torque must be within [-1000, 1000]"
        return torque, None

    def set_active_output(self, torque: int) -> None:
        self.active_output = torque
        self.last_commanded = torque
        timestamp = time.strftime("%H:%M:%S")
        self.status_var.set(f"Holding {torque} (started {timestamp})")

    def clear_active_output(self) -> None:
        self.active_output = None
        if self.is_connected():
            self.status_var.set("Torque idle")

    def on_shutdown(self) -> None:
        self.clear_active_output()

class PositionPoller(threading.Thread):
    def __init__(
        self,
        handles: List[BaseMotorView],
        positions: Dict[int, Optional[float]],
        lock: threading.Lock,
        interval_s: float,
    ) -> None:
        super().__init__(daemon=True)
        self._handles = handles
        self._positions = positions
        self._lock = lock
        self._interval = max(interval_s, 0.05)
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            start = time.monotonic()
            for handle in self._handles:
                controller = handle.controller
                if controller is None:
                    value = None
                else:
                    try:
                        if isinstance(handle, PTMotorView):
                            value = controller.get_position_angle_sdo()
                        else:
                            value = controller.get_position_angle()
                    except Exception as exc:
                        log.warning(
                            "Node 0x%02X: position read failed: %s",
                            handle.node_id,
                            exc,
                        )
                        value = None
                with self._lock:
                    self._positions[handle.node_id] = value
            elapsed = time.monotonic() - start
            remaining = self._interval - elapsed
            if remaining > 0:
                if self._stop_event.wait(remaining):
                    break
        log.info("Position poller stopped")

    def stop(self) -> None:
        self._stop_event.set()


class MultiMotorPPApp:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.bus_cfg = BusConfig(args.bustype, args.channel, args.bitrate)
        self.network = canopen.Network()
        self.network_connected = False
        self.motors: List[BaseMotorView] = []
        self.motor_by_id: Dict[int, BaseMotorView] = {}
        self._positions: Dict[int, Optional[float]] = {}
        self._positions_lock = threading.Lock()
        self.poller: Optional[PositionPoller] = None
        self._closed = False
        self.slider_low = min(args.slider_min, args.slider_max)
        self.slider_high = max(args.slider_min, args.slider_max)
        self._active_torque_handle: Optional[PTMotorView] = None
        self._torque_release_bound = False
        self.sync_helper: Optional[SyncProducerHelper] = None
        self._pending_sync_enable = False
        self._sync_producer_enabled = False

        self.root = tk.Tk()
        self.root.title("Multi-motor controller")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.content = ttk.Frame(self.root, padding=12)
        self.content.grid(row=0, column=0, sticky="nsew")
        self.content.columnconfigure(0, weight=1)
        self.content.rowconfigure(1, weight=1)

        self.toolbar = ttk.Frame(self.content)
        self.toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        self.connect_bus_button = ttk.Button(
            self.toolbar,
            text="Connect bus",
            command=self._connect_bus,
        )
        self.connect_bus_button.pack(side="left")
        self.bus_status_var = tk.StringVar(self.root, value="Bus disconnected")
        ttk.Label(self.toolbar, textvariable=self.bus_status_var).pack(
            side="left", padx=(8, 16)
        )
        self.connect_all_button = ttk.Button(
            self.toolbar,
            text="Connect all",
            command=self._connect_all,
        )
        self.connect_all_button.pack(side="left")

        self.motors_frame = ttk.Frame(self.content)
        self.motors_frame.grid(row=1, column=0, sticky="nsew")
        for col_index in range(2):
            self.motors_frame.columnconfigure(col_index, weight=1)

        self._build_motor_rows()
        self._update_motor_connect_buttons()
        self._update_connect_all_state()

        self.poller = PositionPoller(
            self.motors,
            self._positions,
            self._positions_lock,
            args.poll_interval,
        )
        self.poller.start()
        self.root.after(args.ui_update_ms, self._refresh_positions)

    def _build_motor_rows(self) -> None:
        slider_limits = (self.slider_low, self.slider_high)
        for index, node_id in enumerate(self.args.node_ids):
            row_index = index // 2
            column_index = index % 2
            if node_id == 0x05:
                handle = PTMotorView(self.motors_frame, node_id)
            else:
                handle = PPMotorView(
                    self.motors_frame,
                    node_id,
                    slider_limits,
                    self.args.slider_resolution,
                    self.args.jog_step,
                    self.args.jog_repeat_delay_ms,
                    self.args.jog_repeat_ms,
                )
            padx = (0, 12) if column_index == 0 else (0, 0)
            handle.frame.grid(
                row=row_index,
                column=column_index,
                sticky="nsew",
                padx=padx,
                pady=(0, 12),
            )
            if column_index == 0:
                self.motors_frame.rowconfigure(row_index, weight=1)
            self._bind_motor_callbacks(handle)
            self.motors.append(handle)
            self.motor_by_id[handle.node_id] = handle
            with self._positions_lock:
                self._positions[handle.node_id] = None

    def _bind_motor_callbacks(self, handle: BaseMotorView) -> None:
        handle.connect_button.configure(
            command=lambda h=handle: self._connect_motor(h, show_message=True)
        )
        if isinstance(handle, PPMotorView):
            handle.manual_button.configure(
                command=lambda h=handle: self._on_manual_move(h)
            )
            handle.scale.configure(
                command=lambda value, h=handle: self._on_slider_move(h, value)
            )
            handle.scale.bind(
                "<ButtonRelease-1>",
                lambda _event, h=handle: self._on_slider_release(h),
            )
            handle.slider_button.configure(
                command=lambda h=handle: self._send_target(h, h.slider_var.get())
            )
            handle.jog_minus.configure(
                command=lambda h=handle: self._jog_once(h, -1.0)
            )
            handle.jog_plus.configure(
                command=lambda h=handle: self._jog_once(h, 1.0)
            )
        elif isinstance(handle, PTMotorView):
            for idx, button in enumerate(handle.torque_buttons):
                button.bind(
                    "<ButtonPress-1>",
                    lambda _event, h=handle, index=idx: self._on_torque_press(h, index),
                )
                button.bind(
                    "<ButtonRelease-1>",
                    lambda _event, h=handle: self._on_torque_release(h),
                )
                button.bind(
                    "<Leave>",
                    lambda _event, h=handle: self._on_torque_release(h),
                )

    def _build_pp_config(self, node_id: int) -> PPConfig:
        return PPConfig(
            node_id=node_id,
            eds_path=self.args.eds_path,
            profile_velocity_deg_s=self.args.profile_velocity,
            profile_accel_deg_s2=self.args.profile_accel,
            profile_decel_deg_s2=self.args.profile_decel,
            sync_period_s=self.args.sync_period,
        )

    def _connect_bus(self) -> None:
        if self.network_connected:
            return
        self.connect_bus_button.configure(state=tk.DISABLED, text="Connecting...")
        self.bus_status_var.set("Connecting bus...")
        self.root.update_idletasks()
        try:
            self.network.connect(
                bustype=self.bus_cfg.bustype,
                channel=self.bus_cfg.channel,
                bitrate=self.bus_cfg.bitrate,
            )
        except (can.CanError, OSError) as exc:
            log.exception("CAN connection failed")
            messagebox.showerror(
                "CAN connection failed",
                f"Unable to open CAN interface ({self.bus_cfg.bustype}:{self.bus_cfg.channel}):\n{exc}",
            )
            self.connect_bus_button.configure(state=tk.NORMAL, text="Connect bus")
            self.bus_status_var.set("Bus disconnected")
            return

        self.network_connected = True
        timestamp = time.strftime("%H:%M:%S")
        self.bus_status_var.set(f"Bus connected {timestamp}")
        self.connect_bus_button.configure(text="Bus connected")
        if self.sync_helper is None:
            self.sync_helper = SyncProducerHelper(self.network)
        self._pending_sync_enable = True
        self._update_motor_connect_buttons()
        self._update_connect_all_state()
        self._try_enable_sync_producer()

    def _connect_motor(self, handle: BaseMotorView, *, show_message: bool) -> bool:
        if handle.is_connected():
            return True
        if not self.network_connected:
            if show_message:
                messagebox.showwarning(
                    "Bus not connected",
                    "Please connect the CAN bus before connecting nodes.",
                )
            handle.status_var.set("Bus disconnected")
            self._update_motor_connect_buttons()
            self._update_connect_all_state()
            return False
        handle.status_var.set("Connecting...")
        handle.connect_button.configure(state=tk.DISABLED, text="Connecting...")
        self.root.update_idletasks()

        cfg = self._build_pp_config(handle.node_id)
        controller = ProfilePositionController(self.network, cfg)
        # if isinstance(handle, PTMotorView):
        #     controller._configure_pdus = types.MethodType(lambda _self: None, controller)  # type: ignore[attr-defined]

        try:
            controller.initialise()
            controller.clear_faults()
            controller.enable_operation()
            if isinstance(handle, PTMotorView):
                controller.switch_to_profile_torque_mode(initial_torque=0, pdo_mapping=True)
        except Exception as exc:
            try:
                controller.shutdown()
            except Exception:
                log.exception(
                    "Node 0x%02X: cleanup after failed connect",
                    handle.node_id,
                )
            log.exception("Node 0x%02X: connect failed", handle.node_id)
            if show_message:
                messagebox.showerror(
                    "Connection failed",
                    f"Node 0x{handle.node_id:02X}: {exc}",
                )
            handle.on_disconnected()
            handle.connect_button.configure(state=tk.NORMAL, text="Connect")
            handle.status_var.set("Connect failed")
            with self._positions_lock:
                self._positions[handle.node_id] = None
            self._update_connect_all_state()
            return False

        if isinstance(handle, PTMotorView):
            try:
                current = controller.get_position_angle_sdo()
            except Exception as exc:
                log.warning(
                    "Node 0x%02X: failed to read initial position via SDO: %s",
                    handle.node_id,
                    exc,
                )
                current = 0.0
        else:
            try:
                current = controller.get_position_angle()
                log.debug(
                    "Node 0x%02X: initial position read: %.3f deg",
                    handle.node_id,
                    current,
                )
            except Exception as exc:
                log.warning(
                    "Node 0x%02X: failed to read initial position: %s",
                    handle.node_id,
                    exc,
                )
                current = 0.0

        handle.on_connected(controller, current)
        handle.connect_button.configure(text="Connected", state=tk.DISABLED)
        with self._positions_lock:
            self._positions[handle.node_id] = current
        self._update_connect_all_state()
        if handle.node_id == 0x04:
            self._try_enable_sync_producer()
        return True

    def _connect_all(self) -> None:
        if not self.network_connected:
            messagebox.showwarning(
                "Bus not connected",
                "Please connect the CAN bus before connecting nodes.",
            )
            return
        if all(handle.is_connected() for handle in self.motors):
            self.connect_all_button.configure(state=tk.DISABLED)
            return
        self.connect_all_button.configure(state=tk.DISABLED)
        self.root.update_idletasks()
        try:
            for handle in self.motors:
                if not handle.is_connected():
                    self._connect_motor(handle, show_message=True)
        finally:
            self._update_connect_all_state()

    def _update_connect_all_state(self) -> None:
        if not self.network_connected:
            self.connect_all_button.configure(state=tk.DISABLED)
            return
        if all(handle.is_connected() for handle in self.motors):
            self.connect_all_button.configure(state=tk.DISABLED)
        else:
            self.connect_all_button.configure(state=tk.NORMAL)

    def _update_motor_connect_buttons(self) -> None:
        desired_state = tk.NORMAL if self.network_connected else tk.DISABLED
        for handle in self.motors:
            if handle.is_connected():
                handle.connect_button.configure(state=tk.DISABLED, text="Connected")
            else:
                handle.connect_button.configure(state=desired_state, text="Connect")

    def _try_enable_sync_producer(self) -> None:
        if not self._pending_sync_enable:
            return
        if not self.network_connected or self.sync_helper is None:
            return
        pt_handle = self.motor_by_id.get(0x05)
        if pt_handle is None:
            self._pending_sync_enable = False
            return
        if not pt_handle.is_connected():
            return
        try:
            self.sync_helper.enable_sync_producer(0x05, period_ms=int(1000 / 50))
        except Exception:
            log.exception("Node 0x05: failed to enable SYNC producer")
            pt_handle.status_var.set("SYNC enable failed")
            return
        self._pending_sync_enable = False
        self._sync_producer_enabled = True
        try:
            pt_handle.controller.enable_operation()  # type: ignore[union-attr]
        except Exception:
            log.exception("Node 0x05: re-enable operation after SYNC setup failed")
        pt_handle.status_var.set("SYNC producer @50Hz")

    def _on_manual_move(self, handle: PPMotorView) -> None:
        raw = handle.manual_target_var.get().strip()
        if not raw:
            return
        try:
            target = float(raw)
        except ValueError:
            messagebox.showerror(
                "Invalid input",
                f"Node 0x{handle.node_id:02X}: '{raw}' is not a valid number",
            )
            return
        self._send_target(handle, target)

    def _on_slider_move(self, handle: PPMotorView, value: str) -> None:
        try:
            angle = float(value)
        except (TypeError, ValueError):
            return
        handle.slider_readout_var.set(f"{angle:.3f} deg")

    def _on_slider_release(self, handle: PPMotorView) -> None:
        self._send_target(handle, handle.slider_var.get())

    def _jog_once(self, handle: PPMotorView, direction: float) -> None:
        if not handle.is_connected():
            messagebox.showwarning(
                "Not connected",
                f"Node 0x{handle.node_id:02X} is not connected",
            )
            return
        try:
            step = float(handle.jog_step_var.get())
        except ValueError:
            messagebox.showerror(
                "Invalid jog step",
                f"Node 0x{handle.node_id:02X}: jog step must be numeric",
            )
            return
        if step <= 0:
            messagebox.showerror(
                "Invalid jog step",
                f"Node 0x{handle.node_id:02X}: jog step must be positive",
            )
            return

        base = handle.last_commanded
        if base is None:
            base = self._latest_position(handle)
        if base is None:
            base = 0.0
        target = base + direction * step
        self._send_target(handle, target)

    def _latest_position(self, handle: BaseMotorView) -> Optional[float]:
        with self._positions_lock:
            return self._positions.get(handle.node_id)

    def _send_target(self, handle: PPMotorView, angle: float) -> None:
        if not handle.is_connected():
            messagebox.showwarning(
                "Not connected",
                f"Node 0x{handle.node_id:02X} is not connected",
            )
            return
        try:
            handle.controller.set_target_angle(angle)  # type: ignore[union-attr]
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to set target angle %s",
                handle.node_id,
                angle,
            )
            messagebox.showerror(
                "Command failed",
                f"Node 0x{handle.node_id:02X}: failed to set target angle:\n{exc}",
            )
            return

        clamped = handle.clamp_to_slider(angle)
        handle.last_commanded = angle
        handle.slider_var.set(clamped)
        handle.slider_readout_var.set(f"{angle:.3f} deg")
        handle.manual_target_var.set(f"{angle:.3f}")
        timestamp = time.strftime("%H:%M:%S")
        handle.status_var.set(f"{timestamp} -> {angle:.3f} deg")

    def _on_torque_press(self, handle: PTMotorView, index: int) -> None:
        if not handle.is_connected():
            messagebox.showwarning(
                "Not connected",
                f"Node 0x{handle.node_id:02X} is not connected",
            )
            return
        torque, error = handle.read_torque_value(index)
        if error is not None or torque is None:
            messagebox.showerror(
                "Invalid torque",
                f"Node 0x{handle.node_id:02X}: {error}",
            )
            return

        self._stop_active_torque_output()

        try:
            handle.controller.set_target_torque_sdo(torque)  # type: ignore[union-attr]
        except Exception as exc:
            log.exception(
                "Node 0x%02X: failed to set torque %s",
                handle.node_id,
                torque,
            )
            messagebox.showerror(
                "Command failed",
                f"Node 0x{handle.node_id:02X}: failed to set torque:\n{exc}",
            )
            return

        handle.set_active_output(torque)
        self._active_torque_handle = handle
        if not self._torque_release_bound:
            self.root.bind_all("<ButtonRelease-1>", self._on_global_button_release)
            self._torque_release_bound = True

    def _on_torque_release(self, handle: PTMotorView) -> None:
        if self._active_torque_handle is handle:
            self._stop_active_torque_output()

    def _on_global_button_release(self, _event: tk.Event) -> None:
        self._stop_active_torque_output()

    def _stop_active_torque_output(self) -> None:
        handle = self._active_torque_handle
        if handle is None:
            return
        controller = handle.controller
        if controller is not None:
            try:
                controller.set_target_torque_sdo(0)
            except Exception:
                log.exception("Node 0x%02X: failed to reset torque", handle.node_id)
        handle.clear_active_output()
        self._active_torque_handle = None
        if self._torque_release_bound:
            self.root.unbind_all("<ButtonRelease-1>")
            self._torque_release_bound = False

    def _refresh_positions(self) -> None:
        with self._positions_lock:
            snapshot = dict(self._positions)
        for handle in self.motors:
            value = snapshot.get(handle.node_id)
            if not handle.is_connected():
                handle.update_position_display(None)
                continue
            handle.update_position_display(value)
        if not self._closed:
            self.root.after(self.args.ui_update_ms, self._refresh_positions)

    def on_close(self) -> None:
        self._shutdown()
        self.root.destroy()

    def _shutdown(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_active_torque_output()
        if self.poller is not None:
            self.poller.stop()
            self.poller.join(timeout=2.0)
        for handle in self.motors:
            handle.on_shutdown()
            if not handle.is_connected():
                continue
            try:
                if isinstance(handle, PTMotorView):
                    handle.controller.set_target_torque_sdo(0)  # type: ignore[union-attr]
                handle.controller.shutdown()  # type: ignore[union-attr]
            except Exception:
                log.exception("Node 0x%02X: shutdown failed", handle.node_id)
        if self._sync_producer_enabled and self.sync_helper is not None:
            try:
                self.sync_helper.disable_sync_producer(0x05)
            except Exception:
                log.exception("Node 0x05: failed to disable SYNC producer")
        if self.network_connected:
            try:
                self.network.disconnect()
            except Exception:
                log.exception("CAN network disconnect failed")

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            self._shutdown()
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Tkinter GUI for controlling multiple motors with PP and PT modes.",
    )
    parser.add_argument(
        "--node-ids",
        nargs="+",
        type=lambda value: int(value, 0),
        default=[0x01, 0x02, 0x03, 0x04, 0x05],
        help="List of node IDs to control (hex values allowed).",
    )
    parser.add_argument(
        "--eds-path",
        default="ZeroErr Driver_V1.5.eds",
        help="Path to the EDS file.",
    )
    parser.add_argument("--bustype", default="pcan", help="CAN interface type.")
    parser.add_argument("--channel", default="PCAN_USBBUS1", help="CAN interface channel.")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate.")
    parser.add_argument(
        "--profile-velocity",
        type=float,
        default=30.0,
        help="Default profile velocity in deg/s.",
    )
    parser.add_argument(
        "--profile-accel",
        type=float,
        default=30.0,
        help="Default profile acceleration in deg/s^2.",
    )
    parser.add_argument(
        "--profile-decel",
        type=float,
        default=30.0,
        help="Default profile deceleration in deg/s^2.",
    )
    parser.add_argument(
        "--sync-period",
        type=float,
        default=None,
        help="SYNC period in seconds if the nodes should produce SYNC.",
    )
    parser.add_argument(
        "--slider-min",
        type=float,
        default=-360.0,
        help="Minimum angle for the slider in degrees.",
    )
    parser.add_argument(
        "--slider-max",
        type=float,
        default=360.0,
        help="Maximum angle for the slider in degrees.",
    )
    parser.add_argument(
        "--slider-resolution",
        type=float,
        default=0.1,
        help="Slider step resolution in degrees.",
    )
    parser.add_argument(
        "--jog-step",
        type=float,
        default=5.0,
        help="Default jog step in degrees.",
    )
    parser.add_argument(
        "--jog-repeat-delay-ms",
        type=int,
        default=400,
        help="Delay before jog buttons start repeating (ms).",
    )
    parser.add_argument(
        "--jog-repeat-ms",
        type=int,
        default=150,
        help="Repeat interval while jog buttons are held (ms).",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.1,
        help="Background polling interval in seconds.",
    )
    parser.add_argument(
        "--ui-update-ms",
        type=int,
        default=150,
        help="UI refresh interval in milliseconds.",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        help="Logging level (DEBUG, INFO, ...).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.slider_min >= args.slider_max:
        raise SystemExit("slider-min must be smaller than slider-max")
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(message)s",
    )
    app = MultiMotorPPApp(args)
    try:
        app.run()
    except KeyboardInterrupt:
        log.info("Interrupted by user")


if __name__ == "__main__":
    main()
