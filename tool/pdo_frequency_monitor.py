"""实时监控 CANopen PDO 周期频率的小工具。

启动后，它会按照 1 秒的窗口统计每个节点的 PDO 收发频率，
输出格式示例：

    2025-11-02 16:00:00  master -> node 0x04: 100.0 Hz
    2025-11-02 16:00:00  node 0x04 -> master: 50.0 Hz

按 Ctrl+C 退出。
"""

from __future__ import annotations

import argparse
import collections
import datetime as _dt
import logging
import queue
import sys
import threading
import time
from dataclasses import dataclass
from typing import DefaultDict, Iterable, Optional, Sequence, Tuple

import can
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

log = logging.getLogger(__name__)

# CiA-301 标准 PDO COB-ID 范围
_TPDO_BASES = (0x180, 0x280, 0x380, 0x480)
_RPDO_BASES = (0x200, 0x300, 0x400, 0x500)
_MAX_NODE_ID = 0x7F


@dataclass
class MonitorConfig:
    bustype: str
    channel: str
    bitrate: int
    interval: float
    nodes: Sequence[int]


def _classify_pdo(cob_id: int) -> Optional[Tuple[int, str]]:
    """根据 COB-ID 判断 PDO 方向与节点号。"""
    for base in _TPDO_BASES:
        node_id = cob_id - base
        if 1 <= node_id <= _MAX_NODE_ID:
            return node_id, "node->master"
    for base in _RPDO_BASES:
        node_id = cob_id - base
        if 1 <= node_id <= _MAX_NODE_ID:
            return node_id, "master->node"
    return None


def _parse_node_list(text: str) -> Sequence[int]:
    if not text.strip():
        return []

    nodes = []
    for raw in text.replace(";", ",").split(","):
        part = raw.strip()
        if not part:
            continue
        try:
            value = int(part, 0)
        except ValueError as exc:
            raise ValueError(f"无法解析节点编号: {part}") from exc
        if not (1 <= value <= _MAX_NODE_ID):
            raise ValueError(f"节点编号超出范围 1..0x7F: {value}")
        nodes.append(value)
    return nodes


def _parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor PDO update frequencies on a CANopen bus")
    parser.add_argument("--bustype", default="pcan", help="Underlying python-can bus type (default: pcan)")
    parser.add_argument("--channel", default="PCAN_USBBUS1", help="CAN interface channel (default: PCAN_USBBUS1)")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bus bitrate in bit/s (default: 1_000_000)")
    parser.add_argument("--interval", type=float, default=1.0, help="Reporting interval in seconds (default: 1.0)")
    parser.add_argument("--nodes", default="", help="Comma-separated node IDs to monitor (default: all)")
    parser.add_argument("--log-level", default="INFO", help="Logging level (default: INFO)")
    return parser.parse_args(list(argv))


def _format_timestamp(ts: float) -> str:
    return _dt.datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")


class PDOFrequencyMonitor(threading.Thread):
    """后台线程，在固定时间窗口内统计 PDO 频率。"""

    def __init__(self, cfg: MonitorConfig, output_queue: "queue.Queue[str]", stop_event: threading.Event):
        super().__init__(name="pdo-monitor", daemon=True)
        self.cfg = cfg
        self.output_queue = output_queue
        self.stop_event = stop_event
        self.counts: DefaultDict[Tuple[int, str], int] = collections.defaultdict(int)
        self._last_report = time.monotonic()
        self._bus: Optional[can.BusABC] = None

    def run(self) -> None:
        try:
            self._bus = can.interface.Bus(
                bustype=self.cfg.bustype,
                channel=self.cfg.channel,
                bitrate=self.cfg.bitrate,
            )
        except can.CanError as exc:
            self.output_queue.put(f"打开 CAN 接口失败: {exc}")
            return

        monitored = {n for n in self.cfg.nodes if 1 <= n <= _MAX_NODE_ID}
        if monitored:
            self.output_queue.put(
                f"开始监控节点: {', '.join(f'0x{n:02X}' for n in sorted(monitored))}"
            )
        else:
            self.output_queue.put("监控全部节点的 PDO 流量")

        while not self.stop_event.is_set():
            try:
                msg = self._bus.recv(timeout=0.1)
            except can.CanError as exc:
                self.output_queue.put(f"读取 CAN 帧失败: {exc}")
                continue

            now = time.monotonic()

            if msg is not None:
                classified = _classify_pdo(msg.arbitration_id)
                if classified is not None:
                    node_id, direction = classified
                    if not monitored or node_id in monitored:
                        self.counts[(node_id, direction)] += 1

            if now - self._last_report >= self.cfg.interval:
                wall_ts = _format_timestamp(time.time())
                if self.counts:
                    for (node_id, direction), counter in sorted(self.counts.items()):
                        freq = counter / max(self.cfg.interval, 1e-6)
                        if direction == "master->node":
                            label = f"master -> node 0x{node_id:02X}"
                        else:
                            label = f"node 0x{node_id:02X} -> master"
                        self.output_queue.put(f"{wall_ts}  {label}: {freq:.1f} Hz")
                else:
                    self.output_queue.put(f"{wall_ts}  (无匹配的 PDO 流量)")
                self.counts.clear()
                self._last_report = now

        if self._bus is not None:
            self._bus.shutdown()

class MonitorApp(tk.Tk):
    """Tkinter GUI 外壳，可启动/停止 PDO 频率监控线程。"""

    def __init__(self, defaults: argparse.Namespace):
        super().__init__()
        self.title("CANopen PDO Frequency Monitor")
        self.resizable(False, False)

        self.defaults = defaults
        self.monitor_thread: Optional[PDOFrequencyMonitor] = None
        self.stop_event = threading.Event()
        self.output_queue: "queue.Queue[str]" = queue.Queue()

        self._build_widgets()
        self._apply_defaults()
        self._update_buttons()
        self.after(200, self._poll_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_widgets(self) -> None:
        padding = {"padx": 8, "pady": 4}

        form = ttk.Frame(self)
        form.grid(row=0, column=0, sticky="nsew")

        ttk.Label(form, text="Bustype:").grid(row=0, column=0, sticky="e", **padding)
        self.entry_bustype = ttk.Entry(form, width=16)
        self.entry_bustype.grid(row=0, column=1, sticky="w", **padding)

        ttk.Label(form, text="Channel:").grid(row=1, column=0, sticky="e", **padding)
        self.entry_channel = ttk.Entry(form, width=16)
        self.entry_channel.grid(row=1, column=1, sticky="w", **padding)

        ttk.Label(form, text="Bitrate:").grid(row=2, column=0, sticky="e", **padding)
        self.entry_bitrate = ttk.Entry(form, width=16)
        self.entry_bitrate.grid(row=2, column=1, sticky="w", **padding)

        ttk.Label(form, text="Interval (s):").grid(row=3, column=0, sticky="e", **padding)
        self.entry_interval = ttk.Entry(form, width=16)
        self.entry_interval.grid(row=3, column=1, sticky="w", **padding)

        ttk.Label(form, text="Nodes (comma-separated):").grid(row=4, column=0, sticky="e", **padding)
        self.entry_nodes = ttk.Entry(form, width=24)
        self.entry_nodes.grid(row=4, column=1, sticky="w", **padding)

        button_frame = ttk.Frame(self)
        button_frame.grid(row=1, column=0, sticky="ew")
        self.start_button = ttk.Button(button_frame, text="Start", command=self.start_monitor)
        self.start_button.grid(row=0, column=0, padx=8, pady=6)
        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_monitor)
        self.stop_button.grid(row=0, column=1, padx=8, pady=6)

        output_frame = ttk.Frame(self)
        output_frame.grid(row=2, column=0, sticky="nsew")
        ttk.Label(output_frame, text="Output:").grid(row=0, column=0, sticky="w", padx=8)
        self.output_text = scrolledtext.ScrolledText(output_frame, width=60, height=18, state="disabled")
        self.output_text.grid(row=1, column=0, padx=8, pady=(0, 8))

    def _apply_defaults(self) -> None:
        self.entry_bustype.insert(0, self.defaults.bustype)
        self.entry_channel.insert(0, self.defaults.channel)
        self.entry_bitrate.insert(0, str(self.defaults.bitrate))
        self.entry_interval.insert(0, str(self.defaults.interval))
        self.entry_nodes.insert(0, self.defaults.nodes)

    def _update_buttons(self) -> None:
        running = self.monitor_thread is not None and self.monitor_thread.is_alive()
        self.start_button.config(state="disabled" if running else "normal")
        self.stop_button.config(state="normal" if running else "disabled")

    def _append_output(self, line: str) -> None:
        self.output_text.configure(state="normal")
        self.output_text.insert("end", line + "\n")
        self.output_text.see("end")
        self.output_text.configure(state="disabled")

    def _poll_queue(self) -> None:
        try:
            while True:
                line = self.output_queue.get_nowait()
                self._append_output(line)
        except queue.Empty:
            pass

        if self.monitor_thread and not self.monitor_thread.is_alive():
            self.monitor_thread = None
            self.stop_event = threading.Event()
            self._update_buttons()

        self.after(200, self._poll_queue)

    def start_monitor(self) -> None:
        if self.monitor_thread and self.monitor_thread.is_alive():
            return

        try:
            bitrate = int(self.entry_bitrate.get().strip() or "0")
            interval = float(self.entry_interval.get().strip() or "0")
            nodes = _parse_node_list(self.entry_nodes.get())
        except ValueError as exc:
            messagebox.showerror("Invalid Input", str(exc))
            return

        if bitrate <= 0:
            messagebox.showerror("Invalid Input", "Bitrate 必须为正整数")
            return
        if interval <= 0:
            messagebox.showerror("Invalid Input", "Interval 必须大于 0")
            return

        cfg = MonitorConfig(
            bustype=self.entry_bustype.get().strip() or "pcan",
            channel=self.entry_channel.get().strip() or "PCAN_USBBUS1",
            bitrate=bitrate,
            interval=interval,
            nodes=nodes,
        )

        self.stop_event = threading.Event()
        self.monitor_thread = PDOFrequencyMonitor(cfg, self.output_queue, self.stop_event)
        self.monitor_thread.start()
        self._append_output("监控线程已启动...")
        self._update_buttons()

    def stop_monitor(self) -> None:
        if self.monitor_thread and self.monitor_thread.is_alive():
            self._append_output("正在停止监控线程...")
            self.stop_event.set()
            self.monitor_thread.join(timeout=2.0)
            if self.monitor_thread.is_alive():
                self._append_output("警告: 线程仍在停止，请稍候...")
            else:
                self._append_output("监控线程已停止")
                self.monitor_thread = None
                self.stop_event = threading.Event()
            self._update_buttons()

    def _on_close(self) -> None:
        self.stop_monitor()
        self.destroy()


def main(argv: Optional[Iterable[str]] = None) -> None:
    args = _parse_args(argv or sys.argv[1:])
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO), format="%(asctime)s %(levelname)s %(message)s")
    app = MonitorApp(args)
    app.mainloop()


if __name__ == "__main__":
    main()
