"""Standalone Tkinter GUI for reading/writing controller parameters via CANopen SDO."""

from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText
from typing import Any, Dict, Optional

import can  
import canopen  
import time


@dataclass(frozen=True, slots=True)
class ParameterField:
	key: str
	label: str
	index: int
	subindex: int
	min_value: int
	max_value: int
	is_bool: bool = False


PARAMETER_FIELDS: tuple[ParameterField, ...] = (
	ParameterField("position_kp", "Position P (0x2382:01)", 0x2382, 0x01, 0x0000, 0xFFFF),
	ParameterField("velocity_kp", "Velocity P (0x2381:01)", 0x2381, 0x01, 0x0000, 0xFFFF),
	ParameterField("velocity_ki", "Velocity I (0x2381:02)", 0x2381, 0x02, 0x0000, 0xFFFF),
	ParameterField("current_kp", "Current P (0x2380:01)", 0x2380, 0x01, 0x0000, 0xFFFF),
	ParameterField("current_ki", "Current I (0x2380:02)", 0x2380, 0x02, 0x0000, 0xFFFF),
	ParameterField("velocity_feedforward", "Velocity Feedforward (0x2382:03)", 0x2382, 0x03, 0x0000, 0xFFFF),
	ParameterField("integral_limit_ma", "Integral Limit mA (0x3000:00)", 0x3000, 0x00, 0x0000, 0x00001F40),
	ParameterField("pid_switch", "PID Switch (0x2383:00)", 0x2383, 0x00, 0x0000, 0x0001, is_bool=True),
)


class ControllerParameterGUI:
	def __init__(self, root: tk.Tk) -> None:
		self.root = root
		self.root.title("Controller Parameter Utility")

		self.network: Optional[canopen.Network] = None
		self.node: Optional[canopen.RemoteNode] = None

		self._build_widgets()
		self._set_defaults()
		self.root.protocol("WM_DELETE_WINDOW", self._on_close)

	def _build_widgets(self) -> None:
		main_frame = tk.Frame(self.root, padx=10, pady=10)
		main_frame.pack(fill=tk.BOTH, expand=True)

		conn_frame = tk.LabelFrame(main_frame, text="Connection", padx=8, pady=8)
		conn_frame.pack(fill=tk.X, expand=False, pady=(0, 10))

		self.bus_entries: Dict[str, tk.Entry] = {}
		self.node_id_var = tk.StringVar()
		self.eds_path_var = tk.StringVar()

		def _add_entry(row: int, label: str, key: str, width: int = 18) -> None:
			tk.Label(conn_frame, text=label).grid(row=row, column=0, sticky=tk.W, pady=2)
			entry = tk.Entry(conn_frame, width=width)
			entry.grid(row=row, column=1, sticky=tk.W, pady=2)
			self.bus_entries[key] = entry

		_add_entry(0, "bustype", "bustype")
		_add_entry(1, "channel", "channel")
		_add_entry(2, "bitrate", "bitrate")

		tk.Label(conn_frame, text="node id").grid(row=0, column=2, sticky=tk.W, padx=(20, 0))
		node_entry = tk.Entry(conn_frame, textvariable=self.node_id_var, width=10)
		node_entry.grid(row=0, column=3, sticky=tk.W)

		tk.Label(conn_frame, text="EDS path").grid(row=1, column=2, sticky=tk.W, padx=(20, 0))
		eds_entry = tk.Entry(conn_frame, textvariable=self.eds_path_var, width=28)
		eds_entry.grid(row=1, column=3, sticky=tk.W)

		btn_frame = tk.Frame(conn_frame)
		btn_frame.grid(row=0, column=4, rowspan=3, padx=(20, 0))

		tk.Button(btn_frame, text="Connect", command=self.connect).pack(fill=tk.X)
		tk.Button(btn_frame, text="Disconnect", command=self.disconnect).pack(fill=tk.X, pady=(5, 0))

		param_frame = tk.LabelFrame(main_frame, text="Controller Parameters", padx=8, pady=8)
		param_frame.pack(fill=tk.X, expand=False)

		self.param_widgets: Dict[str, Any] = {}
		for row, field in enumerate(PARAMETER_FIELDS):
			tk.Label(param_frame, text=field.label, anchor="w").grid(row=row, column=0, sticky=tk.W, pady=2)
			if field.is_bool:
				var = tk.IntVar()
				chk = tk.Checkbutton(param_frame, variable=var)
				chk.grid(row=row, column=1, sticky=tk.W, pady=2)
				self.param_widgets[field.key] = var
			else:
				entry = tk.Entry(param_frame, width=16)
				entry.grid(row=row, column=1, sticky=tk.W, pady=2)
				self.param_widgets[field.key] = entry

		action_frame = tk.Frame(main_frame, pady=10)
		action_frame.pack(fill=tk.X)

		tk.Button(action_frame, text="Read From Drive", command=self.read_parameters).pack(side=tk.LEFT)
		tk.Button(action_frame, text="Write Modified", command=self.write_parameters).pack(side=tk.LEFT, padx=10)

		self.log_widget = ScrolledText(main_frame, height=10, state="disabled")
		self.log_widget.pack(fill=tk.BOTH, expand=True, pady=(5, 0))

	def _set_defaults(self) -> None:
		self.bus_entries["bustype"].insert(0, "pcan")
		self.bus_entries["channel"].insert(0, "PCAN_USBBUS1")
		self.bus_entries["bitrate"].insert(0, "1000000")
		self.node_id_var.set("0x04")
		default_eds = Path("ZeroErr Driver_V1.5.eds")
		self.eds_path_var.set(str(default_eds))

	def log(self, message: str) -> None:
		self.log_widget.configure(state="normal")
		self.log_widget.insert(tk.END, message + "\n")
		self.log_widget.configure(state="disabled")
		self.log_widget.see(tk.END)

	def connect(self) -> None:
		if self.network is not None:
			messagebox.showinfo("Info", "Already connected")
			return

		bustype = self.bus_entries["bustype"].get().strip() or "pcan"
		channel = self.bus_entries["channel"].get().strip() or "PCAN_USBBUS1"
		bitrate_str = self.bus_entries["bitrate"].get().strip() or "1000000"
		node_id_str = self.node_id_var.get().strip() or "0x04"
		eds_path = Path(self.eds_path_var.get().strip() or "ZeroErr Driver_V1.5.eds")

		try:
			bitrate = int(bitrate_str)
			node_id = int(node_id_str, 0)
		except ValueError as exc:
			messagebox.showerror("Input Error", f"Invalid bitrate or node id: {exc}")
			return

		if not eds_path.exists():
			messagebox.showerror("File Error", f"EDS file not found: {eds_path}")
			return

		try:
			network = canopen.Network()
			network.connect(bustype=bustype, channel=channel, bitrate=bitrate)
			# canopen 需要字符串路径，这里显式转换，避免 WindowsPath 缺少 rfind 导致的异常
			node = network.add_node(node_id, str(eds_path))
			node.load_configuration()
		except can.CanInterfaceNotImplementedError as exc:
			messagebox.showerror("CAN Error", f"Interface not available: {exc}")
			return
		except Exception as exc:  # pylint: disable=broad-except
			messagebox.showerror("Connection Error", str(exc))
			try:
				network.disconnect()
			except Exception:  # pragma: no cover - best effort
				pass
			return

		self.network = network
		self.node = node
		self.log(f"Connected to node 0x{node_id:02X} on {bustype}:{channel}")

	def disconnect(self) -> None:
		if self.network is None:
			return
		try:
			self.network.disconnect()
		except Exception as exc:  # pragma: no cover - best effort
			self.log(f"Disconnect warning: {exc}")
		finally:
			self.network = None
			self.node = None
			self.log("Disconnected")

	def _ensure_connected(self) -> bool:
		if self.node is None:
			messagebox.showwarning("Not Connected", "Please connect to the drive first")
			return False
		return True

	def read_parameters(self) -> None:
		if not self._ensure_connected():
			return

		assert self.node is not None

		for field in PARAMETER_FIELDS:
			try:
				raw_value = int(self.node.sdo[field.index][field.subindex].raw)
				time.sleep(0.05)  # Small delay to ensure SDO read completes
				if field.is_bool:
					var: tk.IntVar = self.param_widgets[field.key]
					var.set(1 if raw_value else 0)
				else:
					entry: tk.Entry = self.param_widgets[field.key]
					entry.delete(0, tk.END)
					entry.insert(0, str(raw_value))
			except Exception as exc:  # pylint: disable=broad-except
				self.log(
					f"Failed to read {field.label} (0x{field.index:04X}:{field.subindex:02X}): {exc}"
				)
		self.log("Read completed")

	def write_parameters(self) -> None:
		if not self._ensure_connected():
			return

		assert self.node is not None

		written = 0
		for field in PARAMETER_FIELDS:
			widget = self.param_widgets[field.key]
			if field.is_bool:
				value = int(widget.get())
			else:
				entry: tk.Entry = widget
				text = entry.get().strip()
				if not text:
					continue
				try:
					value = int(text, 0)
				except ValueError as exc:
					self.log(f"Invalid value for {field.label}: {exc}")
					continue

			if value < field.min_value or value > field.max_value:
				self.log(
					f"Value out of range for {field.label}: {value} not in [{field.min_value}, {field.max_value}]"
				)
				continue

			try:
				self.node.sdo[field.index][field.subindex].raw = value
				written += 1
				time.sleep(0.05)  # Small delay to ensure SDO write completes
			except Exception as exc:  # pylint: disable=broad-except
				self.log(
					f"Failed to write {field.label} (0x{field.index:04X}:{field.subindex:02X}): {exc}"
				)

		if written:
			self.log(f"Write completed ({written} items updated)")
		else:
			self.log("No parameters written")

	def _on_close(self) -> None:
		self.disconnect()
		self.root.destroy()


def main() -> None:
		root = tk.Tk()
		ControllerParameterGUI(root)
		root.mainloop()


if __name__ == "__main__":
	main()
