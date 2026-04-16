#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WHEELTEC ROS 底盘串口上位机

发送帧格式 (11字节):
    [0x7B][Mode][保留][Vx_H][Vx_L][Vy_H][Vy_L][Vz_H][Vz_L][BCC][0x7D]

接收帧格式 (24字节):
    [0x7B][FlagStop][Vel_X_H][Vel_X_L][Vel_Y_H][Vel_Y_L][Vel_Z_H][Vel_Z_L]
    [Accel_X_H][Accel_X_L][Accel_Y_H][Accel_Y_L][Accel_Z_H][Accel_Z_L]
    [Gyro_X_H][Gyro_X_L][Gyro_Y_H][Gyro_Y_L][Gyro_Z_H][Gyro_Z_L]
    [Power_H][Power_L][BCC][0x7D]

Vx / Vy:
    mm/s, 有符号 short, 大端

Vz:
    mrad/s, 有符号 short, 大端

Accel:
    原始有符号 short

Gyro:
    原始有符号 short

Power_Voltage:
    mV, 无符号 short, 大端
"""

from __future__ import annotations

import csv
import os
import queue
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import filedialog, messagebox, scrolledtext, ttk

import serial
import serial.tools.list_ports


TX_FRAME_LEN = 11
RX_FRAME_LEN = 24
FRAME_HEAD = 0x7B
FRAME_TAIL = 0x7D

BG = "#0d1117"
PANEL = "#161b22"
BORDER = "#30363d"
ACCENT = "#00d4aa"
ACCENT2 = "#ff6b35"
TEXT_PRI = "#e6edf3"
TEXT_SEC = "#8b949e"
TEXT_DIM = "#484f58"
SUCCESS = "#3fb950"
WARNING = "#d29922"
DANGER = "#f85149"
BTN_BG = "#21262d"
PENDING = "#ff9d00"  # 待确认的橙色

CONTROL_MODES = {
    "0: 正常控制": 0x00,
    "1: 自动回充": 0x01,
    "2: 自动回充+导航": 0x02,
    "3: 红外对接": 0x03,
}

SPEED_PRESETS = (
    ("慢速", 50),
    ("中速", 200),
    ("快速", 500),
    ("全速", 1000),
)


@dataclass(slots=True)
class StatusFrame:
    """解析后的24字节机器人状态帧"""
    flag_stop: int
    vel_x: int
    vel_y: int
    vel_z: int
    accel_x: int
    accel_y: int
    accel_z: int
    gyro_x: int
    gyro_y: int
    gyro_z: int
    power_voltage: int
    bcc_valid: bool

    @classmethod
    def decode(cls, frame: bytes) -> "StatusFrame":
        if len(frame) != RX_FRAME_LEN:
            raise ValueError(f"invalid RX frame length: {len(frame)}")
        bcc_valid = calc_bcc(frame[:22]) == frame[22]
        return cls(
            flag_stop=frame[1],
            vel_x=struct.unpack(">h", frame[2:4])[0],
            vel_y=struct.unpack(">h", frame[4:6])[0],
            vel_z=struct.unpack(">h", frame[6:8])[0],
            accel_x=struct.unpack(">h", frame[8:10])[0],
            accel_y=struct.unpack(">h", frame[10:12])[0],
            accel_z=struct.unpack(">h", frame[12:14])[0],
            gyro_x=struct.unpack(">h", frame[14:16])[0],
            gyro_y=struct.unpack(">h", frame[16:18])[0],
            gyro_z=struct.unpack(">h", frame[18:20])[0],
            power_voltage=struct.unpack(">H", frame[20:22])[0],
            bcc_valid=bcc_valid,
        )


@dataclass(slots=True)
class MotionFrame:
    """发送的11字节控制帧"""
    mode: int
    vx: int
    vy: int
    vz: int

    def encode(self) -> bytes:
        frame = bytearray(TX_FRAME_LEN)
        frame[0] = FRAME_HEAD
        frame[1] = self.mode
        frame[2] = 0x00
        frame[3:5] = struct.pack(">h", self.vx)
        frame[5:7] = struct.pack(">h", self.vy)
        frame[7:9] = struct.pack(">h", self.vz)
        frame[9] = calc_bcc(frame[:9])
        frame[10] = FRAME_TAIL
        return bytes(frame)


def calc_bcc(data: bytes) -> int:
    bcc = 0
    for value in data:
        bcc ^= value
    return bcc & 0xFF


def frame_to_hex(frame: bytes) -> str:
    return " ".join(f"{b:02X}" for b in frame)


class ControlButton(tk.Button):
    def __init__(self, parent, **kwargs):
        self.normal_bg = kwargs.pop("bg", BTN_BG)
        self.hover_bg = kwargs.pop("hover_bg", "#1f6feb")
        super().__init__(
            parent,
            bg=self.normal_bg,
            fg=TEXT_PRI,
            activebackground=self.hover_bg,
            activeforeground="white",
            relief="flat",
            bd=0,
            cursor="hand2",
            **kwargs,
        )
        self.bind("<Enter>", lambda _e: self.config(bg=self.hover_bg))
        self.bind("<Leave>", lambda _e: self.config(bg=self.normal_bg))


class WheeltecController:
    AXIS_CONFIG = (
        ("vx", "Vx 前后", "mm/s", -2000, 2000, 10),
        ("vy", "Vy 左右", "mm/s", -2000, 2000, 10),
        ("vz", "Vz 转向", "mrad/s", -3140, 3140, 10),
    )

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("WHEELTEC ROS 底盘串口上位机")
        self.root.configure(bg=BG)
        self.root.minsize(1160, 760)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.ser: serial.Serial | None = None
        self.connected = False
        self.read_thread: threading.Thread | None = None
        self.auto_thread: threading.Thread | None = None
        self.log_queue: queue.Queue[tuple[str, str]] = queue.Queue()
        self.rx_buffer = bytearray()
        self.ui_alive = True

        self.vx_var = tk.IntVar(value=0)
        self.vy_var = tk.IntVar(value=0)
        self.vz_var = tk.IntVar(value=0)
        self.step_var = tk.IntVar(value=100)

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.timeout_var = tk.DoubleVar(value=0.10)
        self.auto_interval_var = tk.DoubleVar(value=0.10)
        self.mode_var = tk.StringVar(value="0: 正常控制")
        self.auto_reconnect_var = tk.BooleanVar(value=False)

        self.tx_count_var = tk.StringVar(value="0")
        self.rx_count_var = tk.StringVar(value="0")
        self.last_send_var = tk.StringVar(value="未发送")
        self.rx_state_var = tk.StringVar(value="等待接收")
        self.rx_value_var = tk.StringVar(value="Vx=0  Vy=0  Vz=0.000 rad/s")
        self.battery_var = tk.StringVar(value="-- V")
        self.battery_pct_var = tk.StringVar(value="--%")

        self.auto_send = False
        self.tx_count = 0
        self.rx_count = 0
        self.last_rx_time = "-"
        self.keys_pressed: set[str] = set()
        self.log_writer: csv.writer | None = None
        self.log_file: object | None = None

        # 自动发送策略
        self.auto_send_on_change_var = tk.BooleanVar(value=True)
        self.auto_send_delay_ms = 80
        self.auto_send_job: str | None = None
        self.last_sent_payload: tuple[int, int, int, int] | None = None

        self.speed_scales: dict[str, tk.Scale] = {}
        self.speed_entries: dict[str, tk.Entry] = {}
        self.speed_labels: dict[str, tk.Label] = {}
        self.byte_labels: list[tk.Label] = []
        self.last_rx_byte_labels: list[tk.Label] = []

        # ========== ACK相关 ==========
        self._pending_acks: dict = {}  # payload -> (log_line_id, send_time_ms)
        self._ack_timeout_ms = 500     # ACK超时时间(ms)
        self._ack_check_job: str | None = None  # ACK检查定时器
        self._acked_line_ids: set = set()  # 已确认的行号集合

        self._build_ui()
        self._bind_variables()
        self._bind_shortcuts()
        self._refresh_ports()
        self._update_preview()
        self._poll_log()

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        self._build_header()

        main = tk.Frame(self.root, bg=BG)
        main.grid(row=1, column=0, sticky="nsew", padx=12, pady=(0, 12))
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        left = tk.Frame(main, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        right = tk.Frame(main, bg=BG)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)

        self._build_connection(left)
        self._build_status(left)
        self._build_mode_panel(left)
        self._build_speed_panel(left)
        self._build_dpad(left)
        self._build_send_panel(left)
        self._build_dashboard(right)
        self._build_frame_preview(right)
        self._build_rx_preview(right)
        self._build_log(right)

    def _build_header(self):
        header = tk.Frame(self.root, bg=PANEL, height=54)
        header.grid(row=0, column=0, sticky="ew")
        header.grid_propagate(False)

        dot = tk.Canvas(header, width=10, height=10, bg=PANEL, highlightthickness=0)
        dot.pack(side="left", padx=(18, 6))
        dot.create_oval(1, 1, 9, 9, fill=ACCENT, outline="")

        tk.Label(
            header,
            text="WHEELTEC 底盘串口控制台",
            bg=PANEL,
            fg=TEXT_PRI,
            font=("Microsoft YaHei UI", 13, "bold"),
        ).pack(side="left", pady=14)

        self.status_lbl = tk.Label(
            header,
            text="● 未连接",
            bg=PANEL,
            fg=DANGER,
            font=("Consolas", 10),
        )
        self.status_lbl.pack(side="right", padx=18)

        tk.Frame(self.root, bg=BORDER, height=1).grid(row=0, column=0, sticky="sew")

    def _build_connection(self, parent: tk.Widget):
        card = self._card(parent, "串口配置")

        row1 = tk.Frame(card, bg=PANEL)
        row1.pack(fill="x", pady=(0, 6))
        tk.Label(row1, text="端口", width=6, anchor="w", bg=PANEL, fg=TEXT_SEC).pack(side="left")
        self.port_cb = ttk.Combobox(row1, textvariable=self.port_var, width=16, state="readonly")
        self.port_cb.pack(side="left", padx=(4, 4))
        ControlButton(row1, text="刷新", width=6, command=self._refresh_ports, hover_bg=ACCENT).pack(side="left")

        row2 = tk.Frame(card, bg=PANEL)
        row2.pack(fill="x", pady=(0, 6))
        tk.Label(row2, text="波特率", width=6, anchor="w", bg=PANEL, fg=TEXT_SEC).pack(side="left")
        self.baud_cb = ttk.Combobox(
            row2,
            textvariable=self.baud_var,
            width=12,
            state="readonly",
            values=["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"],
        )
        self.baud_cb.pack(side="left", padx=(4, 4))

        tk.Label(row2, text="超时", width=4, anchor="e", bg=PANEL, fg=TEXT_SEC).pack(side="left", padx=(10, 0))
        tk.Entry(
            row2,
            textvariable=self.timeout_var,
            width=6,
            bg=BTN_BG,
            fg=TEXT_PRI,
            insertbackground=TEXT_PRI,
            relief="flat",
        ).pack(side="left", padx=(4, 0))

        self.conn_btn = ControlButton(
            card,
            text="连接串口",
            font=("Microsoft YaHei UI", 10, "bold"),
            bg="#1f6feb",
            hover_bg="#388bfd",
            command=self._toggle_connection,
        )
        self.conn_btn.pack(fill="x", pady=(6, 0))

    def _build_status(self, parent: tk.Widget):
        card = self._card(parent, "运行状态")

        rows = (
            ("发送计数", self.tx_count_var),
            ("接收计数", self.rx_count_var),
            ("最近发送", self.last_send_var),
            ("接收状态", self.rx_state_var),
            ("最近上行", self.rx_value_var),
        )
        for title, variable in rows:
            row = tk.Frame(card, bg=PANEL)
            row.pack(fill="x", pady=2)
            tk.Label(row, text=title, width=8, anchor="w", bg=PANEL, fg=TEXT_SEC).pack(side="left")
            tk.Label(
                row,
                textvariable=variable,
                anchor="w",
                justify="left",
                bg=PANEL,
                fg=TEXT_PRI if title != "接收状态" else WARNING,
                font=("Consolas", 9),
                wraplength=220,
            ).pack(side="left", fill="x", expand=True)

    def _build_speed_panel(self, parent: tk.Widget):
        card = self._card(parent, "速度设置")

        for axis_name, label, unit, low, high, resolution in self.AXIS_CONFIG:
            axis_var = getattr(self, f"{axis_name}_var")
            row = tk.Frame(card, bg=PANEL)
            row.pack(fill="x", pady=(0, 2))

            tk.Label(row, text=label, width=8, anchor="w", bg=PANEL, fg=TEXT_SEC).pack(side="left")
            value_label = tk.Label(
                row,
                text="0",
                width=6,
                anchor="e",
                bg=PANEL,
                fg=ACCENT,
                font=("Consolas", 10, "bold"),
            )
            value_label.pack(side="right")
            self.speed_labels[axis_name] = value_label

            tk.Label(row, text=unit, width=8, anchor="e", bg=PANEL, fg=TEXT_DIM).pack(side="right", padx=(0, 6))

            entry = tk.Entry(
                row,
                width=7,
                justify="center",
                bg=BTN_BG,
                fg=TEXT_PRI,
                insertbackground=TEXT_PRI,
                relief="flat",
            )
            entry.insert(0, "0")
            entry.pack(side="right", padx=(0, 8))
            entry.bind("<Return>", lambda _e, n=axis_name: self._apply_entry_value(n))
            entry.bind("<FocusOut>", lambda _e, n=axis_name: self._apply_entry_value(n))
            self.speed_entries[axis_name] = entry

            scale = tk.Scale(
                card,
                from_=low,
                to=high,
                resolution=resolution,
                orient="horizontal",
                variable=axis_var,
                bg=PANEL,
                fg=TEXT_SEC,
                troughcolor=BORDER,
                activebackground=ACCENT,
                highlightthickness=0,
                bd=0,
                showvalue=False,
            )
            scale.pack(fill="x", pady=(0, 4))
            self.speed_scales[axis_name] = scale

        step_row = tk.Frame(card, bg=PANEL)
        step_row.pack(fill="x", pady=(8, 0))
        tk.Label(step_row, text="按键步进", width=8, anchor="w", bg=PANEL, fg=TEXT_SEC).pack(side="left")
        for step in (50, 100, 200, 500):
            ControlButton(
                step_row,
                text=str(step),
                width=4,
                command=lambda s=step: self.step_var.set(s),
                hover_bg=ACCENT,
                font=("Consolas", 9),
            ).pack(side="left", padx=2)
        tk.Label(step_row, textvariable=self.step_var, bg=PANEL, fg=ACCENT2, font=("Consolas", 10, "bold")).pack(side="left", padx=(8, 0))

        ControlButton(
            card,
            text="全部清零",
            bg=BORDER,
            hover_bg=DANGER,
            command=self._zero_speeds,
            font=("Microsoft YaHei UI", 9),
        ).pack(fill="x", pady=(8, 0))

    def _build_mode_panel(self, parent: tk.Widget):
        card = self._card(parent, "控制模式")

        mode_row = tk.Frame(card, bg=PANEL)
        mode_row.pack(fill="x", pady=(0, 4))
        tk.Label(mode_row, text="Mode", width=6, anchor="w", bg=PANEL, fg=TEXT_SEC).pack(side="left")
        self.mode_cb = ttk.Combobox(
            mode_row,
            textvariable=self.mode_var,
            width=18,
            state="readonly",
            values=list(CONTROL_MODES.keys()),
        )
        self.mode_cb.pack(side="left", padx=(4, 0))
        self.mode_cb.current(0)

        preset_row = tk.Frame(card, bg=PANEL)
        preset_row.pack(fill="x", pady=(0, 4))
        tk.Label(preset_row, text="预设速度", width=6, anchor="w", bg=PANEL, fg=TEXT_SEC).pack(side="left")
        for name, step in SPEED_PRESETS:
            ControlButton(
                preset_row,
                text=name,
                width=4,
                command=lambda s=step: self.step_var.set(s),
                hover_bg=ACCENT2,
                font=("Microsoft YaHei UI", 8),
            ).pack(side="left", padx=2)

        reconnect_row = tk.Frame(card, bg=PANEL)
        reconnect_row.pack(fill="x")
        tk.Checkbutton(
            reconnect_row,
            text="断线自动重连",
            variable=self.auto_reconnect_var,
            bg=PANEL,
            fg=TEXT_SEC,
            selectcolor=BTN_BG,
            activebackground=PANEL,
            activeforeground=TEXT_PRI,
        ).pack(side="left")

    def _build_dpad(self, parent: tk.Widget):
        card = self._card(parent, "方向控制 (WASD / QE)")

        grid = tk.Frame(card, bg=PANEL)
        grid.pack()

        pad_cfg = (
            ("↖", 0, 0, lambda: self._set_vel(1, 1, 0)),
            ("↑", 0, 1, lambda: self._set_vel(1, 0, 0)),
            ("↗", 0, 2, lambda: self._set_vel(1, -1, 0)),
            ("⟲", 0, 3, lambda: self._set_vel(0, 0, 1)),
            ("←", 1, 0, lambda: self._set_vel(0, 1, 0)),
            ("■", 1, 1, self._emergency_stop),
            ("→", 1, 2, lambda: self._set_vel(0, -1, 0)),
            ("⟳", 1, 3, lambda: self._set_vel(0, 0, -1)),
            ("↙", 2, 0, lambda: self._set_vel(-1, 1, 0)),
            ("↓", 2, 1, lambda: self._set_vel(-1, 0, 0)),
            ("↘", 2, 2, lambda: self._set_vel(-1, -1, 0)),
        )

        for text, row, column, command in pad_cfg:
            bg = DANGER if text == "■" else BTN_BG
            hover = DANGER if text == "■" else "#1f6feb"
            ControlButton(
                grid,
                text=text,
                width=3,
                command=command,
                bg=bg,
                hover_bg=hover,
                font=("Consolas", 15, "bold"),
            ).grid(row=row, column=column, padx=3, pady=3)

        tk.Label(
            card,
            text="回车发送一次，空格急停；按方向按钮会按当前步进直接设置速度值。",
            bg=PANEL,
            fg=TEXT_DIM,
            font=("Microsoft YaHei UI", 8),
            wraplength=240,
            justify="left",
        ).pack(pady=(4, 0), anchor="w")

    def _build_send_panel(self, parent: tk.Widget):
        card = self._card(parent, "发送控制")

        row = tk.Frame(card, bg=PANEL)
        row.pack(fill="x", pady=(0, 6))
        tk.Label(row, text="周期(s)", width=8, anchor="w", bg=PANEL, fg=TEXT_SEC).pack(side="left")
        tk.Entry(
            row,
            textvariable=self.auto_interval_var,
            width=7,
            bg=BTN_BG,
            fg=TEXT_PRI,
            insertbackground=TEXT_PRI,
            relief="flat",
        ).pack(side="left")

        opt_row = tk.Frame(card, bg=PANEL)
        opt_row.pack(fill="x", pady=(0, 6))
        tk.Checkbutton(
            opt_row,
            text="速度变化自动发送",
            variable=self.auto_send_on_change_var,
            bg=PANEL,
            fg=TEXT_SEC,
            selectcolor=BTN_BG,
            activebackground=PANEL,
            activeforeground=TEXT_PRI,
        ).pack(side="left")
        tk.Label(
            opt_row,
            text="防抖 80ms",
            bg=PANEL,
            fg=TEXT_DIM,
            font=("Microsoft YaHei UI", 8),
        ).pack(side="right")

        btn_row = tk.Frame(card, bg=PANEL)
        btn_row.pack(fill="x")
        ControlButton(
            btn_row,
            text="发送一次",
            command=self._send_once,
            bg="#1f6feb",
            hover_bg="#388bfd",
            font=("Microsoft YaHei UI", 9, "bold"),
        ).pack(side="left", expand=True, fill="x", padx=(0, 4))

        self.auto_btn = ControlButton(
            btn_row,
            text="连续发送",
            command=self._toggle_auto,
            bg=BORDER,
            hover_bg=SUCCESS,
            font=("Microsoft YaHei UI", 9, "bold"),
        )
        self.auto_btn.pack(side="left", expand=True, fill="x", padx=(0, 4))

        ControlButton(
            btn_row,
            text="急停发送",
            command=self._emergency_stop,
            bg="#5c1d1d",
            hover_bg=DANGER,
            font=("Microsoft YaHei UI", 9, "bold"),
        ).pack(side="left", expand=True, fill="x")

    def _build_dashboard(self, parent: tk.Widget):
        card = self._card(parent, "传感器数据面板")
        card.pack(fill="x", pady=(0, 8))

        # 电池状态行
        bat_row = tk.Frame(card, bg=PANEL)
        bat_row.pack(fill="x", pady=(0, 4))
        tk.Label(bat_row, text="电池", width=6, anchor="w", bg=PANEL, fg=TEXT_SEC).pack(side="left")
        self.battery_bar = ttk.Progressbar(
            bat_row, length=120, mode="determinate",
            variable=tk.DoubleVar(value=0),
        )
        self.battery_bar.pack(side="left", padx=(4, 4))
        tk.Label(bat_row, textvariable=self.battery_var, width=8, anchor="e",
                 bg=PANEL, fg=SUCCESS, font=("Consolas", 10, "bold")).pack(side="left")
        tk.Label(bat_row, textvariable=self.battery_pct_var, width=5, anchor="e",
                 bg=PANEL, fg=TEXT_DIM, font=("Consolas", 9)).pack(side="left")

        # IMU 数据显示
        imu_card = tk.Frame(card, bg="#0a0e13", highlightbackground=BORDER, highlightthickness=1)
        imu_card.pack(fill="x", pady=(0, 4))

        self.imu_text = tk.Text(
            imu_card, height=10, bg="#0a0e13", fg=ACCENT,
            font=("Consolas", 9), relief="flat", bd=0,
            insertbackground=TEXT_PRI, state="disabled",
        )
        self.imu_text.pack(fill="x", padx=6, pady=6)

        # 日志记录控制
        log_row = tk.Frame(card, bg=PANEL)
        log_row.pack(fill="x")
        self.log_btn = ControlButton(
            log_row,
            text="开始记录",
            command=self._toggle_logging,
            bg=BORDER,
            hover_bg=SUCCESS,
            font=("Microsoft YaHei UI", 8, "bold"),
        )
        self.log_btn.pack(side="left")

    def _build_frame_preview(self, parent: tk.Widget):
        card = self._card(parent, "下行数据帧预览")
        card.pack(fill="x", pady=(0, 8))

        self.byte_labels = self._build_frame_bar(
            card,
            ("帧头", "预留", "预留", "Vx_H", "Vx_L", "Vy_H", "Vy_L", "Vz_H", "Vz_L", "BCC", "帧尾"),
        )

    def _build_rx_preview(self, parent: tk.Widget):
        card = self._card(parent, "最近上行数据帧 (24字节)")
        card.pack(fill="x", pady=(0, 8))

        rx_fields = (
            "HEAD", "Flag", "Vx_H", "Vx_L", "Vy_H", "Vy_L", "Vz_H", "Vz_L",
            "Ax_H", "Ax_L", "Ay_H", "Ay_L", "Az_H", "Az_L",
            "Gx_H", "Gx_L", "Gy_H", "Gy_L", "Gz_H", "Gz_L",
            "Pwr_H", "Pwr_L", "BCC", "TAIL",
        )
        self.last_rx_byte_labels = self._build_frame_bar(card, rx_fields)
        self._set_frame_labels(self.last_rx_byte_labels, bytes(RX_FRAME_LEN))

    def _build_frame_bar(self, parent: tk.Widget, fields: tuple[str, ...]) -> list[tk.Label]:
        wrap = tk.Frame(parent, bg="#0a0e13", highlightbackground=BORDER, highlightthickness=1)
        wrap.pack(fill="x", padx=4, pady=4)

        labels: list[tk.Label] = []
        for field in fields:
            col = tk.Frame(wrap, bg="#0a0e13")
            col.pack(side="left", padx=5, pady=6)
            value = tk.Label(
                col,
                text="00",
                width=4,
                bg=BTN_BG,
                fg=TEXT_SEC,
                font=("Consolas", 11, "bold"),
            )
            value.pack()
            tk.Label(col, text=field, bg="#0a0e13", fg=TEXT_DIM, font=("Consolas", 7)).pack()
            labels.append(value)
        return labels

    def _build_log(self, parent: tk.Widget):
        card = self._card(parent, "通信日志")
        card.pack(fill="both", expand=True)

        self.log_text = scrolledtext.ScrolledText(
            card,
            bg="#0a0e13",
            fg=TEXT_SEC,
            font=("Consolas", 9),
            relief="flat",
            bd=0,
            wrap="word",
            insertbackground=TEXT_PRI,
            selectbackground=ACCENT,
            state="disabled",
        )
        self.log_text.pack(fill="both", expand=True)
        self.log_text.tag_config("tx", foreground=ACCENT)
        self.log_text.tag_config("rx", foreground=SUCCESS)
        self.log_text.tag_config("warn", foreground=WARNING)
        self.log_text.tag_config("err", foreground=DANGER)
        self.log_text.tag_config("inf", foreground=TEXT_PRI)
        self.log_text.tag_config("ack_ok", foreground=SUCCESS)     # ACK成功-绿色
        self.log_text.tag_config("ack_fail", foreground=DANGER)   # ACK失败-红色
        self.log_text.tag_config("pending", foreground=PENDING)   # 待确认-橙色

        ControlButton(
            card,
            text="清空日志",
            command=self._clear_log,
            bg=BORDER,
            hover_bg=DANGER,
            font=("Microsoft YaHei UI", 8),
        ).pack(anchor="e", pady=(4, 0))

    def _card(self, parent: tk.Widget, title: str) -> tk.Frame:
        outer = tk.Frame(parent, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        outer.pack(fill="x", pady=(0, 8))
        tk.Label(
            outer,
            text=f"  {title}",
            bg=PANEL,
            fg=TEXT_SEC,
            anchor="w",
            font=("Microsoft YaHei UI", 8, "bold"),
        ).pack(fill="x", pady=(6, 2))
        tk.Frame(outer, bg=BORDER, height=1).pack(fill="x")
        inner = tk.Frame(outer, bg=PANEL)
        inner.pack(fill="both", expand=True, padx=10, pady=8)
        return inner

    def _bind_variables(self):
        for axis_name, *_rest in self.AXIS_CONFIG:
            axis_var = getattr(self, f"{axis_name}_var")
            axis_var.trace_add("write", lambda *_args, name=axis_name: self._on_axis_change(name))

    def _bind_shortcuts(self):
        # 多键同时支持：记录每个按键的按下状态
        self.key_map = {
            "w": (1, 0, 0), "s": (-1, 0, 0),
            "a": (0, 1, 0), "d": (0, -1, 0),
            "q": (0, 0, 1), "e": (0, 0, -1),
        }
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.bind("<space>", lambda _e: self._emergency_stop())
        self.root.bind("<Return>", lambda _e: self._send_once())
        self.root.bind("<F5>", lambda _e: self._refresh_ports())

    def _on_key_press(self, event: tk.Event):
        key = event.keysym.lower()
        if key in self.key_map:
            self.keys_pressed.add(key)
            self._apply_keyspeed()

    def _on_key_release(self, event: tk.Event):
        key = event.keysym.lower()
        if key in self.key_map:
            self.keys_pressed.discard(key)
            # 若所有方向键释放且处于连续发送模式则自动停
            if not self.keys_pressed:
                pass  # 不自动清零，保留最后设置值
            else:
                self._apply_keyspeed()

    def _apply_keyspeed(self):
        """根据当前按下的键计算合成速度"""
        step = self.step_var.get()
        vx, vy, vz = 0, 0, 0
        for key in self.keys_pressed:
            dx, dy, dz = self.key_map[key]
            vx += dx * step
            vy += dy * step
            vz += dz * 500
        self.vx_var.set(max(-32767, min(32767, vx)))
        self.vy_var.set(max(-32767, min(32767, vy)))
        self.vz_var.set(max(-32767, min(32767, vz)))

    def _on_axis_change(self, axis_name: str):
        value = getattr(self, f"{axis_name}_var").get()
        self.speed_labels[axis_name].config(text=str(value))
        entry = self.speed_entries[axis_name]
        if entry.get() != str(value):
            entry.delete(0, "end")
            entry.insert(0, str(value))
        self._update_preview()
        self._schedule_auto_send(reason=f"{axis_name} changed")

    def _apply_entry_value(self, axis_name: str):
        entry = self.speed_entries[axis_name]
        raw = entry.get().strip()
        axis_var = getattr(self, f"{axis_name}_var")
        scale = self.speed_scales[axis_name]
        low = int(scale.cget("from"))
        high = int(scale.cget("to"))
        try:
            value = int(float(raw))
        except ValueError:
            value = axis_var.get()
        axis_var.set(max(low, min(high, value)))

    def _build_current_frame(self) -> MotionFrame:
        mode_str = self.mode_var.get()
        mode_val = int(mode_str.split(":")[0])
        return MotionFrame(
            mode=mode_val,
            vx=max(-32768, min(32767, self.vx_var.get())),
            vy=max(-32768, min(32767, self.vy_var.get())),
            vz=max(-32768, min(32767, self.vz_var.get())),
        )

    def _current_payload(self) -> tuple[int, int, int, int]:
        frame = self._build_current_frame()
        return (frame.mode, frame.vx, frame.vy, frame.vz)

    def _schedule_auto_send(self, reason: str = ""):
        if not self.auto_send_on_change_var.get():
            return
        if self.auto_send:
            return
        if not self.connected or self.ser is None or not self.ser.is_open:
            return

        payload = self._current_payload()
        if payload == self.last_sent_payload:
            return

        if self.auto_send_job is not None:
            try:
                self.root.after_cancel(self.auto_send_job)
            except tk.TclError:
                pass
            self.auto_send_job = None

        self.auto_send_job = self.root.after(
            self.auto_send_delay_ms,
            lambda r=reason: self._auto_send_if_needed(r),
        )

    def _auto_send_if_needed(self, reason: str = ""):
        self.auto_send_job = None
        if self.auto_send:
            return
        if not self.auto_send_on_change_var.get():
            return
        if not self.connected or self.ser is None or not self.ser.is_open:
            return

        payload = self._current_payload()
        if payload == self.last_sent_payload:
            return

        self._send_once(auto_reason=reason or "auto")

    def _update_preview(self):
        self._set_frame_labels(self.byte_labels, self._build_current_frame().encode())

    def _set_frame_labels(self, labels: list[tk.Label], frame: bytes):
        highlight = {0: ACCENT2, 9: WARNING, 10: ACCENT2}
        speed_bytes = {3, 4, 5, 6, 7, 8}
        for index, label in enumerate(labels):
            value = frame[index] if index < len(frame) else 0
            if index in highlight:
                fg = highlight[index]
            elif index in speed_bytes:
                fg = ACCENT
            else:
                fg = TEXT_SEC
            label.config(text=f"{value:02X}", fg=fg)

    def _refresh_ports(self):
        current = self.port_var.get()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if current in ports:
            self.port_var.set(current)
        elif ports:
            self.port_var.set(ports[0])
        else:
            self.port_var.set("")
        self._log(f"已刷新串口列表: {', '.join(ports) if ports else '未发现设备'}", "inf")

    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("连接失败", "请先选择串口端口。")
            return

        try:
            baud = int(self.baud_var.get())
            timeout = max(0.01, float(self.timeout_var.get()))
        except ValueError:
            messagebox.showerror("参数错误", "波特率或超时时间格式不正确。")
            return

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        except serial.SerialException as exc:
            messagebox.showerror("连接失败", str(exc))
            return

        self.connected = True
        self.rx_buffer.clear()
        self.last_sent_payload = None
        # 清空ACK相关状态
        self._pending_acks.clear()
        self._acked_line_ids.clear()
        self.conn_btn.config(text="断开串口", bg=DANGER, activebackground="#b91c1c")
        self.status_lbl.config(text=f"● {port} @ {baud}", fg=SUCCESS)
        self.rx_state_var.set("已连接，等待数据")
        self._log(f"已连接 {port}，波特率 {baud}，超时 {timeout:.2f}s", "inf")

        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _disconnect(self, reason: str | None = None):
        self.auto_send = False
        self.connected = False
        if self.auto_send_job is not None:
            try:
                self.root.after_cancel(self.auto_send_job)
            except tk.TclError:
                pass
            self.auto_send_job = None
        # 取消ACK检查定时器
        if self._ack_check_job is not None:
            try:
                self.root.after_cancel(self._ack_check_job)
            except tk.TclError:
                pass
            self._ack_check_job = None
        # 清空待确认列表
        self._pending_acks.clear()
        self._acked_line_ids.clear()

        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except serial.SerialException:
                pass
            finally:
                self.ser = None

        self.auto_btn.config(text="连续发送", bg=BORDER, activebackground=SUCCESS)
        self.conn_btn.config(text="连接串口", bg="#1f6feb", activebackground="#388bfd")
        self.status_lbl.config(text="● 未连接", fg=DANGER)
        self.rx_state_var.set("串口已断开")

        log_msg = reason if reason else "串口已断开"
        self._log(log_msg, "warn" if reason else "inf")

        # 断线自动重连
        if reason and self.auto_reconnect_var.get():
            self.root.after(1500, self._auto_reconnect)

    def _auto_reconnect(self):
        if self.connected or self.ui_alive is False:
            return
        port = self.port_var.get().strip()
        if not port:
            return
        try:
            baud = int(self.baud_var.get())
            timeout = max(0.01, float(self.timeout_var.get()))
            self._log(f"尝试自动重连 {port} @ {baud}...", "inf")
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
            self.connected = True
            self.rx_buffer.clear()
            self.last_sent_payload = None
            self._pending_acks.clear()
            self._acked_line_ids.clear()
            self.conn_btn.config(text="断开串口", bg=DANGER, activebackground="#b91c1c")
            self.status_lbl.config(text=f"● {port} @ {baud} (重连)", fg=SUCCESS)
            self.rx_state_var.set("重连成功，等待数据")
            self._log(f"自动重连成功", "inf")
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
        except serial.SerialException:
            self._log("自动重连失败，将在5秒后重试...", "warn")
            self.root.after(5000, self._auto_reconnect)

    def _read_loop(self):
        while self.connected and self.ser is not None:
            try:
                data = self.ser.read(128)
            except serial.SerialException as exc:
                self.root.after(0, lambda e=exc: self._disconnect(f"读取失败: {e}"))
                return

            if not data:
                continue

            self.rx_buffer.extend(data)
            while True:
                frame = self._extract_frame()
                if frame is None:
                    break
                self.root.after(0, lambda f=frame: self._handle_rx_frame(f))

    def _extract_frame(self) -> bytes | None:
        while self.rx_buffer and self.rx_buffer[0] != FRAME_HEAD:
            self.rx_buffer.pop(0)

        if len(self.rx_buffer) < TX_FRAME_LEN:
            return None

        # 尝试按11字节帧解析（帧尾在位置10）
        if self.rx_buffer[10] == FRAME_TAIL:
            frame = bytes(self.rx_buffer[:TX_FRAME_LEN])
            del self.rx_buffer[:TX_FRAME_LEN]
            return frame

        # 尝试按24字节帧解析（帧尾在位置23）
        if len(self.rx_buffer) >= RX_FRAME_LEN and self.rx_buffer[23] == FRAME_TAIL:
            frame = bytes(self.rx_buffer[:RX_FRAME_LEN])
            del self.rx_buffer[:RX_FRAME_LEN]
            return frame

        # 不足以匹配任何帧，等待更多数据
        return None

    def _handle_rx_frame(self, frame: bytes):
        self.rx_count += 1
        self.rx_count_var.set(str(self.rx_count))
        self.last_rx_time = time.strftime("%H:%M:%S")
        self._set_frame_labels(self.last_rx_byte_labels, frame)

        if len(frame) == RX_FRAME_LEN:
            self._handle_status_frame(frame)
        elif len(frame) == TX_FRAME_LEN:
            self._handle_echo_frame(frame)
        else:
            self.rx_state_var.set(f"未知帧({len(frame)}B) @ {self.last_rx_time}")
            self._log(f"← RX: {frame_to_hex(frame)}  [未知帧]", "warn")

    def _handle_status_frame(self, frame: bytes):
        """解析24字节状态帧（机器人上行数据）"""
        try:
            status = StatusFrame.decode(frame)
        except Exception as e:
            self.rx_value_var.set(f"解析错误: {e}")
            self.rx_state_var.set(f"解析错误 @ {self.last_rx_time}")
            self._log(f"← RX: {frame_to_hex(frame)}  [解析错误: {e}]", "err")
            return

        # 更新状态显示
        battery_v = status.power_voltage / 1000.0
        self.rx_value_var.set(
            f"Vx={status.vel_x}  Vy={status.vel_y}  Vz={status.vel_z}\n"
            f"电压={battery_v:.2f}V  停止={status.flag_stop}"
        )
        self.rx_state_var.set(
            f"BCC{'✓' if status.bcc_valid else '✗'}  {self.last_rx_time}"
        )

        # 更新仪表盘
        self._update_dashboard(status)

        suffix = (
            f" 电池={battery_v:.2f}V "
            f"Accel=({status.accel_x},{status.accel_y},{status.accel_z}) "
            f"Gyro=({status.gyro_x},{status.gyro_y},{status.gyro_z})"
        )
        tag = "rx" if status.bcc_valid else "warn"
        self._log(f"← RX: {frame_to_hex(frame)}  [24B]{suffix}", tag)

        # 写日志文件
        if self.log_writer:
            self.log_writer.writerow([
                time.time(), status.vel_x, status.vel_y, status.vel_z,
                status.accel_x, status.accel_y, status.accel_z,
                status.gyro_x, status.gyro_y, status.gyro_z,
                status.power_voltage, int(status.bcc_valid),
            ])

    def _handle_echo_frame(self, frame: bytes):
        """解析11字节回显帧（发送帧的应答ACK）"""
        valid = calc_bcc(frame[:9]) == frame[9]
        try:
            echo = MotionFrame(
                mode=frame[1],
                vx=struct.unpack(">h", frame[3:5])[0],
                vy=struct.unpack(">h", frame[5:7])[0],
                vz=struct.unpack(">h", frame[7:9])[0],
            )
            self.rx_value_var.set(
                f"回显: Vx={echo.vx}  Vy={echo.vy}  Vz={echo.vz / 1000:.3f} rad/s"
            )
        except Exception:
            echo = None
            self.rx_value_var.set("回显解析失败")

        status = "BCC 正常" if valid else "BCC 错误"
        self.rx_state_var.set(f"回显 {status}  @ {self.last_rx_time}")

        # 尝试更新对应的发送日志为ACK成功状态
        if echo is not None and valid:
            ack_payload = (echo.mode, echo.vx, echo.vy, echo.vz)
            self._mark_ack_received(ack_payload)

        tag = "rx" if valid else "warn"
        suffix = f"  (Vx={echo.vx} Vy={echo.vy} Vz={echo.vz / 1000:.3f})" if echo else ""
        self._log(f"← RX: {frame_to_hex(frame)}  [11B回显] {status}{suffix}", tag)

    def _mark_ack_received(self, payload: tuple):
        """收到ACK后，更新对应发送日志为成功状态"""
        if payload not in self._pending_acks:
            return

        log_line_id, _ = self._pending_acks.pop(payload)
        self._acked_line_ids.add(log_line_id)
        self.root.after(0, lambda: self._update_log_line_color(log_line_id, "ack_ok"))

    def _update_log_line_color(self, line_id: int, tag: str):
        """更新指定行号的日志颜色"""
        try:
            self.log_text.config(state="normal")
            start_index = f"{line_id}.0"
            end_index = f"{line_id}.end"
            self.log_text.tag_remove("pending", start_index, end_index)
            self.log_text.tag_add(tag, start_index, end_index)
            self.log_text.config(state="disabled")
        except tk.TclError:
            pass

    def _check_ack_timeout(self):
        """检查哪些发送记录超时未收到ACK"""
        if not self.ui_alive:
            return

        current_time = time.time() * 1000
        timeout_items = []

        for payload, (log_line_id, send_time) in list(self._pending_acks.items()):
            if log_line_id not in self._acked_line_ids:
                if current_time - send_time > self._ack_timeout_ms:
                    timeout_items.append((payload, log_line_id))

        # 标记超时为失败
        for payload, log_line_id in timeout_items:
            if log_line_id not in self._acked_line_ids:
                self._acked_line_ids.add(log_line_id)
                self._pending_acks.pop(payload, None)
                self._update_log_line_color(log_line_id, "ack_fail")

        # 继续检查
        self._ack_check_job = self.root.after(100, self._check_ack_timeout)

    def _send_once(self, auto_reason: str | None = None):
        if not self.connected or self.ser is None or not self.ser.is_open:
            self._log("串口未连接，无法发送。", "err")
            return

        frame_obj = self._build_current_frame()
        frame = frame_obj.encode()
        try:
            self.ser.write(frame)
        except serial.SerialException as exc:
            self._disconnect(f"发送失败: {exc}")
            return

        self.last_sent_payload = (frame_obj.mode, frame_obj.vx, frame_obj.vy, frame_obj.vz)
        self.tx_count += 1
        self.tx_count_var.set(str(self.tx_count))
        send_ts = time.strftime("%H:%M:%S")
        self.last_send_var.set(send_ts)
        reason_text = f" [{auto_reason}]" if auto_reason else ""

        # 获取当前日志行号（在插入之前）
        self.log_text.update_idletasks()
        current_line_count = int(self.log_text.index("end-1c").split(".")[0])

        # 记录待确认的发送
        payload = (frame_obj.mode, frame_obj.vx, frame_obj.vy, frame_obj.vz)
        self._pending_acks[payload] = (current_line_count, time.time() * 1000)

        # 启动ACK超时检查（如果还没启动）
        if self._ack_check_job is None:
            self._ack_check_job = self.root.after(100, self._check_ack_timeout)

        # 使用pending标签（橙色）标记，待确认状态
        self._log(
            f"→ TX{reason_text}: {frame_to_hex(frame)}  (Vx={frame_obj.vx} Vy={frame_obj.vy} Vz={frame_obj.vz / 1000:.3f} rad/s)",
            "pending",
        )

    def _toggle_logging(self):
        if self.log_writer is not None:
            # 停止记录
            self.log_btn.config(text="开始记录", bg=BORDER, activebackground=SUCCESS)
            if self.log_file:
                self.log_file.close()
            self.log_writer = None
            self.log_file = None
            self._log("数据记录已停止", "inf")
            return

        # 开始记录
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV 文件", "*.csv")],
            title="保存数据记录",
        )
        if not path:
            return

        try:
            self.log_file = open(path, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow([
                "timestamp", "vel_x", "vel_y", "vel_z",
                "accel_x", "accel_y", "accel_z",
                "gyro_x", "gyro_y", "gyro_z",
                "power_mv", "bcc_valid",
            ])
        except OSError as exc:
            messagebox.showerror("记录失败", str(exc))
            self.log_writer = None
            self.log_file = None
            return

        self.log_btn.config(text="停止记录", bg=DANGER, activebackground=DANGER)
        self._log(f"数据记录已开始: {os.path.basename(path)}", "inf")

    def _toggle_auto(self):
        if self.auto_send:
            self.auto_send = False
            self.auto_btn.config(text="连续发送", bg=BORDER, activebackground=SUCCESS)
            self._log("连续发送已停止", "inf")
            return

        if not self.connected:
            self._log("请先连接串口，再启动连续发送。", "err")
            return

        try:
            interval = max(0.02, float(self.auto_interval_var.get()))
        except ValueError:
            self._log("发送周期格式不正确。", "err")
            return

        self.auto_send = True
        self.auto_btn.config(text="停止连发", bg=DANGER, activebackground=DANGER)
        self._log(f"连续发送已启动，周期 {interval:.2f}s", "inf")
        self.auto_thread = threading.Thread(target=self._auto_loop, daemon=True)
        self.auto_thread.start()

    def _auto_loop(self):
        while self.auto_send and self.connected:
            self.root.after(0, self._send_once)
            try:
                interval = max(0.02, float(self.auto_interval_var.get()))
            except (ValueError, tk.TclError):
                interval = 0.10
            time.sleep(interval)

    def _set_vel(self, vx_sign: int, vy_sign: int, vz_sign: int):
        step = self.step_var.get()
        if vx_sign:
            self.vx_var.set(vx_sign * step)
        if vy_sign:
            self.vy_var.set(vy_sign * step)
        if vz_sign:
            self.vz_var.set(vz_sign * 500)

    def _zero_speeds(self):
        self.vx_var.set(0)
        self.vy_var.set(0)
        self.vz_var.set(0)

    def _emergency_stop(self):
        self._zero_speeds()
        if self.connected:
            self._send_once()
        self._log("已执行急停", "warn")

    def _log(self, message: str, tag: str = "inf"):
        timestamp = time.strftime("%H:%M:%S")
        self.log_queue.put((f"[{timestamp}] {message}\n", tag))

    def _poll_log(self):
        if not self.ui_alive:
            return

        try:
            while True:
                text, tag = self.log_queue.get_nowait()
                self.log_text.config(state="normal")
                self.log_text.insert("end", text, tag)
                self.log_text.see("end")
                self.log_text.config(state="disabled")
        except queue.Empty:
            pass

        self.root.after(100, self._poll_log)

    def _clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    def _on_close(self):
        self.ui_alive = False
        self.auto_send = False
        if self.connected:
            self._emergency_stop()
        self._disconnect()
        self.root.destroy()

    def _update_dashboard(self, status: StatusFrame):
        """更新IMU和电池仪表盘"""
        if hasattr(self, 'battery_var'):
            voltage = status.power_voltage / 1000.0
            self.battery_var.set(f"{voltage:.2f} V")
            # 简单电量估计（4S电池 14.8V满电 / 12V低压）
            pct = max(0, min(100, (voltage - 12.0) / (14.8 - 12.0) * 100))
            self.battery_pct_var.set(f"{pct:.0f}%")
            if hasattr(self, 'battery_bar'):
                self.battery_bar['value'] = pct

        if hasattr(self, 'imu_text'):
            self.imu_text.config(state="normal")
            self.imu_text.delete("1.0", "end")
            self.imu_text.insert("end", (
                f"Accel X: {status.accel_x:>6}\n"
                f"Accel Y: {status.accel_y:>6}\n"
                f"Accel Z: {status.accel_z:>6}\n"
                f"─────────────\n"
                f"Gyro  X: {status.gyro_x:>6}\n"
                f"Gyro  Y: {status.gyro_y:>6}\n"
                f"Gyro  Z: {status.gyro_z:>6}\n"
                f"─────────────\n"
                f"Vel X : {status.vel_x:>6} mm/s\n"
                f"Vel Y : {status.vel_y:>6} mm/s\n"
                f"Vel Z : {status.vel_z:>6} mm/s"
            ))
            self.imu_text.config(state="disabled")


def apply_dark_style():
    style = ttk.Style()
    style.theme_use("clam")
    style.configure(
        "TCombobox",
        fieldbackground=BTN_BG,
        background=BTN_BG,
        foreground=TEXT_PRI,
        selectbackground=BORDER,
        selectforeground=TEXT_PRI,
        bordercolor=BORDER,
        arrowcolor=TEXT_SEC,
    )
    style.map(
        "TCombobox",
        fieldbackground=[("readonly", BTN_BG)],
        foreground=[("readonly", TEXT_PRI)],
    )


if __name__ == "__main__":
    root = tk.Tk()
    apply_dark_style()
    app = WheeltecController(root)
    root.mainloop()
