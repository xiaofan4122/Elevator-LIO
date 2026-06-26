#!/usr/bin/env python3
# Copyright (c) 2026 xiaofan
# SPDX-License-Identifier: MIT

import csv
import html
import json
import math
import os
import pty
import re
import shutil
import shlex
import signal
import subprocess
import sys
import time
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

_qt_im_module = os.environ.get("QT_IM_MODULE", "").strip()
_xmodifiers = os.environ.get("XMODIFIERS", "").strip()
_gtk_im_module = os.environ.get("GTK_IM_MODULE", "").strip()

# Qt5 on this machine ships the fcitx plugin, not a separate fcitx5 plugin.
# If the desktop session exports fcitx5 here, PyQt can fail to activate Sogou
# inside QLineEdit/QPlainTextEdit even though the input method daemon is alive.
if _qt_im_module in ("", "fcitx5"):
    os.environ["QT_IM_MODULE"] = "fcitx"
if _xmodifiers in ("", "@im=fcitx5"):
    os.environ["XMODIFIERS"] = "@im=fcitx"
if _gtk_im_module == "":
    os.environ["GTK_IM_MODULE"] = "fcitx"

from PyQt5 import QtCore, QtGui, QtWidgets
import genpy
import rosbag
from std_msgs.msg import Bool
import yaml


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parent.parent
SETUP_SCRIPT = WORKSPACE_ROOT / "devel" / "setup.bash"
LIO_EXECUTABLE = WORKSPACE_ROOT / "devel" / "lib" / "lio" / "lio"
SYSTEM_LIBUSB = Path("/lib/x86_64-linux-gnu/libusb-1.0.so.0")
RVIZ_CONFIG = PACKAGE_ROOT / "rviz" / "LIO.rviz"
YAML_ROOT = PACKAGE_ROOT / "yaml"
DB_PATH = PACKAGE_ROOT / "temp" / "bag_runner_manager.json"
ANALYSIS_ATTACHMENT_ROOT = PACKAGE_ROOT / "temp" / "analysis_attachments"
BATCH_EXPORT_ROOT = PACKAGE_ROOT / "temp" / "batch_exports"
FIXED_CONFIG_NAME = "root_config.yaml"
DEFAULT_RATE = 3.0
WAIT_TIMEOUT_MS = 20000
OTHER_LIO_PROFILES = (
    {
        "key": "fastlio",
        "display_name": "FASTLIO",
        "setup_script": str(Path.home() / "CLionProjects" / "ws_fastlio2" / "devel" / "setup.bash"),
        "launch_command": "export DISABLE_ROS1_EOL_WARNINGS=1; exec roslaunch fast_lio mapping_mid360.launch rviz:=true",
        "ready_node_pattern": r"^/laserMapping$",
        "description": "~/CLionProjects/ws_fastlio2 | roslaunch fast_lio mapping_mid360.launch rviz:=true",
        "extra_lib_paths": (),
    },
    {
        "key": "pointlio",
        "display_name": "PointLIO",
        "setup_script": str(Path.home() / "CLionProjects" / "ws_pointLIO" / "devel" / "setup.bash"),
        "launch_command": "export DISABLE_ROS1_EOL_WARNINGS=1; exec roslaunch point_lio mapping_mid360.launch rviz:=true",
        "ready_node_pattern": r"^/laserMapping$",
        "description": "~/CLionProjects/ws_pointLIO | roslaunch point_lio mapping_mid360.launch rviz:=true",
        "extra_lib_paths": (),
    },
    {
        "key": "lio_ekf",
        "display_name": "LIO-EKF",
        "setup_script": str(Path.home() / "CLionProjects" / "ws_LIO-EKF" / "devel" / "setup.bash"),
        "launch_command": "export DISABLE_ROS1_EOL_WARNINGS=1; exec roslaunch lio_ekf mid360.launch start_driver:=false",
        "ready_node_pattern": r"^/lio_ekf_node$",
        "description": "~/CLionProjects/ws_LIO-EKF | roslaunch lio_ekf mid360.launch start_driver:=false",
        "extra_lib_paths": (),
    },
    {
        "key": "liosam",
        "display_name": "LIOSAM",
        "setup_script": str(Path.home() / "CLionProjects" / "ws_liosam_yd" / "devel" / "setup.bash"),
        "launch_command": "export DISABLE_ROS1_EOL_WARNINGS=1; exec roslaunch lio_sam run_mid360.launch",
        "ready_node_pattern": r"^/lio_sam_mapOptmization$",
        "description": "~/CLionProjects/ws_liosam_yd | roslaunch lio_sam run_mid360.launch",
        "extra_lib_paths": (
            str(Path.home() / "CLionProjects" / "ws_liosam_yd" / ".deps" / "ros-noetic-gtsam" / "opt" / "ros" / "noetic" / "lib" / "x86_64-linux-gnu"),
        ),
    },
)


STYLESHEET = """
QWidget {
    background: #f3f1ec;
    color: #233142;
    font-family: "Noto Sans CJK SC", "Microsoft YaHei", "PingFang SC", sans-serif;
    font-size: 14px;
}
QLabel {
    background: transparent;
}
QLabel#TitleLabel {
    font-size: 24px;
    font-weight: 700;
    color: #182534;
}
QLabel#SubtitleLabel {
    color: #6f7b88;
}
QFrame#Card {
    background: #fbfaf7;
    border: 1px solid #e1ddd5;
    border-radius: 18px;
}
QFrame#InlineField {
    background: #fdfcf9;
    border: 1px solid #dad6ce;
    border-radius: 12px;
}
QFrame#InlineField QLineEdit,
QFrame#InlineField QComboBox {
    background: transparent;
    border: none;
    border-radius: 0px;
    padding: 10px 12px;
}
QFrame#InlineField QPushButton {
    background: transparent;
    border: none;
    border-left: 1px solid #dad6ce;
    border-radius: 0px;
    padding: 10px 16px;
    color: #314253;
}
QFrame#InlineField QPushButton:hover {
    background: #f1ede6;
}
QFrame#InlineField QPushButton:pressed {
    background: #e7e2d9;
}
QFrame#InlineField QComboBox::drop-down {
    border: none;
    width: 22px;
}
QPushButton {
    background: #e8e2d8;
    color: #314253;
    border: none;
    border-radius: 12px;
    padding: 10px 16px;
    font-weight: 600;
}
QPushButton:hover {
    background: #ddd6cb;
}
QPushButton:disabled {
    background: #ece8e0;
    color: #9aa3ad;
}
QPushButton#PrimaryButton {
    background: #506b85;
    color: #ffffff;
}
QPushButton#PrimaryButton:hover {
    background: #435d75;
}
QPushButton#DangerButton {
    background: #a96561;
    color: #ffffff;
}
QPushButton#DangerButton:hover {
    background: #985854;
}
QLineEdit, QPlainTextEdit, QDoubleSpinBox, QTableWidget, QTabWidget::pane {
    background: #fdfcf9;
    border: 1px solid #dad6ce;
    border-radius: 12px;
}
QLineEdit, QDoubleSpinBox {
    padding: 10px 12px;
}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    background: #ece7df;
    border-left: 1px solid #cbc3b8;
    width: 24px;
}
QDoubleSpinBox::up-button {
    border-top-right-radius: 12px;
    border-bottom: 1px solid #d3ccc1;
}
QDoubleSpinBox::down-button {
    border-bottom-right-radius: 12px;
}
QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
    background: #e1dbd1;
}
QDoubleSpinBox::up-button:pressed, QDoubleSpinBox::down-button:pressed {
    background: #d5cec3;
}
QPlainTextEdit {
    padding: 10px 12px;
}
QTableWidget {
    gridline-color: #ebe6dd;
    selection-background-color: #d7e1e8;
    selection-color: #152433;
    alternate-background-color: #f7f5f1;
}
QTreeWidget {
    background: #fdfcf9;
    border: 1px solid #dad6ce;
    border-radius: 12px;
    alternate-background-color: #f7f5f1;
    outline: none;
    show-decoration-selected: 0;
}
QTreeWidget::item {
    padding: 4px 6px;
}
QTreeWidget::item:selected {
    background: #d7e1e8;
    color: #152433;
}
QTreeWidget::item:selected:active {
    background: #cfdbe4;
    color: #152433;
}
QTreeWidget::branch {
    background: transparent;
}
QTreeWidget::branch:selected {
    background: transparent;
}
QHeaderView::section {
    background: #efeae2;
    border: none;
    border-bottom: 1px solid #e0dacf;
    padding: 10px;
    font-weight: 700;
}
QTabWidget::pane {
    background: #fcfbf8;
    border: 1px solid #d7d1c6;
    border-radius: 14px;
    margin-top: 10px;
    padding: 8px;
}
QTabWidget::tab-bar {
    left: 6px;
}
QTabBar {
    background: transparent;
}
QTabBar::tab {
    background: #e8e1d6;
    color: #617181;
    border: 1px solid #d8d1c6;
    border-bottom: none;
    border-top-left-radius: 12px;
    border-top-right-radius: 12px;
    padding: 9px 18px;
    margin-right: 8px;
    min-width: 120px;
    font-weight: 600;
}
QTabBar::tab:hover {
    background: #ddd7cd;
    color: #324556;
}
QTabBar::tab:selected {
    background: #fcfbf8;
    color: #1f3142;
    border-color: #cfd7df;
}
QTabBar::tab:!selected {
    margin-top: 4px;
}
QScrollBar:vertical {
    background: #ebe5dc;
    width: 14px;
    margin: 6px 2px 6px 2px;
    border-radius: 7px;
}
QScrollBar::handle:vertical {
    background: #adb8bf;
    min-height: 28px;
    border-radius: 7px;
}
QScrollBar::handle:vertical:hover {
    background: #8e9da8;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    background: transparent;
    height: 0px;
}
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
    background: transparent;
}
QScrollBar:horizontal {
    background: #ebe5dc;
    height: 14px;
    margin: 2px 6px 2px 6px;
    border-radius: 7px;
}
QScrollBar::handle:horizontal {
    background: #adb8bf;
    min-width: 28px;
    border-radius: 7px;
}
QScrollBar::handle:horizontal:hover {
    background: #8e9da8;
}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    background: transparent;
    width: 0px;
}
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {
    background: transparent;
}
QGroupBox {
    border: none;
    font-weight: 700;
    margin-top: 8px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 0px;
    padding: 0 2px;
}
QLabel#StatusBadge {
    background: #ece8df;
    color: #516173;
    border: 1px solid #d7d0c4;
    border-radius: 10px;
    padding: 8px 12px;
    font-weight: 700;
}
"""


@dataclass
class BagEntry:
    bag_id: str
    alias: str
    bag_path: str
    play_rate: float = DEFAULT_RATE
    note: str = ""
    has_elevator_flag: bool = False
    group: str = "Default"
    marker: str = "none"
    marker_note: str = ""
    record_date: str = ""
    duration_seconds: float = 0.0
    deleted: bool = False


@dataclass
class AppSettings:
    setup_script: str = str(SETUP_SCRIPT)
    lio_executable: str = str(LIO_EXECUTABLE)
    libusb_path: str = str(SYSTEM_LIBUSB)
    root_config: str = FIXED_CONFIG_NAME
    build_before_run: bool = True
    enable_rviz: bool = True


@dataclass
class BatchTestGroup:
    group_id: str
    name: str
    bag_ids: List[str] = field(default_factory=list)
    created_at: str = ""
    updated_at: str = ""


@dataclass
class AnalysisRecord:
    bag_id: str
    bag_path: str
    finished_at: str
    lio_yaml_name: str
    exit_commit_count: int
    delta_z_values: List[float]
    delta_z_total: float
    flag_event_times: List[float]
    yaml_snapshot: dict = field(default_factory=dict)
    yaml_group_name: str = ""
    yaml_diff_keys: List[str] = field(default_factory=list)
    generated_bag_path: str = ""
    terminal_error: float = 0.0
    terminal_z_error: float = 0.0
    total_path_length: float = 0.0
    terminal_pose: List[float] = field(default_factory=list)
    attachments: List[str] = field(default_factory=list)
    comment: str = ""


@dataclass(frozen=True)
class ExternalLioProfile:
    key: str
    display_name: str
    setup_script: str
    launch_command: str
    ready_node_pattern: str
    description: str = ""
    extra_lib_paths: Tuple[str, ...] = ()


class ManagedProcess(QtCore.QObject):
    output = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal(int, str)
    state_changed = QtCore.pyqtSignal()

    def __init__(self, name: str, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self.name = name
        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._handle_output)
        self.process.finished.connect(self._handle_finished)
        self.process.stateChanged.connect(self._handle_state_changed)

    def start(self, command: str, cwd: Path) -> None:
        self.output.emit(f"$ {command}")
        self.process.setProgram("/bin/bash")
        self.process.setArguments(["-lc", command])
        self.process.setWorkingDirectory(str(cwd))
        self.process.start()
        self.state_changed.emit()

    def terminate(self) -> None:
        if self.process.state() == QtCore.QProcess.NotRunning:
            return
        pid = int(self.process.processId())
        if pid > 0:
            try:
                os.killpg(pid, signal.SIGINT)
            except (ProcessLookupError, PermissionError, OSError):
                pass
        self.state_changed.emit()

    def kill(self) -> None:
        if self.process.state() == QtCore.QProcess.NotRunning:
            return
        pid = int(self.process.processId())
        if pid > 0:
            try:
                os.killpg(pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError, OSError):
                pass
        self.process.kill()

    def is_running(self) -> bool:
        return self.process.state() != QtCore.QProcess.NotRunning

    def _handle_output(self) -> None:
        text = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if text:
            self.output.emit(text.rstrip("\n"))

    def _handle_finished(self, exit_code: int, exit_status: QtCore.QProcess.ExitStatus) -> None:
        status = "normal" if exit_status == QtCore.QProcess.NormalExit else "crashed"
        self.finished.emit(exit_code, status)
        self.state_changed.emit()

    def _handle_state_changed(self, _state: QtCore.QProcess.ProcessState) -> None:
        self.state_changed.emit()


class PtyManagedProcess(QtCore.QObject):
    output = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal(int, str)
    state_changed = QtCore.pyqtSignal()

    def __init__(self, name: str, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self.name = name
        self.proc: Optional[subprocess.Popen] = None
        self.master_fd: Optional[int] = None
        self.notifier: Optional[QtCore.QSocketNotifier] = None
        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.setInterval(100)
        self.poll_timer.timeout.connect(self._poll_process)

    def start(self, command: str, cwd: Path) -> None:
        self.output.emit(f"$ {command}")
        master_fd, slave_fd = pty.openpty()
        self.proc = subprocess.Popen(
            ["/bin/bash", "-lc", command],
            cwd=str(cwd),
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            preexec_fn=os.setsid,
            close_fds=True,
        )
        os.close(slave_fd)
        self.master_fd = master_fd
        self.notifier = QtCore.QSocketNotifier(master_fd, QtCore.QSocketNotifier.Read, self)
        self.notifier.activated.connect(self._read_output)
        self.poll_timer.start()
        self.state_changed.emit()

    def write_stdin(self, data: bytes) -> None:
        if self.master_fd is None or not self.is_running():
            return
        try:
            os.write(self.master_fd, data)
        except OSError:
            pass

    def terminate(self) -> None:
        if not self.is_running() or self.proc is None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError, OSError):
            pass
        self.state_changed.emit()

    def kill(self) -> None:
        if not self.is_running() or self.proc is None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            pass
        self.state_changed.emit()

    def is_running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def _read_output(self) -> None:
        if self.master_fd is None:
            return
        try:
            data = os.read(self.master_fd, 4096)
        except OSError:
            if not self.is_running():
                self._cleanup_fd()
            return
        if data:
            self.output.emit(data.decode("utf-8", errors="replace").rstrip("\n"))
            return
        if not self.is_running():
            self._cleanup_fd()

    def _poll_process(self) -> None:
        if self.proc is None:
            return
        ret = self.proc.poll()
        if ret is None:
            return
        self.poll_timer.stop()
        self._cleanup_fd()
        status = "normal" if ret == 0 else "crashed"
        self.finished.emit(ret, status)
        self.proc = None
        self.state_changed.emit()

    def _cleanup_fd(self) -> None:
        if self.notifier is not None:
            self.notifier.setEnabled(False)
            self.notifier.deleteLater()
            self.notifier = None
        if self.master_fd is not None:
            try:
                os.close(self.master_fd)
            except OSError:
                pass
            self.master_fd = None


class FlaggedBagGenerationWorker(QtCore.QObject):
    progress = QtCore.pyqtSignal(int, int, str)
    finished = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, source_bag: str, event_times: List[float], parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self.source_bag = Path(source_bag)
        self.event_times = sorted(event_times)

    def run(self) -> None:
        try:
            target = self.source_bag.with_name(f"{self.source_bag.stem}_with_flag{self.source_bag.suffix}")
            if target.exists():
                target = self.source_bag.with_name(
                    f"{self.source_bag.stem}_with_flag_{int(time.time())}{self.source_bag.suffix}"
                )
            pending_times = list(self.event_times)
            inserted_count = 0
            with rosbag.Bag(str(self.source_bag), "r") as inbag:
                total_messages = int(inbag.get_message_count() or 0)
            self.progress.emit(0, max(total_messages, 1), "Preparing bag rewrite...")
            with rosbag.Bag(str(self.source_bag), "r") as inbag, rosbag.Bag(str(target), "w") as outbag:
                for processed_count, (topic, msg, t) in enumerate(inbag.read_messages(), start=1):
                    while pending_times and pending_times[0] <= t.to_sec():
                        outbag.write("/LIO/set_elevator_flag", Bool(data=True), genpy.Time.from_sec(pending_times.pop(0)))
                        inserted_count += 1
                    outbag.write(topic, msg, t)
                    if processed_count == 1 or processed_count % 500 == 0 or processed_count == total_messages:
                        self.progress.emit(
                            processed_count,
                            max(total_messages, 1),
                            f"Rewriting bag... {processed_count}/{total_messages} messages, flags inserted: {inserted_count}",
                        )
                while pending_times:
                    outbag.write("/LIO/set_elevator_flag", Bool(data=True), genpy.Time.from_sec(pending_times.pop(0)))
                    inserted_count += 1
            self.progress.emit(
                max(total_messages, 1),
                max(total_messages, 1),
                f"Finished. flags inserted: {inserted_count}",
            )
            self.finished.emit(str(target))
        except Exception as exc:
            self.failed.emit(str(exc))


class SettingsDialog(QtWidgets.QDialog):
    def __init__(self, settings: AppSettings, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Settings")
        self.setModal(True)
        self.resize(620, 320)

        layout = QtWidgets.QVBoxLayout(self)
        form = QtWidgets.QFormLayout()

        self.setup_edit = QtWidgets.QLineEdit(settings.setup_script)
        self.lio_edit = QtWidgets.QLineEdit(settings.lio_executable)
        self.libusb_edit = QtWidgets.QLineEdit(settings.libusb_path)
        self.root_config_edit = QtWidgets.QLineEdit(settings.root_config)
        self.build_checkbox = QtWidgets.QCheckBox("Build workspace before starting LIO")
        self.build_checkbox.setChecked(settings.build_before_run)
        self.rviz_checkbox = QtWidgets.QCheckBox(f"Start RViz with {RVIZ_CONFIG}")
        self.rviz_checkbox.setChecked(settings.enable_rviz)

        form.addRow("Setup Script", self.setup_edit)
        form.addRow("LIO Executable", self.lio_edit)
        form.addRow("libusb", self.libusb_edit)
        form.addRow("Root YAML", self.root_config_edit)
        form.addRow("", self.build_checkbox)
        form.addRow("", self.rviz_checkbox)
        layout.addLayout(form)

        info = QtWidgets.QLabel(f"YAML root: {YAML_ROOT}")
        info.setStyleSheet("color: #64748b;")
        layout.addWidget(info)

        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def get_settings(self) -> AppSettings:
        return AppSettings(
            setup_script=self.setup_edit.text().strip(),
            lio_executable=self.lio_edit.text().strip(),
            libusb_path=self.libusb_edit.text().strip(),
            root_config=self.root_config_edit.text().strip() or FIXED_CONFIG_NAME,
            build_before_run=self.build_checkbox.isChecked(),
            enable_rviz=self.rviz_checkbox.isChecked(),
        )


class BatchSelectionDialog(QtWidgets.QDialog):
    def __init__(
        self,
        entries: List[BagEntry],
        batch_groups: List[BatchTestGroup],
        parent: Optional[QtWidgets.QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Batch Test")
        self.resize(860, 620)
        self.entries = list(entries)
        self.entry_ids = {entry.bag_id for entry in self.entries}
        self.batch_groups = [
            BatchTestGroup(
                group_id=group.group_id,
                name=group.name,
                bag_ids=list(group.bag_ids),
                created_at=group.created_at,
                updated_at=group.updated_at,
            )
            for group in batch_groups
        ]
        self.selected_ids: set[str] = set()
        self._handling_item_change = False
        self.marker_icons = {
            name: self._build_marker_icon(name)
            for name in BagRunnerWindow._marker_specs()
        }

        layout = QtWidgets.QVBoxLayout(self)
        info = QtWidgets.QLabel(
            "Select bag entries to test in sequence. Saved batch groups bind reusable test sets. "
            f"Finished batch artifacts are exported under {BATCH_EXPORT_ROOT}."
        )
        info.setObjectName("SubtitleLabel")
        info.setWordWrap(True)
        layout.addWidget(info)

        group_actions = QtWidgets.QHBoxLayout()
        self.batch_group_combo = QtWidgets.QComboBox()
        self.load_group_btn = QtWidgets.QPushButton("Load")
        self.save_group_btn = QtWidgets.QPushButton("Save As Group")
        self.update_group_btn = QtWidgets.QPushButton("Update Group")
        self.delete_group_btn = QtWidgets.QPushButton("Delete Group")
        group_actions.addWidget(QtWidgets.QLabel("Batch Group"))
        group_actions.addWidget(self.batch_group_combo, 1)
        group_actions.addWidget(self.load_group_btn)
        group_actions.addWidget(self.save_group_btn)
        group_actions.addWidget(self.update_group_btn)
        group_actions.addWidget(self.delete_group_btn)
        layout.addLayout(group_actions)

        actions = QtWidgets.QHBoxLayout()
        select_all_btn = QtWidgets.QPushButton("Select All")
        clear_btn = QtWidgets.QPushButton("Clear")
        self.mark_filter_combo = QtWidgets.QComboBox()
        self.mark_filter_combo.addItem("All Marks", "all")
        self.mark_filter_combo.addItem(self.marker_icons["warn"], "Warning", "warn")
        self.mark_filter_combo.addItem(self.marker_icons["ok"], "OK", "ok")
        self.mark_filter_combo.addItem(self.marker_icons["bad"], "Blocked", "bad")
        self.mark_filter_combo.addItem(self.marker_icons["none"], "No Mark", "none")
        self.selection_summary_label = QtWidgets.QLabel("")
        self.selection_summary_label.setObjectName("SubtitleLabel")
        actions.addWidget(select_all_btn)
        actions.addWidget(clear_btn)
        actions.addSpacing(12)
        actions.addWidget(QtWidgets.QLabel("Mark"))
        actions.addWidget(self.mark_filter_combo)
        actions.addStretch(1)
        actions.addWidget(self.selection_summary_label)
        layout.addLayout(actions)

        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderLabels(["Alias", "Group", "Date", "Duration", "Note", "Mark"])
        self.tree.header().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.tree.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(4, QtWidgets.QHeaderView.Stretch)
        self.tree.header().setSectionResizeMode(5, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.itemChanged.connect(self._handle_item_changed)
        layout.addWidget(self.tree, 1)

        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        layout.addWidget(buttons)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        select_all_btn.clicked.connect(lambda: self._set_all_children(QtCore.Qt.Checked))
        clear_btn.clicked.connect(lambda: self._set_all_children(QtCore.Qt.Unchecked))
        self.mark_filter_combo.currentIndexChanged.connect(self._refresh_tree)
        self.batch_group_combo.currentIndexChanged.connect(self._update_group_buttons)
        self.load_group_btn.clicked.connect(self._load_selected_batch_group)
        self.save_group_btn.clicked.connect(self._save_current_selection_as_group)
        self.update_group_btn.clicked.connect(self._update_selected_batch_group)
        self.delete_group_btn.clicked.connect(self._delete_selected_batch_group)

        self._refresh_group_combo()
        self._refresh_tree()

    def _build_marker_icon(self, marker: str) -> QtGui.QIcon:
        symbol, color, _ = BagRunnerWindow._marker_specs().get(marker, BagRunnerWindow._marker_specs()["none"])
        pixmap = QtGui.QPixmap(18, 18)
        pixmap.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(pixmap)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(color)
        painter.drawEllipse(1, 1, 16, 16)
        if symbol:
            font = QtGui.QFont("Noto Sans CJK SC", 10)
            font.setBold(True)
            painter.setFont(font)
            painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff")))
            painter.drawText(pixmap.rect(), QtCore.Qt.AlignCenter, symbol)
        painter.end()
        return QtGui.QIcon(pixmap)

    def _active_group_bag_ids(self, group: BatchTestGroup) -> List[str]:
        return [bag_id for bag_id in group.bag_ids if bag_id in self.entry_ids]

    def _current_batch_group(self) -> Optional[BatchTestGroup]:
        group_id = str(self.batch_group_combo.currentData() or "")
        if not group_id:
            return None
        return next((group for group in self.batch_groups if group.group_id == group_id), None)

    def _refresh_group_combo(self, selected_group_id: str = "") -> None:
        current_group_id = selected_group_id or str(self.batch_group_combo.currentData() or "")
        self.batch_groups.sort(key=lambda group: BagRunnerWindow._natural_sort_key(group.name))
        self.batch_group_combo.blockSignals(True)
        self.batch_group_combo.clear()
        self.batch_group_combo.addItem("Manual Selection", "")
        for group in self.batch_groups:
            active_count = len(self._active_group_bag_ids(group))
            total_count = len(group.bag_ids)
            count_text = f"{active_count}/{total_count}" if active_count != total_count else str(total_count)
            self.batch_group_combo.addItem(f"{group.name} ({count_text})", group.group_id)
        if current_group_id:
            index = self.batch_group_combo.findData(current_group_id)
            if index >= 0:
                self.batch_group_combo.setCurrentIndex(index)
        self.batch_group_combo.blockSignals(False)
        self._update_group_buttons()

    def _update_selection_summary(self) -> None:
        selected_count = len(self.selected_ids)
        self.selection_summary_label.setText(f"{selected_count} selected")

    def _update_group_buttons(self) -> None:
        has_selection = bool(self.selected_ids)
        has_group = self._current_batch_group() is not None
        self.load_group_btn.setEnabled(has_group)
        self.save_group_btn.setEnabled(has_selection)
        self.update_group_btn.setEnabled(has_group and has_selection)
        self.delete_group_btn.setEnabled(has_group)
        self._update_selection_summary()

    def _set_selection(self, bag_ids: List[str]) -> None:
        self.selected_ids = {str(bag_id) for bag_id in bag_ids if str(bag_id) in self.entry_ids}
        self._refresh_tree()

    def _load_selected_batch_group(self) -> None:
        group = self._current_batch_group()
        if group is None:
            return
        missing_count = len(group.bag_ids) - len(self._active_group_bag_ids(group))
        self._set_selection(group.bag_ids)
        if missing_count > 0:
            QtWidgets.QMessageBox.information(
                self,
                "Group Loaded",
                f"Loaded '{group.name}'. {missing_count} deleted or missing bag entry was skipped.",
            )

    def _save_current_selection_as_group(self) -> None:
        selected_bag_ids = self.selected_bag_ids()
        if not selected_bag_ids:
            QtWidgets.QMessageBox.information(self, "No Selection", "Select at least one bag entry first.")
            return
        name, accepted = QtWidgets.QInputDialog.getText(
            self,
            "Create Batch Group",
            "Group name:",
        )
        name = name.strip()
        if not accepted or not name:
            return
        if any(group.name == name for group in self.batch_groups):
            QtWidgets.QMessageBox.warning(
                self,
                "Name Exists",
                "A batch group with this name already exists. Load it and use Update Group instead.",
            )
            return
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        group = BatchTestGroup(
            group_id=uuid.uuid4().hex,
            name=name,
            bag_ids=selected_bag_ids,
            created_at=now,
            updated_at=now,
        )
        self.batch_groups.append(group)
        self._refresh_group_combo(group.group_id)

    def _update_selected_batch_group(self) -> None:
        group = self._current_batch_group()
        if group is None:
            return
        selected_bag_ids = self.selected_bag_ids()
        if not selected_bag_ids:
            QtWidgets.QMessageBox.information(self, "No Selection", "Select at least one bag entry first.")
            return
        group.bag_ids = selected_bag_ids
        group.updated_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._refresh_group_combo(group.group_id)

    def _delete_selected_batch_group(self) -> None:
        group = self._current_batch_group()
        if group is None:
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            "Delete Batch Group",
            f"Delete batch group '{group.name}'?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self.batch_groups = [item for item in self.batch_groups if item.group_id != group.group_id]
        self._refresh_group_combo()

    def _refresh_tree(self) -> None:
        marker_filter = self.mark_filter_combo.currentData()
        filtered_entries = list(self.entries)
        if marker_filter and marker_filter != "all":
            filtered_entries = [entry for entry in filtered_entries if (entry.marker or "none") == marker_filter]
        filtered_entries.sort(
            key=lambda entry: (
                BagRunnerWindow._natural_sort_key(entry.group or "Default"),
                BagRunnerWindow._natural_sort_key(entry.alias),
            ),
        )
        self._handling_item_change = True
        self.tree.clear()
        group_nodes = {}
        for entry in filtered_entries:
            group_name = entry.group or "Default"
            if group_name not in group_nodes:
                parent_item = QtWidgets.QTreeWidgetItem([group_name, "", "", "", "", ""])
                parent_item.setFirstColumnSpanned(True)
                parent_item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsUserCheckable)
                parent_item.setCheckState(0, QtCore.Qt.Unchecked)
                parent_item.setIcon(0, QtWidgets.QApplication.style().standardIcon(QtWidgets.QStyle.SP_ArrowDown))
                font = parent_item.font(0)
                font.setBold(True)
                parent_item.setFont(0, font)
                self.tree.addTopLevelItem(parent_item)
                group_nodes[group_name] = parent_item
            marker_note = entry.marker_note.strip()
            item = QtWidgets.QTreeWidgetItem([
                entry.alias,
                group_name,
                entry.record_date,
                BagRunnerWindow._format_duration(entry.duration_seconds),
                entry.note,
                marker_note,
            ])
            item.setFlags(
                QtCore.Qt.ItemIsEnabled
                | QtCore.Qt.ItemIsSelectable
                | QtCore.Qt.ItemIsUserCheckable
                | QtCore.Qt.ItemNeverHasChildren
            )
            item.setCheckState(0, QtCore.Qt.Checked if entry.bag_id in self.selected_ids else QtCore.Qt.Unchecked)
            item.setData(0, QtCore.Qt.UserRole, entry.bag_id)
            item.setToolTip(0, entry.bag_path)
            item.setToolTip(5, marker_note or BagRunnerWindow._marker_specs().get(entry.marker, ("", "", "No mark"))[2])
            item.setIcon(5, self.marker_icons.get(entry.marker or "none", QtGui.QIcon()))
            group_nodes[group_name].addChild(item)

        for idx in range(self.tree.topLevelItemCount()):
            parent = self.tree.topLevelItem(idx)
            self._update_parent_state(parent)
            parent.setExpanded(True)
        self._handling_item_change = False
        self._update_group_buttons()

    def _set_all_children(self, state: QtCore.Qt.CheckState) -> None:
        self._handling_item_change = True
        for i in range(self.tree.topLevelItemCount()):
            parent = self.tree.topLevelItem(i)
            for j in range(parent.childCount()):
                child = parent.child(j)
                child.setCheckState(0, state)
                bag_id = child.data(0, QtCore.Qt.UserRole)
                if bag_id:
                    if state == QtCore.Qt.Checked:
                        self.selected_ids.add(str(bag_id))
                    else:
                        self.selected_ids.discard(str(bag_id))
            self._update_parent_state(parent)
        self._handling_item_change = False
        self._update_group_buttons()

    def _handle_item_changed(self, item: QtWidgets.QTreeWidgetItem, _column: int) -> None:
        if self._handling_item_change:
            return
        self._handling_item_change = True
        try:
            if item.childCount() > 0:
                state = QtCore.Qt.Unchecked if item.checkState(0) == QtCore.Qt.Unchecked else QtCore.Qt.Checked
                item.setCheckState(0, state)
                for j in range(item.childCount()):
                    child = item.child(j)
                    child.setCheckState(0, state)
                    bag_id = child.data(0, QtCore.Qt.UserRole)
                    if bag_id:
                        if state == QtCore.Qt.Checked:
                            self.selected_ids.add(str(bag_id))
                        else:
                            self.selected_ids.discard(str(bag_id))
                self._update_parent_state(item)
            else:
                bag_id = item.data(0, QtCore.Qt.UserRole)
                if bag_id:
                    if item.checkState(0) == QtCore.Qt.Checked:
                        self.selected_ids.add(str(bag_id))
                    else:
                        self.selected_ids.discard(str(bag_id))
                parent = item.parent()
                if parent is not None:
                    self._update_parent_state(parent)
        finally:
            self._handling_item_change = False
            self._update_group_buttons()

    def _update_parent_state(self, parent: QtWidgets.QTreeWidgetItem) -> None:
        checked = 0
        total = parent.childCount()
        for j in range(total):
            if parent.child(j).checkState(0) == QtCore.Qt.Checked:
                checked += 1
        if checked == 0:
            parent.setCheckState(0, QtCore.Qt.Unchecked)
        elif checked == total:
            parent.setCheckState(0, QtCore.Qt.Checked)
        else:
            parent.setCheckState(0, QtCore.Qt.PartiallyChecked)

    def selected_bag_ids(self) -> List[str]:
        return [entry.bag_id for entry in self.entries if entry.bag_id in self.selected_ids]

    def selected_batch_name(self) -> str:
        selected_ids = set(self.selected_bag_ids())
        if selected_ids:
            for group in self.batch_groups:
                if selected_ids == set(self._active_group_bag_ids(group)):
                    return group.name
        return "Manual Selection"

    def batch_test_groups_result(self) -> List[BatchTestGroup]:
        return [
            BatchTestGroup(
                group_id=group.group_id,
                name=group.name,
                bag_ids=list(group.bag_ids),
                created_at=group.created_at,
                updated_at=group.updated_at,
            )
            for group in self.batch_groups
        ]


class RecoverEntryDialog(QtWidgets.QDialog):
    def __init__(self, entries: List[BagEntry], parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.entries = list(entries)
        self.marker_icons = {
            name: self._build_marker_icon(name)
            for name in BagRunnerWindow._marker_specs()
        }
        self.setWindowTitle("Recover Deleted Entry")
        self.resize(900, 500)

        layout = QtWidgets.QVBoxLayout(self)
        info = QtWidgets.QLabel("Select one deleted bag entry to recover.")
        info.setObjectName("SubtitleLabel")
        layout.addWidget(info)

        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderLabels(["Alias", "Group", "Date", "Duration", "Note", "Mark"])
        self.tree.header().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.tree.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(4, QtWidgets.QHeaderView.Stretch)
        self.tree.header().setSectionResizeMode(5, QtWidgets.QHeaderView.ResizeToContents)
        layout.addWidget(self.tree, 1)

        group_nodes = {}
        sorted_entries = sorted(
            self.entries,
            key=lambda entry: (
                BagRunnerWindow._natural_sort_key(entry.group or "Default"),
                BagRunnerWindow._natural_sort_key(entry.alias),
            ),
        )
        for entry in sorted_entries:
            group_name = entry.group or "Default"
            if group_name not in group_nodes:
                parent_item = QtWidgets.QTreeWidgetItem([group_name, "", "", "", "", ""])
                parent_item.setFirstColumnSpanned(True)
                parent_item.setFlags(parent_item.flags() & ~QtCore.Qt.ItemIsSelectable)
                font = parent_item.font(0)
                font.setBold(True)
                parent_item.setFont(0, font)
                parent_item.setIcon(0, QtWidgets.QApplication.style().standardIcon(QtWidgets.QStyle.SP_ArrowDown))
                self.tree.addTopLevelItem(parent_item)
                group_nodes[group_name] = parent_item
            marker_note = entry.marker_note.strip()
            item = QtWidgets.QTreeWidgetItem([
                entry.alias,
                group_name,
                entry.record_date,
                BagRunnerWindow._format_duration(entry.duration_seconds),
                entry.note,
                marker_note,
            ])
            item.setData(0, QtCore.Qt.UserRole, entry.bag_id)
            item.setToolTip(0, entry.bag_path)
            item.setToolTip(5, marker_note or BagRunnerWindow._marker_specs().get(entry.marker, ("", "", "No mark"))[2])
            item.setIcon(5, self.marker_icons.get(entry.marker or "none", QtGui.QIcon()))
            group_nodes[group_name].addChild(item)

        for idx in range(self.tree.topLevelItemCount()):
            self.tree.topLevelItem(idx).setExpanded(True)

        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        layout.addWidget(buttons)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

    def selected_bag_id(self) -> str:
        item = self.tree.currentItem()
        if item is None:
            return ""
        data = item.data(0, QtCore.Qt.UserRole)
        return str(data) if data else ""

    def _build_marker_icon(self, marker: str) -> QtGui.QIcon:
        symbol, color, _ = BagRunnerWindow._marker_specs().get(marker, BagRunnerWindow._marker_specs()["none"])
        pixmap = QtGui.QPixmap(18, 18)
        pixmap.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(pixmap)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(color)
        painter.drawEllipse(1, 1, 16, 16)
        if symbol:
            font = QtGui.QFont("Noto Sans CJK SC", 10)
            font.setBold(True)
            painter.setFont(font)
            painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff")))
            painter.drawText(pixmap.rect(), QtCore.Qt.AlignCenter, symbol)
        painter.end()
        return QtGui.QIcon(pixmap)


class OtherLioDialog(QtWidgets.QDialog):
    def __init__(self, profiles: List[ExternalLioProfile], parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Start Other LIO")
        self.resize(640, 360)
        self.profiles = list(profiles)

        layout = QtWidgets.QVBoxLayout(self)
        info = QtWidgets.QLabel(
            "Select one external LIO profile. This mode only starts the node and plays the bag. No analysis will be recorded."
        )
        info.setObjectName("SubtitleLabel")
        info.setWordWrap(True)
        layout.addWidget(info)

        self.list_widget = QtWidgets.QListWidget()
        self.list_widget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        for profile in self.profiles:
            item = QtWidgets.QListWidgetItem(profile.display_name)
            item.setData(QtCore.Qt.UserRole, profile.key)
            item.setToolTip(profile.description or profile.launch_command)
            self.list_widget.addItem(item)
        if self.list_widget.count() > 0:
            self.list_widget.setCurrentRow(0)
        layout.addWidget(self.list_widget, 1)

        self.detail_label = QtWidgets.QLabel("")
        self.detail_label.setObjectName("SubtitleLabel")
        self.detail_label.setWordWrap(True)
        layout.addWidget(self.detail_label)

        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        layout.addWidget(buttons)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        self.list_widget.currentItemChanged.connect(self._update_detail)
        self.list_widget.itemDoubleClicked.connect(lambda _item: self.accept())
        self._update_detail()

    def _update_detail(self, *_args) -> None:
        item = self.list_widget.currentItem()
        if item is None:
            self.detail_label.setText("")
            return
        key = str(item.data(QtCore.Qt.UserRole) or "")
        profile = next((profile for profile in self.profiles if profile.key == key), None)
        self.detail_label.setText(profile.description if profile is not None else "")

    def selected_profile(self) -> Optional[ExternalLioProfile]:
        item = self.list_widget.currentItem()
        if item is None:
            return None
        key = str(item.data(QtCore.Qt.UserRole) or "")
        for profile in self.profiles:
            if profile.key == key:
                return profile
        return None


class BagRunnerWindow(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.entries: List[BagEntry] = []
        self.batch_test_groups: List[BatchTestGroup] = []
        self.analysis_records: List[AnalysisRecord] = []
        self.settings = AppSettings()
        self.started_roscore = False
        self.pending_entry: Optional[BagEntry] = None
        self.session_running = False
        self.stopping_session = False
        self.session_start_epoch = 0.0
        self.current_log_session_dir: Optional[Path] = None
        self.stop_deadline_wall = 0.0
        self.wait_elapsed_ms = 0
        self.playback_started_wall = 0.0
        self.playback_paused = False
        self.pause_started_wall = 0.0
        self.total_paused_wall = 0.0
        self.current_bag_start_time = 0.0
        self.current_bag_end_time = 0.0
        self.flag_event_times: List[float] = []
        self.current_pause_service = ""
        self.current_analysis_index = -1
        self.lio_ready_detected = False
        self.lio_startup_progress_detected = False
        self.lio_startup_error_detected = False
        self.visible_analysis_indices: List[int] = []
        self.close_requested = False
        self.flag_generation_thread: Optional[QtCore.QThread] = None
        self.flag_generation_worker: Optional[FlaggedBagGenerationWorker] = None
        self.flag_generation_dialog: Optional[QtWidgets.QProgressDialog] = None
        self.flag_generation_bag_id = ""
        self.flag_generation_analysis_index = -1
        self.flag_generation_event_times: List[float] = []
        self.visible_entry_indices: List[int] = []
        self.batch_mode_active = False
        self.batch_cancelled = False
        self.batch_queue_ids: List[str] = []
        self.batch_run_ids: List[str] = []
        self.batch_total_count = 0
        self.batch_completed_count = 0
        self.batch_results: List[str] = []
        self.batch_result_by_bag_id: Dict[str, str] = {}
        self.batch_group_name = ""
        self.batch_started_at = ""
        self.batch_new_record_keys: List[Tuple[str, str]] = []
        self.yaml_favorites: List[str] = []
        self.yaml_favorite_aliases: Dict[str, str] = {}
        self._entry_selection_guard = False
        self.external_lio_profiles = [ExternalLioProfile(**profile) for profile in OTHER_LIO_PROFILES]
        self.current_external_lio_profile: Optional[ExternalLioProfile] = None
        self.current_session_records_analysis = True
        self.waiting_for_lio_analysis = False

        self.build_proc = ManagedProcess("build", self)
        self.roscore_proc = ManagedProcess("roscore", self)
        self.lio_proc = ManagedProcess("lio", self)
        self.rviz_proc = ManagedProcess("rviz", self)
        self.play_proc = PtyManagedProcess("rosbag", self)

        self.node_wait_timer = QtCore.QTimer(self)
        self.node_wait_timer.setInterval(1000)
        self.node_wait_timer.timeout.connect(self._poll_lio_node)
        self.stop_timer = QtCore.QTimer(self)
        self.stop_timer.setInterval(200)
        self.stop_timer.timeout.connect(self._poll_stop_session)
        self.status_eta_timer = QtCore.QTimer(self)
        self.status_eta_timer.setInterval(1000)
        self.status_eta_timer.timeout.connect(self._update_runtime_status_eta)

        self._setup_ui()
        self._connect_processes()
        self._load_state()
        self.fixed_yaml_badge.setText(f"Root YAML: {self.settings.root_config}")
        self._refresh_table()
        self._refresh_analysis_table()
        self._update_buttons()

    def _group_indicator_icon(self, expanded: bool) -> QtGui.QIcon:
        style = QtWidgets.QApplication.style()
        pixmap = QtWidgets.QStyle.SP_ArrowDown if expanded else QtWidgets.QStyle.SP_ArrowRight
        return style.standardIcon(pixmap)

    def _set_group_item_indicator(self, item: QtWidgets.QTreeWidgetItem, expanded: bool) -> None:
        if item is None or item.childCount() == 0:
            return
        item.setIcon(0, self._group_indicator_icon(expanded))

    def _setup_ui(self) -> None:
        self.setWindowTitle("Bag Runner")
        self.resize(1360, 860)
        self.setStyleSheet(STYLESHEET)

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(16)

        header = QtWidgets.QFrame()
        header.setObjectName("Card")
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(22, 18, 22, 18)

        header_text = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel("Bag Runner")
        title.setObjectName("TitleLabel")
        subtitle = QtWidgets.QLabel("管理常用 rosbag，记录别名和备注，一键编译后启动 LIO 并播包。")
        subtitle.setObjectName("SubtitleLabel")
        header_text.addWidget(title)
        header_text.addWidget(subtitle)
        header_layout.addLayout(header_text, 1)

        self.fixed_yaml_badge = QtWidgets.QLabel(f"Root YAML: {self.settings.root_config}")
        self.fixed_yaml_badge.setStyleSheet(
            "background: #e1e8eb; color: #425b70; border: 1px solid #cfdbe1; border-radius: 10px; padding: 8px 12px; font-weight: 700;"
        )
        header_layout.addWidget(self.fixed_yaml_badge)

        self.settings_btn = QtWidgets.QPushButton("Settings")
        header_layout.addWidget(self.settings_btn)
        root.addWidget(header)

        body = QtWidgets.QHBoxLayout()
        body.setSpacing(16)
        root.addLayout(body, 1)

        left_card = QtWidgets.QFrame()
        left_card.setObjectName("Card")
        left_layout = QtWidgets.QVBoxLayout(left_card)
        left_layout.setContentsMargins(18, 18, 18, 18)
        left_layout.setSpacing(12)

        left_title = QtWidgets.QLabel("Bag List")
        left_title.setStyleSheet("font-size: 18px; font-weight: 700;")
        left_layout.addWidget(left_title)

        list_controls = QtWidgets.QHBoxLayout()
        self.group_filter_combo = QtWidgets.QComboBox()
        self.sort_combo = QtWidgets.QComboBox()
        self.sort_combo.addItems(["Group", "Alias", "Date", "Duration", "Analyses", "Note", "Mark", "Manual Trigger"])
        list_controls.addWidget(QtWidgets.QLabel("Group"))
        list_controls.addWidget(self.group_filter_combo, 1)
        list_controls.addWidget(QtWidgets.QLabel("Sort By"))
        list_controls.addWidget(self.sort_combo, 1)
        left_layout.addLayout(list_controls)

        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderLabels(["Alias", "Date", "Duration", "Analyses", "Note", "Manual Trigger", "Mark"])
        self.tree.header().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(4, QtWidgets.QHeaderView.Stretch)
        self.tree.header().setSectionResizeMode(5, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(6, QtWidgets.QHeaderView.ResizeToContents)
        self.tree.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.tree.setAlternatingRowColors(True)
        self.tree.setIndentation(12)
        self.tree.currentItemChanged.connect(self._handle_entry_current_item_changed)
        self.tree.itemExpanded.connect(lambda item: self._set_group_item_indicator(item, True))
        self.tree.itemCollapsed.connect(lambda item: self._set_group_item_indicator(item, False))
        left_layout.addWidget(self.tree, 1)

        table_actions = QtWidgets.QHBoxLayout()
        self.add_btn = QtWidgets.QPushButton("Add")
        self.update_btn = QtWidgets.QPushButton("Update")
        self.remove_btn = QtWidgets.QPushButton("Remove")
        self.clear_btn = QtWidgets.QPushButton("Recover")
        self.batch_btn = QtWidgets.QPushButton("Batch Test")
        table_actions.addWidget(self.add_btn)
        table_actions.addWidget(self.update_btn)
        table_actions.addWidget(self.remove_btn)
        table_actions.addWidget(self.clear_btn)
        table_actions.addWidget(self.batch_btn)
        left_layout.addLayout(table_actions)

        analysis_card = QtWidgets.QFrame()
        analysis_card.setObjectName("Card")
        analysis_layout = QtWidgets.QVBoxLayout(analysis_card)
        analysis_layout.setContentsMargins(18, 18, 18, 18)
        analysis_layout.setSpacing(12)

        analysis_header = QtWidgets.QHBoxLayout()
        analysis_title = QtWidgets.QLabel("Analysis DB")
        analysis_title.setStyleSheet("font-size: 18px; font-weight: 700;")
        self.export_analysis_btn = QtWidgets.QPushButton("Export Artifacts")
        analysis_header.addWidget(analysis_title, 1)
        analysis_header.addWidget(self.export_analysis_btn)
        analysis_layout.addLayout(analysis_header)

        self.analysis_table = QtWidgets.QTreeWidget()
        self.analysis_table.setHeaderLabels(
            ["Finished", "Count", "Total Z", "Total", "End Err", "End Z Err", "X", "Y", "Z", "Img"]
        )
        self.analysis_table.header().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.analysis_table.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.analysis_table.header().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.analysis_table.header().setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        self.analysis_table.header().setSectionResizeMode(4, QtWidgets.QHeaderView.ResizeToContents)
        self.analysis_table.header().setSectionResizeMode(5, QtWidgets.QHeaderView.ResizeToContents)
        self.analysis_table.header().setSectionResizeMode(6, QtWidgets.QHeaderView.ResizeToContents)
        self.analysis_table.header().setSectionResizeMode(7, QtWidgets.QHeaderView.ResizeToContents)
        self.analysis_table.header().setSectionResizeMode(8, QtWidgets.QHeaderView.ResizeToContents)
        self.analysis_table.header().setSectionResizeMode(9, QtWidgets.QHeaderView.ResizeToContents)
        self.analysis_table.setRootIsDecorated(True)
        self.analysis_table.setItemsExpandable(True)
        self.analysis_table.setUniformRowHeights(True)
        self.analysis_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.analysis_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.analysis_table.setWordWrap(False)
        self.analysis_table.setIndentation(12)
        self.analysis_table.itemSelectionChanged.connect(self._load_selected_analysis_detail)
        self.analysis_table.itemDoubleClicked.connect(self._show_selected_analysis_detail_dialog)
        self.analysis_table.itemExpanded.connect(lambda item: self._set_group_item_indicator(item, True))
        self.analysis_table.itemCollapsed.connect(lambda item: self._set_group_item_indicator(item, False))
        analysis_layout.addWidget(self.analysis_table, 1)

        right_panel = QtWidgets.QVBoxLayout()
        right_panel.setSpacing(16)

        form_card = QtWidgets.QFrame()
        form_card.setObjectName("Card")
        form_layout = QtWidgets.QVBoxLayout(form_card)
        form_layout.setContentsMargins(18, 18, 18, 18)
        form_layout.setSpacing(14)

        form_title = QtWidgets.QLabel("Entry Detail")
        form_title.setStyleSheet("font-size: 18px; font-weight: 700;")
        form_layout.addWidget(form_title)

        grid = QtWidgets.QFormLayout()
        grid.setLabelAlignment(QtCore.Qt.AlignLeft)
        grid.setFormAlignment(QtCore.Qt.AlignTop)
        grid.setSpacing(12)

        self.alias_edit = QtWidgets.QLineEdit()
        self.group_edit = QtWidgets.QLineEdit()
        self.bag_path_edit = QtWidgets.QLineEdit()
        self.marker_combo = QtWidgets.QComboBox()
        self.marker_combo.addItem("", "none")
        self.marker_combo.addItem("", "warn")
        self.marker_combo.addItem("", "ok")
        self.marker_combo.addItem("", "bad")
        self._init_marker_icons()
        self.marker_note_edit = QtWidgets.QLineEdit()
        self.marker_note_edit.setPlaceholderText("Mark note")
        self.rate_spin = QtWidgets.QDoubleSpinBox()
        self.rate_spin.setRange(0.1, 50.0)
        self.rate_spin.setDecimals(2)
        self.rate_spin.setSingleStep(0.5)
        self.rate_spin.setValue(DEFAULT_RATE)
        self.note_edit = QtWidgets.QPlainTextEdit()
        self.note_edit.setPlaceholderText("记录这个包的用途、场景、楼层或实验备注。")
        self.note_edit.setFixedHeight(130)
        for widget in [self.alias_edit, self.group_edit, self.bag_path_edit, self.marker_note_edit, self.note_edit]:
            widget.setAttribute(QtCore.Qt.WA_InputMethodEnabled, True)
            if isinstance(widget, QtWidgets.QLineEdit):
                widget.setInputMethodHints(QtCore.Qt.ImhNone)
            else:
                widget.setInputMethodHints(QtCore.Qt.ImhNone)

        bag_row = QtWidgets.QHBoxLayout()
        bag_row.setContentsMargins(0, 0, 0, 0)
        bag_row.setSpacing(8)
        self.bag_browse_btn = QtWidgets.QPushButton("Browse")
        self.bag_browse_btn.setMinimumHeight(self.bag_path_edit.sizeHint().height())
        self.bag_browse_btn.setMinimumWidth(84)
        bag_row.addWidget(self.bag_path_edit, 1)
        bag_row.addWidget(self.bag_browse_btn)

        mark_row = QtWidgets.QHBoxLayout()
        mark_row.setContentsMargins(0, 0, 0, 0)
        mark_row.setSpacing(8)
        mark_row.addWidget(self.marker_combo, 1)
        mark_row.addWidget(self.marker_note_edit, 1)

        grid.addRow("Group", self.group_edit)
        grid.addRow("Alias", self.alias_edit)
        grid.addRow("Bag Path", self._wrap_layout(bag_row))
        grid.addRow("Mark", self._wrap_layout(mark_row))
        grid.addRow("Play Rate", self.rate_spin)
        grid.addRow("Note", self.note_edit)
        form_layout.addLayout(grid)

        action_row = QtWidgets.QHBoxLayout()
        self.start_btn = QtWidgets.QPushButton("Build + Start LIO + Play Bag")
        self.start_btn.setObjectName("PrimaryButton")
        self.other_btn = QtWidgets.QPushButton("Others")
        self.pause_btn = QtWidgets.QPushButton("Pause Bag")
        self.flag_btn = QtWidgets.QPushButton("Set Elevator Flag")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.stop_btn.setObjectName("DangerButton")
        action_row.addWidget(self.start_btn, 1)
        action_row.addWidget(self.other_btn)
        action_row.addWidget(self.pause_btn)
        action_row.addWidget(self.flag_btn)
        action_row.addWidget(self.stop_btn)
        form_layout.addLayout(action_row)

        self.status_label = QtWidgets.QLabel("Idle")
        self.status_label.setObjectName("StatusBadge")
        form_layout.addWidget(self.status_label)

        logs_card = QtWidgets.QFrame()
        logs_card.setObjectName("Card")
        logs_layout = QtWidgets.QVBoxLayout(logs_card)
        logs_layout.setContentsMargins(18, 18, 18, 18)
        logs_layout.setSpacing(12)

        logs_title = QtWidgets.QLabel("Output")
        logs_title.setStyleSheet("font-size: 18px; font-weight: 700;")
        logs_layout.addWidget(logs_title)

        self.log_tabs = QtWidgets.QTabWidget()
        self.lio_log_edit = QtWidgets.QPlainTextEdit()
        self.rosbag_log_edit = QtWidgets.QPlainTextEdit()
        for edit in [self.lio_log_edit, self.rosbag_log_edit]:
            edit.setReadOnly(True)
            edit.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
            edit.setFont(QtGui.QFont("DejaVu Sans Mono", 10))
        self.log_tabs.addTab(self.lio_log_edit, "LIO Output")
        self.log_tabs.addTab(self.rosbag_log_edit, "Rosbag Output")
        logs_layout.addWidget(self.log_tabs, 1)

        right_panel.addWidget(form_card, 0)
        right_panel.addWidget(logs_card, 1)

        left_stack = QtWidgets.QVBoxLayout()
        left_stack.setSpacing(16)
        left_stack.addWidget(left_card, 5)
        left_stack.addWidget(analysis_card, 3)

        body.addLayout(left_stack, 6)
        body.addLayout(right_panel, 5)

        self.add_btn.clicked.connect(self._add_entry)
        self.update_btn.clicked.connect(self._update_entry)
        self.remove_btn.clicked.connect(self._remove_entry)
        self.clear_btn.clicked.connect(self._recover_entry)
        self.batch_btn.clicked.connect(self._open_batch_test_dialog)
        self.group_filter_combo.currentTextChanged.connect(self._refresh_table)
        self.sort_combo.currentTextChanged.connect(self._refresh_table)
        self.bag_browse_btn.clicked.connect(self._browse_bag)
        self.start_btn.clicked.connect(self._start_selected_entry)
        self.other_btn.clicked.connect(self._start_other_lio_entry)
        self.pause_btn.clicked.connect(self._toggle_pause_playback)
        self.flag_btn.clicked.connect(self._publish_elevator_flag)
        self.stop_btn.clicked.connect(self._stop_session)
        self.settings_btn.clicked.connect(self._open_settings)
        self.export_analysis_btn.clicked.connect(self._export_selected_analysis_artifacts)

        self.pause_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Space), self)
        self.pause_shortcut.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
        self.pause_shortcut.activated.connect(self._handle_space_shortcut)
        self.delete_analysis_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Delete), self.analysis_table)
        self.delete_analysis_shortcut.setContext(QtCore.Qt.WidgetShortcut)
        self.delete_analysis_shortcut.activated.connect(self._remove_selected_analysis_record)

    def _connect_processes(self) -> None:
        self.build_proc.output.connect(self._append_lio_log)
        self.roscore_proc.output.connect(self._append_lio_log)
        self.lio_proc.output.connect(self._append_lio_log)
        self.rviz_proc.output.connect(self._append_lio_log)
        self.play_proc.output.connect(self._append_rosbag_log)

        self.build_proc.finished.connect(self._on_build_finished)
        self.roscore_proc.finished.connect(self._on_roscore_finished)
        self.lio_proc.finished.connect(self._on_lio_finished)
        self.rviz_proc.finished.connect(self._on_rviz_finished)
        self.play_proc.finished.connect(self._on_play_finished)

        for proc in [self.build_proc, self.roscore_proc, self.lio_proc, self.rviz_proc, self.play_proc]:
            proc.state_changed.connect(self._update_buttons)

    def _wrap_layout(self, layout: QtWidgets.QLayout) -> QtWidgets.QWidget:
        widget = QtWidgets.QFrame()
        widget.setObjectName("InlineField")
        widget.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        widget.setLayout(layout)
        return widget

    def _append_log(self, widget: QtWidgets.QPlainTextEdit, text: str) -> None:
        for line in text.splitlines():
            widget.appendPlainText(line)
        scrollbar = widget.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _append_lio_log(self, text: str) -> None:
        self._append_log(self.lio_log_edit, text)
        ansi_free_text = re.sub(r"\x1b\[[0-9;]*m", "", text)
        session_match = re.search(r"log session dir:\s*(.+)", ansi_free_text)
        if self.session_running and session_match:
            session_path = Path(session_match.group(1).strip()).expanduser()
            self.current_log_session_dir = session_path
            self._append_log(self.lio_log_edit, f"[info] bound run artifacts to {session_path}")
        startup_markers = (
            "Config path:",
            "Loading config from:",
            "LIO Configuration",
            "log session dir:",
            "Current NodeHandler namespace:",
            "LIONode is running!",
        )
        if self.session_running and any(marker in text for marker in startup_markers):
            self.lio_startup_progress_detected = True
        external_progress_markers = (
            "started core service [/rosout]",
            "process[laserMapping-",
            "process[rviz-",
            "started with pid",
        )
        if (
            self.session_running
            and self.current_external_lio_profile is not None
            and any(marker in text for marker in external_progress_markers)
        ):
            self.lio_startup_progress_detected = True
        external_error_markers = (
            "symbol lookup error",
            "RLException",
            "[FATAL]",
            "Segmentation fault",
            "Traceback (most recent call last):",
        )
        if self.session_running and any(marker in text for marker in external_error_markers):
            self.lio_startup_error_detected = True
        external_node_name = self._current_external_ready_node_name()
        external_ready_markers = tuple(
            marker
            for marker in (
                f"process[{external_node_name}-" if external_node_name else "",
                f"Started node [/{external_node_name}]" if external_node_name else "",
                f"[/{external_node_name}]" if external_node_name else "",
            )
            if marker
        )
        if (
            self.session_running
            and self.current_external_lio_profile is not None
            and self.node_wait_timer.isActive()
            and not self.lio_ready_detected
            and any(marker in text for marker in external_ready_markers)
        ):
            self._handle_lio_ready_detected()
        if (
            self.session_running
            and self.node_wait_timer.isActive()
            and not self.lio_ready_detected
            and (
                "Current NodeHandler namespace:" in text
                or "LIONode is running!" in text
            )
        ):
            self._handle_lio_ready_detected()

    def _append_rosbag_log(self, text: str) -> None:
        self._append_log(self.rosbag_log_edit, text)

    def _lio_log_tail(self, max_lines: int = 12) -> str:
        text = self.lio_log_edit.toPlainText().strip()
        if not text:
            return ""
        lines = text.splitlines()
        return "\n".join(lines[-max_lines:])

    def _current_session_setup_script(self) -> str:
        if self.current_external_lio_profile is not None:
            return self.current_external_lio_profile.setup_script
        return self.settings.setup_script

    def _current_external_ready_node_name(self) -> str:
        if self.current_external_lio_profile is None:
            return ""
        match = re.fullmatch(r"\^/([^$]+)\$", self.current_external_lio_profile.ready_node_pattern)
        if match is None:
            return ""
        return match.group(1)

    def _handle_space_shortcut(self) -> None:
        focus = QtWidgets.QApplication.focusWidget()
        if isinstance(focus, (QtWidgets.QLineEdit, QtWidgets.QPlainTextEdit, QtWidgets.QAbstractSpinBox)):
            return
        self._toggle_pause_playback()

    def _status_style(self, text: str) -> str:
        normalized = text.strip().lower()
        background = "#ece8df"
        foreground = "#516173"
        border = "#d7d0c4"
        if "paused" in normalized:
            background = "#f5e6c7"
            foreground = "#84561a"
            border = "#e6d1a0"
        elif "playing" in normalized:
            background = "#dceadf"
            foreground = "#2b6240"
            border = "#c0d4c3"
        elif "building workspace" in normalized or "starting" in normalized:
            background = "#dce6ee"
            foreground = "#365b79"
            border = "#c6d3df"
        elif "finished" in normalized or "completed" in normalized:
            background = "#d9e9de"
            foreground = "#2b5c42"
            border = "#bcd1c4"
        elif "failed" in normalized or "error" in normalized:
            background = "#f2dedd"
            foreground = "#873838"
            border = "#ddc0be"
        elif "stopping" in normalized or "stopped" in normalized:
            background = "#ebe1d6"
            foreground = "#77553a"
            border = "#d6c6b8"
        return (
            "QLabel#StatusBadge {"
            f"background: {background};"
            f"color: {foreground};"
            f"border: 1px solid {border};"
            "border-radius: 10px;"
            "padding: 8px 12px;"
            "font-weight: 700;"
            "}"
        )

    def _set_status(self, text: str, log: bool = True) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(self._status_style(text))
        if log:
            self._append_lio_log(f"[status] {text}")

    def _status_with_batch_progress(self, text: str) -> str:
        if not self.batch_mode_active or self.batch_total_count <= 0:
            return text
        current_index = min(self.batch_completed_count + 1, self.batch_total_count)
        return f"Batch {current_index}/{self.batch_total_count} | {text}"

    def _format_eta_text(self, seconds: float) -> str:
        seconds = max(0.0, seconds)
        whole = int(round(seconds))
        hours = whole // 3600
        minutes = (whole % 3600) // 60
        secs = whole % 60
        if hours > 0:
            return f"{hours:d}:{minutes:02d}:{secs:02d}"
        return f"{minutes:02d}:{secs:02d}"

    def _entry_remaining_wall_seconds(self, entry: Optional[BagEntry], current_progress_only: bool = False) -> float:
        if entry is None:
            return 0.0
        total_wall = 0.0
        duration = max(0.0, entry.duration_seconds)
        rate = max(0.1, float(entry.play_rate))
        total_wall = duration / rate if duration > 0.0 else 0.0
        if current_progress_only and self.playback_started_wall > 0.0:
            bag_time = self._current_bag_time()
            if bag_time is not None and self.current_bag_end_time > self.current_bag_start_time:
                bag_remaining = max(0.0, self.current_bag_end_time - bag_time)
                total_wall = bag_remaining / rate
        return total_wall

    def _remaining_batch_wall_seconds(self) -> float:
        remaining = 0.0
        remaining += self._entry_remaining_wall_seconds(self.pending_entry, current_progress_only=True)
        for bag_id in self.batch_queue_ids:
            entry = self._find_entry_by_bag_id(bag_id)
            remaining += self._entry_remaining_wall_seconds(entry)
        return remaining

    def _batch_eta_suffix(self, include_current_pending: bool = True) -> str:
        if not self.batch_mode_active:
            return ""
        remaining = 0.0
        if include_current_pending:
            remaining += self._entry_remaining_wall_seconds(self.pending_entry)
        for bag_id in self.batch_queue_ids:
            entry = self._find_entry_by_bag_id(bag_id)
            remaining += self._entry_remaining_wall_seconds(entry)
        if remaining <= 0.0:
            return ""
        return f" | Batch left {self._format_eta_text(remaining)}"

    def _update_runtime_status_eta(self) -> None:
        if not self.session_running or self.stopping_session:
            self.status_eta_timer.stop()
            return
        if self.pending_entry is not None and self.play_proc.is_running():
            bag_remaining = self._entry_remaining_wall_seconds(self.pending_entry, current_progress_only=True)
            if self.playback_paused:
                status = self._status_with_batch_progress(
                    f"Playback paused | ETA {self._format_eta_text(bag_remaining)}"
                )
            else:
                rate = self.pending_entry.play_rate
                status = self._status_with_batch_progress(
                    f"Playing bag at {rate:.2f}x | ETA {self._format_eta_text(bag_remaining)}"
                )
            if self.batch_mode_active:
                total_remaining = self._remaining_batch_wall_seconds()
                status += f" | Batch left {self._format_eta_text(total_remaining)}"
            self._set_status(status, log=False)

    @staticmethod
    def _natural_sort_key(text: str):
        return [int(part) if part.isdigit() else part.lower() for part in re.split(r"(\d+)", text or "")]

    def _load_state(self) -> None:
        self.entries = []
        self.batch_test_groups = []
        self.analysis_records = []
        self.settings = AppSettings()
        DB_PATH.parent.mkdir(parents=True, exist_ok=True)
        if not DB_PATH.exists():
            return
        try:
            raw = json.loads(DB_PATH.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError) as exc:
            self._append_lio_log(f"[warn] failed to read {DB_PATH}: {exc}")
            return

        settings_raw = raw.get("settings", {})
        try:
            self.settings = AppSettings(**settings_raw)
        except TypeError:
            self.settings = AppSettings()
        yaml_favorites_raw = raw.get("yaml_favorites", [])
        if isinstance(yaml_favorites_raw, list):
            self.yaml_favorites = [str(item) for item in yaml_favorites_raw if str(item).strip()]
        else:
            self.yaml_favorites = []
        yaml_favorite_aliases_raw = raw.get("yaml_favorite_aliases", {})
        if isinstance(yaml_favorite_aliases_raw, dict):
            self.yaml_favorite_aliases = {
                str(key): str(value).strip()
                for key, value in yaml_favorite_aliases_raw.items()
                if str(key).strip() and str(value).strip()
            }
        else:
            self.yaml_favorite_aliases = {}

        for item in raw.get("batch_test_groups", []):
            if not isinstance(item, dict):
                continue
            name = str(item.get("name", "")).strip()
            if not name:
                continue
            bag_ids_raw = item.get("bag_ids", [])
            bag_ids = [str(bag_id) for bag_id in bag_ids_raw] if isinstance(bag_ids_raw, list) else []
            group_id = str(item.get("group_id", "")).strip() or uuid.uuid4().hex
            created_at = str(item.get("created_at", "")).strip()
            updated_at = str(item.get("updated_at", "")).strip()
            self.batch_test_groups.append(
                BatchTestGroup(
                    group_id=group_id,
                    name=name,
                    bag_ids=bag_ids,
                    created_at=created_at,
                    updated_at=updated_at,
                )
            )
        self.batch_test_groups.sort(key=lambda group: self._natural_sort_key(group.name))

        entries_metadata_updated = False
        analyses_metadata_updated = False
        for item in raw.get("entries", []):
            if "config_name" in item:
                item = {
                    "bag_id": item.get("bag_id", uuid.uuid4().hex),
                    "alias": item.get("alias", ""),
                    "bag_path": item.get("bag_path", ""),
                    "play_rate": item.get("play_rate", DEFAULT_RATE),
                    "note": item.get("note", ""),
                    "has_elevator_flag": item.get("has_elevator_flag", False),
                    "group": item.get("group", "Default"),
                    "marker": item.get("marker", "none"),
                    "marker_note": item.get("marker_note", ""),
                    "record_date": item.get("record_date", ""),
                    "duration_seconds": item.get("duration_seconds", 0.0),
                    "deleted": item.get("deleted", False),
                }
            if "has_elevator_flag" not in item:
                item["has_elevator_flag"] = False
            if "bag_id" not in item or not item["bag_id"]:
                item["bag_id"] = uuid.uuid4().hex
            if "group" not in item or not item["group"]:
                item["group"] = "Default"
            if "marker" not in item or not item["marker"]:
                item["marker"] = "none"
            if "marker_note" not in item:
                item["marker_note"] = ""
            if "record_date" not in item:
                item["record_date"] = ""
            if "duration_seconds" not in item:
                item["duration_seconds"] = 0.0
            if "deleted" not in item:
                item["deleted"] = False
            try:
                entry = BagEntry(**item)
            except TypeError:
                continue
            entries_metadata_updated = self._hydrate_entry_metadata(entry) or entries_metadata_updated
            self.entries.append(entry)

        self.entries.sort(
            key=lambda entry: (
                self._natural_sort_key(entry.group or "Default"),
                self._natural_sort_key(entry.alias),
            )
        )

        for item in raw.get("analyses", []):
            if "bag_id" not in item or not item["bag_id"]:
                item["bag_id"] = self._find_entry_id_by_path(
                    item.get("generated_bag_path") or item.get("bag_path", "")
                )
                analyses_metadata_updated = True
            elif not self._entry_exists_for_bag_id(item["bag_id"]):
                rebound_bag_id = self._find_entry_id_by_path(
                    item.get("generated_bag_path") or item.get("bag_path", "")
                )
                if rebound_bag_id:
                    item["bag_id"] = rebound_bag_id
                    analyses_metadata_updated = True
            if "lio_yaml_name" not in item or not item["lio_yaml_name"]:
                item["lio_yaml_name"] = self._resolve_effective_lio_yaml_name()
            if "yaml_snapshot" not in item or item["yaml_snapshot"] is None:
                item["yaml_snapshot"] = self._load_yaml_snapshot_by_name(item["lio_yaml_name"])
                analyses_metadata_updated = True
            if "yaml_group_name" not in item:
                item["yaml_group_name"] = ""
                analyses_metadata_updated = True
            if "yaml_diff_keys" not in item or item["yaml_diff_keys"] is None:
                item["yaml_diff_keys"] = []
                analyses_metadata_updated = True
            if "flag_event_times" not in item:
                item["flag_event_times"] = []
            if "terminal_error" not in item:
                item["terminal_error"] = 0.0
            if "terminal_z_error" not in item:
                item["terminal_z_error"] = 0.0
            if "total_path_length" not in item:
                item["total_path_length"] = 0.0
            if "terminal_pose" not in item or item["terminal_pose"] is None:
                item["terminal_pose"] = []
            if "attachments" not in item or item["attachments"] is None:
                item["attachments"] = []
            if "comment" not in item or item["comment"] is None:
                item["comment"] = ""
            expected_delta_total = self._compute_delta_z_total(item.get("delta_z_values", []))
            if not math.isclose(
                float(item.get("delta_z_total", 0.0)),
                expected_delta_total,
                rel_tol=1e-9,
                abs_tol=1e-9,
            ):
                item["delta_z_total"] = expected_delta_total
                analyses_metadata_updated = True
            if "alias" in item:
                item.pop("alias", None)
            if "source_file" in item:
                item.pop("source_file", None)
            try:
                self.analysis_records.append(AnalysisRecord(**item))
            except TypeError:
                continue
        for bag_id in {record.bag_id for record in self.analysis_records if record.bag_id}:
            self._rebuild_yaml_groups_for_bag(bag_id)
            analyses_metadata_updated = True
        if entries_metadata_updated or analyses_metadata_updated:
            self._save_state()

    @staticmethod
    def _compute_delta_z_total(delta_z_values: List[float]) -> float:
        return sum(abs(value) for value in delta_z_values)

    @staticmethod
    def _format_duration(seconds: float) -> str:
        total_seconds = max(0.0, seconds)
        whole_seconds = int(total_seconds)
        hours = whole_seconds // 3600
        minutes = (whole_seconds % 3600) // 60
        secs = whole_seconds % 60
        if hours > 0:
            return f"{hours:d}:{minutes:02d}:{secs:02d}"
        return f"{minutes:02d}:{secs:02d}"

    @staticmethod
    def _read_bag_metadata(bag_path: Path) -> Tuple[str, float]:
        if not bag_path.is_file():
            return "", 0.0
        with rosbag.Bag(str(bag_path), "r") as bag:
            start_time = bag.get_start_time()
            end_time = bag.get_end_time()
        record_date = datetime.fromtimestamp(start_time).strftime("%Y-%m-%d %H:%M:%S")
        duration_seconds = max(0.0, end_time - start_time)
        return record_date, duration_seconds

    def _hydrate_entry_metadata(self, entry: BagEntry) -> bool:
        if entry.record_date and entry.duration_seconds > 0.0:
            return False
        try:
            record_date, duration_seconds = self._read_bag_metadata(Path(entry.bag_path))
        except Exception:
            return False
        entry.record_date = record_date
        entry.duration_seconds = duration_seconds
        return True

    def _find_entry_id_by_path(self, bag_path: str) -> str:
        normalized = str(Path(bag_path)) if bag_path else ""
        for entry in self.entries:
            if str(Path(entry.bag_path)) == normalized:
                return entry.bag_id
        return ""

    def _entry_exists_for_bag_id(self, bag_id: str) -> bool:
        if not bag_id:
            return False
        return any(entry.bag_id == bag_id for entry in self.entries)

    def _analysis_count_for_bag(self, bag_id: str) -> int:
        if not bag_id:
            return 0
        return sum(1 for record in self.analysis_records if record.bag_id == bag_id)

    def _root_config_path(self) -> Path:
        configured = Path(self.settings.root_config or FIXED_CONFIG_NAME).expanduser()
        return configured.resolve() if configured.is_absolute() else (YAML_ROOT / configured).resolve()

    def _root_config_argument(self) -> Optional[str]:
        try:
            return str(self._root_config_path().relative_to(YAML_ROOT.resolve()))
        except ValueError:
            return None

    def _resolve_effective_lio_yaml_name(self) -> str:
        return self._root_config_argument() or (self.settings.root_config or FIXED_CONFIG_NAME)

    @staticmethod
    def _merge_yaml_mapping(target: dict, source: dict) -> None:
        for key, value in source.items():
            if key in target and isinstance(target[key], dict) and isinstance(value, dict):
                BagRunnerWindow._merge_yaml_mapping(target[key], value)
            else:
                target[key] = value

    def _load_split_yaml_snapshot(self, root_yaml_path: Path, yaml_base: Optional[Path] = None) -> dict:
        root_data = self._load_yaml_file(root_yaml_path)
        if not isinstance(root_data, dict):
            return {}

        config_base = yaml_base or YAML_ROOT
        snapshot: dict = {"config_files": dict(root_data)}
        for selector in ("sensor_config", "runtime_config", "logging_config"):
            relative_name = str(root_data.get(selector, "")).strip()
            if not relative_name:
                continue
            component_data = self._load_yaml_file(config_base / relative_name)
            if isinstance(component_data, dict):
                self._merge_yaml_mapping(snapshot, component_data)
        return snapshot

    def _load_yaml_snapshot_by_name(self, yaml_name: str) -> dict:
        requested_path = YAML_ROOT / yaml_name
        if requested_path.is_file() and requested_path.name != Path(FIXED_CONFIG_NAME).name:
            data = self._load_yaml_file(requested_path)
            # Backward compatibility for analysis records made with the former single-YAML layout.
            if isinstance(data, dict) and not all(
                key in data for key in ("sensor_config", "runtime_config", "logging_config")
            ):
                return data
        root_path = requested_path if requested_path.is_file() else self._root_config_path()
        return self._load_split_yaml_snapshot(root_path, YAML_ROOT)

    def _load_session_yaml_snapshot(self, session_dir: Path) -> dict:
        root_arg = self._root_config_argument() or FIXED_CONFIG_NAME
        archived_root = session_dir / "yaml" / root_arg
        if archived_root.is_file():
            return self._load_split_yaml_snapshot(archived_root, session_dir / "yaml")
        return self._load_split_yaml_snapshot(self._root_config_path(), YAML_ROOT)

    @staticmethod
    def _yaml_snapshot_signature(snapshot: dict) -> str:
        try:
            return json.dumps(snapshot or {}, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        except TypeError:
            return json.dumps({}, ensure_ascii=False, sort_keys=True)

    def _yaml_diff_keys(self, left, right, prefix: str = "") -> List[str]:
        diffs: List[str] = []
        if isinstance(left, dict) and isinstance(right, dict):
            keys = sorted(set(left.keys()) | set(right.keys()))
            for key in keys:
                key_prefix = f"{prefix}.{key}" if prefix else str(key)
                if key not in left or key not in right:
                    diffs.append(key_prefix)
                    continue
                diffs.extend(self._yaml_diff_keys(left[key], right[key], key_prefix))
            return diffs
        if isinstance(left, list) and isinstance(right, list):
            if left != right:
                diffs.append(prefix or "list")
            return diffs
        if left != right:
            diffs.append(prefix or "value")
        return diffs

    @staticmethod
    def _yaml_group_rank(name: str) -> int:
        match = re.fullmatch(r"yaml(\d+)", (name or "").strip())
        return int(match.group(1)) if match else 999999

    def _rebuild_yaml_groups_for_bag(self, bag_id: str) -> None:
        if not bag_id:
            return
        records = [record for record in self.analysis_records if record.bag_id == bag_id]
        if not records:
            return
        records.sort(key=lambda record: record.finished_at or "")
        group_map = {}
        baseline_snapshot = {}
        next_index = 1
        for record in records:
            if not record.yaml_snapshot:
                record.yaml_snapshot = self._load_yaml_snapshot_by_name(record.lio_yaml_name)
            signature = self._yaml_snapshot_signature(record.yaml_snapshot)
            if signature not in group_map:
                group_name = f"yaml{next_index}"
                group_map[signature] = group_name
                next_index += 1
            record.yaml_group_name = group_map[signature]
            if record.yaml_group_name == "yaml1":
                baseline_snapshot = record.yaml_snapshot or {}
                record.yaml_diff_keys = []
            else:
                record.yaml_diff_keys = self._yaml_diff_keys(baseline_snapshot, record.yaml_snapshot or {})

    def _yaml_group_display_text(self, record: AnalysisRecord) -> str:
        group_name = record.yaml_group_name or "yaml1"
        if not record.yaml_diff_keys:
            return group_name
        preview = record.yaml_diff_keys[:4]
        suffix = "" if len(record.yaml_diff_keys) <= 4 else f", +{len(record.yaml_diff_keys) - 4}"
        return f"{group_name} ({', '.join(preview)}{suffix})"

    def _yaml_value_at_path(self, snapshot, dotted_key: str):
        current = snapshot
        for part in dotted_key.split("."):
            if not isinstance(current, dict) or part not in current:
                return None
            current = current[part]
        return current

    def _yaml_display_key(self, dotted_key: str) -> str:
        alias = self.yaml_favorite_aliases.get(dotted_key, "").strip()
        return alias or dotted_key

    def _yaml_display_key_with_source(self, dotted_key: str) -> str:
        alias = self.yaml_favorite_aliases.get(dotted_key, "").strip()
        if not alias:
            return dotted_key
        return f"{alias} [{dotted_key}]"

    def _yaml_group_summary_text(self, group_name: str, snapshot: dict, diff_keys: List[str]) -> str:
        if not diff_keys:
            return group_name
        parts = []
        for key in diff_keys:
            value = self._yaml_value_at_path(snapshot, key)
            rendered = "(missing)" if value is None else self._format_scalar(value)
            parts.append(f"{self._yaml_display_key(key)}={rendered}")
        return f"{group_name} ({', '.join(parts)})"

    @staticmethod
    def _format_scalar(value) -> str:
        if isinstance(value, bool):
            return "ON" if value else "OFF"
        if isinstance(value, float):
            return f"{value:.6g}"
        return str(value)

    def _load_yaml_file(self, yaml_path: Path):
        try:
            return yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
        except Exception as exc:
            return exc

    @staticmethod
    def _yaml_anchor_name(path_parts: List[str]) -> str:
        base = "__".join(re.sub(r"[^0-9A-Za-z_]+", "_", part) for part in path_parts)
        return base.strip("_") or "root"

    @staticmethod
    def _yaml_dotted_path(path_parts: List[str]) -> str:
        return ".".join(str(part) for part in path_parts)

    def _build_yaml_outline(self, data, parent_item: QtWidgets.QTreeWidgetItem, path_parts: Optional[List[str]] = None) -> None:
        if path_parts is None:
            path_parts = []
        if not isinstance(data, dict):
            return
        for key, value in data.items():
            key_text = str(key)
            current_path = path_parts + [key_text]
            item = QtWidgets.QTreeWidgetItem([key_text])
            item.setData(0, QtCore.Qt.UserRole, self._yaml_anchor_name(current_path))
            item.setData(0, QtCore.Qt.UserRole + 1, self._yaml_dotted_path(current_path))
            is_leaf = not isinstance(value, dict)
            item.setData(0, QtCore.Qt.UserRole + 2, is_leaf)
            parent_item.addChild(item)
            if isinstance(value, dict):
                self._build_yaml_outline(value, item, current_path)

    def _format_yaml_for_display(self, yaml_label: str, data, highlighted_anchor: str = "", snapshot_mode: bool = False) -> str:
        if isinstance(data, Exception):
            return (
                "<html><body style='background:#f6f4ef;color:#223244;font-family:\"DejaVu Sans Mono\";'>"
                f"<div style='padding:18px;'>Failed to load YAML: {html.escape(str(data))}</div>"
            "</body></html>"
        )

        def render_scalar(value) -> str:
            if isinstance(value, bool):
                color = "#2c6b43" if value else "#9b3d36"
                text = "ON" if value else "OFF"
                return (
                    f"<span style='color:{color};font-weight:700;background:#f3efe6;"
                    "padding:2px 8px;border-radius:999px;'>"
                    f"{text}</span>"
                )
            if isinstance(value, (int, float)):
                return f"<span style='color:#355f84;font-weight:700;'>{html.escape(self._format_scalar(value))}</span>"
            return f"<span style='color:#37475a;'>{html.escape(self._format_scalar(value))}</span>"

        def render_list(values, indent: int) -> str:
            items = []
            margin = indent * 18
            for item in values:
                if isinstance(item, dict):
                    items.append(
                        f"<div style='margin-left:{margin}px;margin-top:6px;"
                        "padding-left:12px;border-left:2px solid #d7dcd2;'>"
                        f"{render_dict(item, indent + 1, nested=True)}</div>"
                    )
                elif isinstance(item, list):
                    items.append(
                        f"<div style='margin-left:{margin}px;margin-top:6px;'>"
                        f"<span style='color:#8a6a2f;'>•</span>{render_list(item, indent + 1)}</div>"
                    )
                else:
                    items.append(
                        f"<div style='margin-left:{margin}px;color:#6f7f90;margin-top:4px;'>"
                        f"<span style='color:#8a6a2f;'>•</span> {render_scalar(item)}</div>"
                    )
            return "".join(items)

        def render_dict(mapping, indent: int = 0, nested: bool = False, path_parts: Optional[List[str]] = None) -> str:
            if path_parts is None:
                path_parts = []
            blocks = []
            for key, value in mapping.items():
                current_path = path_parts + [str(key)]
                anchor = self._yaml_anchor_name(current_path)
                key_html = html.escape(str(key))
                highlighted = anchor == highlighted_anchor
                if isinstance(value, dict):
                    header_style = (
                        "margin-top:14px;" if not nested else "margin-top:10px;"
                    )
                    header_background = "#f4e6a8" if highlighted else "#e8ece7"
                    header_border = "#d3b75a" if highlighted else "#d9ddd4"
                    header_shadow = "box-shadow:0 0 0 2px rgba(211,183,90,0.20);" if highlighted else ""
                    blocks.append(
                        f"<a name='{anchor}'></a>"
                        f"<div style='{header_style}padding:8px 12px;background:{header_background};"
                        f"border:1px solid {header_border};border-radius:10px;{header_shadow}"
                        "color:#1f3244;font-weight:700;'>"
                        f"{key_html}</div>"
                    )
                    blocks.append(
                        f"<div style='margin-left:14px;margin-top:8px;padding-left:14px;"
                        "border-left:2px solid #d8ddd3;'>"
                        f"{render_dict(value, indent + 1, nested=True, path_parts=current_path)}</div>"
                    )
                elif isinstance(value, list):
                    row_border = "#d3b75a" if highlighted else "#ddd7cc"
                    row_background = "background:#fcf3c5;" if highlighted else ""
                    blocks.append(
                        f"<a name='{anchor}'></a>"
                        f"<div style='margin-top:10px;padding:8px 0 2px 0;{row_background}"
                        f"border-top:1px dashed {row_border};'>"
                        f"<span style='color:#1f3244;font-weight:700;'>{key_html}</span>"
                        "</div>"
                    )
                    blocks.append(render_list(value, indent + 1))
                else:
                    row_border = "#d8b85b" if highlighted else "#eee9df"
                    row_background = "#fcf3c5" if highlighted else "transparent"
                    blocks.append(
                        f"<a name='{anchor}'></a>"
                        "<div style='display:flex;align-items:flex-start;gap:18px;"
                        f"padding:8px 10px;border-top:1px solid {row_border};"
                        f"background:{row_background};border-radius:8px;'>"
                        f"<div style='min-width:260px;color:#2a3b4d;font-weight:700;'>{key_html}</div>"
                        f"<div style='flex:1;'>{render_scalar(value)}</div>"
                        "</div>"
                    )
            return "".join(blocks)

        body = render_dict(data)
        subtitle = f"YAML Snapshot: {yaml_label}" if snapshot_mode else f"YAML: {yaml_label}"
        return (
            "<html><body style='background:#f3f1ec;color:#233142;"
            "font-family:\"Noto Sans CJK SC\",\"Microsoft YaHei\",\"PingFang SC\",sans-serif;"
            "margin:0;'>"
            "<a name='root'></a>"
            "<div style='padding:22px 24px 28px 24px;'>"
            "<div style='font-size:22px;font-weight:800;color:#182534;'>LIO Configuration</div>"
            f"<div style='margin-top:6px;color:#6d7885;font-size:13px;'>{html.escape(subtitle)}</div>"
            "<div style='margin-top:14px;height:1px;background:#ddd7cc;'></div>"
            f"<div style='margin-top:18px;'>{body}</div>"
            "</div></body></html>"
        )

    def _format_yaml_favorites_html(self, yaml_data) -> str:
        if isinstance(yaml_data, Exception):
            return (
                "<html><body style='background:#fcfbf8;color:#6f7b88;"
                "font-family:\"Noto Sans CJK SC\",\"Microsoft YaHei\",\"PingFang SC\",sans-serif;"
                "margin:0;padding:14px 16px;'>Favorites unavailable.</body></html>"
            )
        if not self.yaml_favorites:
            return (
                "<html><body style='background:#fcfbf8;color:#6f7b88;"
                "font-family:\"Noto Sans CJK SC\",\"Microsoft YaHei\",\"PingFang SC\",sans-serif;"
                "margin:0;padding:14px 16px;'>No favorite YAML keys yet. "
                "Right-click a leaf item in Outline to add one.</body></html>"
            )

        rows = []
        for key in self.yaml_favorites:
            value = self._yaml_value_at_path(yaml_data, key)
            rendered = "(missing)" if value is None else self._format_scalar(value)
            color = "#9b3d36" if value is None else "#355f84"
            key_label = self._yaml_display_key(key)
            key_source = "" if key_label == key else f"<div style='margin-top:3px;color:#7d8791;font-size:12px;'>{html.escape(key)}</div>"
            rows.append(
                "<div style='display:flex;gap:16px;align-items:flex-start;"
                "padding:8px 0;border-top:1px solid #ece6dc;'>"
                f"<div style='min-width:320px;color:#1f3244;font-weight:700;'>{html.escape(key_label)}{key_source}</div>"
                f"<div style='flex:1;color:{color};font-weight:700;'>{html.escape(rendered)}</div>"
                "</div>"
            )
        return (
            "<html><body style='background:#fcfbf8;color:#233142;"
            "font-family:\"Noto Sans CJK SC\",\"Microsoft YaHei\",\"PingFang SC\",sans-serif;"
            "margin:0;padding:12px 16px;'>"
            "<div style='font-size:14px;font-weight:800;color:#182534;'>Favorite Keys</div>"
            "<div style='margin-top:6px;color:#6d7885;font-size:12px;'>Shared across all YAML viewers</div>"
            f"<div style='margin-top:10px;'>{''.join(rows)}</div>"
            "</body></html>"
        )

    def _open_yaml_viewer(self, yaml_name: str, yaml_snapshot: Optional[dict] = None) -> None:
        yaml_path = PACKAGE_ROOT / "yaml" / yaml_name
        snapshot_mode = isinstance(yaml_snapshot, dict) and bool(yaml_snapshot)
        yaml_data = yaml_snapshot if snapshot_mode else self._load_yaml_snapshot_by_name(yaml_name)
        yaml_label = yaml_name
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(f"YAML Viewer - {yaml_label}")
        dialog.resize(1120, 760)
        layout = QtWidgets.QVBoxLayout(dialog)

        search_row = QtWidgets.QHBoxLayout()
        search_edit = QtWidgets.QLineEdit()
        search_edit.setPlaceholderText("Search in YAML")
        prev_button = QtWidgets.QPushButton("Previous")
        next_button = QtWidgets.QPushButton("Next")
        search_row.addWidget(search_edit, 1)
        search_row.addWidget(prev_button)
        search_row.addWidget(next_button)
        layout.addLayout(search_row)

        favorites_view = QtWidgets.QTextBrowser()
        favorites_view.setReadOnly(True)
        favorites_view.setOpenExternalLinks(False)
        favorites_view.setMaximumHeight(168)
        favorites_view.setStyleSheet(
            "QTextBrowser { background:#fcfbf8; border:1px solid #ddd7cc; border-radius:14px; padding:0; }"
        )
        favorites_view.setHtml(self._format_yaml_favorites_html(yaml_data))
        layout.addWidget(favorites_view)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        outline = QtWidgets.QTreeWidget()
        outline.setHeaderHidden(True)
        outline.setRootIsDecorated(True)
        outline.setItemsExpandable(True)
        outline.setMaximumWidth(280)
        outline.setMinimumWidth(220)
        outline.setStyleSheet(
            "QTreeWidget { background:#fbfaf7; border:1px solid #ddd7cc; border-radius:14px; padding:6px; }"
            "QTreeWidget::item { padding:6px 8px; }"
            "QTreeWidget::branch { background:transparent; }"
        )
        outline.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        root_item = QtWidgets.QTreeWidgetItem(["Outline"])
        root_item.setData(0, QtCore.Qt.UserRole, "root")
        root_item.setIcon(0, self._group_indicator_icon(True))
        outline.addTopLevelItem(root_item)
        if not isinstance(yaml_data, Exception):
            self._build_yaml_outline(yaml_data, root_item)
        root_item.setExpanded(True)

        viewer = QtWidgets.QTextBrowser()
        viewer.setReadOnly(True)
        viewer.setOpenExternalLinks(False)
        viewer.setStyleSheet(
            "QTextBrowser { background:#f3f1ec; border:1px solid #ddd7cc; border-radius:14px; padding:0; }"
        )
        viewer.setHtml(self._format_yaml_for_display(yaml_label, yaml_data, snapshot_mode=snapshot_mode))
        splitter.addWidget(outline)
        splitter.addWidget(viewer)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        layout.addWidget(splitter)
        highlight_timer = QtCore.QTimer(dialog)
        highlight_timer.setSingleShot(True)
        highlight_state = {"anchor": ""}

        def apply_anchor_highlight(anchor: str) -> None:
            if not dialog.isVisible():
                return
            viewer.setHtml(self._format_yaml_for_display(yaml_label, yaml_data, anchor, snapshot_mode=snapshot_mode))
            viewer.scrollToAnchor(anchor)

        def clear_anchor_highlight(anchor: str) -> None:
            if not dialog.isVisible():
                return
            viewer.setHtml(self._format_yaml_for_display(yaml_label, yaml_data, snapshot_mode=snapshot_mode))
            viewer.scrollToAnchor(anchor)

        def on_highlight_timeout() -> None:
            anchor = highlight_state["anchor"]
            if anchor:
                clear_anchor_highlight(anchor)

        def jump_to_outline_item(item: QtWidgets.QTreeWidgetItem) -> None:
            anchor = item.data(0, QtCore.Qt.UserRole)
            if anchor:
                anchor = str(anchor)
                highlight_state["anchor"] = anchor
                highlight_timer.stop()
                apply_anchor_highlight(anchor)
                highlight_timer.start(900)

        highlight_timer.timeout.connect(on_highlight_timeout)
        dialog.finished.connect(highlight_timer.stop)

        outline.itemClicked.connect(jump_to_outline_item)
        outline.itemExpanded.connect(lambda item: self._set_group_item_indicator(item, True))
        outline.itemCollapsed.connect(lambda item: self._set_group_item_indicator(item, False))
        for idx in range(outline.topLevelItemCount()):
            self._set_group_item_indicator(outline.topLevelItem(idx), True)
        iterator = QtWidgets.QTreeWidgetItemIterator(outline)
        while iterator.value():
            item = iterator.value()
            if item.childCount() > 0:
                self._set_group_item_indicator(item, item.isExpanded())
            else:
                item.setIcon(0, QtWidgets.QApplication.style().standardIcon(QtWidgets.QStyle.SP_FileIcon))
            iterator += 1

        def refresh_favorites_view() -> None:
            favorites_view.setHtml(self._format_yaml_favorites_html(yaml_data))

        def show_outline_context_menu(position: QtCore.QPoint) -> None:
            item = outline.itemAt(position)
            if item is None:
                return
            is_leaf = bool(item.data(0, QtCore.Qt.UserRole + 2))
            dotted_path = str(item.data(0, QtCore.Qt.UserRole + 1) or "")
            if not is_leaf or not dotted_path:
                return
            menu = QtWidgets.QMenu(outline)
            if dotted_path in self.yaml_favorites:
                alias_action = menu.addAction("Set Alias")
                clear_alias_action = None
                if dotted_path in self.yaml_favorite_aliases:
                    clear_alias_action = menu.addAction("Clear Alias")
                menu.addSeparator()
                unfavorite_action = menu.addAction("Unfavorite")
                chosen = menu.exec_(outline.viewport().mapToGlobal(position))
                if chosen == alias_action:
                    current_alias = self.yaml_favorite_aliases.get(dotted_path, "")
                    alias_text, ok = QtWidgets.QInputDialog.getText(
                        dialog,
                        "Favorite Alias",
                        f"Set alias for:\n{dotted_path}",
                        QtWidgets.QLineEdit.Normal,
                        current_alias,
                    )
                    if ok:
                        alias_text = alias_text.strip()
                        if alias_text:
                            self.yaml_favorite_aliases[dotted_path] = alias_text
                        else:
                            self.yaml_favorite_aliases.pop(dotted_path, None)
                        self._save_state()
                        refresh_favorites_view()
                elif clear_alias_action is not None and chosen == clear_alias_action:
                    self.yaml_favorite_aliases.pop(dotted_path, None)
                    self._save_state()
                    refresh_favorites_view()
                elif chosen == unfavorite_action:
                    self.yaml_favorites = [key for key in self.yaml_favorites if key != dotted_path]
                    self.yaml_favorite_aliases.pop(dotted_path, None)
                    self._save_state()
                    refresh_favorites_view()
            else:
                favorite_action = menu.addAction("Favorite")
                favorite_alias_action = menu.addAction("Favorite With Alias")
                chosen = menu.exec_(outline.viewport().mapToGlobal(position))
                if chosen == favorite_action:
                    self.yaml_favorites.append(dotted_path)
                    self.yaml_favorites = sorted(set(self.yaml_favorites), key=self._natural_sort_key)
                    self._save_state()
                    refresh_favorites_view()
                elif chosen == favorite_alias_action:
                    alias_text, ok = QtWidgets.QInputDialog.getText(
                        dialog,
                        "Favorite Alias",
                        f"Set alias for:\n{dotted_path}",
                    )
                    if ok:
                        self.yaml_favorites.append(dotted_path)
                        self.yaml_favorites = sorted(set(self.yaml_favorites), key=self._natural_sort_key)
                        alias_text = alias_text.strip()
                        if alias_text:
                            self.yaml_favorite_aliases[dotted_path] = alias_text
                        self._save_state()
                        refresh_favorites_view()

        outline.customContextMenuRequested.connect(show_outline_context_menu)

        def run_find(backward: bool = False) -> None:
            pattern = search_edit.text().strip()
            if not pattern:
                return
            flags = QtGui.QTextDocument.FindFlags()
            if backward:
                flags |= QtGui.QTextDocument.FindBackward
            if not viewer.find(pattern, flags):
                cursor = viewer.textCursor()
                cursor.movePosition(
                    QtGui.QTextCursor.End if backward else QtGui.QTextCursor.Start
                )
                viewer.setTextCursor(cursor)
                viewer.find(pattern, flags)

        ctrl_f = QtWidgets.QShortcut(QtGui.QKeySequence.Find, dialog)
        ctrl_f.activated.connect(search_edit.setFocus)
        ctrl_f.activated.connect(search_edit.selectAll)
        next_button.clicked.connect(lambda: run_find(False))
        prev_button.clicked.connect(lambda: run_find(True))
        search_edit.returnPressed.connect(lambda: run_find(False))

        prev_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence("Shift+Return"), search_edit)
        prev_shortcut.activated.connect(lambda: run_find(True))

        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close)
        buttons.rejected.connect(dialog.reject)
        buttons.accepted.connect(dialog.accept)
        buttons.button(QtWidgets.QDialogButtonBox.Close).clicked.connect(dialog.accept)
        layout.addWidget(buttons)
        dialog.exec_()

    @staticmethod
    def _marker_specs() -> dict:
        return {
            "none": ("", QtGui.QColor("#c8cdd2"), "No mark"),
            "warn": ("!", QtGui.QColor("#d29b1f"), "Warning marker"),
            "ok": ("✓", QtGui.QColor("#2f8f4e"), "OK marker"),
            "bad": ("✕", QtGui.QColor("#c64a42"), "Blocked marker"),
        }

    def _build_marker_icon(self, marker: str) -> QtGui.QIcon:
        symbol, color, _ = self._marker_specs().get(marker, self._marker_specs()["none"])
        pixmap = QtGui.QPixmap(18, 18)
        pixmap.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(pixmap)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(color)
        painter.drawEllipse(1, 1, 16, 16)
        if symbol:
            font = QtGui.QFont("Noto Sans CJK SC", 10)
            font.setBold(True)
            painter.setFont(font)
            painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff")))
            painter.drawText(pixmap.rect(), QtCore.Qt.AlignCenter, symbol)
        painter.end()
        return QtGui.QIcon(pixmap)

    def _init_marker_icons(self) -> None:
        self.marker_icons = {name: self._build_marker_icon(name) for name in self._marker_specs()}
        for idx in range(self.marker_combo.count()):
            marker = self.marker_combo.itemData(idx)
            self.marker_combo.setItemIcon(idx, self.marker_icons.get(marker, QtGui.QIcon()))

    @staticmethod
    def _compute_terminal_pose(post_state_path: Path) -> List[float]:
        with post_state_path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
            reader = csv.DictReader(handle)
            last_row = None
            for row in reader:
                last_row = row
        if last_row is None:
            return []
        keys = ["x", "y", "z", "qx", "qy", "qz", "qw"]
        return [float(last_row[key]) for key in keys]

    def _save_state(self) -> None:
        payload = {
            "settings": asdict(self.settings),
            "yaml_favorites": list(self.yaml_favorites),
            "yaml_favorite_aliases": dict(self.yaml_favorite_aliases),
            "batch_test_groups": [asdict(group) for group in self.batch_test_groups],
            "entries": [asdict(entry) for entry in self.entries],
            "analyses": [asdict(record) for record in self.analysis_records],
        }
        DB_PATH.parent.mkdir(parents=True, exist_ok=True)
        DB_PATH.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    def _refresh_table(self, preserve_form: bool = False) -> None:
        selected_row_before = self._selected_row()
        current_filter = self.group_filter_combo.currentText()
        groups = sorted({entry.group or "Default" for entry in self.entries if not entry.deleted})
        self.group_filter_combo.blockSignals(True)
        self.group_filter_combo.clear()
        self.group_filter_combo.addItem("All Groups")
        self.group_filter_combo.addItems(groups)
        if current_filter and self.group_filter_combo.findText(current_filter) >= 0:
            self.group_filter_combo.setCurrentText(current_filter)
        self.group_filter_combo.blockSignals(False)

        sort_key = self.sort_combo.currentText() or "Group"
        filtered_entries = [entry for entry in self.entries if not entry.deleted]
        selected_group = self.group_filter_combo.currentText()
        if selected_group and selected_group != "All Groups":
            filtered_entries = [entry for entry in filtered_entries if entry.group == selected_group]

        def entry_sort_key(entry: BagEntry):
            group = entry.group or "Default"
            if sort_key == "Alias":
                return (self._natural_sort_key(group), self._natural_sort_key(entry.alias), self._natural_sort_key(entry.bag_path))
            if sort_key == "Date":
                return (self._natural_sort_key(group), entry.record_date or "", self._natural_sort_key(entry.alias))
            if sort_key == "Duration":
                return (self._natural_sort_key(group), entry.duration_seconds, self._natural_sort_key(entry.alias))
            if sort_key == "Analyses":
                return (self._natural_sort_key(group), self._analysis_count_for_bag(entry.bag_id), self._natural_sort_key(entry.alias))
            if sort_key == "Note":
                return (self._natural_sort_key(group), self._natural_sort_key(entry.note), self._natural_sort_key(entry.alias))
            if sort_key == "Mark":
                rank = {"warn": 0, "ok": 1, "bad": 2, "none": 3}.get(entry.marker, 3)
                return (self._natural_sort_key(group), rank, self._natural_sort_key(entry.alias))
            if sort_key == "Manual Trigger":
                return (self._natural_sort_key(group), 0 if entry.has_elevator_flag else 1, self._natural_sort_key(entry.alias))
            return (self._natural_sort_key(group), self._natural_sort_key(entry.alias), self._natural_sort_key(entry.bag_path))

        filtered_entries.sort(key=entry_sort_key)
        self.visible_entry_indices = [self.entries.index(entry) for entry in filtered_entries]

        self._entry_selection_guard = True
        self.tree.clear()
        group_nodes = {}
        for entry in filtered_entries:
            self._hydrate_entry_metadata(entry)
            group_name = entry.group or "Default"
            if group_name not in group_nodes:
                parent = QtWidgets.QTreeWidgetItem([group_name, "", "", "", "", "", ""])
                parent.setFirstColumnSpanned(True)
                parent.setFlags(parent.flags() & ~QtCore.Qt.ItemIsSelectable)
                parent.setIcon(0, self._group_indicator_icon(True))
                font = parent.font(0)
                font.setBold(True)
                parent.setFont(0, font)
                group_nodes[group_name] = parent
                self.tree.addTopLevelItem(parent)
            _, _, marker_tip = self._marker_specs().get(entry.marker, self._marker_specs()["none"])
            if entry.marker_note.strip():
                marker_tip = f"{marker_tip}\n{entry.marker_note.strip()}"
            marker_text = entry.marker_note.strip()
            manual_trigger_text = "True" if entry.has_elevator_flag else "False"
            duration_text = self._format_duration(entry.duration_seconds)
            analysis_count_text = str(self._analysis_count_for_bag(entry.bag_id))
            item = QtWidgets.QTreeWidgetItem([
                entry.alias,
                entry.record_date,
                duration_text,
                analysis_count_text,
                entry.note,
                manual_trigger_text,
                marker_text,
            ])
            item.setData(0, QtCore.Qt.UserRole, self.entries.index(entry))
            item.setTextAlignment(2, QtCore.Qt.AlignCenter)
            item.setTextAlignment(3, QtCore.Qt.AlignCenter)
            item.setTextAlignment(5, QtCore.Qt.AlignCenter)
            item.setToolTip(0, f"Bag: {entry.bag_path}\nRate: {entry.play_rate:.2f}x")
            item.setToolTip(1, f"Bag: {entry.bag_path}\nRate: {entry.play_rate:.2f}x")
            item.setToolTip(2, f"Bag: {entry.bag_path}\nRate: {entry.play_rate:.2f}x")
            item.setToolTip(3, f"Bag: {entry.bag_path}\nRate: {entry.play_rate:.2f}x")
            item.setToolTip(4, f"Bag: {entry.bag_path}\nRate: {entry.play_rate:.2f}x")
            item.setTextAlignment(1, QtCore.Qt.AlignCenter)
            if entry.has_elevator_flag:
                item.setToolTip(5, "Manual trigger topic exists in this bag.")
                item.setForeground(5, QtGui.QBrush(QtGui.QColor("#24613a")))
            else:
                item.setToolTip(5, "No manual trigger topic in this bag.")
                item.setForeground(5, QtGui.QBrush(QtGui.QColor("#5f6472")))
            item.setToolTip(6, marker_tip)
            item.setIcon(6, self.marker_icons.get(entry.marker, QtGui.QIcon()))
            group_nodes[group_name].addChild(item)

        for idx in range(self.tree.topLevelItemCount()):
            self.tree.topLevelItem(idx).setExpanded(True)
        if preserve_form and selected_row_before >= 0:
            self._select_entry(selected_row_before)
        self._entry_selection_guard = False
        self._update_buttons()

    def _refresh_analysis_table(self) -> None:
        selected_row = self._selected_row()
        selected_bag_id = ""
        if 0 <= selected_row < len(self.entries):
            selected_bag_id = self.entries[selected_row].bag_id
        self.visible_analysis_indices = [
            idx for idx, record in enumerate(self.analysis_records) if record.bag_id == selected_bag_id
        ] if selected_bag_id else []
        self.visible_analysis_indices.sort(
            key=lambda idx: (
                self._yaml_group_rank(self.analysis_records[idx].yaml_group_name),
                self.analysis_records[idx].finished_at,
            )
        )
        self.analysis_table.clear()
        self.analysis_table.setHeaderLabels(
            ["Finished", "Count", "Total Z", "Total", "End Err", "End Z Err", "X", "Y", "Z", "Img"]
        )
        group_records = {}
        all_diff_keys = []
        for record_index in self.visible_analysis_indices:
            record = self.analysis_records[record_index]
            group_name = record.yaml_group_name or "yaml1"
            group_records.setdefault(group_name, []).append(record_index)
            for key in record.yaml_diff_keys:
                if key not in all_diff_keys:
                    all_diff_keys.append(key)

        for group_name in sorted(group_records.keys(), key=self._yaml_group_rank):
            record_indices = group_records[group_name]
            group_record = self.analysis_records[record_indices[0]]
            parent = QtWidgets.QTreeWidgetItem([
                self._yaml_group_summary_text(group_name, group_record.yaml_snapshot, all_diff_keys),
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
            ])
            parent.setFirstColumnSpanned(True)
            parent.setFlags(parent.flags() & ~QtCore.Qt.ItemIsSelectable)
            parent.setIcon(0, self._group_indicator_icon(True))
            font = parent.font(0)
            font.setBold(True)
            parent.setFont(0, font)
            if all_diff_keys:
                parent.setToolTip(
                    0,
                    "\n".join(self._yaml_display_key_with_source(key) for key in all_diff_keys),
                )
            self.analysis_table.addTopLevelItem(parent)
            self.analysis_table.setFirstItemColumnSpanned(parent, True)

            for record_index in record_indices:
                record = self.analysis_records[record_index]
                pose = record.terminal_pose if len(record.terminal_pose) >= 3 else [0.0, 0.0, 0.0]
                child = QtWidgets.QTreeWidgetItem([
                    record.finished_at,
                    str(record.exit_commit_count),
                    f"{record.delta_z_total:.6f}",
                    f"{record.total_path_length:.6f}",
                    f"{record.terminal_error:.6f}",
                    f"{record.terminal_z_error:.6f}",
                    f"{pose[0]:.6f}",
                    f"{pose[1]:.6f}",
                    f"{pose[2]:.6f}",
                    "✓" if record.attachments else "✕",
                ])
                child.setData(0, QtCore.Qt.UserRole, record_index)
                for col in range(10):
                    child.setTextAlignment(col, QtCore.Qt.AlignCenter)
                parent.addChild(child)

        for idx in range(self.analysis_table.topLevelItemCount()):
            self.analysis_table.topLevelItem(idx).setExpanded(True)

    def _selected_row(self) -> int:
        item = self.tree.currentItem()
        if item is None:
            return -1
        data = item.data(0, QtCore.Qt.UserRole)
        return int(data) if data is not None else -1

    @staticmethod
    def _row_from_tree_item(item: Optional[QtWidgets.QTreeWidgetItem]) -> int:
        if item is None:
            return -1
        data = item.data(0, QtCore.Qt.UserRole)
        return int(data) if data is not None else -1

    def _selected_analysis_row(self) -> int:
        item = self.analysis_table.currentItem()
        if item is None:
            return -1
        data = item.data(0, QtCore.Qt.UserRole)
        return int(data) if data is not None else -1

    def _select_analysis_record(self, record_index: int) -> None:
        if record_index < 0:
            return
        for i in range(self.analysis_table.topLevelItemCount()):
            parent = self.analysis_table.topLevelItem(i)
            for j in range(parent.childCount()):
                child = parent.child(j)
                data = child.data(0, QtCore.Qt.UserRole)
                if data is not None and int(data) == record_index:
                    self.analysis_table.setCurrentItem(child)
                    return

    def _remove_selected_analysis_record(self) -> None:
        row = self._selected_analysis_row()
        if row < 0 or row >= len(self.analysis_records):
            return
        record = self.analysis_records[row]
        reply = QtWidgets.QMessageBox.question(
            self,
            "Delete Analysis",
            f"Delete analysis record from {record.finished_at}?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        for attachment in record.attachments:
            path = Path(attachment)
            try:
                if path.is_dir():
                    shutil.rmtree(path)
                else:
                    path.unlink()
            except OSError:
                pass
        try:
            self._analysis_attachment_dir(record).rmdir()
        except OSError:
            pass
        del self.analysis_records[row]
        self._save_state()
        self._refresh_analysis_table()

    def _show_selected_analysis_detail_dialog(self) -> None:
        row = self._selected_analysis_row()
        if row < 0 or row >= len(self.analysis_records):
            return
        record = self.analysis_records[row]
        dialog = QtWidgets.QDialog(self)
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        dialog.setWindowTitle("Analysis Detail")
        dialog.resize(880, 560)
        layout = QtWidgets.QVBoxLayout(dialog)
        detail_edit = QtWidgets.QPlainTextEdit()
        detail_edit.setReadOnly(True)
        detail_edit.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        detail_edit.setFont(QtGui.QFont("DejaVu Sans Mono", 10))
        detail_edit.setPlainText(self._analysis_detail_text(record))
        layout.addWidget(detail_edit)

        comment_title = QtWidgets.QLabel("Comment")
        comment_title.setStyleSheet("font-size: 15px; font-weight: 700;")
        layout.addWidget(comment_title)

        comment_edit = QtWidgets.QLineEdit(record.comment)
        comment_edit.setPlaceholderText("Add a short note for this analysis record")
        layout.addWidget(comment_edit)

        attachment_title = QtWidgets.QLabel("Attachments")
        attachment_title.setStyleSheet("font-size: 15px; font-weight: 700;")
        layout.addWidget(attachment_title)

        attachment_list = QtWidgets.QListWidget()
        attachment_list.setViewMode(QtWidgets.QListView.IconMode)
        attachment_list.setResizeMode(QtWidgets.QListView.Adjust)
        attachment_list.setMovement(QtWidgets.QListView.Static)
        attachment_list.setIconSize(QtCore.QSize(200, 125))
        attachment_list.setSpacing(10)
        attachment_list.setMinimumHeight(180)
        attachment_list.setWordWrap(True)
        attachment_list.itemDoubleClicked.connect(self._open_analysis_attachment)
        self._refresh_analysis_attachment_list(attachment_list, record)
        layout.addWidget(attachment_list)

        attachment_actions = QtWidgets.QHBoxLayout()
        paste_button = QtWidgets.QPushButton("Paste Image")
        select_button = QtWidgets.QPushButton("Select Image")
        attachment_hint = QtWidgets.QLabel("Ctrl+V to paste from clipboard")
        attachment_hint.setStyleSheet("color: #6f7b88;")
        attachment_actions.addWidget(paste_button)
        attachment_actions.addWidget(select_button)
        attachment_actions.addStretch(1)
        attachment_actions.addWidget(attachment_hint)
        layout.addLayout(attachment_actions)

        paste_button.clicked.connect(lambda: self._paste_analysis_attachment(row, attachment_list))
        select_button.clicked.connect(lambda: self._select_analysis_attachment(row, attachment_list))
        paste_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence.Paste, dialog)
        paste_shortcut.activated.connect(lambda: self._paste_analysis_attachment(row, attachment_list))
        delete_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence.Delete, dialog)
        delete_shortcut.activated.connect(
            lambda: self._delete_analysis_attachment(row, attachment_list, detail_edit)
        )
        comment_edit.editingFinished.connect(
            lambda: self._save_analysis_comment(row, comment_edit.text().strip(), detail_edit)
        )

        button_row = QtWidgets.QHBoxLayout()
        button_row.addStretch(1)
        open_yaml_button = QtWidgets.QPushButton("Open YAML")
        open_yaml_button.clicked.connect(
            lambda: self._open_yaml_viewer(record.lio_yaml_name, record.yaml_snapshot)
        )
        button_row.addWidget(open_yaml_button)
        layout.addLayout(button_row)
        dialog.exec_()

    def _current_entry_from_form(
        self,
        metadata_override: Optional[Tuple[str, float]] = None,
        has_elevator_flag_override: Optional[bool] = None,
        show_errors: bool = True,
    ) -> Optional[BagEntry]:
        alias = self.alias_edit.text().strip()
        bag_path = self.bag_path_edit.text().strip()
        note = self.note_edit.toPlainText().strip()
        if not alias:
            if show_errors:
                QtWidgets.QMessageBox.warning(self, "Missing Alias", "Alias is required.")
            return None
        if not bag_path:
            if show_errors:
                QtWidgets.QMessageBox.warning(self, "Missing Bag", "Bag path is required.")
            return None
        if metadata_override is not None:
            record_date, duration_seconds = metadata_override
        else:
            record_date = ""
            duration_seconds = 0.0
            try:
                record_date, duration_seconds = self._read_bag_metadata(Path(bag_path))
            except Exception:
                record_date = ""
                duration_seconds = 0.0
        if has_elevator_flag_override is not None:
            has_elevator_flag = has_elevator_flag_override
        else:
            has_elevator_flag = self._bag_contains_elevator_flag(Path(bag_path))
        return BagEntry(
            bag_id=uuid.uuid4().hex,
            alias=alias,
            bag_path=bag_path,
            play_rate=float(self.rate_spin.value()),
            note=note,
            has_elevator_flag=has_elevator_flag,
            group=self.group_edit.text().strip() or "Default",
            marker=self.marker_combo.currentData() or "none",
            marker_note=self.marker_note_edit.text().strip(),
            record_date=record_date,
            duration_seconds=duration_seconds,
        )

    def _add_entry(self) -> None:
        entry = self._current_entry_from_form()
        if entry is None:
            return
        duplicate_row = next(
            (row for row, existing in enumerate(self.entries) if self._entries_match(existing, entry)),
            -1,
        )
        if duplicate_row >= 0:
            duplicate = self.entries[duplicate_row]
            deleted_hint = (
                "\n\nThe matching entry is currently deleted. Use Recover if you want to restore it."
                if duplicate.deleted
                else ""
            )
            QtWidgets.QMessageBox.warning(
                self,
                "Duplicate Bag Entry",
                "An identical Bag List entry already exists, so nothing was added.\n\n"
                f"Alias: {duplicate.alias}\n"
                f"Bag: {duplicate.bag_path}\n\n"
                "Browse a different bag, or change the entry configuration before clicking Add again."
                f"{deleted_hint}",
            )
            return
        self.entries.append(entry)
        self._save_state()
        self._refresh_table()
        self._select_entry(len(self.entries) - 1)

    def _update_entry(self) -> None:
        row = self._selected_row()
        if row < 0:
            QtWidgets.QMessageBox.warning(self, "No Selection", "Select one row to update.")
            return
        saved_entry = self.entries[row]
        entry = self._current_entry_from_form(
            metadata_override=(saved_entry.record_date, saved_entry.duration_seconds),
            has_elevator_flag_override=saved_entry.has_elevator_flag,
        )
        if entry is None:
            return
        if Path(entry.bag_path).expanduser() != Path(saved_entry.bag_path).expanduser():
            QtWidgets.QMessageBox.information(
                self,
                "Use Add Instead",
                "Bag path changed. To keep analysis results bound to the original bag entry, "
                "path changes must create a new entry with Add, not Update.",
            )
            return
        entry.bag_id = saved_entry.bag_id
        self.entries[row] = entry
        self._save_state()
        self._refresh_table()
        self._select_entry(row)

    def _remove_entry(self) -> None:
        row = self._selected_row()
        if row < 0:
            QtWidgets.QMessageBox.warning(self, "No Selection", "Select one row to remove.")
            return
        entry = self.entries[row]
        if entry.deleted:
            QtWidgets.QMessageBox.information(self, "Already Deleted", f"'{entry.alias}' is already marked deleted.")
            return
        alias = entry.alias
        reply = QtWidgets.QMessageBox.question(
            self,
            "Confirm Remove",
            f"Mark bag entry '{alias}' as deleted?\n\nIt will not be removed from the database and can be restored with Recover.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self.entries[row].deleted = True
        self._save_state()
        self._refresh_table()
        self._select_entry(row)

    def _recover_entry(self) -> None:
        deleted_entries = [entry for entry in self.entries if entry.deleted]
        if not deleted_entries:
            QtWidgets.QMessageBox.information(self, "No Deleted Entries", "There are no deleted bag entries to recover.")
            return
        dialog = RecoverEntryDialog(deleted_entries, self)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        bag_id = dialog.selected_bag_id()
        if not bag_id:
            QtWidgets.QMessageBox.information(self, "No Selection", "Select one deleted bag entry to recover.")
            return
        recover_row = next((idx for idx, entry in enumerate(self.entries) if entry.bag_id == bag_id), -1)
        if recover_row < 0:
            QtWidgets.QMessageBox.warning(self, "Missing Entry", "The selected deleted entry could not be found.")
            return
        self.entries[recover_row].deleted = False
        self._save_state()
        self._refresh_table()
        self._select_entry(recover_row)

    def _clear_form(self) -> None:
        self._entry_selection_guard = True
        self.alias_edit.clear()
        self.group_edit.setText("Default")
        self.bag_path_edit.clear()
        self.marker_combo.setCurrentIndex(0)
        self.marker_note_edit.clear()
        self.note_edit.clear()
        self.rate_spin.setValue(DEFAULT_RATE)
        self.tree.clearSelection()
        self._entry_selection_guard = False
        self._refresh_analysis_table()
        self._update_buttons()

    def _load_selected_entry_into_form(self) -> None:
        row = self._selected_row()
        if row < 0 or row >= len(self.entries):
            self._update_buttons()
            return
        entry = self.entries[row]
        self.group_edit.setText(entry.group or "Default")
        self.alias_edit.setText(entry.alias)
        self.bag_path_edit.setText(entry.bag_path)
        marker_index = self.marker_combo.findData(entry.marker or "none")
        self.marker_combo.setCurrentIndex(marker_index if marker_index >= 0 else 0)
        self.marker_note_edit.setText(entry.marker_note)
        self.note_edit.setPlainText(entry.note)
        self.rate_spin.setValue(entry.play_rate)
        self._refresh_analysis_table()
        self._update_buttons()

    def _handle_entry_current_item_changed(
        self,
        current: Optional[QtWidgets.QTreeWidgetItem],
        previous: Optional[QtWidgets.QTreeWidgetItem],
    ) -> None:
        if self._entry_selection_guard:
            return
        previous_row = self._row_from_tree_item(previous)
        current_row = self._row_from_tree_item(current)
        if previous_row >= 0 and previous_row != current_row and self._entry_form_has_unsaved_changes(previous_row):
            action = self._prompt_entry_save_on_switch(previous_row)
            if action == "cancel":
                self._entry_selection_guard = True
                self.tree.setCurrentItem(previous)
                self._entry_selection_guard = False
                return
            if action == "save":
                if not self._save_form_changes_for_entry(previous_row):
                    self._entry_selection_guard = True
                    self.tree.setCurrentItem(previous)
                    self._entry_selection_guard = False
                    return
                if current_row >= 0:
                    self._entry_selection_guard = True
                    self._select_entry(current_row)
                    self._entry_selection_guard = False
        self._load_selected_entry_into_form()

    def _entry_form_has_unsaved_changes(self, row: int) -> bool:
        if row < 0 or row >= len(self.entries):
            return False
        saved_entry = self.entries[row]
        form_entry = self._current_entry_from_form(
            metadata_override=(saved_entry.record_date, saved_entry.duration_seconds),
            has_elevator_flag_override=saved_entry.has_elevator_flag,
            show_errors=False,
        )
        if form_entry is None:
            return False
        form_entry.bag_id = saved_entry.bag_id
        form_entry.record_date = saved_entry.record_date
        form_entry.duration_seconds = saved_entry.duration_seconds
        form_entry.has_elevator_flag = saved_entry.has_elevator_flag
        return not self._entries_match(saved_entry, form_entry)

    def _prompt_entry_save_on_switch(self, row: int) -> str:
        saved_entry = self.entries[row]
        form_entry = self._current_entry_from_form(
            metadata_override=(saved_entry.record_date, saved_entry.duration_seconds),
            has_elevator_flag_override=saved_entry.has_elevator_flag,
            show_errors=False,
        )
        if form_entry is None:
            return "discard"
        changes = self._entry_change_summary(saved_entry, form_entry)
        same_bag_path = Path(saved_entry.bag_path).expanduser() == Path(form_entry.bag_path).expanduser()
        dialog = QtWidgets.QMessageBox(self)
        dialog.setIcon(QtWidgets.QMessageBox.Question)
        dialog.setWindowTitle("Unsaved Changes")
        dialog.setText("Entry Detail has unsaved changes.")
        if same_bag_path:
            dialog.setInformativeText(
                "Save changes to the current bag entry before switching?\n\n"
                + "\n".join(f"- {change}" for change in changes)
            )
        else:
            dialog.setInformativeText(
                "Bag path changed, so saving will create a new bag entry before switching.\n\n"
                + "\n".join(f"- {change}" for change in changes)
            )
        save_button = dialog.addButton("Save", QtWidgets.QMessageBox.AcceptRole)
        discard_button = dialog.addButton("Discard", QtWidgets.QMessageBox.DestructiveRole)
        cancel_button = dialog.addButton(QtWidgets.QMessageBox.Cancel)
        dialog.setDefaultButton(save_button)
        dialog.exec_()
        clicked = dialog.clickedButton()
        if clicked == save_button:
            return "save"
        if clicked == discard_button:
            return "discard"
        if clicked == cancel_button:
            return "cancel"
        return "cancel"

    def _save_form_changes_for_entry(self, row: int) -> bool:
        if row < 0 or row >= len(self.entries):
            return False
        saved_entry = self.entries[row]
        current_path = self.bag_path_edit.text().strip()
        same_bag_path = Path(saved_entry.bag_path).expanduser() == Path(current_path).expanduser()
        if same_bag_path:
            entry = self._current_entry_from_form(
                metadata_override=(saved_entry.record_date, saved_entry.duration_seconds),
                has_elevator_flag_override=saved_entry.has_elevator_flag,
            )
            if entry is None:
                return False
            entry.bag_id = saved_entry.bag_id
            entry.record_date = saved_entry.record_date
            entry.duration_seconds = saved_entry.duration_seconds
            entry.has_elevator_flag = saved_entry.has_elevator_flag
            self.entries[row] = entry
        else:
            entry = self._current_entry_from_form()
            if entry is None:
                return False
            self.entries.append(entry)
        self._save_state()
        self._refresh_table()
        return True

    def _select_entry(self, row: int) -> None:
        for idx in range(self.tree.topLevelItemCount()):
            parent = self.tree.topLevelItem(idx)
            for child_idx in range(parent.childCount()):
                child = parent.child(child_idx)
                data = child.data(0, QtCore.Qt.UserRole)
                if data is not None and int(data) == row:
                    self.tree.setCurrentItem(child)
                    return

    def _browse_bag(self) -> None:
        current_path = self.bag_path_edit.text().strip()
        start_dir = WORKSPACE_ROOT
        if current_path:
            current_bag = Path(current_path).expanduser()
            if current_bag.is_file():
                start_dir = current_bag.parent
            elif current_bag.is_dir():
                start_dir = current_bag
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select Bag",
            str(start_dir),
            "ROS Bag (*.bag);;All Files (*)",
        )
        if path:
            self.bag_path_edit.setText(path)
            self.alias_edit.setText(Path(path).stem)
            self.note_edit.clear()

    def _complete_batch_entry(
        self,
        entry: Optional[BagEntry],
        result_text: str,
        fallback_label: str = "",
    ) -> None:
        label = entry.alias if entry is not None else (fallback_label or "unknown")
        self.batch_results.append(f"{label}: {result_text}")
        if entry is not None:
            if entry.bag_id not in self.batch_result_by_bag_id:
                self.batch_completed_count += 1
            self.batch_result_by_bag_id[entry.bag_id] = result_text
        else:
            self.batch_completed_count += 1

    def _open_batch_test_dialog(self) -> None:
        if self.session_running:
            QtWidgets.QMessageBox.information(self, "Busy", "A session is already running.")
            return
        active_entries = [entry for entry in self.entries if not entry.deleted]
        if not active_entries:
            QtWidgets.QMessageBox.information(self, "No Entries", "No bag entries are available for batch testing.")
            return
        dialog = BatchSelectionDialog(active_entries, self.batch_test_groups, self)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        self.batch_test_groups = dialog.batch_test_groups_result()
        self._save_state()
        bag_ids = dialog.selected_bag_ids()
        if not bag_ids:
            QtWidgets.QMessageBox.information(self, "No Selection", "Select at least one bag entry.")
            return
        self.batch_mode_active = True
        self.batch_cancelled = False
        self.batch_queue_ids = list(bag_ids)
        self.batch_run_ids = list(bag_ids)
        self.batch_total_count = len(bag_ids)
        self.batch_completed_count = 0
        self.batch_results = []
        self.batch_result_by_bag_id = {}
        self.batch_group_name = dialog.selected_batch_name()
        self.batch_started_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.batch_new_record_keys = []
        self.lio_log_edit.clear()
        self.rosbag_log_edit.clear()
        self._append_lio_log(
            f"[batch] starting batch test '{self.batch_group_name}' with {self.batch_total_count} entries"
        )
        self._start_next_batch_entry()

    def _start_next_batch_entry(self) -> None:
        if not self.batch_mode_active or self.session_running:
            return
        while self.batch_queue_ids:
            bag_id = self.batch_queue_ids.pop(0)
            entry = self._find_entry_by_bag_id(bag_id)
            if entry is None:
                self._complete_batch_entry(None, "missing entry", bag_id)
                continue
            self.pending_entry = entry
            self._set_status(
                f"Batch {self.batch_completed_count + 1}/{self.batch_total_count}: {entry.alias}"
                f"{self._batch_eta_suffix(include_current_pending=True)}"
            )
            self._append_lio_log(
                f"[batch] ({self.batch_completed_count + 1}/{self.batch_total_count}) starting {entry.alias}"
            )
            self._start_entry_session(entry)
            return
        self._finish_batch_mode()

    def _finish_batch_mode(self) -> None:
        if not self.batch_mode_active:
            return
        summary_title = "Batch Test Finished"
        if self.batch_cancelled:
            summary_title = "Batch Test Cancelled"
        summary_lines = self.batch_results or ["(no results)"]
        export_dir = None
        try:
            export_dir = self._export_batch_analysis_artifacts(summary_title, summary_lines)
        except OSError as exc:
            self._append_lio_log(f"[warn] batch export failed: {exc}")
        self.batch_mode_active = False
        self.batch_cancelled = False
        self.batch_queue_ids = []
        self.batch_run_ids = []
        self.batch_total_count = 0
        self.batch_completed_count = 0
        self.batch_result_by_bag_id = {}
        self.batch_group_name = ""
        self.batch_started_at = ""
        self.batch_new_record_keys = []
        self._set_status(summary_title)
        self._update_buttons()
        if export_dir is not None:
            summary_lines = summary_lines + [f"", f"Export: {export_dir}"]
        QtWidgets.QMessageBox.information(self, summary_title, "\n".join(summary_lines))

    def _open_settings(self) -> None:
        dialog = SettingsDialog(self.settings, self)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        self.settings = dialog.get_settings()
        self.fixed_yaml_badge.setText(f"Root YAML: {self.settings.root_config}")
        self._save_state()
        self._append_lio_log("[info] settings updated")

    def _load_selected_analysis_detail(self) -> None:
        self._update_buttons()

    def _analysis_detail_text(self, record: AnalysisRecord) -> str:
        detail_lines = [
            f"Bag: {record.bag_path}",
            f"Finished: {record.finished_at}",
            f"LIO YAML: {record.lio_yaml_name}",
            f"YAML Group: {record.yaml_group_name or 'yaml1'}",
            f"exit_commit count: {record.exit_commit_count}",
            f"Comment: {record.comment or '(empty)'}",
            "delta_z_commit values:",
        ]
        if record.yaml_diff_keys:
            detail_lines.append("YAML diff keys:")
            detail_lines.extend(
                f"  - {self._yaml_display_key_with_source(key)}" for key in record.yaml_diff_keys
            )
        if record.delta_z_values:
            detail_lines.extend(f"  {idx + 1}. {value:.6f}" for idx, value in enumerate(record.delta_z_values))
        else:
            detail_lines.append("  (none)")
        detail_lines.append(f"Total Z: {record.delta_z_total:.6f}")
        detail_lines.append(f"Total Path Length: {record.total_path_length:.6f}")
        detail_lines.append(f"Terminal error: {record.terminal_error:.6f}")
        detail_lines.append(f"Terminal z error: {record.terminal_z_error:.6f}")
        if record.terminal_pose:
            detail_lines.append(
                "Terminal pose: "
                f"x={record.terminal_pose[0]:.6f}, y={record.terminal_pose[1]:.6f}, z={record.terminal_pose[2]:.6f}, "
                f"qx={record.terminal_pose[3]:.6f}, qy={record.terminal_pose[4]:.6f}, "
                f"qz={record.terminal_pose[5]:.6f}, qw={record.terminal_pose[6]:.6f}"
            )
        detail_lines.append("Flag event times:")
        if record.flag_event_times:
            detail_lines.extend(f"  {idx + 1}. {value:.6f}" for idx, value in enumerate(record.flag_event_times))
        else:
            detail_lines.append("  (none)")
        if record.generated_bag_path:
            detail_lines.append(f"Generated bag: {record.generated_bag_path}")
        detail_lines.append(f"Attachments: {len(record.attachments)}")
        return "\n".join(detail_lines)

    def _save_analysis_comment(
        self,
        record_index: int,
        comment: str,
        detail_widget: Optional[QtWidgets.QPlainTextEdit] = None,
    ) -> None:
        if record_index < 0 or record_index >= len(self.analysis_records):
            return
        record = self.analysis_records[record_index]
        if record.comment == comment:
            return
        record.comment = comment
        self._save_state()
        if detail_widget is not None:
            detail_widget.setPlainText(self._analysis_detail_text(record))

    @staticmethod
    def _sanitize_filename_component(text: str) -> str:
        sanitized = re.sub(r"[^0-9A-Za-z._-]+", "_", text.strip())
        return sanitized.strip("_") or "item"

    def _analysis_attachment_dir(self, record: AnalysisRecord) -> Path:
        finished_tag = self._sanitize_filename_component(record.finished_at.replace(" ", "_").replace(":", "-"))
        return ANALYSIS_ATTACHMENT_ROOT / record.bag_id / finished_tag

    def _unique_child_path(self, directory: Path, name: str) -> Path:
        candidate = directory / name
        if not candidate.exists():
            return candidate
        stem = candidate.stem
        suffix = candidate.suffix
        for idx in range(2, 1000):
            numbered = directory / f"{stem}_{idx}{suffix}"
            if not numbered.exists():
                return numbered
        return directory / f"{stem}_{uuid.uuid4().hex[:8]}{suffix}"

    def _next_analysis_attachment_path(self, record: AnalysisRecord, suffix: str) -> Path:
        suffix = suffix if suffix.startswith(".") else f".{suffix}"
        directory = self._analysis_attachment_dir(record)
        directory.mkdir(parents=True, exist_ok=True)
        return directory / f"attachment_{uuid.uuid4().hex[:12]}{suffix.lower()}"

    def _refresh_analysis_attachment_list(self, widget: QtWidgets.QListWidget, record: AnalysisRecord) -> None:
        widget.clear()
        for attachment in record.attachments:
            path = Path(attachment)
            item = QtWidgets.QListWidgetItem(path.name)
            item.setToolTip(str(path))
            item.setData(QtCore.Qt.UserRole, str(path))
            pixmap = QtGui.QPixmap(str(path))
            if pixmap.isNull():
                fallback = QtGui.QPixmap(240, 150)
                fallback.fill(QtGui.QColor("#ece8df"))
                painter = QtGui.QPainter(fallback)
                painter.setPen(QtGui.QPen(QtGui.QColor("#516173")))
                label = "Folder" if path.is_dir() else path.suffix.upper().lstrip(".") or "File"
                painter.drawText(fallback.rect(), QtCore.Qt.AlignCenter, label)
                painter.end()
                pixmap = fallback
            item.setIcon(QtGui.QIcon(pixmap.scaled(240, 150, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)))
            widget.addItem(item)

    def _open_analysis_attachment(self, item: QtWidgets.QListWidgetItem) -> None:
        path = item.data(QtCore.Qt.UserRole)
        if not path:
            return
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))

    def _paste_analysis_attachment(self, record_index: int, widget: QtWidgets.QListWidget) -> None:
        if record_index < 0 or record_index >= len(self.analysis_records):
            return
        clipboard = QtWidgets.QApplication.clipboard()
        image = clipboard.image()
        if image.isNull():
            QtWidgets.QMessageBox.information(self, "No Image", "Clipboard does not contain an image.")
            return
        record = self.analysis_records[record_index]
        target = self._next_analysis_attachment_path(record, ".png")
        if not image.save(str(target), "PNG"):
            QtWidgets.QMessageBox.warning(self, "Paste Failed", "Failed to save clipboard image.")
            return
        record.attachments.append(str(target))
        self._save_state()
        self._refresh_analysis_attachment_list(widget, record)

    def _select_analysis_attachment(self, record_index: int, widget: QtWidgets.QListWidget) -> None:
        if record_index < 0 or record_index >= len(self.analysis_records):
            return
        paths, _ = QtWidgets.QFileDialog.getOpenFileNames(
            self,
            "Select Images",
            str(WORKSPACE_ROOT),
            "Images (*.png *.jpg *.jpeg *.bmp *.gif *.webp);;All Files (*)",
        )
        if not paths:
            return
        record = self.analysis_records[record_index]
        copied = 0
        for path_text in paths:
            source = Path(path_text)
            if not source.is_file():
                continue
            target = self._next_analysis_attachment_path(record, source.suffix or ".png")
            try:
                target.write_bytes(source.read_bytes())
            except OSError:
                continue
            record.attachments.append(str(target))
            copied += 1
        if copied == 0:
            QtWidgets.QMessageBox.warning(self, "Select Failed", "No images were imported.")
            return
        self._save_state()
        self._refresh_analysis_attachment_list(widget, record)

    def _delete_analysis_attachment(
        self,
        record_index: int,
        widget: QtWidgets.QListWidget,
        detail_widget: Optional[QtWidgets.QPlainTextEdit] = None,
    ) -> None:
        if record_index < 0 or record_index >= len(self.analysis_records):
            return
        item = widget.currentItem()
        if item is None:
            return
        path_text = str(item.data(QtCore.Qt.UserRole) or "")
        if not path_text:
            return
        path = Path(path_text)
        reply = QtWidgets.QMessageBox.question(
            self,
            "Delete Attachment",
            f"Delete attachment?\n\n{path.name}",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        record = self.analysis_records[record_index]
        record.attachments = [attachment for attachment in record.attachments if attachment != path_text]
        try:
            if path.is_dir():
                shutil.rmtree(path)
            else:
                path.unlink()
        except OSError:
            pass
        self._save_state()
        self._refresh_analysis_attachment_list(widget, record)
        if detail_widget is not None:
            detail_widget.setPlainText(self._analysis_detail_text(record))

    def _copy_path_into_directory(self, source: Path, directory: Path) -> Optional[Path]:
        if not source.exists():
            return None
        directory.mkdir(parents=True, exist_ok=True)
        target = self._unique_child_path(directory, source.name)
        try:
            if source.is_dir():
                shutil.copytree(source, target)
            else:
                shutil.copy2(source, target)
        except OSError as exc:
            self._append_lio_log(f"[warn] failed to copy artifact {source}: {exc}")
            return None
        return target

    def _export_analysis_record_bundle(self, record: AnalysisRecord, export_dir: Path) -> int:
        export_dir.mkdir(parents=True, exist_ok=True)
        copied = 0
        for attachment in record.attachments:
            copied_path = self._copy_path_into_directory(Path(attachment), export_dir)
            if copied_path is not None:
                copied += 1
        (export_dir / "analysis_detail.txt").write_text(self._analysis_detail_text(record), encoding="utf-8")
        (export_dir / "analysis_record.json").write_text(
            json.dumps(asdict(record), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        return copied

    def _batch_analysis_records(self) -> List[AnalysisRecord]:
        records: List[AnalysisRecord] = []
        used_indices: set[int] = set()
        for bag_id, finished_at in self.batch_new_record_keys:
            for idx, record in enumerate(self.analysis_records):
                if idx in used_indices:
                    continue
                if record.bag_id == bag_id and record.finished_at == finished_at:
                    records.append(record)
                    used_indices.add(idx)
                    break
        return records

    def _export_batch_analysis_artifacts(self, summary_title: str, summary_lines: List[str]) -> Path:
        now = datetime.now()
        finished_at = now.strftime("%Y-%m-%d %H:%M:%S")
        finished_tag = now.strftime("%Y%m%d_%H%M%S")
        batch_name = self.batch_group_name or "Manual Selection"
        batch_component = self._sanitize_filename_component(batch_name)
        export_dir = self._unique_child_path(BATCH_EXPORT_ROOT, f"batch_{finished_tag}_{batch_component}")
        export_dir.mkdir(parents=True, exist_ok=True)

        records_by_bag_id: Dict[str, List[AnalysisRecord]] = {}
        for record in self._batch_analysis_records():
            records_by_bag_id.setdefault(record.bag_id, []).append(record)
        for records in records_by_bag_id.values():
            records.sort(key=lambda item: item.finished_at or "")

        rows = []
        for order, bag_id in enumerate(self.batch_run_ids, start=1):
            entry = self._find_entry_by_bag_id(bag_id)
            alias = entry.alias if entry is not None else bag_id
            entry_dir_name = f"{order:02d}_{self._sanitize_filename_component(alias)}"
            entry_dir = export_dir / entry_dir_name
            entry_dir.mkdir(parents=True, exist_ok=True)
            bag_records = records_by_bag_id.get(bag_id, [])
            copied_artifacts = 0
            for record_index, record in enumerate(bag_records, start=1):
                record_tag = self._sanitize_filename_component(
                    record.finished_at.replace(" ", "_").replace(":", "-")
                )
                record_dir = entry_dir / f"{record_index:02d}_{record_tag}"
                copied_artifacts += self._export_analysis_record_bundle(record, record_dir)

            latest_record = bag_records[-1] if bag_records else None
            status = self.batch_result_by_bag_id.get(bag_id, "not run")
            entry_summary = {
                "order": order,
                "alias": alias,
                "entry_group": entry.group if entry is not None else "",
                "bag_id": bag_id,
                "bag_path": entry.bag_path if entry is not None else "",
                "play_rate": entry.play_rate if entry is not None else "",
                "status": status,
                "analysis_count": len(bag_records),
                "copied_artifacts": copied_artifacts,
            }
            if latest_record is not None:
                entry_summary.update(
                    {
                        "latest_finished_at": latest_record.finished_at,
                        "exit_commit_count": latest_record.exit_commit_count,
                        "delta_z_total": latest_record.delta_z_total,
                        "total_path_length": latest_record.total_path_length,
                        "terminal_error": latest_record.terminal_error,
                        "terminal_z_error": latest_record.terminal_z_error,
                    }
                )
            else:
                entry_summary.update(
                    {
                        "latest_finished_at": "",
                        "exit_commit_count": "",
                        "delta_z_total": "",
                        "total_path_length": "",
                        "terminal_error": "",
                        "terminal_z_error": "",
                    }
                )
            (entry_dir / "entry_summary.json").write_text(
                json.dumps(entry_summary, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
            rows.append({**entry_summary, "export_subdir": entry_dir_name})

        csv_fields = [
            "order",
            "alias",
            "entry_group",
            "bag_id",
            "bag_path",
            "play_rate",
            "status",
            "analysis_count",
            "copied_artifacts",
            "latest_finished_at",
            "exit_commit_count",
            "delta_z_total",
            "total_path_length",
            "terminal_error",
            "terminal_z_error",
            "export_subdir",
        ]
        with (export_dir / "batch_summary.csv").open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=csv_fields)
            writer.writeheader()
            writer.writerows(rows)

        (export_dir / "batch_summary.json").write_text(
            json.dumps(
                {
                    "batch_name": batch_name,
                    "started_at": self.batch_started_at,
                    "finished_at": finished_at,
                    "status": summary_title,
                    "results": list(summary_lines),
                    "entries": rows,
                },
                indent=2,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )
        result_text = [
            f"Batch: {batch_name}",
            f"Started: {self.batch_started_at}",
            f"Finished: {finished_at}",
            f"Status: {summary_title}",
            "",
            *summary_lines,
        ]
        (export_dir / "batch_results.txt").write_text("\n".join(result_text) + "\n", encoding="utf-8")
        self._append_lio_log(f"[batch] exported batch artifacts to {export_dir}")
        return export_dir

    def _export_selected_analysis_artifacts(self) -> None:
        row = self._selected_analysis_row()
        if row < 0 or row >= len(self.analysis_records):
            QtWidgets.QMessageBox.information(self, "No Selection", "Select one analysis record first.")
            return
        record = self.analysis_records[row]
        base_dir = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "Export Analysis Artifacts",
            str(WORKSPACE_ROOT),
        )
        if not base_dir:
            return
        finished_tag = self._sanitize_filename_component(record.finished_at.replace(" ", "_").replace(":", "-"))
        export_dir = self._unique_child_path(Path(base_dir), f"bag_runner_export_{finished_tag}")
        copied = self._export_analysis_record_bundle(record, export_dir)
        QtWidgets.QMessageBox.information(
            self,
            "Export Finished",
            f"Exported {copied} artifact(s) to:\n{export_dir}",
        )

    def _validate_runtime_paths(self, entry: BagEntry, external_profile: Optional[ExternalLioProfile] = None) -> bool:
        errors = []
        if not Path(entry.bag_path).is_file():
            errors.append(f"Bag not found: {entry.bag_path}")
        if external_profile is None:
            if not Path(self.settings.setup_script).is_file():
                errors.append(f"setup.bash not found: {self.settings.setup_script}")
            if not Path(self.settings.lio_executable).is_file():
                errors.append(f"LIO executable not found: {self.settings.lio_executable}")
            if not Path(self.settings.libusb_path).is_file():
                errors.append(f"libusb not found: {self.settings.libusb_path}")
            root_config_path = self._root_config_path()
            if not root_config_path.is_file():
                errors.append(f"Root YAML not found: {root_config_path}")
            elif self._root_config_argument() is None:
                errors.append(f"Root YAML must be inside {YAML_ROOT}: {root_config_path}")
            else:
                root_data = self._load_yaml_file(root_config_path)
                if not isinstance(root_data, dict):
                    errors.append(f"Failed to parse root YAML: {root_config_path}")
                else:
                    for selector in ("sensor_config", "runtime_config", "logging_config"):
                        selected_file = str(root_data.get(selector, "")).strip()
                        if not selected_file:
                            errors.append(f"Root YAML is missing '{selector}': {root_config_path}")
                            continue
                        selected_path = YAML_ROOT / selected_file
                        if not selected_path.is_file():
                            errors.append(f"Selected YAML not found ({selector}): {selected_path}")
            if self._should_start_rviz() and not RVIZ_CONFIG.is_file():
                errors.append(f"RViz config not found: {RVIZ_CONFIG}")
        else:
            if not Path(external_profile.setup_script).is_file():
                errors.append(f"{external_profile.display_name} setup.bash not found: {external_profile.setup_script}")
        if errors:
            QtWidgets.QMessageBox.critical(self, "Invalid Runtime Paths", "\n".join(errors))
            return False
        return True

    def _should_start_rviz(self) -> bool:
        return self.settings.enable_rviz and not self.batch_mode_active and self.current_external_lio_profile is None

    def _find_entry_by_bag_id(self, bag_id: str) -> Optional[BagEntry]:
        for entry in self.entries:
            if entry.bag_id == bag_id:
                return entry
        return None

    def _bag_contains_elevator_flag(self, bag_path: Path) -> bool:
        if not bag_path.is_file():
            return False
        try:
            with rosbag.Bag(str(bag_path), "r") as bag:
                topics = bag.get_type_and_topic_info().topics
                return "/LIO/set_elevator_flag" in topics
        except Exception:
            return False

    @staticmethod
    def _entries_match(left: BagEntry, right: BagEntry) -> bool:
        return (
            left.alias == right.alias
            and left.group == right.group
            and left.bag_path == right.bag_path
            and left.marker == right.marker
            and left.marker_note == right.marker_note
            and abs(left.play_rate - right.play_rate) < 1e-9
            and left.note == right.note
            and left.has_elevator_flag == right.has_elevator_flag
        )

    @staticmethod
    def _entry_change_summary(left: BagEntry, right: BagEntry) -> List[str]:
        changes = []
        if left.group != right.group:
            changes.append(f"Group: {left.group} -> {right.group}")
        if left.alias != right.alias:
            changes.append(f"Alias: {left.alias} -> {right.alias}")
        if left.bag_path != right.bag_path:
            changes.append(f"Bag Path: {left.bag_path} -> {right.bag_path}")
        if abs(left.play_rate - right.play_rate) >= 1e-9:
            changes.append(f"Play Rate: {left.play_rate:.2f} -> {right.play_rate:.2f}")
        if left.marker != right.marker:
            changes.append(f"Mark: {left.marker or 'none'} -> {right.marker or 'none'}")
        if left.marker_note != right.marker_note:
            changes.append(
                f"Mark Note: {(left.marker_note or '(empty)')} -> {(right.marker_note or '(empty)')}"
            )
        if left.note != right.note:
            changes.append(f"Note: {(left.note or '(empty)')} -> {(right.note or '(empty)')}")
        if left.has_elevator_flag != right.has_elevator_flag:
            changes.append(
                f"Manual Trigger: {left.has_elevator_flag} -> {right.has_elevator_flag}"
            )
        return changes

    def _resolve_entry_for_start(self, row: int) -> Optional[BagEntry]:
        saved_entry = self.entries[row]
        form_entry = self._current_entry_from_form()
        if form_entry is None:
            return None
        if self._entries_match(saved_entry, form_entry):
            return saved_entry
        changes = self._entry_change_summary(saved_entry, form_entry)
        same_bag_path = Path(saved_entry.bag_path).expanduser() == Path(form_entry.bag_path).expanduser()

        dialog = QtWidgets.QMessageBox(self)
        dialog.setIcon(QtWidgets.QMessageBox.Question)
        dialog.setWindowTitle("Entry Changed")
        dialog.setText("Current form values differ from the selected bag entry.")
        if same_bag_path:
            informative_text = (
                "Bag path is unchanged, so this will update the selected entry and keep its analysis results.\n\n"
                + "\n".join(f"- {change}" for change in changes)
            )
        else:
            informative_text = (
                "Bag path changed, so this cannot update the selected entry. "
                "To keep analysis results bound to the original bag, you must create a new entry.\n\n"
                + "\n".join(f"- {change}" for change in changes)
            )
        dialog.setInformativeText(informative_text)
        update_button = None
        create_button = None
        if same_bag_path:
            update_button = dialog.addButton("Update Selected", QtWidgets.QMessageBox.AcceptRole)
        else:
            create_button = dialog.addButton("Create New", QtWidgets.QMessageBox.ActionRole)
        cancel_button = dialog.addButton(QtWidgets.QMessageBox.Cancel)
        dialog.setDefaultButton(update_button if update_button is not None else create_button)
        dialog.exec_()

        clicked = dialog.clickedButton()
        if update_button is not None and clicked == update_button:
            form_entry.bag_id = saved_entry.bag_id
            form_entry.record_date = saved_entry.record_date
            form_entry.duration_seconds = saved_entry.duration_seconds
            form_entry.has_elevator_flag = saved_entry.has_elevator_flag
            self.entries[row] = form_entry
            self._save_state()
            self._refresh_table()
            self._select_entry(row)
            return self.entries[row]
        if create_button is not None and clicked == create_button:
            self.entries.append(form_entry)
            self._save_state()
            self._refresh_table()
            self._select_entry(len(self.entries) - 1)
            return self.entries[-1]
        if clicked == cancel_button:
            return None
        return None

    def _start_selected_entry(self) -> None:
        if self.session_running:
            QtWidgets.QMessageBox.information(self, "Busy", "A session is already running.")
            return
        row = self._selected_row()
        if row < 0:
            QtWidgets.QMessageBox.warning(self, "No Selection", "Select one bag entry first.")
            return

        entry = self._resolve_entry_for_start(row)
        if entry is None:
            return
        self.lio_log_edit.clear()
        self.rosbag_log_edit.clear()
        self._start_entry_session(entry)

    def _start_other_lio_entry(self) -> None:
        if self.session_running:
            QtWidgets.QMessageBox.information(self, "Busy", "A session is already running.")
            return
        row = self._selected_row()
        if row < 0 or row >= len(self.entries):
            QtWidgets.QMessageBox.warning(self, "No Selection", "Select one bag entry first.")
            return
        entry = self.entries[row]
        if entry.deleted:
            QtWidgets.QMessageBox.warning(self, "Deleted Entry", "Recover this bag entry before starting it.")
            return
        dialog = OtherLioDialog(self.external_lio_profiles, self)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        profile = dialog.selected_profile()
        if profile is None:
            return
        self.lio_log_edit.clear()
        self.rosbag_log_edit.clear()
        self._start_entry_session(entry, external_profile=profile)

    def _start_entry_session(self, entry: BagEntry, external_profile: Optional[ExternalLioProfile] = None) -> None:
        if not self._validate_runtime_paths(entry, external_profile):
            return
        self.current_external_lio_profile = external_profile
        self.current_session_records_analysis = external_profile is None
        self.pending_entry = entry
        self.session_running = True
        self.session_start_epoch = time.time()
        self.current_log_session_dir = None
        self.playback_started_wall = 0.0
        self.playback_paused = False
        self.pause_started_wall = 0.0
        self.total_paused_wall = 0.0
        self.flag_event_times = []
        self.current_pause_service = ""
        self.current_analysis_index = -1
        self.lio_ready_detected = False
        self.lio_startup_error_detected = False
        self.waiting_for_lio_analysis = False
        try:
            with rosbag.Bag(entry.bag_path, "r") as bag:
                self.current_bag_start_time = bag.get_start_time()
                self.current_bag_end_time = bag.get_end_time()
        except Exception:
            self.current_bag_start_time = 0.0
            self.current_bag_end_time = 0.0
        self.status_eta_timer.start()
        self._update_buttons()
        if external_profile is None and self.settings.build_before_run:
            self._set_status("Building workspace")
            workspace_build = WORKSPACE_ROOT / "build"
            if (workspace_build / "CMakeCache.txt").is_file():
                command = f"exec cmake --build {shlex.quote(str(workspace_build))} -j2"
            else:
                command = "exec catkin_make"
            self.build_proc.start(command, WORKSPACE_ROOT)
        else:
            self._ensure_roscore_then_start_lio()

    def _ensure_roscore_then_start_lio(self) -> None:
        if self._roscore_ready():
            self.started_roscore = False
            self._start_lio()
            return
        self.started_roscore = True
        self._set_status(self._status_with_batch_progress("Starting roscore"))
        self.roscore_proc.start("exec roscore", WORKSPACE_ROOT)
        QtCore.QTimer.singleShot(1500, self._wait_roscore_ready)

    def _wait_roscore_ready(self) -> None:
        if not self.session_running:
            return
        if self._roscore_ready():
            self._append_lio_log("[info] roscore is ready")
            self._start_lio()
            return
        QtCore.QTimer.singleShot(1000, self._wait_roscore_ready)

    def _roscore_ready(self) -> bool:
        setup = shlex.quote(self._current_session_setup_script())
        cmd = f"source {setup} >/dev/null 2>&1 && rosparam get /rosversion >/dev/null 2>&1"
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", cmd],
                cwd=str(WORKSPACE_ROOT),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
                timeout=2.0,
            )
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            return False

    def _lio_node_ready(self) -> bool:
        setup = shlex.quote(self._current_session_setup_script())
        node_pattern = r"^/(lio|lio_node)$"
        if self.current_external_lio_profile is not None:
            node_pattern = self.current_external_lio_profile.ready_node_pattern
        cmd = f"source {setup} >/dev/null 2>&1 && rosnode list 2>/dev/null | grep -Eq {shlex.quote(node_pattern)}"
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", cmd],
                cwd=str(WORKSPACE_ROOT),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
                timeout=2.0,
            )
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            return False

    def _start_lio(self) -> None:
        if self.pending_entry is None:
            return
        if self.current_external_lio_profile is not None:
            profile = self.current_external_lio_profile
            setup = shlex.quote(profile.setup_script)
            external_workspace = Path(profile.setup_script).expanduser().resolve().parents[1]
            libusb = shlex.quote(str(SYSTEM_LIBUSB))
            extra_lib_paths = ":".join(profile.extra_lib_paths)
            ld_library_path = (
                f"/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/noetic/lib:{external_workspace}/devel/lib"
            )
            if extra_lib_paths:
                ld_library_path = f"{ld_library_path}:{extra_lib_paths}"
            cmd = (
                "unset ALLUSERSPROFILE MVCAM_COMMON_RUNENV MVCAM_GENICAM_CLPROTOCOL MVCAM_SDK_PATH MVCAM_SDK_VERSION;"
                "unset GENICAM_GENTL32_PATH GENICAM_GENTL64_PATH;"
                f"source {setup};"
                f"export LD_LIBRARY_PATH=\"{ld_library_path}\";"
                f"export LD_PRELOAD={libusb};"
                f"{profile.launch_command}"
            )
            self._append_lio_log(f"[info] starting external LIO: {profile.display_name}")
            self._set_status(self._status_with_batch_progress(f"Starting {profile.display_name} for {self.pending_entry.alias}"))
            self.lio_proc.start(f"exec setsid bash -lc {shlex.quote(cmd)}", WORKSPACE_ROOT)
            self.wait_elapsed_ms = 0
            self.lio_startup_progress_detected = False
            self.node_wait_timer.start()
            QtCore.QTimer.singleShot(0, self._poll_lio_node)
            return
        setup = shlex.quote(self.settings.setup_script)
        lio_exe = shlex.quote(self.settings.lio_executable)
        libusb = shlex.quote(self.settings.libusb_path)
        config_name = shlex.quote(self._root_config_argument() or FIXED_CONFIG_NAME)
        cmd = (
            "unset ALLUSERSPROFILE MVCAM_COMMON_RUNENV MVCAM_GENICAM_CLPROTOCOL MVCAM_SDK_PATH MVCAM_SDK_VERSION;"
            "unset GENICAM_GENTL32_PATH GENICAM_GENTL64_PATH;"
            f"source {setup};"
            f"export LD_LIBRARY_PATH=\"/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/noetic/lib:{WORKSPACE_ROOT}/devel/lib\";"
            f"export LD_PRELOAD={libusb};"
            f"exec {lio_exe} _config_path:={config_name}"
        )
        self._set_status(self._status_with_batch_progress(f"Starting LIO for {self.pending_entry.alias}"))
        self.lio_proc.start(f"exec setsid bash -lc {shlex.quote(cmd)}", WORKSPACE_ROOT)
        self.wait_elapsed_ms = 0
        self.lio_startup_progress_detected = False
        self.node_wait_timer.start()
        QtCore.QTimer.singleShot(0, self._poll_lio_node)
        if self._should_start_rviz():
            self._start_rviz()

    def _handle_lio_ready_detected(self) -> None:
        if not self.session_running or self.pending_entry is None or self.lio_ready_detected:
            return
        self.lio_ready_detected = True
        if self.node_wait_timer.isActive():
            self.node_wait_timer.stop()
        self._append_lio_log("[info] lio node is ready")
        self._start_bag_play()

    def _start_rviz(self) -> None:
        setup = shlex.quote(self.settings.setup_script)
        rviz_cfg = shlex.quote(str(RVIZ_CONFIG))
        cmd = f"source {setup}; export DISABLE_ROS1_EOL_WARNINGS=1; exec rviz -d {rviz_cfg}"
        self._append_lio_log(f"[info] starting rviz with {RVIZ_CONFIG}")
        self.rviz_proc.start(f"exec setsid bash -lc {shlex.quote(cmd)}", WORKSPACE_ROOT)

    def _poll_lio_node(self) -> None:
        if not self.session_running:
            self.node_wait_timer.stop()
            return
        if self.lio_ready_detected:
            self.node_wait_timer.stop()
            return
        if self._lio_node_ready():
            self._handle_lio_ready_detected()
            return
        if self.lio_startup_error_detected:
            self.node_wait_timer.stop()
            tail = self._lio_log_tail()
            if self.batch_mode_active and self.pending_entry is not None:
                self._complete_batch_entry(self.pending_entry, "lio startup error")
                if tail:
                    self._append_lio_log(f"[batch] lio startup error for {self.pending_entry.alias}\n{tail}")
            else:
                message = "LIO reported a startup error before the node became ready."
                if tail:
                    message += f"\n\nRecent LIO output:\n{tail}"
                QtWidgets.QMessageBox.critical(self, "LIO Start Failed", message)
            self._stop_session()
            return
        if self.lio_startup_progress_detected and self.wait_elapsed_ms >= 3000:
            self._append_lio_log("[warn] rosnode readiness check is slow; falling back to startup-log readiness")
            self._handle_lio_ready_detected()
            return
        waited_s = self.wait_elapsed_ms / 1000.0
        if self.pending_entry is not None:
            self._set_status(
                self._status_with_batch_progress(
                    f"Waiting for LIO node ({waited_s:.0f}s/{WAIT_TIMEOUT_MS / 1000:.0f}s): {self.pending_entry.alias}"
                )
            )
        if not self.lio_startup_progress_detected and self.wait_elapsed_ms >= 8000:
            self.node_wait_timer.stop()
            tail = self._lio_log_tail()
            message = "LIO produced no recognizable startup output in time."
            if tail:
                message += f"\n\nRecent LIO output:\n{tail}"
            if self.batch_mode_active and self.pending_entry is not None:
                self._complete_batch_entry(self.pending_entry, "no startup progress")
                if tail:
                    self._append_lio_log(f"[batch] no startup progress for {self.pending_entry.alias}\n{tail}")
            else:
                QtWidgets.QMessageBox.critical(self, "LIO Start Failed", message)
            self._stop_session()
            return
        if not self.lio_proc.is_running():
            self.node_wait_timer.stop()
            tail = self._lio_log_tail()
            if self.batch_mode_active and self.pending_entry is not None:
                self._complete_batch_entry(self.pending_entry, "lio start failed")
                if tail:
                    self._append_lio_log(f"[batch] lio start failed for {self.pending_entry.alias}\n{tail}")
            else:
                message = "lio process exited before the node became ready."
                if tail:
                    message += f"\n\nRecent LIO output:\n{tail}"
                QtWidgets.QMessageBox.critical(self, "LIO Start Failed", message)
            self._stop_session()
            return
        self.wait_elapsed_ms += self.node_wait_timer.interval()
        if self.wait_elapsed_ms >= WAIT_TIMEOUT_MS:
            self.node_wait_timer.stop()
            tail = self._lio_log_tail()
            if self.batch_mode_active and self.pending_entry is not None:
                self._complete_batch_entry(self.pending_entry, "lio node timeout")
                if tail:
                    self._append_lio_log(f"[batch] lio node timeout for {self.pending_entry.alias}\n{tail}")
            else:
                message = "lio node did not appear in time."
                if tail:
                    message += f"\n\nRecent LIO output:\n{tail}"
                QtWidgets.QMessageBox.critical(self, "LIO Start Failed", message)
            self._stop_session()

    def _start_bag_play(self) -> None:
        if self.pending_entry is None:
            return
        setup = shlex.quote(self._current_session_setup_script())
        bag_path = shlex.quote(self.pending_entry.bag_path)
        rate = self.pending_entry.play_rate
        cmd = f"source {setup}; exec rosbag play -r {rate} {bag_path}"
        self._set_status(self._status_with_batch_progress(f"Playing bag at {rate:.2f}x"))
        self.playback_started_wall = time.time()
        self.playback_paused = False
        self.pause_started_wall = 0.0
        self.total_paused_wall = 0.0
        self.play_proc.start(cmd, WORKSPACE_ROOT)

    def _toggle_pause_playback(self) -> None:
        if not self.play_proc.is_running():
            QtWidgets.QMessageBox.information(self, "Not Playing", "rosbag is not currently playing.")
            return
        requested_pause = not self.playback_paused
        self.play_proc.write_stdin(b" ")
        now = time.time()
        if requested_pause:
            self.playback_paused = True
            self.pause_started_wall = now
            self._set_status("Playback paused")
            self._append_rosbag_log("[control] playback paused")
        else:
            if self.pause_started_wall > 0.0:
                self.total_paused_wall += max(0.0, now - self.pause_started_wall)
            self.pause_started_wall = 0.0
            self.playback_paused = False
            if self.pending_entry is not None:
                self._set_status(f"Playing bag at {self.pending_entry.play_rate:.2f}x")
            else:
                self._set_status("Playing bag")
            self._append_rosbag_log("[control] playback resumed")
        self._update_buttons()

    def _current_bag_time(self) -> Optional[float]:
        if self.playback_started_wall <= 0.0 or self.pending_entry is None:
            return None
        now = time.time()
        paused_extra = self.total_paused_wall
        if self.playback_paused and self.pause_started_wall > 0.0:
            paused_extra += max(0.0, now - self.pause_started_wall)
        played_wall = max(0.0, now - self.playback_started_wall - paused_extra)
        bag_time = self.current_bag_start_time + played_wall * self.pending_entry.play_rate
        if self.current_bag_end_time > self.current_bag_start_time:
            bag_time = min(bag_time, self.current_bag_end_time)
        return bag_time

    def _publish_elevator_flag(self) -> None:
        if not self.current_session_records_analysis:
            QtWidgets.QMessageBox.information(self, "Unavailable", "Set Elevator Flag is only available for the internal LIO run.")
            return
        if not self.play_proc.is_running():
            QtWidgets.QMessageBox.information(self, "Not Playing", "rosbag is not currently playing.")
            return
        bag_time = self._current_bag_time()
        if bag_time is None:
            QtWidgets.QMessageBox.warning(self, "No Timestamp", "Could not determine current bag timestamp.")
            return
        setup = shlex.quote(self.settings.setup_script)
        cmd = f"source {setup} >/dev/null 2>&1 && rostopic pub /LIO/set_elevator_flag std_msgs/Bool \"data: true\" -1"
        try:
            started = QtCore.QProcess.startDetached("/bin/bash", ["-lc", cmd], str(WORKSPACE_ROOT))
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Publish Failed", f"Failed to start rostopic pub:\n{exc}")
            return
        if isinstance(started, tuple):
            ok = bool(started[0])
        else:
            ok = bool(started)
        if not ok:
            QtWidgets.QMessageBox.warning(self, "Publish Failed", "Failed to start rostopic pub.")
            return
        self.flag_event_times.append(bag_time)
        self._append_rosbag_log(f"[control] published /LIO/set_elevator_flag at bag time {bag_time:.6f}")
        self._append_lio_log(f"[control] published /LIO/set_elevator_flag at bag time {bag_time:.6f}")

    def _any_managed_process_running(self) -> bool:
        return any(
            proc.is_running()
            for proc in [self.build_proc, self.play_proc, self.rviz_proc, self.lio_proc, self.roscore_proc]
        )

    def _stop_session(self) -> None:
        if self.stopping_session or not self._any_managed_process_running():
            return
        if self.batch_mode_active:
            if self.pending_entry is not None and self.pending_entry.bag_id not in self.batch_result_by_bag_id:
                self._complete_batch_entry(self.pending_entry, "cancelled")
            self.batch_cancelled = True
            self.batch_queue_ids = []
        self.node_wait_timer.stop()
        self.stopping_session = True
        self.stop_deadline_wall = time.time() + 8.0
        for proc in [self.build_proc, self.play_proc, self.rviz_proc, self.lio_proc]:
            proc.terminate()
        if self.started_roscore and self.roscore_proc.is_running():
            self.roscore_proc.terminate()
        self._set_status("Stopping session")
        self.stop_timer.start()
        self._update_buttons()

    def _force_stop_remaining(self) -> None:
        for proc in [self.play_proc, self.rviz_proc, self.lio_proc, self.roscore_proc]:
            if proc.is_running():
                proc.kill()

    def _poll_stop_session(self) -> None:
        if not self.stopping_session:
            self.stop_timer.stop()
            return
        runtime_procs = [self.build_proc, self.play_proc, self.rviz_proc, self.lio_proc]
        if any(proc.is_running() for proc in runtime_procs):
            if time.time() > self.stop_deadline_wall:
                self._force_stop_remaining()
                self.stop_timer.stop()
                self._finish_session("Session stopped")
            return
        if self.started_roscore and self.roscore_proc.is_running():
            self.roscore_proc.terminate()
            if time.time() > self.stop_deadline_wall:
                self.roscore_proc.kill()
            return
        self.stop_timer.stop()
        self._finish_session("Session stopped")

    def _maybe_close_rviz_after_playback(self) -> None:
        if not self.rviz_proc.is_running():
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            "Close RViz",
            "Playback finished. Close RViz as well?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self._append_lio_log("[info] closing rviz after playback")
            self.rviz_proc.terminate()
        else:
            self._append_lio_log("[info] keeping rviz open after playback")

    def _handle_external_playback_finished(self) -> None:
        reply = QtWidgets.QMessageBox.question(
            self,
            "Close External LIO",
            "Playback finished. Close the external LIO and RViz as well?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply == QtWidgets.QMessageBox.Yes:
            if self.lio_proc.is_running():
                self.lio_proc.terminate()
            if self.started_roscore and self.roscore_proc.is_running():
                self.roscore_proc.terminate()
            self._maybe_close_rviz_after_playback()
            self._finish_session("Playback finished")
            return
        self._append_lio_log("[info] keeping external LIO running after playback")
        self._set_status("Playback finished; external LIO still running")
        self._update_buttons()

    def _finish_session(self, status: str) -> None:
        self._maybe_generate_flagged_bag()
        self.stop_timer.stop()
        self.status_eta_timer.stop()
        self.session_running = False
        self.stopping_session = False
        self.pending_entry = None
        self.started_roscore = False
        self.session_start_epoch = 0.0
        self.stop_deadline_wall = 0.0
        self.playback_started_wall = 0.0
        self.playback_paused = False
        self.pause_started_wall = 0.0
        self.total_paused_wall = 0.0
        self.current_bag_start_time = 0.0
        self.current_bag_end_time = 0.0
        self.current_pause_service = ""
        self.current_log_session_dir = None
        self.flag_event_times = []
        self.current_analysis_index = -1
        self.lio_ready_detected = False
        self.current_external_lio_profile = None
        self.current_session_records_analysis = True
        self.waiting_for_lio_analysis = False
        self._set_status(status)
        self._update_buttons()
        if self.batch_mode_active and self.flag_generation_thread is None and not self.close_requested:
            if self.batch_cancelled or not self.batch_queue_ids:
                QtCore.QTimer.singleShot(0, self._finish_batch_mode)
            else:
                QtCore.QTimer.singleShot(0, self._start_next_batch_entry)
            return
        if self.close_requested and self.flag_generation_thread is None:
            self._save_state()
            QtCore.QTimer.singleShot(0, self.close)

    def _start_flagged_bag_generation(self, source_bag: Path, event_times: List[float], bag_id: str, analysis_index: int) -> None:
        self.flag_generation_bag_id = bag_id
        self.flag_generation_analysis_index = analysis_index
        self.flag_generation_event_times = list(event_times)
        self._set_status("Generating with_flag bag")
        self._append_rosbag_log(f"[control] generating flagged bag from {source_bag}")

        dialog = QtWidgets.QProgressDialog("Preparing bag rewrite...", "", 0, 100, self)
        dialog.setWindowTitle("Generating with_flag Bag")
        dialog.setWindowModality(QtCore.Qt.ApplicationModal)
        dialog.setMinimumDuration(0)
        dialog.setAutoClose(False)
        dialog.setAutoReset(False)
        dialog.setCancelButton(None)
        dialog.setValue(0)
        self.flag_generation_dialog = dialog

        thread = QtCore.QThread(self)
        worker = FlaggedBagGenerationWorker(str(source_bag), list(event_times))
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.progress.connect(self._on_flag_generation_progress)
        worker.finished.connect(self._on_flag_generation_finished)
        worker.failed.connect(self._on_flag_generation_failed)
        worker.finished.connect(thread.quit)
        worker.failed.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        worker.failed.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)

        self.flag_generation_thread = thread
        self.flag_generation_worker = worker
        dialog.show()
        thread.start()

    def _cleanup_flag_generation(self) -> None:
        if self.flag_generation_dialog is not None:
            self.flag_generation_dialog.close()
            self.flag_generation_dialog.deleteLater()
            self.flag_generation_dialog = None
        self.flag_generation_thread = None
        self.flag_generation_worker = None

    def _on_flag_generation_progress(self, current: int, total: int, message: str) -> None:
        if self.flag_generation_dialog is None:
            return
        total = max(total, 1)
        self.flag_generation_dialog.setMaximum(total)
        self.flag_generation_dialog.setValue(min(current, total))
        self.flag_generation_dialog.setLabelText(message)

    def _on_flag_generation_finished(self, generated_path: str) -> None:
        self._cleanup_flag_generation()
        bag_id = self.flag_generation_bag_id
        analysis_index = self.flag_generation_analysis_index
        event_times = list(self.flag_generation_event_times)
        self.flag_generation_bag_id = ""
        self.flag_generation_analysis_index = -1
        self.flag_generation_event_times = []

        generated = Path(generated_path)
        for entry in self.entries:
            if entry.bag_id == bag_id:
                entry.bag_path = generated_path
                entry.has_elevator_flag = True
                entry.record_date, entry.duration_seconds = self._read_bag_metadata(generated)
                break
        self._refresh_table(preserve_form=True)
        self._append_rosbag_log(f"[control] generated flagged bag: {generated_path}")

        if 0 <= analysis_index < len(self.analysis_records):
            latest = self.analysis_records[analysis_index]
            latest.flag_event_times = event_times
            latest.generated_bag_path = generated_path
        self._save_state()
        self._refresh_analysis_table()
        self._set_status("Flagged bag generated")
        if self.batch_mode_active and not self.close_requested:
            if self.batch_cancelled or not self.batch_queue_ids:
                QtCore.QTimer.singleShot(0, self._finish_batch_mode)
            else:
                QtCore.QTimer.singleShot(0, self._start_next_batch_entry)
            return
        if self.close_requested:
            QtCore.QTimer.singleShot(0, self.close)

    def _on_flag_generation_failed(self, error_text: str) -> None:
        self._cleanup_flag_generation()
        analysis_index = self.flag_generation_analysis_index
        event_times = list(self.flag_generation_event_times)
        self.flag_generation_bag_id = ""
        self.flag_generation_analysis_index = -1
        self.flag_generation_event_times = []

        if 0 <= analysis_index < len(self.analysis_records):
            latest = self.analysis_records[analysis_index]
            latest.flag_event_times = event_times
            latest.generated_bag_path = ""
        self._save_state()
        self._refresh_analysis_table()
        self._set_status("Generate failed")
        QtWidgets.QMessageBox.warning(self, "Generate Failed", f"Failed to create with_flag bag:\n{error_text}")
        if self.batch_mode_active and not self.close_requested:
            if self.batch_cancelled or not self.batch_queue_ids:
                QtCore.QTimer.singleShot(0, self._finish_batch_mode)
            else:
                QtCore.QTimer.singleShot(0, self._start_next_batch_entry)
            return
        if self.close_requested:
            QtCore.QTimer.singleShot(0, self.close)

    def _maybe_generate_flagged_bag(self) -> None:
        if not self.current_session_records_analysis or not self.flag_event_times or self.pending_entry is None:
            return
        reply = QtWidgets.QMessageBox.question(
            self,
            "Generate with_flag Bag",
            "Playback used /LIO/set_elevator_flag. Generate a new bag with suffix 'with_flag' and update this dataset to point to it?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.Yes,
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self._start_flagged_bag_generation(
                Path(self.pending_entry.bag_path),
                list(self.flag_event_times),
                self.pending_entry.bag_id,
                self.current_analysis_index,
            )
            return
        if 0 <= self.current_analysis_index < len(self.analysis_records):
            latest = self.analysis_records[self.current_analysis_index]
            latest.flag_event_times = list(self.flag_event_times)
            latest.generated_bag_path = ""
            self._save_state()
            self._refresh_analysis_table()
            self._select_analysis_record(self.current_analysis_index)

    def _find_current_run_dir(self) -> Optional[Path]:
        if self.current_log_session_dir is not None and self.current_log_session_dir.is_dir():
            return self.current_log_session_dir

        candidates = []
        for path in (PACKAGE_ROOT / "temp").glob("20??-??-??_??-??-??"):
            if not path.is_dir():
                continue
            try:
                stat = path.stat()
            except OSError:
                continue
            if self.session_start_epoch > 0 and stat.st_mtime + 1.0 < self.session_start_epoch:
                continue
            candidates.append((stat.st_mtime, path))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]

    def _find_latest_elevator_zupt_file(self) -> Optional[Path]:
        run_dir = self._find_current_run_dir()
        if run_dir is None:
            return None
        path = run_dir / "debug" / "elevator_zupt.txt"
        return path if path.is_file() else None

    @staticmethod
    def _compute_terminal_errors(post_state_path: Path) -> Tuple[float, float, float]:
        with post_state_path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
            reader = csv.DictReader(handle)
            first_row = None
            last_row = None
            prev_xyz = None
            total_path_length = 0.0
            for row in reader:
                if first_row is None:
                    first_row = row
                xyz = (float(row["x"]), float(row["y"]), float(row["z"]))
                if prev_xyz is not None:
                    dx = xyz[0] - prev_xyz[0]
                    dy = xyz[1] - prev_xyz[1]
                    dz = xyz[2] - prev_xyz[2]
                    total_path_length += math.sqrt(dx * dx + dy * dy + dz * dz)
                prev_xyz = xyz
                last_row = row
        if first_row is None or last_row is None:
            return 0.0, 0.0, 0.0
        x0, y0, z0 = float(first_row["x"]), float(first_row["y"]), float(first_row["z"])
        x1, y1, z1 = float(last_row["x"]), float(last_row["y"]), float(last_row["z"])
        dx = x1 - x0
        dy = y1 - y0
        dz = z1 - z0
        return math.sqrt(dx * dx + dy * dy + dz * dz), abs(dz), total_path_length

    def _parse_run_analysis(self, run_dir: Path, zupt_path: Optional[Path] = None) -> AnalysisRecord:
        text = ""
        if zupt_path is not None and zupt_path.is_file():
            text = zupt_path.read_text(encoding="utf-8", errors="replace")
        deltas = []
        for match in re.finditer(r"\[exit_commit\].*?delta_z_commit:([-+0-9.eE]+)", text):
            deltas.append(float(match.group(1)))
        finished_at = run_dir.name
        bag_id = self.pending_entry.bag_id if self.pending_entry else ""
        bag_path = self.pending_entry.bag_path if self.pending_entry else ""
        post_state_path = run_dir / "state" / "post_state.csv"
        terminal_error = 0.0
        terminal_z_error = 0.0
        total_path_length = 0.0
        terminal_pose: List[float] = []
        if post_state_path.is_file():
            try:
                terminal_error, terminal_z_error, total_path_length = self._compute_terminal_errors(post_state_path)
                terminal_pose = self._compute_terminal_pose(post_state_path)
            except (OSError, ValueError, KeyError):
                terminal_error = 0.0
                terminal_z_error = 0.0
                total_path_length = 0.0
                terminal_pose = []
        return AnalysisRecord(
            bag_id=bag_id,
            bag_path=bag_path,
            finished_at=finished_at,
            lio_yaml_name=self._resolve_effective_lio_yaml_name(),
            yaml_snapshot=self._load_session_yaml_snapshot(run_dir),
            exit_commit_count=len(deltas),
            delta_z_values=deltas,
            delta_z_total=self._compute_delta_z_total(deltas),
            flag_event_times=list(self.flag_event_times),
            terminal_error=terminal_error,
            terminal_z_error=terminal_z_error,
            total_path_length=total_path_length,
            terminal_pose=terminal_pose,
        )

    def _latest_pcd_after_session_start(self, ikdtree: bool) -> Optional[Path]:
        pcd_dir = PACKAGE_ROOT / "PCD"
        if not pcd_dir.is_dir():
            return None
        candidates = []
        for path in pcd_dir.glob("*.pcd"):
            is_ikdtree_file = "_ikdtree_scans.pcd" in path.name
            if ikdtree != is_ikdtree_file:
                continue
            if not is_ikdtree_file and not path.name.endswith("_scans.pcd"):
                continue
            try:
                stat = path.stat()
            except OSError:
                continue
            if self.session_start_epoch > 0 and stat.st_mtime + 1.0 < self.session_start_epoch:
                continue
            candidates.append((stat.st_mtime, path))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0], reverse=True)
        return candidates[0][1]

    def _archive_run_artifacts(self, record: AnalysisRecord, run_dir: Optional[Path] = None) -> None:
        artifact_root = self._analysis_attachment_dir(record) / "artifacts"
        added: List[Path] = []
        run_temp_dir = run_dir or (PACKAGE_ROOT / "temp" / record.finished_at)
        if run_temp_dir.is_dir():
            copied = self._copy_path_into_directory(run_temp_dir, artifact_root / "temp")
            if copied is not None:
                added.append(copied)
        else:
            self._append_lio_log(f"[warn] temp run directory not found for artifact archive: {run_temp_dir}")

        for label, path in (
            ("merged", self._latest_pcd_after_session_start(ikdtree=False)),
            ("ikdtree", self._latest_pcd_after_session_start(ikdtree=True)),
        ):
            if path is None:
                self._append_lio_log(f"[warn] no {label} PCD found for artifact archive")
                continue
            copied = self._copy_path_into_directory(path, artifact_root / "pcd")
            if copied is not None:
                added.append(copied)

        existing = set(record.attachments)
        for path in added:
            path_text = str(path)
            if path_text not in existing:
                record.attachments.append(path_text)
                existing.add(path_text)
        if added:
            self._append_lio_log(f"[analysis] archived {len(added)} artifact(s) to {artifact_root}")

    def _record_playback_analysis(self) -> None:
        run_dir = self._find_current_run_dir()
        if run_dir is None:
            self._append_lio_log("[warn] no log session directory found for this run")
            return
        zupt_file = self._find_latest_elevator_zupt_file()
        if zupt_file is None:
            self._append_lio_log("[info] elevator_zupt.txt is disabled; recording trajectory-only analysis")
        try:
            record = self._parse_run_analysis(run_dir, zupt_file)
        except (OSError, ValueError) as exc:
            self._append_lio_log(f"[warn] failed to parse run artifacts in {run_dir}: {exc}")
            return
        self._archive_run_artifacts(record, run_dir)
        self.analysis_records.insert(0, record)
        if self.batch_mode_active:
            self.batch_new_record_keys.append((record.bag_id, record.finished_at))
        self._rebuild_yaml_groups_for_bag(record.bag_id)
        self.current_analysis_index = 0
        self._save_state()
        self._refresh_analysis_table()
        self._select_analysis_record(self.current_analysis_index)
        self._refresh_table(preserve_form=True)
        self._append_lio_log(
            f"[analysis] yaml_group={record.yaml_group_name or 'yaml1'} "
            f"exit_commit_count={record.exit_commit_count} "
            f"delta_z_total={record.delta_z_total:.6f} "
            f"total_path_length={record.total_path_length:.6f} "
            f"terminal_error={record.terminal_error:.6f} "
            f"terminal_z_error={record.terminal_z_error:.6f}"
        )

    def _on_build_finished(self, exit_code: int, status: str) -> None:
        if not self.session_running:
            return
        if exit_code != 0 or status != "normal":
            if self.batch_mode_active and self.pending_entry is not None:
                self._complete_batch_entry(self.pending_entry, "build failed")
            if not self.batch_mode_active:
                QtWidgets.QMessageBox.critical(self, "Build Failed", "Workspace build failed. Check the LIO Output tab.")
            self._finish_session("Build failed")
            return
        self._append_lio_log("[info] workspace build finished successfully")
        self._ensure_roscore_then_start_lio()

    def _on_roscore_finished(self, exit_code: int, status: str) -> None:
        if self.session_running and self.started_roscore and not self.play_proc.is_running():
            self._append_lio_log(f"[info] roscore exited ({status}, code={exit_code})")

    def _on_lio_finished(self, exit_code: int, status: str) -> None:
        if not self.session_running:
            return
        if self.waiting_for_lio_analysis:
            self.waiting_for_lio_analysis = False
            self._append_lio_log(f"[info] lio exited after playback ({status}, code={exit_code})")
            self._record_playback_analysis()
            self._maybe_close_rviz_after_playback()
            self._finish_session("Playback finished")
            return
        if self.stopping_session:
            self._append_lio_log(f"[info] lio exited during shutdown ({status}, code={exit_code})")
            return
        if not self.play_proc.is_running() and self.node_wait_timer.isActive():
            self.node_wait_timer.stop()
            tail = self._lio_log_tail()
            if self.batch_mode_active and self.pending_entry is not None:
                self._complete_batch_entry(self.pending_entry, "lio crashed before node ready")
                if tail:
                    self._append_lio_log(f"[batch] lio crashed before node ready\n{tail}")
            else:
                message = f"LIO exited before the node became ready ({status}, code={exit_code})."
                if tail:
                    message += f"\n\nRecent LIO output:\n{tail}"
                QtWidgets.QMessageBox.critical(self, "LIO Start Failed", message)
            self._stop_session()
            return
        if self.play_proc.is_running():
            if self.batch_mode_active and self.pending_entry is not None:
                self._complete_batch_entry(self.pending_entry, "lio exited during playback")
                self._append_lio_log(f"[batch] lio exited during playback ({status}, code={exit_code})")
            else:
                QtWidgets.QMessageBox.warning(self, "LIO Exited", f"LIO exited during playback ({status}, code={exit_code}).")
            self._stop_session()
            return
        self._append_lio_log(f"[info] lio exited ({status}, code={exit_code})")
        if self.current_external_lio_profile is not None and not self.play_proc.is_running():
            if self.started_roscore and self.roscore_proc.is_running():
                self.roscore_proc.terminate()
            self._finish_session("External LIO stopped")

    def _on_rviz_finished(self, exit_code: int, status: str) -> None:
        if not self.session_running:
            return
        if self.stopping_session:
            self._append_lio_log(f"[info] rviz exited during shutdown ({status}, code={exit_code})")
            return
        self._append_lio_log(f"[info] rviz exited ({status}, code={exit_code})")

    def _on_play_finished(self, exit_code: int, status: str) -> None:
        if not self.session_running:
            return
        self._append_rosbag_log(f"[info] rosbag play finished ({status}, code={exit_code})")
        if self.stopping_session:
            return
        if self.batch_mode_active and self.pending_entry is not None:
            result_text = "ok" if exit_code == 0 and status == "normal" else f"failed ({status}, code={exit_code})"
            self._complete_batch_entry(self.pending_entry, result_text)
        if exit_code == 0 and status == "normal":
            if self.current_session_records_analysis:
                if self.lio_proc.is_running():
                    self.waiting_for_lio_analysis = True
                    self._set_status("Waiting for LIO to save artifacts")
                    self.lio_proc.terminate()
                    if self.started_roscore and self.roscore_proc.is_running():
                        self.roscore_proc.terminate()
                    return
                if self.started_roscore and self.roscore_proc.is_running():
                    self.roscore_proc.terminate()
                self._record_playback_analysis()
                self._maybe_close_rviz_after_playback()
                self._finish_session("Playback finished")
            else:
                self._append_lio_log("[info] external LIO session finished; analysis skipped")
                self._handle_external_playback_finished()
        else:
            if self.lio_proc.is_running():
                self.lio_proc.terminate()
            if self.started_roscore and self.roscore_proc.is_running():
                self.roscore_proc.terminate()
            self._maybe_close_rviz_after_playback()
            self._finish_session("Playback failed")

    def _update_buttons(self) -> None:
        has_selection = self._selected_row() >= 0
        selected_entry = self.entries[self._selected_row()] if has_selection and 0 <= self._selected_row() < len(self.entries) else None
        selected_deleted = bool(selected_entry.deleted) if selected_entry is not None else False
        has_deleted_entries = any(entry.deleted for entry in self.entries)
        self.add_btn.setEnabled(True)
        self.update_btn.setEnabled(has_selection and not selected_deleted)
        self.remove_btn.setEnabled((not self.session_running) and has_selection and not selected_deleted)
        self.clear_btn.setEnabled(has_deleted_entries)
        self.batch_btn.setEnabled((not self.session_running) and any(not entry.deleted for entry in self.entries))
        self.settings_btn.setEnabled(not self.session_running)
        self.export_analysis_btn.setEnabled(self._selected_analysis_row() >= 0)
        self.start_btn.setEnabled((not self.session_running) and has_selection and not selected_deleted)
        self.other_btn.setEnabled((not self.session_running) and has_selection and not selected_deleted)
        self.pause_btn.setEnabled(self.play_proc.is_running() and not self.stopping_session)
        self.pause_btn.setText("Resume Bag" if self.playback_paused else "Pause Bag")
        self.flag_btn.setEnabled(self.play_proc.is_running() and not self.stopping_session and self.current_session_records_analysis)
        self.stop_btn.setEnabled(self.session_running and not self.stopping_session)

    def closeEvent(self, event) -> None:
        if self.flag_generation_thread is not None:
            self.close_requested = True
            event.ignore()
            return
        if self._any_managed_process_running():
            self.close_requested = True
            self._stop_session()
            event.ignore()
            return
        self._save_state()
        super().closeEvent(event)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    window = BagRunnerWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
