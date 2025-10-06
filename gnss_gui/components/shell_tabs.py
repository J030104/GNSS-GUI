"""Tabbed shell area that contains the Log (non-closable) and multiple shells.

Each shell runs the host's system shell using QProcess and provides a simple
input field and output area. The LogViewer is inserted as the first non-closable
tab so logs remain visible while shells can be added/removed.
"""
from typing import Optional
import sys
import shutil

from PyQt5.QtCore import Qt, QProcess, QEvent
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QPlainTextEdit,
    QLineEdit,
    QTabWidget,
    QPushButton,
    QHBoxLayout,
)
import os
import glob

from .log_viewer import LogViewer


class _ShellWidget(QWidget):
    def __init__(self, shell_cmd: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._proc = QProcess(self)
        self._proc.setProcessChannelMode(QProcess.MergedChannels)

        self._out = QPlainTextEdit()
        self._out.setReadOnly(True)
        self._in = QLineEdit()
        self._in.setPlaceholderText('Type a command and press Enter')
        # Input history and completion state
        self._history = []
        self._hist_index = -1
        self._completions = []
        self._comp_index = -1

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._out)
        layout.addWidget(self._in)
        self.setLayout(layout)

        self._proc.readyReadStandardOutput.connect(self._read_output)
        self._proc.readyReadStandardError.connect(self._read_output)
        self._in.returnPressed.connect(self._handle_input)
        # Install event filter on input to capture Up/Down/Tab
        self._in.installEventFilter(self)

        # Start an interactive shell where possible. Note: full tty behaviour
        # (e.g. curses apps) may not work because QProcess doesn't provide a pty.
        args = []
        if sys.platform == 'win32':
            program = shutil.which('cmd') or 'cmd'
            # cmd runs interactive by default
        else:
            # Prefer bash if available; start as interactive so aliases are loaded
            bash = shutil.which('bash')
            sh = shutil.which('sh')
            if bash:
                program = bash
                args = ['-i']
            elif sh:
                program = sh
                args = ['-i']
            else:
                program = '/bin/sh'

        try:
            self._proc.start(program, args)
        except Exception as e:
            self._out.appendPlainText(f'Failed to start shell: {e}')

    def _read_output(self) -> None:
        try:
            data = self._proc.readAllStandardOutput().data().decode(errors='replace')
            if data:
                self._out.appendPlainText(data)
        except Exception:
            pass

    def _handle_input(self) -> None:
        txt = self._in.text()
        if not txt:
            return
        # Save to history
        try:
            if not self._history or (self._history and self._history[-1] != txt):
                self._history.append(txt)
        except Exception:
            pass
        self._hist_index = -1
        # Write the command to the process stdin
        try:
            self._proc.write((txt + '\n').encode())
        except Exception:
            self._out.appendPlainText('Failed to send input to shell')
        self._in.clear()

    def close(self) -> None:
        try:
            if self._proc.state() != QProcess.NotRunning:
                self._proc.kill()
                self._proc.waitForFinished(500)
        except Exception:
            pass
        super().close()

    def eventFilter(self, obj, event) -> bool:  # type: ignore[override]
        # Handle key navigation and tab completion on the input
        if obj is self._in and event.type() == QEvent.KeyPress:
            key = event.key()
            if key == Qt.Key_Up:
                # history up
                if self._history:
                    if self._hist_index == -1:
                        self._hist_index = len(self._history) - 1
                    else:
                        self._hist_index = max(0, self._hist_index - 1)
                    try:
                        self._in.setText(self._history[self._hist_index])
                    except Exception:
                        pass
                    return True
            if key == Qt.Key_Down:
                # history down
                if self._history:
                    if self._hist_index == -1:
                        return True
                    self._hist_index = min(len(self._history) - 1, self._hist_index + 1)
                    if self._hist_index == len(self._history) - 1:
                        self._in.setText(self._history[self._hist_index])
                    else:
                        try:
                            self._in.setText(self._history[self._hist_index])
                        except Exception:
                            pass
                    return True
            if key == Qt.Key_Tab:
                # Simple completion: complete last token using files and PATH commands
                text = self._in.text()
                cursor_pos = self._in.cursorPosition()
                prefix = text[:cursor_pos]
                # get last token
                token = prefix.split()[-1] if prefix.split() else ''
                if not token:
                    return True
                # build candidates
                candidates = []
                # files in cwd
                try:
                    for p in glob.glob(token + '*'):
                        candidates.append(p)
                except Exception:
                    pass
                # commands in PATH
                try:
                    for d in os.environ.get('PATH', '').split(os.pathsep):
                        try:
                            if not d:
                                continue
                            for fname in os.listdir(d):
                                if fname.startswith(token):
                                    fpath = os.path.join(d, fname)
                                    if os.access(fpath, os.X_OK):
                                        candidates.append(fname)
                        except Exception:
                            pass
                except Exception:
                    pass
                candidates = sorted(set(candidates))
                if not candidates:
                    return True
                # cycle through completions
                if self._comp_index == -1 or self._completions != candidates:
                    self._completions = candidates
                    self._comp_index = 0
                else:
                    self._comp_index = (self._comp_index + 1) % len(self._completions)
                comp = self._completions[self._comp_index]
                # replace last token in input
                new_text = prefix[: prefix.rfind(token)] + comp + text[cursor_pos:]
                self._in.setText(new_text)
                # place cursor at end of completed token
                self._in.setCursorPosition((prefix[: prefix.rfind(token)] + comp).__len__())
                return True

        return super().eventFilter(obj, event)


class ShellTabs(QTabWidget):
    def __init__(self, log_viewer: Optional[LogViewer] = None, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        # Make tabs closable in general and handle close requests
        self.setTabsClosable(True)
        self.tabCloseRequested.connect(self._on_tab_close_requested)

        # Add Log tab as first, non-closable
        if log_viewer is None:
            log_viewer = LogViewer()
        self._log = log_viewer
        self.addTab(self._log, 'Log')
        self.setTabsClosable(False)

        # Add a small add-button in the corner to create new shells
        btn = QPushButton('+')
        btn.setFixedSize(18, 18)
        btn.setToolTip('Open a new shell')
        btn.clicked.connect(self.add_shell)
        self.setCornerWidget(btn, Qt.TopRightCorner)

        self._shell_count = 0

    def add_shell(self) -> None:
        self._shell_count += 1
        shell = _ShellWidget('', parent=self)
        title = f'Shell {self._shell_count}'
        idx = self.addTab(shell, title)
        self.setCurrentWidget(shell)
        # Ensure this new tab is closable even if the tab widget was set to
        # non-closable earlier for the log; mark this index closable.
        try:
            self.setTabClosable(idx, True)
        except Exception:
            pass
        # Auto-close the tab when the underlying process finishes
        try:
            shell._proc.finished.connect(lambda ec, st, w=shell: self._on_shell_finished(w))
        except Exception:
            pass

    def _on_tab_close_requested(self, index: int) -> None:
        # Prevent closing the log tab (index 0)
        if index == 0:
            return
        widget = self.widget(index)
        try:
            if hasattr(widget, 'close'):
                widget.close()
        except Exception:
            pass
        self.removeTab(index)

    def _on_shell_finished(self, shell_widget: QWidget) -> None:
        """Remove the tab for a shell widget when its process finishes."""
        try:
            idx = self.indexOf(shell_widget)
            if idx != -1:
                # Attempt to close the widget and remove the tab
                try:
                    if hasattr(shell_widget, 'close'):
                        shell_widget.close()
                except Exception:
                    pass
                self.removeTab(idx)
        except Exception:
            pass

    def log_viewer(self) -> LogViewer:
        return self._log
