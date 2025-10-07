"""Tabbed shell area that contains the Log (non-closable) and multiple shells.

Each shell runs the host's system shell using QProcess and provides a simple
input field and output area. The LogViewer is inserted as the first non-closable
tab so logs remain visible while shells can be added/removed.
"""
from typing import Optional
import sys
import shutil

from PyQt5.QtCore import Qt, QProcess, QEvent
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QPlainTextEdit,
    QLineEdit,
    QTabWidget,
    QPushButton,
    QHBoxLayout,
    QSplitter,
    QTabBar,
    QLabel,
)
from PyQt5.QtCore import QMimeData, QDataStream, QByteArray, QPoint
from PyQt5.QtGui import QDrag
import uuid
import weakref
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


class ShellTabs(QWidget):
    """A container that holds shell tabs and the log viewer.

    It can present the log either as the first non-closable tab or in a
    resizable split to the right of the shell tabs (like VS Code's panel split).
    """

    def __init__(self, log_viewer: Optional[LogViewer] = None, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        # Internal tab widget that will hold shells (and optionally the log)
        # Use a custom tab bar that can initiate drags of tabs.
        self._tabs = QTabWidget()
        self._tab_bar = QTabBar()
        self._tab_bar.setMovable(True)
        # Install our draggable behaviour by subclassing QTabBar in-place
        # (we'll monkey-patch event handlers below since simple subclassing
        # through apply_patch would be more invasive in this edit.)
        self._tabs.setTabBar(self._tab_bar)
        self._tabs.setTabsClosable(True)
        self._tabs.tabCloseRequested.connect(self._on_tab_close_requested)
        # Prevent the Log tab (index 0 when present) from being moved.
        try:
            self._tab_bar.tabMoved.connect(self._on_tab_moved)
        except Exception:
            pass

        # Log viewer instance may be provided by caller; otherwise create one
        if log_viewer is None:
            log_viewer = LogViewer()
        self._log = log_viewer

        # By default, keep the log as the first non-closable tab
        self._log_in_tab = True
        self._tabs.addTab(self._log, 'Log')
        # Ensure the log tab is not closable
        try:
            log_idx = self._tabs.indexOf(self._log)
            if log_idx != -1:
                self._tabs.setTabClosable(log_idx, False)
        except Exception:
            pass

        # Buttons in the corner: split toggle and add-shell
        corner = QWidget()
        corner_layout = QHBoxLayout()
        corner_layout.setContentsMargins(0, 0, 0, 0)
        corner.setLayout(corner_layout)

        self._add_btn = QPushButton('+')
        self._add_btn.setFixedSize(18, 18)
        self._add_btn.setToolTip('Open a new shell')
        self._add_btn.clicked.connect(self.add_shell)

        corner_layout.addWidget(self._add_btn)

        self._tabs.setCornerWidget(corner, Qt.TopRightCorner)
        # Ensure initial tab closable flags
        self._update_tab_closable_flags()

        # Optional splitter used when split mode is active
        self._splitter: Optional[QSplitter] = None
        self._is_split = False

        # Layout for this widget
        self._layout = QVBoxLayout()
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.addWidget(self._tabs)
        self.setLayout(self._layout)

        self._shell_count = 0
        # Mapping of drag ids to widgets and metadata so we can transfer
        # widgets during a drag/drop operation.
        self._drag_map: dict[str, dict] = {}

        # Enable drops on the main widget and on the tab widget
        self.setAcceptDrops(True)
        self._tabs.setAcceptDrops(True)

        # Replace the QTabBar with our draggable behaviour by connecting
        # its mouse events to local handlers.
        self._tab_bar.mousePressEvent = self._tabbar_mouse_press  # type: ignore[assignment]
        self._tab_bar.mouseMoveEvent = self._tabbar_mouse_move  # type: ignore[assignment]
        self._tab_bar.mouseReleaseEvent = self._tabbar_mouse_release  # type: ignore[assignment]
        self._drag_start_pos: Optional[QPoint] = None

    def add_shell(self) -> None:
        """Add a new shell tab."""
        self._shell_count += 1
        shell = _ShellWidget('', parent=self)
        title = f'Shell {self._shell_count}'
        idx = self._tabs.addTab(shell, title)
        self._tabs.setCurrentWidget(shell)
        # Ensure this new tab is closable
        try:
            self._tabs.setTabClosable(idx, True)
        except Exception:
            pass
        # Ensure log remains unclosable if present
        self._update_tab_closable_flags()
        try:
            shell._proc.finished.connect(lambda ec, st, w=shell: self._on_shell_finished(w))
        except Exception:
            pass
        # no split button; nothing to update

    # --- Drag helpers on the tab bar -------------------------------------------------
    def _tabbar_mouse_press(self, event) -> None:  # type: ignore[override]
        # Record position for potential drag start
        try:
            self._drag_start_pos = event.pos()
            # Call the original to preserve selection behaviour
            QTabBar.mousePressEvent(self._tab_bar, event)
        except Exception:
            pass

    def _tabbar_mouse_move(self, event) -> None:  # type: ignore[override]
        try:
            if self._drag_start_pos is None:
                QTabBar.mouseMoveEvent(self._tab_bar, event)
                return

            dx = abs(event.pos().x() - self._drag_start_pos.x())
            dy = abs(event.pos().y() - self._drag_start_pos.y())
            dist = max(dx, dy)
            if dist < 10:
                QTabBar.mouseMoveEvent(self._tab_bar, event)
                return

            # If horizontal movement dominates, treat as a reorder and pass
            # to the QTabBar. If vertical movement dominates, start a split drag.
            # If Control is held, force split-drag regardless of direction.
            ctrl_force = False
            try:
                ctrl_force = bool(event.modifiers() & Qt.ControlModifier)
            except Exception:
                ctrl_force = False
            if dx >= dy and not ctrl_force:
                QTabBar.mouseMoveEvent(self._tab_bar, event)
                # Reordering handled by QTabBar; clear drag start so we don't
                # accidentally start an external split-drag afterwards.
                try:
                    self._drag_start_pos = None
                except Exception:
                    pass
                return

            # Vertical-dominant: start external split drag
            idx = self._tab_bar.tabAt(self._drag_start_pos)
            if idx == -1:
                return
            widget = self._tabs.widget(idx)
            if widget is None:
                return

            # Generate a drag id and store metadata
            drag_id = uuid.uuid4().hex
            self._drag_map[drag_id] = {
                'widget_ref': weakref.ref(widget),
                'text': self._tab_bar.tabText(idx),
                'index': idx,
            }

            mime = QMimeData()
            mime.setData('application/x-gnss-tab', drag_id.encode())

            drag = QDrag(self._tab_bar)
            drag.setMimeData(mime)
            # Execute drag (allow move)
            result = drag.exec_()
            # If the drag was accepted and the widget moved, remove its tab
            if result == Qt.MoveAction:
                try:
                    cur_idx = self._tabs.indexOf(widget)
                    if cur_idx != -1:
                        self._tabs.removeTab(cur_idx)
                except Exception:
                    pass
            # Schedule a recovery check: if the widget is lost (neither in
            # tabs nor in splitter), reinsert it as a tab to avoid disappearing.
            try:
                QTimer.singleShot(0, lambda did=drag_id: self._recover_lost_widget(did))
            except Exception:
                pass

            # After any potential change, ensure closable flags are correct
            self._update_tab_closable_flags()
        except Exception:
            pass

    def _tabbar_mouse_release(self, event) -> None:  # type: ignore[override]
        # Clear drag start position on release and call the original handler
        try:
            self._drag_start_pos = None
            QTabBar.mouseReleaseEvent(self._tab_bar, event)
        except Exception:
            pass

    # --- Drag/drop events for the ShellTabs widget -------------------------------
    def dragEnterEvent(self, event) -> None:  # type: ignore[override]
        if event.mimeData().hasFormat('application/x-gnss-tab'):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dragMoveEvent(self, event) -> None:  # type: ignore[override]
        if event.mimeData().hasFormat('application/x-gnss-tab'):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dropEvent(self, event) -> None:  # type: ignore[override]
        # Determine drop side relative to this widget: left half -> tabs,
        # right half -> split.
        try:
            if not event.mimeData().hasFormat('application/x-gnss-tab'):
                event.ignore()
                return
            data = bytes(event.mimeData().data('application/x-gnss-tab')).decode()
            drag_id = data
            meta = self._drag_map.get(drag_id)
            if meta is None:
                event.ignore()
                return
            widget = meta.get('widget_ref')()
            if widget is None:
                event.ignore()
                return
            # Decide where to drop based on position
            pos = event.pos()
            w = self.width()
            if pos.x() > w // 2:
                # Drop on right half -> ensure splitter and add the widget to right
                if not self._is_split:
                    # Remove tabs widget from layout and create splitter
                    idx = self._layout.indexOf(self._tabs)
                    if idx != -1:
                        self._layout.removeWidget(self._tabs)
                    self._splitter = QSplitter(Qt.Horizontal)
                    # Ensure left tabs have a sensible minimum width so the
                    # add/split buttons remain visible after splitting.
                    try:
                        self._tabs.setMinimumWidth(200)
                    except Exception:
                        pass
                    self._splitter.addWidget(self._tabs)
                    # Wrap widget with a small header for drag-back
                    wrapper = self._make_split_wrapper(widget, meta.get('text', ''))
                    self._splitter.addWidget(wrapper)
                    self._layout.addWidget(self._splitter)
                    # Equalize sizes across splitter panes (handles N panes)
                    try:
                        self._equalize_splitter()
                    except Exception:
                        pass
                    self._is_split = True
                    self._log_in_tab = False
                else:
                    # Add into existing splitter as right-most widget
                    wrapper = self._make_split_wrapper(widget, meta.get('text', ''))
                    self._splitter.addWidget(wrapper)
                    try:
                        self._equalize_splitter()
                    except Exception:
                        pass
                event.acceptProposedAction()
                return
            else:
                # Drop on left half -> ensure the tabs are visible and insert
                # the widget as a tab
                if self._is_split and self._splitter is not None:
                    # Remove wrapper(s) corresponding to this widget from splitter
                    for i in range(self._splitter.count() - 1, -1, -1):
                        wdg = self._splitter.widget(i)
                        # If wrapper has attribute original_widget, match it
                        if hasattr(wdg, 'original_widget') and getattr(wdg, 'original_widget') is widget:
                            # Remove the wrapper cleanly from splitter
                            wdg.setParent(None)
                            try:
                                wdg.deleteLater()
                            except Exception:
                                pass
                    # If multiple panes remain, equalize sizes; otherwise remove splitter
                    try:
                        if self._splitter.count() > 1:
                            self._equalize_splitter()
                    except Exception:
                        pass
                    if self._splitter.count() == 1:
                        self._layout.removeWidget(self._splitter)
                        self._layout.addWidget(self._tabs)
                        self._splitter = None
                        self._is_split = False
                        # Restore state: log is back in tabs and should be non-closable
                        try:
                            self._log_in_tab = True
                        except Exception:
                            pass
                        try:
                            log_idx = self._tabs.indexOf(self._log)
                            if log_idx != -1:
                                self._tabs.setTabClosable(log_idx, False)
                        except Exception:
                            pass
                        # Ensure all closable flags are correct
                        try:
                            self._update_tab_closable_flags()
                        except Exception:
                            pass
                # Insert widget as a new tab
                title = meta.get('text', '')
                idx = self._tabs.addTab(widget, title)
                self._tabs.setCurrentIndex(idx)
                event.acceptProposedAction()
                return
        except Exception:
            event.ignore()

    def _make_split_wrapper(self, widget: QWidget, title: str) -> QWidget:
        """Wrap a widget with a small draggable header so it can be dragged back."""
        container = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        header = QLabel(title)
        header.setFixedHeight(20)
        header.setStyleSheet('background: #ddd; padding-left: 6px;')
        # Attach drag start behaviour to the header
        header.mousePressEvent = lambda ev, h=header, w=widget: self._split_header_press(ev, h, w)  # type: ignore[assignment]
        header.mouseMoveEvent = lambda ev, h=header, w=widget: self._split_header_move(ev, h, w)  # type: ignore[assignment]
        layout.addWidget(header)
        # Reparent the widget into the wrapper to ensure correct ownership
        try:
            widget.setParent(container)
        except Exception:
            pass
        layout.addWidget(widget)
        try:
            widget.show()
        except Exception:
            pass
        container.setLayout(layout)
        # Keep reference to the original widget so we can find it later
        setattr(container, 'original_widget', widget)
        return container

    def _split_header_press(self, event, header: QLabel, widget: QWidget) -> None:
        header._drag_start = event.pos()

    def _split_header_move(self, event, header: QLabel, widget: QWidget) -> None:
        try:
            if not hasattr(header, '_drag_start'):
                return
            dist = (event.pos() - header._drag_start).manhattanLength()
            if dist < 10:
                return
            # Find a drag_id for this widget (reverse lookup)
            drag_id = None
            for k, v in self._drag_map.items():
                if v.get('widget_ref')() is widget:
                    drag_id = k
                    break
            if drag_id is None:
                drag_id = uuid.uuid4().hex
                self._drag_map[drag_id] = {'widget_ref': weakref.ref(widget), 'text': header.text()}
            mime = QMimeData()
            mime.setData('application/x-gnss-tab', drag_id.encode())
            drag = QDrag(header)
            drag.setMimeData(mime)
            result = drag.exec_()
            if result == Qt.MoveAction:
                # The drop handler will reparent the widget; remove the wrapper
                parent = header.parent()
                if parent is not None:
                    parent.setParent(None)
            # Keep closable flags updated
            self._update_tab_closable_flags()
        except Exception:
            pass

    def _equalize_splitter(self) -> None:
        """Set equal sizes for all widgets in the splitter (N-way equal).

        This works for any number of panes: 2 -> 50/50, 3 -> 33/33/33, etc.
        """
        try:
            if self._splitter is None:
                return
            count = max(1, self._splitter.count())
            if count <= 1:
                return
            total = max(300, self.width())
            size = total // count
            sizes = [size] * count
            # Adjust last piece to fill remainder
            remainder = total - size * count
            if remainder > 0:
                sizes[-1] += remainder
            self._splitter.setSizes(sizes)
        except Exception:
            pass

    def _recover_lost_widget(self, drag_id: str) -> None:
        """If a dragged widget is no longer present in tabs or splitter,
        reinsert it as a new tab to avoid it disappearing.
        """
        try:
            meta = self._drag_map.get(drag_id)
            if meta is None:
                return
            wref = meta.get('widget_ref')
            if wref is None:
                return
            widget = wref()
            if widget is None:
                return
            # Check tabs
            if self._tabs.indexOf(widget) != -1:
                return
            # Check splitter
            if self._splitter is not None:
                for i in range(self._splitter.count()):
                    if self._splitter.widget(i) is widget or hasattr(self._splitter.widget(i), 'original_widget') and getattr(self._splitter.widget(i), 'original_widget') is widget:
                        return
            # Not found: reinsert as a tab
            title = meta.get('text', 'Shell')
            try:
                idx = self._tabs.addTab(widget, title)
                self._tabs.setCurrentIndex(idx)
                self._update_tab_closable_flags()
            except Exception:
                pass
        except Exception:
            pass

    def _on_tab_moved(self, from_index: int, to_index: int) -> None:
        """Prevent the log tab (index 0 when present) from being moved.

        If an attempt is made to move the log tab, revert the move.
        """
        try:
            if not self._log_in_tab:
                return
            if from_index == 0 or to_index == 0:
                # Revert the move
                try:
                    self._tab_bar.blockSignals(True)
                    # moveTab(from, to) moves a tab; to revert swap back
                    self._tab_bar.moveTab(to_index, from_index)
                finally:
                    self._tab_bar.blockSignals(False)
        except Exception:
            pass
        finally:
            # Ensure closable flags remain correct after any movement
            try:
                self._update_tab_closable_flags()
            except Exception:
                pass

    def _update_tab_closable_flags(self) -> None:
        """Set tab closable: log (index 0 when present) is not closable; others are."""
        try:
            for i in range(self._tabs.count()):
                closable = True
                if self._log_in_tab and i == 0:
                    closable = False
                try:
                    self._tabs.setTabClosable(i, closable)
                except Exception:
                    pass
        except Exception:
            pass

    # split button removed; splits are handled by drag gestures (Ctrl to force)

    def _on_tab_close_requested(self, index: int) -> None:
        # If the log is present in tabs it's at index 0 and should not be closed
        if self._log_in_tab and index == 0:
            return
        widget = self._tabs.widget(index)
        try:
            # Mark widget as being closed by user to avoid race between
            # widget.close() emitting finished and this handler removing the tab
            # and the finished-slot also attempting to remove the tab.
            if widget is not None:
                try:
                    setattr(widget, '_closing_by_user', True)
                except Exception:
                    pass
            if hasattr(widget, 'close'):
                widget.close()
        except Exception:
            pass
        self._tabs.removeTab(index)
        # no split button; nothing to update

    def _on_shell_finished(self, shell_widget: QWidget) -> None:
        """Remove the tab for a shell widget when its process finishes."""
        try:
            # If the shell was closed by the user and the tab close handler
            # is already handling removal, skip to avoid double-removal.
            try:
                if getattr(shell_widget, '_closing_by_user', False):
                    # Clear the flag for cleanliness and do nothing
                    try:
                        delattr(shell_widget, '_closing_by_user')
                    except Exception:
                        pass
                    return
            except Exception:
                pass
            idx = self._tabs.indexOf(shell_widget)
            if idx != -1:
                try:
                    if hasattr(shell_widget, 'close'):
                        shell_widget.close()
                except Exception:
                    pass
                self._tabs.removeTab(idx)
        except Exception:
            pass
        # no split button; nothing to update

    # toggle_split removed; split behavior is via drag gestures (Ctrl to force)

    def log_viewer(self) -> LogViewer:
        return self._log
