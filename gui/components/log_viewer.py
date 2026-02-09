"""Log viewer widget.

Provides a scrollable text area for displaying log messages, system
diagnostics and debug output.  Other parts of the program can call
``append`` to add messages to the view.
"""

from typing import Optional

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPlainTextEdit


class LogViewer(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._edit = QPlainTextEdit()
        self._edit.setReadOnly(True)

        layout = QVBoxLayout()
        layout.addWidget(self._edit)
        self.setLayout(layout)

    def append(self, message: str) -> None:
        """Append a line of text to the log viewer."""
        self._edit.appendPlainText(message)
        # Scroll to bottom
        cursor = self._edit.textCursor()
        cursor.movePosition(cursor.End)
        self._edit.setTextCursor(cursor)