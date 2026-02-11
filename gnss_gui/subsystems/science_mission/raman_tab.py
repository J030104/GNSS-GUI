from typing import Optional

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel


class RamanSpectroscopyTab(QWidget):
    def __init__(self, site_name: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.site_name = site_name
        self._init_ui()

    def _init_ui(self):
        """Initialize the Raman Spectroscopy layout."""
        layout = QVBoxLayout()
        layout.addWidget(QLabel(f"Raman Spectroscopy ({self.site_name})"))
        self.setLayout(layout)
