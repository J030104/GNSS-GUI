"""Video layout tab widget for GNSSCommWidget.

This component provides a vertical tab bar on the right, where each tab represents a camera layout mode. Each mode can have a distinct arrangement of video viewers. Layouts are parameterized for easy customization and extension.
"""

from typing import List, Dict, Any, Optional
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, QGridLayout
)
from PyQt5.QtCore import Qt
from .video_viewer import VideoViewer

class VideoLayoutTabWidget(QWidget):
    """A widget with vertical tabs for switching between camera layouts."""
    def __init__(self, layouts: List[Dict[str, Any]], parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.tab_widget = QTabWidget()
        self.tab_widget.setTabPosition(QTabWidget.West)
        self.tab_widget.setMovable(False)
        self.tab_widget.setDocumentMode(True)
        # Remove tab widget styling to minimize gaps
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: 5px;
                margin: 0px;
                padding: 0px;
            }
            QTabWidget::tab-bar {
                alignment: left;
            }
        """)
        self._layouts = layouts
        self._video_viewers = []  # type: List[List[VideoViewer]]
        self._init_tabs()
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.tab_widget)
        self.setLayout(layout)

    def _init_tabs(self):
        self._video_viewers.clear()
        for layout_def in self._layouts:
            tab = QWidget()
            grid = QGridLayout()
            grid.setContentsMargins(0, 0, 0, 0)
            grid.setSpacing(2)
            viewers = []
            for i, box in enumerate(layout_def['boxes']):
                camera_name = box.get('camera_name', f"{layout_def['name']} Cam {i+1}")
                viewer = VideoViewer(fps=10, camera_name=camera_name)
                viewers.append(viewer)
                grid.addWidget(viewer, box['row'], box['col'], box.get('rowspan', 1), box.get('colspan', 1))
                # Only set size constraints if explicitly specified in the layout definition
                # Otherwise let the VideoViewer use its default small size
                if 'min_size' in box:
                    viewer.setMinimumSize(*box['min_size'])
                if 'max_size' in box:
                    viewer.setMaximumSize(*box['max_size'])
            tab.setLayout(grid)
            self.tab_widget.addTab(tab, layout_def['name'])
            self._video_viewers.append(viewers)

    def get_video_viewers(self, tab_index: int = None) -> List[VideoViewer]:
        """Return the list of VideoViewer widgets for the given tab (default: current)."""
        if tab_index is None:
            tab_index = self.tab_widget.currentIndex()
        return self._video_viewers[tab_index]

    def get_all_video_viewers(self) -> List[VideoViewer]:
        """Return a flat list of all VideoViewer instances across all tabs."""
        viewers = []
        for vlist in self._video_viewers:
            viewers.extend(vlist)
        return viewers

    def add_layout(self, layout_def: Dict[str, Any]):
        """Add a new layout tab at runtime."""
        self._layouts.append(layout_def)
        self._init_tabs()

    def set_layouts(self, layouts: List[Dict[str, Any]]):
        """Replace all layouts."""
        self._layouts = layouts
        self._init_tabs()
