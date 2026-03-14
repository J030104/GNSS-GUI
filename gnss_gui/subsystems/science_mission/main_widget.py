"""Placeholder widget for the science mission subsystem.

Shows dummy sensor readings and a log viewer.  Teams can integrate
real instruments (e.g. spectrometers, microscopes) and control
interfaces in this tab.
"""

from typing import Optional

from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QTabWidget,
)

from .dashboard_tab import DashboardTab
from .uv_tab import UVFluorescenceTab
from .raman_tab import RamanSpectroscopyTab


class ScienceMissionWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.sub_tabs = QTabWidget()
        
        # specific style for the main tabs
        main_tab_style = """
            QTabBar::tab {
                height: 40px;
                width: 150px;
                font-size: 14px;
                font-weight: bold;
            }
            QTabWidget::pane {
                border: 1px solid #C4C4C3;
                top: -1px;
            }
        """
        self.sub_tabs.setStyleSheet(main_tab_style)
        
        # specific style for the nested tabs (wider)
        nested_tab_style = """
            QTabBar::tab {
                height: 30px;
                width: 200px; /* Wider for full text visibility */
                font-size: 12px;
                font-weight: bold;
            }
            QTabWidget::pane {
                border: 1px solid #C4C4C3;
                top: -1px;
            }
        """
        
        # --- Dashboard Tab ---
        self.dashboard_tab = DashboardTab()
        
        # --- Test Site 1 ---
        self.test_site_1_tab = QWidget()
        ts1_layout = QVBoxLayout()
        ts1_layout.setContentsMargins(0, 0, 0, 0)
        
        self.ts1_tabs = QTabWidget()
        self.ts1_tabs.setStyleSheet(nested_tab_style)
        
        # TS1: UV Fluorescence
        self.ts1_uv_tab = UVFluorescenceTab("Site 1")
        
        # TS1: Raman Spectroscopy
        self.ts1_raman_tab = RamanSpectroscopyTab("Site 1")
        
        self.ts1_tabs.addTab(self.ts1_uv_tab, "UV Fluorescence")
        self.ts1_tabs.addTab(self.ts1_raman_tab, "Raman Spectroscopy")
        
        ts1_layout.addWidget(self.ts1_tabs)
        self.test_site_1_tab.setLayout(ts1_layout)
        
        # --- Test Site 2 ---
        self.test_site_2_tab = QWidget()
        ts2_layout = QVBoxLayout()
        ts2_layout.setContentsMargins(0, 0, 0, 0)
        
        self.ts2_tabs = QTabWidget()
        self.ts2_tabs.setStyleSheet(nested_tab_style)

        # TS2: UV Fluorescence
        self.ts2_uv_tab = UVFluorescenceTab("Site 2")
        
        # TS2: Raman Spectroscopy
        self.ts2_raman_tab = RamanSpectroscopyTab("Site 2")
        
        self.ts2_tabs.addTab(self.ts2_uv_tab, "UV Fluorescence")
        self.ts2_tabs.addTab(self.ts2_raman_tab, "Raman Spectroscopy")
        
        ts2_layout.addWidget(self.ts2_tabs)
        self.test_site_2_tab.setLayout(ts2_layout)
        
        self.sub_tabs.addTab(self.dashboard_tab, "Dashboard")
        self.sub_tabs.addTab(self.test_site_1_tab, "Test Site 1")
        self.sub_tabs.addTab(self.test_site_2_tab, "Test Site 2")
        
        layout.addWidget(self.sub_tabs)
        self.setLayout(layout)
