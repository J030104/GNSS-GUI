"""Placeholder widget for the science mission subsystem.

Shows dummy sensor readings and a log viewer.  Teams can integrate
real instruments (e.g. spectrometers, microscopes) and control
interfaces in this tab.
"""

from typing import Optional

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QTabWidget


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
        
        # Dashboard tab
        self.dashboard_tab = QWidget()
        dash_layout = QVBoxLayout()
        dash_layout.addWidget(QLabel("Dashboard (Empty)"))
        self.dashboard_tab.setLayout(dash_layout)
        
        # --- Test Site 1 ---
        self.test_site_1_tab = QWidget()
        ts1_layout = QVBoxLayout()
        ts1_layout.setContentsMargins(0, 0, 0, 0)
        
        self.ts1_tabs = QTabWidget()
        self.ts1_tabs.setStyleSheet(nested_tab_style)
        
        # TS1: UV Fluorescence
        self.ts1_uv_tab = QWidget()
        uv1_layout = QVBoxLayout()
        uv1_layout.addWidget(QLabel("UV Fluorescence (Site 1)"))
        self.ts1_uv_tab.setLayout(uv1_layout)
        
        # TS1: Raman Spectroscopy
        self.ts1_raman_tab = QWidget()
        raman1_layout = QVBoxLayout()
        raman1_layout.addWidget(QLabel("Raman Spectroscopy (Site 1)"))
        self.ts1_raman_tab.setLayout(raman1_layout)
        
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
        self.ts2_uv_tab = QWidget()
        uv2_layout = QVBoxLayout()
        uv2_layout.addWidget(QLabel("UV Fluorescence (Site 2)"))
        self.ts2_uv_tab.setLayout(uv2_layout)
        
        # TS2: Raman Spectroscopy
        self.ts2_raman_tab = QWidget()
        raman2_layout = QVBoxLayout()
        raman2_layout.addWidget(QLabel("Raman Spectroscopy (Site 2)"))
        self.ts2_raman_tab.setLayout(raman2_layout)
        
        self.ts2_tabs.addTab(self.ts2_uv_tab, "UV Fluorescence")
        self.ts2_tabs.addTab(self.ts2_raman_tab, "Raman Spectroscopy")
        
        ts2_layout.addWidget(self.ts2_tabs)
        self.test_site_2_tab.setLayout(ts2_layout)
        
        self.sub_tabs.addTab(self.dashboard_tab, "Dashboard")
        self.sub_tabs.addTab(self.test_site_1_tab, "Test Site 1")
        self.sub_tabs.addTab(self.test_site_2_tab, "Test Site 2")
        
        layout.addWidget(self.sub_tabs)
        self.setLayout(layout)


