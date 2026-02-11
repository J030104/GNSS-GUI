
import math
import random
from typing import List, Tuple, Optional

from PyQt5.QtCore import Qt, QTimer, QPointF
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QBrush
from PyQt5.QtWidgets import QWidget, QSizePolicy


class SpectralPlotWidget(QWidget):
    """A widget that draws a spectral plot (Intensity vs Raman Shift)."""

    def __init__(self, parent: Optional[QWidget] = None, title: Optional[str] = None) -> None:
        super().__init__(parent)
        self.plot_title = title
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(300, 200)

        # Data storage
        self.x_data: List[float] = []
        self.y_data: List[float] = []
        
        # Plot styling
        self.bg_color = Qt.white
        self.axis_color = Qt.black
        self.line_color = QColor(66, 135, 245) # Blue-ish
        self.line_width = 2
        
        # Margins for axes
        self.margin_left = 60
        self.margin_right = 20
        self.margin_top = 40
        self.margin_bottom = 50

        # Animation state
        self._t = 0.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_simulation)
        # Start animation by default for prototype
        self.start_simulation()

    def set_data(self, x: List[float], y: List[float]) -> None:
        """Update the plot data."""
        self.x_data = x
        self.y_data = y
        self.update() # Trigger repaint

    def start_simulation(self):
        """Start the dynamic data generation."""
        self._timer.start(100) # 10 FPS

    def stop_simulation(self):
        """Stop the dynamic data generation."""
        self._timer.stop()

    def _update_simulation(self):
        """Generate simulated Raman data."""
        self._t += 0.1
        
        # Generate X axis (0 to 1500 cm^-1)
        num_points = 300
        x = [i * (1500 / num_points) for i in range(num_points)]
        
        # Generate Y axis (peaks with some noise and movement)
        y = []
        
        # Define some peaks: center, width, height
        peaks = [
            (250, 20, 500),
            (350, 30, 800),
            (450, 15, 1500),
            (820, 10, 9000),      # Major peak 1
            (850, 15, 5000),      # Major peak 2
            (950, 25, 1200),
            (1100, 8, 11000)      # Sharp peak
        ]
        
        noise_level = 100
        
        for val in x:
            intensity = 200 # Baseline
            
            # Add peaks
            for center, width, height in peaks:
                # Add slight wobble to peak positions
                wobble = math.sin(self._t + center) * 5
                
                # Lorentzian-ish shape
                delta = val - (center + wobble)
                peak_val = height / (1 + (delta / width) ** 2)
                intensity += peak_val
            
            # Add random noise
            intensity += random.uniform(-noise_level, noise_level)
            
            y.append(intensity)
            
        self.set_data(x, y)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Fill background
        painter.fillRect(self.rect(), self.bg_color)
        
        w = self.width()
        h = self.height()
        
        # Drawing area
        # If no title, reduce top margin significantly
        if not self.plot_title:
            self.margin_top = 10
            
        plot_x = self.margin_left
        plot_y = self.margin_top
        plot_w = w - self.margin_left - self.margin_right
        plot_h = h - self.margin_top - self.margin_bottom
        
        # Draw Title
        painter.setPen(Qt.black)
        font = painter.font()
        font.setBold(True)
        font.setPointSize(12)
        painter.setFont(font)
        if self.plot_title:
            painter.drawText(0, 0, w, self.margin_top, Qt.AlignCenter, self.plot_title)
        
        # Draw Axes Labels
        font.setBold(False)
        font.setPointSize(10)
        painter.setFont(font)
        
        # X Axis Label
        painter.drawText(plot_x, plot_y + plot_h + 35, plot_w, 20, Qt.AlignCenter, "Raman Shift (cm⁻¹)")
        
        # Y Axis Label (rotated)
        painter.save()
        painter.translate(15, plot_y + plot_h / 2)
        painter.rotate(-90)
        painter.drawText(-plot_h // 2, -15, plot_h, 30, Qt.AlignCenter, "Intensity")
        painter.restore()
        
        # Draw Axes Lines
        painter.setPen(QPen(self.axis_color, 2))
        # Y Axis
        painter.drawLine(plot_x, plot_y, plot_x, plot_y + plot_h)
        # X Axis
        painter.drawLine(plot_x, plot_y + plot_h, plot_x + plot_w, plot_y + plot_h)
        
        # Determine scales
        if not self.x_data or not self.y_data:
            return
            
        min_x, max_x = min(self.x_data), max(self.x_data)
        # Fixed Y range for stability (0 to 16000 based on simulation peaks)
        min_y, max_y = 0, 16000
        
        if max_x == min_x: max_x += 1
        if max_y == min_y: max_y += 1
        
        scale_x = plot_w / (max_x - min_x)
        scale_y = plot_h / (max_y - min_y)
        
        # Helper to map data to pixels
        def map_pt(dx, dy):
            px = plot_x + (dx - min_x) * scale_x
            py = plot_y + plot_h - (dy - min_y) * scale_y
            return QPointF(px, py)
        
        # Draw Grid / Ticks (Simplified)
        painter.setPen(QPen(Qt.gray, 1, Qt.DotLine))
        # Y Ticks
        num_yticks = 5
        for i in range(num_yticks + 1):
            val = min_y + (max_y - min_y) * i / num_yticks
            y_pos = plot_y + plot_h - (val - min_y) * scale_y
            painter.drawLine(plot_x, int(y_pos), plot_x + plot_w, int(y_pos))
            painter.drawText(5, int(y_pos) - 10, self.margin_left - 10, 20, Qt.AlignRight | Qt.AlignVCenter, f"{int(val)}")

        # X Ticks
        num_xticks = 5
        for i in range(num_xticks + 1):
            val = min_x + (max_x - min_x) * i / num_xticks
            x_pos = plot_x + (val - min_x) * scale_x
            # painter.drawLine(int(x_pos), plot_y, int(x_pos), plot_y + plot_h) # Optional grid
            painter.drawText(int(x_pos) - 25, plot_y + plot_h + 5, 50, 20, Qt.AlignCenter, f"{int(val)}")

        # Draw Data Line
        painter.setPen(QPen(self.line_color, self.line_width))
        
        points = []
        for x, y in zip(self.x_data, self.y_data):
            points.append(map_pt(x, y))
            
        if points:
            painter.drawPolyline(*points)
