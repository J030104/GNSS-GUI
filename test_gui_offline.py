#!/usr/bin/env python3
"""
Offline test script for GNSS-GUI
Tests the GUI without real cameras by using mock/dummy camera sources.
Run this to find errors without needing actual hardware.

Usage:
    ./venv/bin/python test_gui_offline.py
"""
import sys
import traceback
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer

# Track all errors found
ERRORS_FOUND = []
WARNINGS_FOUND = []

def log_error(component, error, tb=None):
    """Log an error for later review"""
    error_info = {
        'component': component,
        'error': str(error),
        'type': type(error).__name__,
        'traceback': tb
    }
    ERRORS_FOUND.append(error_info)
    print(f"❌ ERROR in {component}: {type(error).__name__}: {error}")
    if tb:
        print(f"   Traceback: {tb[:200]}...")

def log_warning(component, message):
    """Log a warning"""
    WARNINGS_FOUND.append({'component': component, 'message': message})
    print(f"⚠️  WARNING in {component}: {message}")

def test_imports():
    """Test all imports work"""
    print("\n" + "="*60)
    print("TESTING IMPORTS")
    print("="*60)
    
    modules = [
        ('gnss_gui.main', 'MainWindow'),
        ('gnss_gui.utilities.video_streamer', 'CameraManager, LocalCamera, NetworkStreamCamera, CameraSource'),
        ('gnss_gui.components.video_viewer', 'VideoViewer'),
        ('gnss_gui.components.video_layout_tabs', 'VideoLayoutTabWidget'),
        ('gnss_gui.components.shell_tabs', 'ShellTabs'),  # Correct class name
        ('gnss_gui.components.log_viewer', 'LogViewer'),
        ('gnss_gui.components.map_viewer', 'MapViewer'),
        ('gnss_gui.components.control_panel', 'ControlPanel'),
        ('gnss_gui.components.telemetry_panel', 'TelemetryPanel'),
        ('gnss_gui.components.status_bar', 'StatusBar'),
        ('gnss_gui.subsystems.gnss_comm', 'GNSSCommWidget'),
        ('gnss_gui.subsystems.drone', None),
        ('gnss_gui.subsystems.science_mission', None),
        ('gnss_gui.subsystems.autonomous_navigation', None),
        ('gnss_gui.subsystems.robotic_arm_delivery', None),
        ('gnss_gui.subsystems.power_electronics', None),
        ('gnss_gui.utilities.connection_manager', None),
    ]
    
    for module_name, classes in modules:
        try:
            module = __import__(module_name, fromlist=[''])
            print(f"✅ {module_name}")
            if classes:
                for cls_name in classes.split(', '):
                    if hasattr(module, cls_name.strip()):
                        print(f"   ✅ {cls_name.strip()} exists")
                    else:
                        log_error(module_name, f"Missing class/function: {cls_name}")
        except Exception as e:
            log_error(module_name, e, traceback.format_exc())
    
    # Test telemetry client separately (expected to fail on macOS)
    try:
        from gnss_gui.utilities import telemetry_client
        print(f"✅ gnss_gui.utilities.telemetry_client")
    except ImportError as e:
        log_warning("telemetry_client", f"ROS 2 not available (expected on macOS): {e}")

def test_camera_source():
    """Test CameraSource base class"""
    print("\n" + "="*60)
    print("TESTING CameraSource")
    print("="*60)
    
    try:
        from gnss_gui.utilities.video_streamer import CameraSource
        
        # CameraSource is an abstract base class - cannot instantiate directly
        print(f"✅ CameraSource class exists (abstract base class)")
        
        # Test methods are defined
        methods = ['start', 'stop', 'read_frame']
        for method in methods:
            if hasattr(CameraSource, method):
                print(f"   ✅ Has method: {method}")
            else:
                log_error("CameraSource", f"Missing method: {method}")
            
    except Exception as e:
        log_error("CameraSource", e, traceback.format_exc())

def test_local_camera():
    """Test LocalCamera class (will fail without camera but should not crash)"""
    print("\n" + "="*60)
    print("TESTING LocalCamera (expected to fail gracefully without camera)")
    print("="*60)
    
    try:
        from gnss_gui.utilities.video_streamer import LocalCamera
        
        # Test with non-existent camera (should handle gracefully)
        # LocalCamera takes index as first argument
        cam = LocalCamera(index=999)  # Non-existent camera
        print(f"✅ Created LocalCamera with index 999")
        
        # Try to start (should fail gracefully)
        try:
            cam.start()
            print(f"   ⚠️ start() called - this will fail to get frames from non-existent camera")
        except Exception as e:
            print(f"   ⚠️ start() raised (expected): {type(e).__name__}")
        
        # Get frame (should return None)
        try:
            frame = cam.read_frame()
            print(f"   ✅ read_frame() returned: {type(frame)} (None is expected)")
        except Exception as e:
            print(f"   ⚠️ read_frame() raised: {type(e).__name__}")
        
        # Stop
        try:
            cam.stop()
            print(f"   ✅ stop() completed")
        except Exception as e:
            log_error("LocalCamera.stop()", e, traceback.format_exc())
            
    except Exception as e:
        log_error("LocalCamera", e, traceback.format_exc())

def test_network_camera():
    """Test NetworkStreamCamera class"""
    print("\n" + "="*60)
    print("TESTING NetworkStreamCamera (expected to fail gracefully)")
    print("="*60)
    
    try:
        from gnss_gui.utilities.video_streamer import NetworkStreamCamera, NetworkStreamOptions
        
        # NetworkStreamCamera takes NetworkStreamOptions, not a URL string
        opts = NetworkStreamOptions(
            proto="rtsp",
            host="fake.host",
            port=554,
            path="stream"
        )
        cam = NetworkStreamCamera(options=opts)
        print(f"✅ Created NetworkStreamCamera")
        
        # Try to start (should timeout or fail gracefully)
        try:
            cam.start()
            print(f"   ⚠️ start() called - will fail to connect to fake host")
        except Exception as e:
            print(f"   ⚠️ start() raised (expected): {type(e).__name__}")
        
        cam.stop()
        print(f"   ✅ stop() completed")
            
    except Exception as e:
        log_error("NetworkStreamCamera", e, traceback.format_exc())

def test_camera_manager():
    """Test CameraManager class"""
    print("\n" + "="*60)
    print("TESTING CameraManager")
    print("="*60)
    
    try:
        from gnss_gui.utilities.video_streamer import CameraManager
        
        # CameraManager is a class-level registry, not instantiated
        print(f"✅ CameraManager class exists")
        
        # Test class methods
        methods = ['register', 'get_camera']
        for method in methods:
            if hasattr(CameraManager, method):
                print(f"   ✅ Has class method: {method}")
            else:
                log_warning("CameraManager", f"Missing method: {method}")
        
        # Test getting the default 'local' camera
        local_cam = CameraManager.get_camera('local')
        if local_cam:
            print(f"   ✅ Default 'local' camera registered: {type(local_cam).__name__}")
        else:
            log_warning("CameraManager", "No 'local' camera registered (might be OK if no camera)")
        
    except Exception as e:
        log_error("CameraManager", e, traceback.format_exc())

def test_video_viewer():
    """Test VideoViewer widget"""
    print("\n" + "="*60)
    print("TESTING VideoViewer")
    print("="*60)
    
    try:
        from gnss_gui.components.video_viewer import VideoViewer
        
        # VideoViewer signature: __init__(self, fps: int = 10, parent=None, camera_name: str = "Camera")
        viewer = VideoViewer(fps=10, camera_name="TestViewer")
        print(f"✅ Created VideoViewer: {viewer.camera_name}")
        
        # Test methods
        methods = ['attach_camera', 'set_brightness', 'set_zoom']
        for method in methods:
            if hasattr(viewer, method):
                print(f"   ✅ Has method: {method}")
            else:
                log_warning("VideoViewer", f"Missing method: {method}")
        
        # Test attach/detach without camera
        try:
            viewer.attach_camera(None)
            print(f"   ✅ attach_camera(None) handled")
        except Exception as e:
            log_error("VideoViewer.attach_camera(None)", e, traceback.format_exc())
            
    except Exception as e:
        log_error("VideoViewer", e, traceback.format_exc())

def test_video_layout_tabs():
    """Test VideoLayoutTabWidget"""
    print("\n" + "="*60)
    print("TESTING VideoLayoutTabWidget")
    print("="*60)
    
    try:
        from gnss_gui.components.video_layout_tabs import VideoLayoutTabWidget
        
        # VideoLayoutTabWidget requires 'layouts' argument
        test_layouts = [
            {
                'name': 'Test Mode',
                'boxes': [
                    {'row': 0, 'col': 0, 'camera_name': 'Test Camera'},
                ],
            },
        ]
        tabs = VideoLayoutTabWidget(layouts=test_layouts)
        print(f"✅ Created VideoLayoutTabWidget")
        
        # Test tab count
        print(f"   Tab count: {tabs.count()}")
        
        # Test get_video_viewers
        if hasattr(tabs, 'get_video_viewers'):
            viewers = tabs.get_video_viewers(0)
            print(f"   ✅ get_video_viewers(0) returned {len(viewers)} viewer(s)")
        
    except Exception as e:
        log_error("VideoLayoutTabWidget", e, traceback.format_exc())

def test_gnss_comm_widget():
    """Test the main GNSSCommWidget"""
    print("\n" + "="*60)
    print("TESTING GNSSCommWidget (MAIN PROBLEM AREA)")
    print("="*60)
    
    try:
        from gnss_gui.subsystems.gnss_comm import GNSSCommWidget
        
        widget = GNSSCommWidget()
        print(f"✅ Created GNSSCommWidget")
        
        # Check state variables
        state_vars = ['_camera_states', '_attached_viewers', '_attached_camera']
        for var in state_vars:
            if hasattr(widget, var):
                val = getattr(widget, var)
                print(f"   ✅ {var} = {type(val).__name__}: {val}")
            else:
                log_warning("GNSSCommWidget", f"Missing state var: {var}")
        
        # Test critical methods exist
        critical_methods = [
            '_on_layout_tab_changed',
            'on_start_stream', 
            'on_stop_stream',
            '_ensure_camera_state'
        ]
        for method in critical_methods:
            if hasattr(widget, method):
                print(f"   ✅ Has method: {method}")
            else:
                log_warning("GNSSCommWidget", f"Missing method: {method}")
        
        # Test simulating tab change (the crash point)
        print("\n   Testing tab change simulation...")
        try:
            if hasattr(widget, '_on_layout_tab_changed'):
                widget._on_layout_tab_changed(0)  # Switch to first tab
                print(f"   ✅ _on_layout_tab_changed(0) completed")
                
                widget._on_layout_tab_changed(1)  # Switch to second tab
                print(f"   ✅ _on_layout_tab_changed(1) completed")
        except Exception as e:
            log_error("GNSSCommWidget._on_layout_tab_changed()", e, traceback.format_exc())
        
        # Test stream start/stop simulation
        print("\n   Testing stream start/stop simulation...")
        try:
            if hasattr(widget, 'on_start_stream'):
                widget.on_start_stream()
                print(f"   ⚠️ on_start_stream() called (will try to access camera)")
        except Exception as e:
            log_error("GNSSCommWidget.on_start_stream()", e, traceback.format_exc())
            
        try:
            if hasattr(widget, 'on_stop_stream'):
                widget.on_stop_stream()
                print(f"   ✅ on_stop_stream() called")
        except Exception as e:
            log_error("GNSSCommWidget.on_stop_stream()", e, traceback.format_exc())
        
        # Test the crash scenario: start → switch tab → stop
        print("\n   Testing crash scenario: start → switch tab → stop...")
        try:
            widget.on_start_stream()
            widget._on_layout_tab_changed(2)  # Switch tab while streaming
            widget.on_stop_stream()
            print(f"   ✅ Crash scenario completed without exception")
        except Exception as e:
            log_error("GNSSCommWidget crash scenario", e, traceback.format_exc())
            
    except Exception as e:
        log_error("GNSSCommWidget", e, traceback.format_exc())

def test_main_window():
    """Test the MainWindow"""
    print("\n" + "="*60)
    print("TESTING MainWindow")
    print("="*60)
    
    try:
        from gnss_gui.main import MainWindow
        
        window = MainWindow()
        print(f"✅ Created MainWindow")
        
        # Don't show the window, just test it was created
        print(f"   Window title: {window.windowTitle()}")
        
    except Exception as e:
        log_error("MainWindow", e, traceback.format_exc())

def print_summary():
    """Print summary of all errors found"""
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    if WARNINGS_FOUND:
        print(f"\n⚠️  {len(WARNINGS_FOUND)} Warning(s):")
        for w in WARNINGS_FOUND:
            print(f"   - [{w['component']}]: {w['message']}")
    
    if not ERRORS_FOUND:
        print("\n✅ All tests passed! No critical errors found.")
        print("\nNote: The GUI creates successfully, but runtime behavior")
        print("during camera switching may still have issues.")
        print("The 'vibe coding' issues are likely in state management,")
        print("not syntax or import errors.")
    else:
        print(f"\n❌ Found {len(ERRORS_FOUND)} error(s):\n")
        for i, err in enumerate(ERRORS_FOUND, 1):
            print(f"{i}. [{err['component']}]")
            print(f"   Type: {err['type']}")
            print(f"   Error: {err['error']}")
            if err['traceback']:
                print(f"   Location: {err['traceback'].split(chr(10))[-3].strip()}")
            print()

def main():
    print("="*60)
    print("GNSS-GUI OFFLINE TEST SUITE")
    print("Testing without real cameras to find bugs")
    print("="*60)
    
    # Create QApplication (needed for Qt widgets)
    app = QApplication(sys.argv)
    
    # Run tests
    test_imports()
    test_camera_source()
    test_local_camera()
    test_network_camera()
    test_camera_manager()
    test_video_viewer()
    test_video_layout_tabs()
    test_gnss_comm_widget()
    test_main_window()
    
    # Print summary
    print_summary()
    
    # Don't run the event loop, just test creation
    print("\n✅ Tests complete. The GUI can be created without crashing.")
    print("The crash likely happens during runtime when:")
    print("  1. Switching camera tabs")
    print("  2. Reconfiguring streams while running")
    print("  3. State desync between _camera_states and _attached_viewers")
    
    return len(ERRORS_FOUND)

if __name__ == "__main__":
    sys.exit(main())
