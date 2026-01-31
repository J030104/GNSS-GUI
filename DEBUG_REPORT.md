# GNSS-GUI Debug Report

**Branch**: `siddhant/cleanup-and-debug`  
**Date**: 2026-01-31  
**Tested by**: Siddhant (with AI assistance)

---

## Summary

The GUI **can launch and create windows** without crashing. However, there are several issues that could cause crashes during **runtime** (when actually using cameras/streams).

---

## Issues Found

### 🔴 Critical Issues

#### 1. ROS 2 Dependency (`rclpy`) - macOS Incompatible
**File**: `gnss_gui/utilities/telemetry_client.py`  
**Issue**: Imports `rclpy` which is only available in ROS 2 environment  
**Impact**: The telemetry client cannot work on macOS/Windows for development  
**Fix**: Already handled with try/except, but logs error on startup

#### 2. Broken venv (FIXED)
**Issue**: The `venv/` folder was pointing to an old path  
**Fix**: Recreated venv with `python3 -m venv venv`

---

### 🟡 Architecture Issues (Potential Crash Sources)

#### 1. Complex State Management
**File**: `gnss_gui/subsystems/gnss_comm.py`  
**Variables**:
- `_camera_states` - dict mapping camera names to state
- `_attached_viewers` - list of attached viewers
- `_attached_camera` - current attached camera

**Problem**: Three separate places tracking state that can get out of sync.

**Evidence** (lines ~295-400):
```python
# In _on_layout_tab_changed():
state['attached_viewers'] = new_attached  # Updates dict
self._attached_viewers = attached  # Also updates separate list
```

**Crash Scenario**: 
1. User starts stream on Camera A
2. `_camera_states['Camera A']['streaming'] = True`
3. User switches tab
4. `_on_layout_tab_changed()` rebuilds `attached_viewers`
5. User reconfigures stream
6. State desync → crash

#### 2. Exception Swallowing
**File**: Multiple files  
**Issue**: Lots of `except Exception: pass` hiding real errors

**Examples**:
```python
# gnss_comm.py line ~303
except Exception:
    pass  # WHAT WENT WRONG?

# gnss_comm.py line ~347  
except Exception:
    pass  # HIDING ERRORS
```

**Fix**: Add logging:
```python
except Exception as e:
    self.log_viewer.append(f"Error: {e}")
```

#### 3. Fuzzy Camera Name Matching
**File**: `gnss_comm.py` lines ~295-320  
**Issue**: Uses string containment for matching cameras to viewers

```python
kn = key.lower().replace(' ', '')
vn = vname.lower().replace(' ', '')
if kn in vn or vn in kn:
    matched_key = key
```

**Problem**: "Camera 1" matches "Camera 10", etc.

---

### 🟢 Minor Issues

#### 1. Missing Class Export (`ShellTabsWidget`)
**File**: `gnss_gui/components/shell_tabs.py`  
**Issue**: Class is named `ShellTabs`, not `ShellTabsWidget`  
**Impact**: None (correct name used in gnss_comm.py)

#### 2. VideoViewer Constructor Signature
**File**: `gnss_gui/components/video_viewer.py`  
**Signature**: `__init__(self, fps: int = 10, parent=None, camera_name: str = "Camera")`  
**Issue**: First argument is `fps`, not a name string  
**Impact**: Test script was wrong, actual usage is correct

---

## Files Analyzed

| File | Lines | Status |
|------|-------|--------|
| `main.py` | 57 | ✅ OK |
| `subsystems/gnss_comm.py` | 636 | ⚠️ State management issues |
| `components/video_viewer.py` | 305 | ✅ OK |
| `components/video_layout_tabs.py` | 80 | ✅ OK |
| `components/shell_tabs.py` | 730 | ✅ OK |
| `utilities/video_streamer.py` | 468 | ✅ OK |
| `utilities/telemetry_client.py` | ? | ⚠️ ROS 2 dependency |

---

## Recommended Fixes

### Priority 1: Add Real Error Logging
Replace all `except Exception: pass` with proper logging.

### Priority 2: Simplify State Management
Replace the three state variables with a single source of truth:

```python
# Instead of:
self._camera_states = {}      # dict
self._attached_viewers = []   # list  
self._attached_camera = None  # single value

# Use single dict:
self._streams = {
    'camera_name': {
        'source': CameraSource,
        'viewers': [VideoViewer, ...],
        'is_running': bool,
        'settings': {...}
    }
}
```

### Priority 3: Strict Camera Matching
Replace fuzzy matching with exact matching:

```python
# Instead of:
if kn in vn or vn in kn:

# Use:
if key == vname or CAMERA_MAPPING.get(key) == vname:
```

---

## How to Test Without Cameras

Run the offline test:
```bash
cd GNSS-GUI
./venv/bin/python test_gui_offline.py
```

Launch the GUI (will show blank video boxes):
```bash
cd GNSS-GUI
./venv/bin/python -m gnss_gui.main
```

---

## Next Steps

1. [ ] Fix exception handling (add logging)
2. [ ] Test on actual Jetson with cameras
3. [ ] Simplify state management
4. [ ] Add unit tests for stream start/stop/reconfigure cycle
