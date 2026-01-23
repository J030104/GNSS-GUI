
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from gnss_gui.components.shell_tabs import ShellTabs

def main():
    app = QApplication(sys.argv)
    win = QMainWindow()
    tabs = ShellTabs()
    win.setCentralWidget(tabs)
    win.resize(600, 400)
    win.show()
    
    # Add a couple of shells to test
    tabs.add_shell()
    tabs.add_shell()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
