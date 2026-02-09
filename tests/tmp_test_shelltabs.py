from gui.components.shell_tabs import ShellTabs
from PyQt5.QtWidgets import QApplication

app = QApplication([])
w = ShellTabs()
print('ShellTabs instance created successfully')
app.quit()
