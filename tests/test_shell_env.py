
import sys
import os
from PyQt5.QtCore import QProcess, QProcessEnvironment, QCoreApplication

app = QCoreApplication(sys.argv)
proc = QProcess()

env = QProcessEnvironment.systemEnvironment()
env.insert('PS1', '$ ')
env.insert('PROMPT', '%# ')
proc.setProcessEnvironment(env)

def on_out():
    print("STDOUT:", proc.readAllStandardOutput().data().decode())
    
def on_err():
    print("STDERR:", proc.readAllStandardError().data().decode())

def on_finished(exit_code, exit_status):
    print(f"Finished: {exit_code}, {exit_status}")
    app.quit()

proc.readyReadStandardOutput.connect(on_out)
proc.readyReadStandardError.connect(on_err)
proc.finished.connect(on_finished)

proc.start('zsh', ['-i'])
print("Started")

# timer to quit if it hangs
from PyQt5.QtCore import QTimer
QTimer.singleShot(2000, lambda: (print("Timeout"), proc.kill()))

sys.exit(app.exec_())
