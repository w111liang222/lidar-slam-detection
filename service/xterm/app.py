from flask import Flask, render_template
from flask_cors import CORS
from flask_socketio import SocketIO
import pty
import os
import subprocess
import select
import termios
import struct
import fcntl

app = Flask(__name__, template_folder=".", static_folder=".", static_url_path="")
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
CORS(app)

app.config["SECRET_KEY"] = "secret!"
app.config["fd"] = None
app.config["child_pid"] = None
socketio = SocketIO(app)

os.environ["TERM"] = "xterm-256color"

def set_winsize(fd, row, col, xpix=0, ypix=0):
    winsize = struct.pack("HHHH", row, col, xpix, ypix)
    fcntl.ioctl(fd, termios.TIOCSWINSZ, winsize)

def read_and_forward_pty_output():
    while True:
        try:
            socketio.sleep(0.01)
            if app.config["fd"] is None:
                continue
            data_ready, _, _ = select.select([app.config["fd"]], [], [], 0)
            if data_ready:
                output = os.read(app.config["fd"], 1024 * 1024).decode()
                socketio.emit("pty-output", {"output": output}, namespace="/pty")
        except Exception as e:
            print(e)

@app.route("/")
def index():
    return render_template("index.html")

@socketio.on("pty-input", namespace="/pty")
def pty_input(data):
    """write to the child pty. The pty sees this as if you are typing in a real
    terminal.
    """
    try:
        if app.config["fd"]:
            os.write(app.config["fd"], data["input"].encode())
    except Exception as e:
        print(e)

@socketio.on("resize", namespace="/pty")
def resize(data):
    try:
        if app.config["fd"]:
            set_winsize(app.config["fd"], data["rows"], data["cols"])
    except Exception as e:
        print(e)

@socketio.on("connect", namespace="/pty")
def connect():
    """new client connected"""
    if app.config["child_pid"]:
        # already started child process, don't start another
        return

    # create child process attached to a pty we can read from and write to
    child_pid, fd = pty.fork()
    if child_pid == 0:
        # this is the child process fork.
        # anything printed here will show up in the pty, including the output
        # of this subprocess
        while True:
            subprocess.run(["su", "-l", "znqc"])
    else:
        # this is the parent process fork.
        app.config["fd"] = fd
        app.config["child_pid"] = child_pid
        socketio.start_background_task(target=read_and_forward_pty_output)
