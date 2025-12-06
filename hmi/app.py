import os
from flask import Flask, render_template, request

app = Flask(__name__)

status = [0, 0, 0, 0]
selected_map = -1

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/status")
def get_status():
    return { "status": status }



@app.route("/mod")
def mod():
    return render_template("mod.html")

@app.route("/mod/teleop")
def mod_teleop():
    return render_template("mod/teleop.html")

@app.route("/mod/pose")
def mod_pose():
    return render_template("mod/pose.html")


@app.route("/actions/teleop/start")
def start_teleop():
    print("starting teleop...")
    status[0] = 1
    return { "status": "success" }

@app.route("/actions/teleop/stop")
def stop_teleop():
    print("stopping teleop...")
    status[0] = 0
    return { "status": "success" }




@app.route("/actions/pose/list")
def list_maps():
    path = "static/maps"
    contents = os.listdir(path)
    return { "maps": contents }

@app.route("/actions/pose/start")
def start_pose():
    map = request.args.get('map')
    print("starting nav to pose with map", map, "...")
    status[2] = 1
    return { "status": "success" }

@app.route("/actions/pose/stop")
def stop_pose():
    print("stopping nav to pose...")
    status[2] = 0
    return { "status": "success" }