"""Exercise the live mouse UI in headless Chrome/Edge. Optional dependency: websocket-client."""

import argparse
from contextlib import contextmanager
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
from threading import Thread
import time
from urllib.request import urlopen

from quest_pose_receiver import make_server
from replay import inspect_episode
from station import Station


def wait_for(read, label, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = read()
        if result:
            return result
        time.sleep(.04)
    raise AssertionError(f"Timed out waiting for {label}")


class Browser:
    def __init__(self, socket):
        self.socket = socket
        self.count = 0
        self.errors = []

    def call(self, method, **params):
        self.count += 1
        self.socket.send(json.dumps(dict(id=self.count, method=method, params=params)))
        while True:
            response = json.loads(self.socket.recv())
            if response.get("method") == "Runtime.exceptionThrown":
                self.errors.append(response["params"]["exceptionDetails"])
            if response.get("id") == self.count:
                if "error" in response:
                    raise RuntimeError(response["error"])
                return response.get("result", {})

    def evaluate(self, expression):
        data = self.call("Runtime.evaluate", expression=expression, returnByValue=True)
        if "exceptionDetails" in data:
            raise RuntimeError(data["exceptionDetails"])
        return data["result"].get("value")

    def center(self, element):
        return self.evaluate(
            f"(() => {{ const e=document.getElementById({json.dumps(element)}); "
            "e.scrollIntoView({block:'center',behavior:'instant'}); const r=e.getBoundingClientRect(); "
            "return [r.x+r.width/2,r.y+r.height/2] })()"
        )

    def pointer(self, kind, xy, button="none", buttons=0, **extra):
        self.call("Input.dispatchMouseEvent", type=kind, x=xy[0], y=xy[1],
                  button=button, buttons=buttons, clickCount=1, **extra)

    def click(self, element):
        check = f"document.getElementById({json.dumps(element)})"
        wait_for(lambda: self.evaluate(
            f"(() => {{ const e = {check}; return !!e && !e.disabled && e.getClientRects().length > 0 }})()"
        ), f"{element} control visible and enabled")
        xy = self.center(element)
        self.pointer("mouseMoved", xy)
        xy = self.center(element)
        assert self.evaluate(f"{check}.contains(document.elementFromPoint({xy[0]},{xy[1]}))"), element
        self.pointer("mousePressed", xy, "left", 1)
        self.pointer("mouseReleased", xy, "left")

    def key(self, code, key, down):
        self.call("Input.dispatchKeyEvent", type="keyDown" if down else "keyUp", code=code, key=key)


def locate_browser():
    candidates = ["google-chrome", "chromium", "chromium-browser", "msedge",
                  "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
                  r"C:\Program Files\Google\Chrome\Application\chrome.exe",
                  r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"]
    for path in candidates:
        binary = shutil.which(path) or (path if Path(path).is_file() else None)
        if binary:
            return binary
    raise SystemExit("Specify an installed Chrome/Chromium/Edge executable using --browser.")


@contextmanager
def browser(binary, profile):
    try:
        import websocket
    except ImportError:
        raise SystemExit("This optional browser test needs python -m pip install websocket-client")
    process = subprocess.Popen([binary, "--headless", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
                                "--no-first-run",
                                "--disable-background-networking", "--disable-extensions",
                                "--disable-component-update", "--remote-debugging-port=0",
                                "--window-size=1280,920", f"--user-data-dir={profile}", "about:blank"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    sock = None
    try:
        port_file = profile / "DevToolsActivePort"
        wait_for(lambda: port_file.is_file() and port_file.stat().st_size, "headless browser", 18)
        port = int(port_file.read_text().splitlines()[0])
        with urlopen(f"http://127.0.0.1:{port}/json/list", timeout=5) as stream:
            target = next(page for page in json.load(stream) if page["type"] == "page")
        sock = websocket.create_connection(target["webSocketDebuggerUrl"], timeout=8, suppress_origin=True)
        client = Browser(sock)
        client.call("Page.enable")
        client.call("Runtime.enable")
        yield client
    finally:
        if sock:
            try:
                sock.send(json.dumps({"id":999999,"method":"Browser.close"}))
            except OSError:
                pass
            sock.close()
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


def same_pose(a, b):
    return all(math.isclose(x, y, abs_tol=1e-10) for field in ("p","q")
               for x,y in zip(a[field],b[field]))


def rotation_deg(a, b):
    # q and -q represent the same rotation.
    dot = abs(sum(x*y for x, y in zip(a, b)))
    return math.degrees(2 * math.acos(min(1, dot)))


def check_robot(hand):
    robot = hand["robot"]
    assert len(robot["joint_angles_rad"]) == len(robot["joint_positions_m"]) == 7
    assert all(math.isfinite(v) for v in robot["joint_angles_rad"])
    target, actual = hand["tool"], robot["tcp"]
    assert math.dist(target["p"], actual["p"]) <= 0.0005, (target, actual)
    assert rotation_deg(target["q"], actual["q"]) <= 0.3, (target, actual)
    assert abs(math.dist(*robot["fingers_m"]) - robot["jaw_opening_m"]) < 1e-6
    return robot


def robot_image(client):
    # Sample the robot pane independently of the hand and jaw close-ups.
    png = client.evaluate("""(() => {
      const pad = document.getElementById('pad'), crop = document.createElement('canvas');
      crop.width = 180; crop.height = 120;
      crop.getContext('2d').drawImage(pad, 0,0,pad.width*.65,pad.height, 0,0,crop.width,crop.height);
      return crop.toDataURL();
    })()""")
    return hash(png)


def exercise(client, address, station):
    client.call("Page.navigate", url=address)
    wait_for(lambda: client.evaluate("document.readyState === 'complete' && !!document.getElementById('pad')"),
             "studio to load")
    client.click("mouse-start")
    def hand():
        return station.state()["hands"]["right"]
    wait_for(lambda: hand()["initial"], "first mouse sample")
    assert not station.state()["held"]
    left_home = station.state()["hands"]["left"]["tool"]
    tool0 = hand()["tool"]
    robot0 = check_robot(hand())
    left_joints = station.state()["hands"]["left"]["robot"]["joint_angles_rad"]
    client.click("face-a")
    wait_for(lambda: station.recorder.status()["active"] and client.evaluate(
        "document.getElementById('record').textContent === 'Save & stay'"), "recording start")
    client.click("capture")
    wait_for(lambda: "captured for right" in client.evaluate("document.getElementById('notice').textContent"),
             "manual reference capture")
    time.sleep(.35)
    xy = client.center("pad")
    client.pointer("mousePressed", xy, "right", 2)
    wait_for(lambda: hand()["clutched"], "right grip engage")
    initial_scene = robot_image(client)
    xy[0] += 46
    xy[1] -= 24
    client.pointer("mouseMoved", xy, "right", 2)
    wait_for(lambda: math.isclose(hand()["delta"]["p"][0], .046), "46 mm lateral drag")
    assert math.isclose(hand()["tool"]["p"][1] - tool0["p"][1], .024)
    time.sleep(.35)
    client.pointer("mouseWheel", xy, "right", 2, deltaX=0, deltaY=-100)
    wait_for(lambda: math.isclose(hand()["delta"]["p"][2], -.04), "40 mm depth scroll")
    moved = check_robot(hand())
    assert math.dist(moved["joint_angles_rad"], robot0["joint_angles_rad"]) > .01
    assert any(math.dist(p, q) > .005 for p, q in zip(
        moved["joint_positions_m"][1:], robot0["joint_positions_m"][1:]))
    wait_for(lambda: robot_image(client) != initial_scene, "redrawn moving arm")
    position = hand()["tool"]["p"]
    client.pointer("mouseReleased", xy, "right")
    wait_for(lambda: not hand()["clutched"], "release before switching gesture")
    client.click("mode-rotate")
    xy = client.center("pad")
    client.pointer("mousePressed", xy, "right", 2)
    wait_for(lambda: hand()["clutched"], "rotate grip")
    xy[0] += 30
    xy[1] += 10
    client.pointer("mouseMoved", xy, "right", 2)
    wait_for(lambda: abs(hand()["tool"]["q"][1]) > .05, "pitch/yaw rotation")
    client.pointer("mouseWheel", xy, "right", 2, deltaX=0, deltaY=120)
    wait_for(lambda: abs(hand()["tool"]["q"][2]) > .05, "wrist roll")
    assert hand()["tool"]["p"] == position
    rotated = check_robot(hand())
    assert math.dist(moved["tcp"]["p"], rotated["tcp"]["p"]) < .001
    assert rotation_deg(moved["tcp"]["q"], rotated["tcp"]["q"]) > 5
    assert math.dist(moved["joint_angles_rad"], rotated["joint_angles_rad"]) > .01
    time.sleep(.35)
    client.pointer("mouseReleased", xy, "right")
    wait_for(lambda: not hand()["clutched"], "rotate grip release")
    client.click("mode-gripper")
    xy = client.center("pad")
    client.pointer("mousePressed", xy, "right", 2)
    wait_for(lambda: hand()["clutched"], "gripper grip")
    xy[0] += 64
    client.pointer("mouseMoved", xy, "right", 2)
    wait_for(lambda: math.isclose(hand()["closure"], .4), "trigger pickup to 40% closure")
    grasp = check_robot(hand())
    assert math.isclose(grasp["jaw_opening_m"], .6 * robot0["jaw_opening_m"], abs_tol=1e-6)
    wait_for(lambda: client.evaluate("parseFloat(document.getElementById('jaw-opening').textContent)") ==
             round(grasp["jaw_opening_m"]*1000, 1), "gripper opening displayed in millimetres")
    time.sleep(.35)
    client.pointer("mouseReleased", xy, "right")
    wait_for(lambda: not hand()["clutched"], "clutch release")
    held_tool = hand()["tool"]
    client.click("mode-move")
    xy = client.center("pad")
    client.pointer("mousePressed", xy, "left", 1)
    xy[0] -= 75
    client.pointer("mouseMoved", xy, "left", 1)
    client.pointer("mouseReleased", xy, "left")
    for _ in range(8):
        client.key("BracketLeft", "[", True)
        client.key("BracketLeft", "[", False)
    wait_for(lambda: hand()["buttons"][0]["value"] < .01 and hand()["delta"]["p"][0] < 0,
             "released hand reposition and trigger reset")
    client.key("Space", " ", True)
    wait_for(lambda: hand()["clutched"], "Space re-clutch")
    assert same_pose(hand()["tool"], held_tool) and math.isclose(hand()["closure"], .4), (held_tool, hand())
    assert not hand()["trigger_live"]
    assert check_robot(hand())["joint_angles_rad"] == grasp["joint_angles_rad"]
    time.sleep(.35)
    client.key("Space", " ", False)
    wait_for(lambda: not hand()["clutched"], "Space release")
    client.click("mode-gripper")
    xy = client.center("pad")
    client.pointer("mousePressed", xy, "right", 2)
    wait_for(lambda: hand()["clutched"], "grip for opening")
    xy[0] += 64
    client.pointer("mouseMoved", xy, "right", 2)
    wait_for(lambda: hand()["trigger_live"], "trigger matches retained grasp")
    xy[0] -= 64
    client.pointer("mouseMoved", xy, "right", 2)
    wait_for(lambda: math.isclose(hand()["closure"], 0, abs_tol=1e-6), "trigger opens gripper")
    assert math.isclose(check_robot(hand())["jaw_opening_m"], robot0["jaw_opening_m"], abs_tol=1e-6)
    client.pointer("mouseReleased", xy, "right")
    wait_for(lambda: not hand()["clutched"], "release open gripper")
    client.click("mark")
    wait_for(lambda: client.evaluate("document.getElementById('notice').hidden"), "marker action settled")
    client.click("view")
    wait_for(lambda: client.evaluate("document.getElementById('camera-name').textContent") != "Overview"
             and station.recorder.status()["selected_camera"] == "left_wrist"
             and "left" in client.evaluate("document.getElementById('camera-name').textContent").lower(),
             "left wrist displayed")
    client.click("face-b")
    wait_for(lambda: station.state()["held"] and not hand()["buttons"][5]["pressed"], "software hold and B release")
    client.click("resume")
    wait_for(lambda: not station.state()["held"], "deliberate local resume")
    assert same_pose(hand()["tool"], held_tool)
    assert station.state()["hands"]["left"]["tool"] == left_home
    assert station.state()["hands"]["left"]["robot"]["joint_angles_rad"] == left_joints
    client.click("face-a")
    wait_for(lambda: not station.recorder.status()["active"], "episode save")
    wait_for(lambda: station.state()["trial_return"]["status"] == "stayed" and
             "saved" in client.evaluate("document.getElementById('rec-state').textContent").lower(),
             "save and stay confirmation")
    assert same_pose(station.state()["hands"]["right"]["tool"], held_tool)
    assert not client.errors, client.errors
    d = [1000*(b-a) for a,b in zip(robot0["tcp"]["p"], moved["tcp"]["p"])]
    print("Mouse FK displacement (mm):", [round(x,2) for x in d],
          "wrist change (deg):", round(rotation_deg(moved["tcp"]["q"], rotated["tcp"]["q"]),2),
          "open / grasped jaw (mm):", [round(1000*r["jaw_opening_m"],2) for r in (robot0,grasp)])
    return station.recorder.status()["episode"]


def check_episode(path):
    report = inspect_episode(path)
    assert report["status"] == "saved" and report["streams"]["actions"] > 1
    assert report["dropped_input_records"] == 0
    assert all(camera["frames"] > 1 for camera in report["cameras"].values())
    with (path / "actions.jsonl").open(encoding="utf-8") as actions, \
            (path / "feedback.jsonl").open(encoding="utf-8") as feedback:
        for command, actual in zip(actions, feedback):
            command, actual = json.loads(command), json.loads(actual)
            assert command["seq"] == actual["seq"]
            for side in ("left", "right"):
                robot = check_robot(command["state"]["hands"][side])
                assert actual["hands"][side]["robot"] == robot
    print("Recorded", report["streams"]["actions"], "paired actions/feedback samples;",
          "camera frames", [cam["frames"] for cam in report["cameras"].values()])


def exercise_quest(client, address, station):
    """Inject XR poses/buttons, then use the application's Start Quest entry point."""
    client.click("stop")
    wait_for(lambda: station.state()["owner"] is None, "mouse disconnect")
    fixture = Path(__file__).parent / "tests" / "quest-input-fixture.js"
    client.call("Page.addScriptToEvaluateOnNewDocument", source=fixture.read_text(encoding="utf-8"))
    client.call("Page.navigate", url=address)
    wait_for(lambda: client.evaluate("!!window.questFixture && !!document.getElementById('xr-start')"),
             "mock headset and studio")
    client.click("xr-start")
    def hand():
        return station.state()["hands"]["right"]
    wait_for(lambda: hand()["profile"] == "oculus-touch-v3", "Quest input through receiver")
    assert station.state()["frame"] == "local-floor"
    wait_for(lambda: client.evaluate("Object.values(questFixture.meshVertices).every(n => n > 100)"),
             "articulated mesh drawn into both mock eye viewports")
    eye_vertices = client.evaluate("questFixture.meshVertices")
    assert eye_vertices["left"] == eye_vertices["right"]
    pixels = client.evaluate("questFixture.robotPixels")
    assert min(pixels.values()) > 30, pixels
    wait_for(lambda: station.state()["grip_only"] and hand()["status"] == "released"
             and client.evaluate("document.getElementById('hold-panel').hidden"), "Quest grip controls")
    client.click("restore_initial")
    def restored():
        result = station.state()["trial_return"]
        return result.get("target") == "home" and result["status"] == "completed"
    wait_for(restored, "workstation button restores the simulator's initial pose")
    before = check_robot(hand())
    def button(index, value, side="right"):
        client.evaluate(f"questFixture.button({index},{value},{json.dumps(side)})")
        wait_for(lambda: station.state()["hands"][side]["buttons"][index]["pressed"] == bool(value),
                 f"raw {side} button {index} = {value}")

    def restore_chord(first, second):
        ack = station.state()["ack"]
        camera = station.recorder.status()["selected_camera"]
        active = station.recorder.status()["active"]
        button(4,1,first)
        assert station.state()["ack"] == ack
        button(4,1,second)
        wait_for(lambda: station.state()["ack"] == ack + 1, "one A + X restore event")
        # Re-pressing one button while the other remains down is the same gesture.
        button(4,0,first)
        button(4,1,first)
        button(4,0,second)
        button(4,0,first)
        wait_for(restored, "Quest chord restores the initial joints")
        assert station.state()["ack"] == ack + 1
        assert station.recorder.status()["active"] == active
        assert station.recorder.status()["selected_camera"] == camera

    button(3,1,"left")
    button(4,1)
    button(4,0)
    wait_for(lambda: station.recorder.status()["active"], "tapping A starts Quest trial with left stick clicked")
    button(3,0,"left")
    wait_for(lambda: client.evaluate("document.getElementById('finish_return').hidden"
                                     " && document.getElementById('face-b').hidden"), "Quest recording controls")
    button(5,1)
    client.evaluate("questFixture.button(1, 1)")
    wait_for(lambda: hand()["clutched"], "Quest side grip")
    client.evaluate("questFixture.hand.position.x -= .025")
    wait_for(lambda: math.isclose(hand()["clutch_delta"]["p"][0], -.025), "Quest translation")
    moved = check_robot(hand())
    assert abs(before["tcp"]["p"][0] - moved["tcp"]["p"][0] - .025) < .001
    assert math.dist(before["joint_angles_rad"], moved["joint_angles_rad"]) > .01
    button(5,0)
    client.evaluate("Object.assign(questFixture.hand.orientation, {y: Math.sin(.1), w: Math.cos(.1)})")
    wait_for(lambda: math.isclose(hand()["clutch_delta"]["q"][1], math.sin(.1)), "Quest wrist rotation")
    rotated = check_robot(hand())
    assert abs(rotation_deg(moved["tcp"]["q"], rotated["tcp"]["q"]) - math.degrees(.2)) < .6
    assert math.dist(moved["tcp"]["p"], rotated["tcp"]["p"]) < .001
    client.evaluate("questFixture.button(0, .65)")
    wait_for(lambda: math.isclose(hand()["closure"], .65), "Quest index trigger")
    grasp = check_robot(hand())
    assert math.isclose(grasp["jaw_opening_m"], .35 * before["jaw_opening_m"], abs_tol=1e-6)
    client.evaluate("questFixture.button(1, 0); questFixture.button(0, 0); questFixture.hand.position.x += .08")
    wait_for(lambda: hand()["buttons"][0]["value"] == 0 and not hand()["clutched"], "Quest release")
    assert hand()["robot"]["joint_angles_rad"] == grasp["joint_angles_rad"]
    assert hand()["robot"]["jaw_opening_m"] == grasp["jaw_opening_m"]
    old_view = station.recorder.status()["selected_camera"]
    button(4,1,"left")
    button(4,0,"left")
    wait_for(lambda: station.recorder.status()["selected_camera"] != old_view, "tapping X switches the view")
    restore_chord("right", "left")
    assert hand()["robot"]["joint_angles_rad"] == before["joint_angles_rad"]
    assert hand()["robot"]["jaw_opening_m"] == grasp["jaw_opening_m"]
    button(3,1,"left")
    button(4,1)
    button(4,0)
    wait_for(lambda: not station.recorder.status()["active"], "left stick click + A tap saves")
    button(3,0,"left")
    wait_for(lambda: station.state()["trial_return"]["status"] == "stayed", "Quest trial ends in place")
    endpoint = check_robot(hand())
    assert endpoint["joint_angles_rad"] == before["joint_angles_rad"]
    assert endpoint["jaw_opening_m"] == grasp["jaw_opening_m"]
    wait_for(lambda: client.evaluate("questFixture.hudLines.includes('Trial saved')"), "save painted in headset")
    restore_chord("left", "right")
    client.click("stop")
    wait_for(lambda: station.state()["owner"] is None, "Quest disconnect")
    assert not client.errors, client.errors
    print("Synthetic Quest Touch grip, 25 mm move with B pressed, 11.46 degree wrist rotation and 65% closure passed;",
          eye_vertices["right"], "mesh vertices per eye;",
          pixels, "visible robot pixels in sampled regions.")


def file_cameras(directory):
    """Generate three distinct color video sources for an offline OpenCV test."""
    import cv2
    import numpy as np
    from live_cameras import load_camera_config

    settings = {"width":96, "height":72, "fps":12, "cameras":{}}
    for i, name in enumerate(("overview","left_wrist","right_wrist")):
        path = directory / (name+".avi")
        writer = cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*"MJPG"),12,(96,72))
        try:
            assert writer.isOpened(), "OpenCV requires an MJPEG writer for this fixture"
            frame = np.zeros((72,96,3), dtype=np.uint8)
            frame[:,:,2-i] = 235  # Red overview, green left wrist, blue right wrist.
            for _ in range(480):
                writer.write(frame)
        finally:
            writer.release()
        settings["cameras"][name] = {"file":path.name}
    config = directory / "cameras.json"
    config.write_text(json.dumps(settings), encoding="utf-8")
    return load_camera_config(config)


def exercise_camera_workflow(client, address, station):
    client.call("Page.navigate", url=address)
    wait_for(lambda: client.evaluate("document.getElementById('jaw-control-label')?.textContent === 'Thumbstick Y'"),
             "workstation stick mode")
    wait_for(lambda: client.evaluate("document.getElementById('camera-age').textContent.startsWith('Live')"),
             "initial camera displayed")
    client.click("mouse-start")
    def hand():
        return station.state()["hands"]["right"]
    wait_for(lambda: station.state()["seq"] >= 6 and hand()["status"] == "released", "released mouse samples")
    xy = client.center("pad")
    client.pointer("mousePressed",xy,"right",2)
    wait_for(lambda: hand()["clutched"], "mouse grip")
    client.key("ArrowDown","ArrowDown",True)
    wait_for(lambda: hand()["closure"] > .15, "down closes jaw")
    client.key("ArrowDown","ArrowDown",False)
    wait_for(lambda: hand()["axes"][3] == 0, "mouse stick centered")
    held = hand()["closure"]
    tool = hand()["tool"]
    xy[0] += 18
    client.pointer("mouseMoved",xy,"right",2)
    wait_for(lambda: abs(hand()["tool"]["p"][0]-tool["p"][0]) > .016, "arm moves with retained opening")
    assert hand()["closure"] == held
    client.key("ArrowUp","ArrowUp",True)
    wait_for(lambda: hand()["closure"] < held-.05, "up opens jaw")
    client.key("ArrowUp","ArrowUp",False)
    client.pointer("mouseReleased",xy,"right")
    client.click("stop")

    fixture = Path(__file__).parent / "tests" / "quest-input-fixture.js"
    client.call("Page.addScriptToEvaluateOnNewDocument", source=fixture.read_text(encoding="utf-8"))
    client.call("Page.navigate",url=address)
    wait_for(lambda: client.evaluate("!!window.questFixture && !!document.getElementById('xr-start')"),
             "mock Quest with three video sources")
    client.click("xr-start")
    wait_for(lambda: hand()["profile"] == "oculus-touch-v3", "Quest controller stream")
    wait_for(lambda: station.state()["grip_only"] and not station.state()["held"], "Quest takes control")
    def button(index, pressed, side):
        client.evaluate(f"questFixture.button({index},{int(pressed)},{json.dumps(side)})")
        wait_for(lambda: station.state()["hands"][side]["buttons"][index]["pressed"] == pressed, "button delivery")
    button(4,True,"right")
    button(4,False,"right")
    wait_for(lambda: station.recorder.status()["active"], "file camera recording")
    for i, name in enumerate(("overview","left_wrist","right_wrist")):
        if i:
            button(4,True,"left")
            button(4,False,"left")
        wait_for(lambda: station.recorder.status()["selected_camera"] == name and
                 client.evaluate("document.getElementById('camera-name').textContent") ==
                 station.recorder.status()["camera_label"] and
                 client.evaluate("document.getElementById('camera-age').textContent.startsWith('Live')"),
                 "desktop displays a fresh frame from "+name)
        label = station.recorder.status()["camera_label"]
        wait_for(lambda: client.evaluate("questFixture.hudLines.some(s => s.startsWith("+
                         json.dumps(label+" · Live")+"))"), "Quest displays "+name)
        client.evaluate("questFixture.panelPixels={}; questFixture.samplePanel=true")
        wait_for(lambda: client.evaluate("Object.keys(questFixture.panelPixels).length === 2"), "stereo frame")
        for rgba in client.evaluate("Object.values(questFixture.panelPixels)"):
            assert rgba[i] >= 200 and all(rgba[j]<45 for j in range(3) if i!=j), (name,rgba)
        assert not any(client.evaluate("Object.values(questFixture.meshVertices)"))
    button(1,True,"right")
    client.evaluate("questFixture.axis(3,1)")
    wait_for(lambda: hand()["closure"] > .18, "Quest stick drives the jaw")
    client.evaluate("questFixture.axis(3,0); questFixture.button(0,1)")
    wait_for(lambda: hand()["axes"][3] == 0 and hand()["buttons"][0]["pressed"], "Quest centered stick")
    held = hand()["closure"]
    client.evaluate("questFixture.hand.position.x -= .02")
    wait_for(lambda: hand()["clutch_delta"] and hand()["clutch_delta"]["p"][0] < -.018, "Quest arm moves")
    assert hand()["closure"] == held
    button(1,False,"right")
    button(4,True,"right")
    button(4,False,"right")
    wait_for(lambda: not station.recorder.status()["active"], "file camera trial saved")
    name = station.recorder.status()["episode"]
    wait_for(lambda: client.evaluate("questFixture.hudLines.includes('Trial saved')"), "Quest save receipt")
    client.evaluate("""window.workingFetch=window.fetch;
        window.fetch=(path,...args)=>path==='/camera' ? Promise.reject(Error('fixture drop')) : workingFetch(path,...args)""")
    wait_for(lambda: not station.state()["view_valid"], "client reports missing display frames")
    assert not station.recorder.status()["camera_stalled"]
    button(1, True, "right")
    wait_for(lambda: hand()["clutched"], "grip engages during display loss")
    previous = hand()["tool"]["p"]
    client.evaluate("questFixture.hand.position.x += .01")
    wait_for(lambda: math.dist(hand()["tool"]["p"], previous) > .008, "simulated arm moves during display loss")
    button(1, False, "right")
    client.evaluate("window.fetch=workingFetch")
    wait_for(lambda: client.evaluate("document.getElementById('camera-age').textContent.startsWith('Live')"),
             "camera display recovered")
    assert not station.state()["held"]
    assert not client.errors, client.errors
    print("Mouse and Quest stick centering retained jaw opening during arm motion. "
          "Both eyes showed the selected red, green and blue feeds; grip control continued during display loss.")
    return name


def exercise_robot_ui(client, address, station):
    """Use a mock encoder snapshot and real camera-file frames without a motor adapter."""
    joints = list(range(1,8))
    arm = {"phase":"waiting", "enabled":False, "joints":joints, "measured_deg":[12.5]*7}
    station.physical_arm = {"mode":"shadow", "arms":{"right":arm}, "error":None}
    fixture = Path(__file__).parent / "tests" / "quest-input-fixture.js"
    client.call("Page.addScriptToEvaluateOnNewDocument", source=fixture.read_text(encoding="utf-8"))
    client.call("Page.navigate", url=address)
    wait_for(lambda: client.evaluate("document.getElementById('hardware-status')?.textContent.includes('Robot read-only')"),
             "read-only hardware indicated")
    client.click("xr-start")
    wait_for(lambda: station.state()["hands"]["right"]["tracked"], "mock hardware Quest session")
    assert client.evaluate("document.getElementById('mouse-start').disabled && document.getElementById('restore_initial').hidden")
    wait_for(lambda: client.evaluate("document.getElementById('encoder-joints').textContent.includes('J7 12.5°')"),
             "seven physical encoder readings")
    wait_for(lambda: client.evaluate("questFixture.hudLines.some(s => s.includes('Live')) && "
                                     "Object.values(questFixture.robotPixels).every(n=>n>30)"),
             "camera HUD and both stereo robot meshes")
    client.evaluate("questFixture.panelPoint=[81,65];questFixture.samplePanel=true")
    wait_for(lambda: client.evaluate("Object.keys(questFixture.panelPixels).length===2"), "stereo camera pixels")
    for pixel in client.evaluate("Object.values(questFixture.panelPixels)"):
        assert pixel[0] > 200 and max(pixel[1:3]) < 45, pixel

    def chord(down):
        value = int(down)
        client.evaluate(f"questFixture.button(4,{value},'left');questFixture.button(4,{value},'right')")
        wait_for(lambda: all(hand["buttons"][4]["pressed"] == down for hand in station.state()["hands"].values()),
                 "A and X delivered")

    ack = station.state()["ack"]
    camera = station.recorder.status()["selected_camera"]
    chord(True)
    chord(False)
    assert station.hardware_return is None and station.state()["ack"] == ack
    assert not station.recorder.status()["active"] and station.recorder.status()["selected_camera"] == camera
    with station.lock:
        station.physical_arm = {"mode":"drive", "arms":{"right":dict(arm, phase="holding", enabled=True)}, "error":None}
    wait_for(lambda: client.evaluate("document.getElementById('hardware-status').textContent.includes('motors enabled')"),
             "robot drive and enable state shown")
    if station.state()["held"]:
        client.click("resume")
    client.click("restore_initial")
    wait_for(lambda: station.hardware_return, "explicit physical workstation button")
    previous = station.hardware_return["id"]
    client.evaluate("questFixture.button(1,1)")
    wait_for(lambda: station.hardware_return is None, "side grip cancels workstation return")
    client.evaluate("questFixture.button(1,0)")
    wait_for(lambda: not station.state()["hands"]["right"]["buttons"][1]["pressed"], "side grip released")
    chord(True)
    wait_for(lambda: station.hardware_return and station.hardware_return["id"] > previous,
             "fresh physical controller return request")
    chord(False)
    assert station.state()["ack"] == ack+1 and station.recorder.status()["selected_camera"] == camera
    assert not station.recorder.status()["active"]
    client.evaluate("questFixture.button(5,1)")
    wait_for(lambda: station.state()["held"] and station.hardware_return is None, "physical right B stops return")
    assert not client.errors, client.errors
    print("Read-only and drive modes, physical return/stop, encoder display and stereo model with camera passed.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--browser", help="Chrome/Chromium/Edge executable")
    parser.add_argument("--episodes", type=Path, help="Keep the browser-generated episode in this directory")
    scenario = parser.add_mutually_exclusive_group()
    scenario.add_argument("--stick-cameras", action="store_true",
                          help="Test stick-mode and three OpenCV file sources through the Quest camera panel")
    scenario.add_argument("--robot-ui", action="store_true",
                          help="Test robot-mode Quest controls with fake encoders and camera files")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="openarm-ui-") as temp:
        tmp = Path(temp)
        episodes = args.episodes or tmp / "episodes"
        cfg = file_cameras(tmp) if args.stick_cameras or args.robot_ui else None
        station = Station(episodes, gripper_mode="stick" if cfg else "trigger", camera_config=cfg)
        try:
            with make_server("127.0.0.1", 0, station) as httpd:
                thread = Thread(target=httpd.serve_forever, daemon=True)
                thread.start()
                try:
                    with browser(args.browser or locate_browser(), tmp / "profile") as client:
                        address = f"http://127.0.0.1:{httpd.server_port}/"
                        try:
                            if args.robot_ui:
                                exercise_robot_ui(client, address, station)
                                return
                            name = (exercise_camera_workflow if cfg else exercise)(client, address, station)
                            check_episode(episodes / name)
                            if not cfg:
                                exercise_quest(client, address, station)
                        except AssertionError:
                            state = station.state()
                            h = state["hands"]["right"]
                            print("Input at failure:", state["frame"], state["seq"], state["reason"], h["status"],
                                  h["buttons"][:2], state["camera"]["id"], "browser:", client.errors)
                            raise
                finally:
                    httpd.shutdown()
                    thread.join()
        finally:
            station.close()
        print("Browser mouse and mock Quest control of the articulated robot passed.")
        if args.episodes:
            print((episodes / name).resolve())


if __name__ == "__main__":
    main()
