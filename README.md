# OpenArm teleoperation and data collection with Meta Quest 2

Use Quest Touch controllers to move a simulated OpenArm, see one of three robot-mounted cameras in the headset, and record a trial on the workstation. The recorder can use live video from your cameras. Joint positions, gripper openings and return-to-start motion come from the simulator. The supplied Mac console can drive physical motors independently. Connecting Quest motion to those motors still requires a calibrated hardware bridge.

## Run at the lab

On the computer you will use with the Quest, use a GitHub account with access to this private repository. Open a terminal and run these commands once.

```sh
git clone https://github.com/xzhu399-dotcom/openarm-quest-teleop.git
cd openarm-quest-teleop
```

Run the receiver from the directory containing this README. Put the Quest 2 and workstation on the same LAN and allow incoming connections to the receiver (TCP 8443 by default). The headset runs the existing Meta Quest Browser.

### Just a Quest 2 and a computer

Run the receiver with `--pose-report` for a text readout of the headset, both controllers, grippers and recording events. It works without an arm, cameras or OpenCV. Use Python 3.9+ and OpenSSL for the default HTTPS pairing certificate.

**macOS** (use the active network interface if it differs from `en0`)

```sh
python3 quest-pose/quest_pose_receiver.py --host "$(ipconfig getifaddr en0)" --pose-report --gripper-mode stick
```

**Windows PowerShell**

```powershell
ipconfig
$labIP = Read-Host 'Enter the IPv4 address of the adapter on the Quest network'
python quest-pose/quest_pose_receiver.py --host $labIP --pose-report --gripper-mode stick
```

1. Open the **complete printed HTTPS URL** in Meta Quest Browser, including `?key=...`, and select **Start Quest**. The terminal displays the latest input. You can also open the same URL in the computer's browser to observe or select **Capture initial pose**.
2. Move one handle with its side grip released. Its position (XYZ, metres), orientation (quaternion XYZW) and change from its initial pose update. Hold the side grip and turn the same handle to change the **simulated tool target**. While gripping, tilt the thumbstick down to close or up to open. Center it to hold the displayed jaw opening in millimetres. That number comes from the simulated gripper.
3. Tap **right A** to start recording, then tap **right A** again to stop and save. Check the start and stop messages, episode ID and final status in the terminal. Saved trials are under `episodes/<episode-id>/` and contain the Quest input, simulated robot data and synthetic camera views.

For a rehearsal without the headset, run `python quest-pose/quest_pose_receiver.py --sim --pose-report --gripper-mode stick` and select **Simulate Quest with mouse** at the printed desktop URL. On macOS use `python3`. For an image inside the terminal, choose the `--terminal-view` workflow below.

### With workstation cameras

**1. Prepare the workstation.** The receiver needs Python 3.9 or later. Install OpenCV for live cameras and the terminal picture. Create the local camera configuration once.

**macOS**

```sh
python3 -m venv .venv-teleop
.venv-teleop/bin/python -m pip install opencv-python
test -e cameras.lab.json || cp quest-pose/cameras.example.json cameras.lab.json
```

**Windows PowerShell**

```powershell
python -m venv .venv-teleop
.\.venv-teleop\Scripts\python.exe -m pip install opencv-python
if (!(Test-Path cameras.lab.json)) { Copy-Item quest-pose/cameras.example.json cameras.lab.json }
```

Edit `cameras.lab.json`. Give `overview`, `left_wrist` and `right_wrist` the device indices for the three cameras actually attached to the workstation. On macOS, grant Camera access to the app running Python. The [camera template](quest-pose/cameras.example.json) uses example indices.

**2. Start the receiver in a terminal.** On the Mac, use the interface with the LAN address reachable from the Quest. Substitute the interface name if it differs from `en0`.

```sh
.venv-teleop/bin/python quest-pose/quest_pose_receiver.py --host "$(ipconfig getifaddr en0)" --cameras cameras.lab.json --gripper-mode stick --terminal-view
```

On Windows, find the IPv4 address of the active adapter with `ipconfig`, then enter it when prompted.

```powershell
$labIP = Read-Host 'Workstation LAN IPv4 address'
.\.venv-teleop\Scripts\python.exe quest-pose/quest_pose_receiver.py --host $labIP --cameras cameras.lab.json --gripper-mode stick --terminal-view
```

The default launch uses OpenSSL to generate a short-lived HTTPS certificate. If the headset requires a trusted certificate, supply `--certfile lab-cert.pem --keyfile lab-key.pem` using a certificate trusted by the headset and valid for that LAN IP. To rehearse before connecting cameras, omit `--cameras cameras.lab.json` to use synthetic views.

**3. Pair and verify.** In Meta Quest Browser, open the **full HTTPS URL printed in the terminal**, including the port and `?key=...`. Select **Start Quest**. Open the same URL on the workstation to see the inspector. With both grips released, move each handle and check that its reported position changes. Press **left X** three times to verify the camera labels and confirm that the headset and terminal show the same feed.

**4. Record one trial.** With grips released, select **Capture initial pose** on the workstation. Tap **right A** to start and check for **REC**. Hold the side grip to move or rotate that arm. With the side grip held, tilt the thumbstick up to open or down to close and let it center to preserve the opening. Release both side grips to finish. Tap **right A** to save and stay, or hold **left thumbstick click** and tap **right A** to save and return the simulated arms to their trial-start positions. All three videos are recorded even while only one is displayed. Check the save status before starting the next trial. From the same Python environment, run `python quest-pose/replay.py episodes/EPISODE_ID` with the directory name shown by the station. Substitute the virtual environment’s Python path if necessary. [Full collection guide and button reference](QUEST2_DATA_COLLECTION.md)

## Which code runs where?

| Role | Folder and entry points | Responsibility |
| --- | --- | --- |
| Physical robot control on the Mac | [OpenArm hardware CLI](openarm-cli-smooth-accel-20261015-155332/), [openarm_cli.py](openarm-cli-smooth-accel-20261015-155332/openarm_cli.py), [openarm_motor_test.py](openarm-cli-smooth-accel-20261015-155332/openarm_motor_test.py) | The console controls J1–J7 and the J8 gripper through the arm’s USB CANable adapter. |
| Meta Quest input and display | [quest-pose/xr.js](quest-pose/xr.js), [app.js](quest-pose/app.js), [index.html](quest-pose/index.html) | Browser code reads headset and controller poses, triggers, face buttons and thumbsticks, sends the samples, and draws the selected camera in both eyes. |
| Quest ↔ workstation communication | [quest-pose/quest_pose_receiver.py](quest-pose/quest_pose_receiver.py), [station.py](quest-pose/station.py) | The HTTPS receiver accepts `POST /sample` and `POST /control`. The workstation sends state and the selected camera back through `GET /state` and `GET /camera`. |
| Workstation → simulated robot | [quest-pose/teleop.py](quest-pose/teleop.py), [robot_sim.py](quest-pose/robot_sim.py), [trial.py](quest-pose/trial.py) | The station feeds controller displacement and rotation into the teleoperation engine, which computes joint positions, jaw opening and the optional trial return. |
| Cameras, recording and review | [quest-pose/live_cameras.py](quest-pose/live_cameras.py), [recording.py](quest-pose/recording.py), [replay.py](quest-pose/replay.py) | The workstation captures and saves all three streams in `episodes/<episode-id>/`. The [examples](examples/) are replayable mouse demonstrations. |

### Check the physical robot connection on the Mac

Use a separate terminal in the hardware CLI folder to inspect the connected CANable adapters. This check reads USB capabilities and works with motor power off. The Quest receiver can run independently.

```sh
cd openarm-cli-smooth-accel-20261015-155332
python3 -m venv .venv-can-diagnostics
.venv-can-diagnostics/bin/python -m pip install -r requirements.txt
.venv-can-diagnostics/bin/python canable_inspect.py
test -e openarm_cli_config.json || cp openarm_cli_config.template.json openarm_cli_config.json
```

On a fresh checkout, fill in the local `openarm_cli_config.json` with the verified left/right adapter serial numbers before using the console. The template has placeholder serial numbers and empty pose and limit maps. Use the hardware guide for serial identification, console preview, calibration prerequisites and supervised motor operation. The hardware CLI uses macOS terminal facilities. [Hardware setup](openarm-cli-smooth-accel-20261015-155332/README.txt) · [OpenArm console guide](openarm-cli-smooth-accel-20261015-155332/OPENARM_CLI_GUIDE.txt)

## Try the right controller with a mouse

From the directory containing this README, run:

```powershell
python quest-pose/quest_pose_receiver.py --sim
```

On macOS use `python3` in place of `python`. Open **http://127.0.0.1:8000/** in a desktop browser and click **Simulate Quest with mouse**. If that port is busy, add `--port 0` and open the address printed by the server. The first valid pose becomes the initial pose automatically. The left controller is untracked in mouse mode.

1. **Set a baseline.** Left-drag the controller to a comfortable position and release. Click **Capture initial pose**. This saves `episodes/initial_poses.json` even before you start a demonstration. In **Pose inspector**, *From initial* should read zero metres and the identity quaternion `[0, 0, 0, 1]`. Click **Start recording** to save subsequent samples.
2. **Translate.** In **Move**, hold the right mouse button and drag 50 pixels to the right. The virtual handle requests **+0.050 m in X**. Watch the arm reposition its links to reach it. Drag vertically for Y, or scroll while holding the right button for depth. Scroll up moves forward along -Z.
3. **Reposition.** Release the right button and left-drag the hand elsewhere. *From initial* includes this reposition. The tool holds its pose. The next right-drag starts from a fresh clutch anchor.
4. **Rotate.** Select **Rotate**, then right-drag to turn the virtual wrist in pitch and yaw. Scroll with the right button held for roll. The arm solves for the new orientation around the point between the fingers.
5. **Grasp.** In the default trigger mode, select **Gripper**, then right-drag right to squeeze or left to ease off. Half-squeeze gives a 44 mm gap in the 88 mm model. To keep this gap while relaxing the trigger, release the side grip first. For thumbstick control launch with `--gripper-mode stick`. Clutch and tilt up to open, down to close, or center to keep the gap while moving the arm.
6. **Save.** Try **Switch view** or **Add marker**, then choose to save and stay or save and return. The return restores trial-start simulated arm joints and keeps the final jaw openings. Episodes are under `episodes/<episode-id>/`. Use `--episodes PATH` to change the output directory. **Hold** freezes both arms. Resume locally with fresh input, release the grip, then squeeze again.

The workspace shows both arms, the selected handle and a gripper detail. The inspector reports metres and XYZW quaternions. Both use +X right, +Y up and -Z forward. The raw-data panel shows the headset, both controllers, buttons, axes, sequence number and timing. Mouse mode keeps the left arm stationary. On the headset each hand controls its corresponding arm.

The simulator uses the published OpenArm v1.0 joint origins, axes, limits and finger travel. Damped inverse kinematics solves for J1-J7. Forward kinematics places the links, palm and fingertips. When a pose is unreachable, the arm holds its last accepted configuration. Release, reposition and re-clutch to continue. The link graphics are schematic. This kinematic model omits contact, gravity, collision response and motor dynamics. The lab rig's revision and calibration still need verification. [Pinned OpenArm model](https://github.com/enactic/openarm_description/blob/624e3ddadba10304f1fb3bb5df1f8c65977e3b25/assets/robot/openarm_v1.0/urdf/example/v1.urdf)

## How the two references work

The **initial pose** is the first valid pose of each handle in the stationary tracking space, or its most recent explicit capture while released. It persists across ordinary clutch releases. Each successful explicit capture replaces `initial_poses.json` under the configured episode root. The file is a record of that session's coordinates. A new session or tracking-space reset establishes a new reporting origin. Each accepted sample during recording contains this reference as well as the current pose and difference.

The **clutch anchor** is a second pair: the hand pose and current tool pose when that side's grip engages. In the default simulation, tracking axes and base axes align and translation scale is one:

```text
reported displacement = current_hand_position − initial_hand_position
reported rotation     = current_hand_orientation × inverse(initial_hand_orientation)

tool_position    = tool_position_at_grip + (current_hand_position − hand_position_at_grip)
tool_orientation = (current_hand_orientation × inverse(hand_orientation_at_grip)) × tool_orientation_at_grip
```

Quaternion multiplication order matters. The default tool reference is a synthetic point between the jaws. Rotating a hand in place changes orientation around this point. Moving the head has no term in either tool calculation. The first sample on a clutch edge preserves the tool pose and closure. Grips engage at 0.65 and release at 0.35. Each side has its own grip, closure and anchors.

A missing hand freezes that side. Loss of both hands, the headset, the input session or visibility latches a hold on both sides. So do a tracking-space reset and a gap of more than 0.4 seconds in delivered input. The inspector's arrival age is time since the server received a sample. The reported source lag is additional delay relative to the best observed browser/host clock offset. Neither value measures physical motion-to-display latency. The solver rejects targets outside its joint limits or reach.

## Quest Touch controls

Only one input session controls the simulation at once. Use **Disconnect input** before switching between the mouse and headset. The workstation browser can observe a Quest session and resume after a hold.

The page reads viewer pose and each controller's grip pose in one stationary `local-floor` or `local` space. Inspect the actual profile and raw buttons on the borrowed controllers. The named mapping below uses the Oculus Touch profile. [WebXR Device API](https://www.w3.org/TR/webxr/) · [Oculus Touch v3 input profile](https://raw.githubusercontent.com/immersive-web/webxr-input-profiles/main/packages/registry/profiles/oculus/oculus-touch-v3.json)

| Quest input | Simulator action |
| --- | --- |
| Left or right side grip | Clutch that side's tool |
| Index trigger while clutched, trigger mode | Squeeze to close continuously and ease off to open |
| Vertical thumbstick while clutched, stick mode | Up opens, down closes, center holds the opening |
| Rotate the clutched controller | Pitch, yaw and roll the gripper around its tool center |
| Right A | Start an episode, or save and stay at its endpoint |
| Hold left thumbstick click + tap right A during recording | With side grips released, save and return the simulated arm joints to their trial-start positions |
| Right B | Latch hold for both tools |
| Left X | Cycle overview, left wrist and right wrist |
| Left Y | Insert an episode marker |
| Headset | Look around without moving a tool |

Synthetic mode shows the robot in stereo and one selected synthetic camera panel. With `--cameras cameras.lab.json`, OpenCV acquires and records three physical cameras or local video files. The headset shows only the selected camera on its panel. Add `--terminal-view` to see the same feed and input telemetry inside the workstation terminal. Preview uses same-origin HTTPS image requests. End-to-end image latency still needs measurement on the lab rig.

The original terminal XYZ diagnostic remains available with `--legacy` on the same HTTPS command. Open its full pairing URL. This page uses the headset-relative viewer frame. Use the main simulator page for stationary-frame measurements and teleoperation.

## Inspect a demonstration

The [articulated robot demonstration](examples/mouse-robot/manifest.json) uses the mouse as a right Quest handle. Replay also supports the earlier [tool-target demonstration](examples/mouse-right/manifest.json). From the project directory run the commands below.

```sh
python quest-pose/replay.py examples/mouse-robot
python quest-pose/replay.py examples/mouse-robot --at 1 --frames replay-frames
python quest-pose/replay.py examples/mouse-robot --play
```

The first command verifies and summarizes the saved streams. The second reports the poses and writes the last available frame from each camera at or before the requested time. The third prints the recorded tool trajectory at its recorded timing. For your own completed episodes, `--outcome success`, `failure` or `aborted` records the reviewed outcome in the manifest.

| Episode file | Contents |
| --- | --- |
| `manifest.json` | Simulation and calibration labels, units, clock information, outcome and dropped-record counts |
| `input.jsonl` | Raw samples, browser monotonic time and receiver arrival timestamps |
| `actions.jsonl` | Per-hand initial and clutch references, pose differences, simulated tool targets, closure, control state and IK outputs |
| `feedback.jsonl` | Simulated joint angles in radians, joint origins, achieved TCP, jaw opening and fingertips in metres. Physical measurements are null |
| `events.jsonl` | Record boundaries, markers, hold transitions, explicit captures and camera switches |
| `overview.png`, `left_wrist.png`, `right_wrist.png` | Independently readable animated PNG camera streams |
| `overview.mjpg`, `left_wrist.mjpg`, `right_wrist.mjpg` | Full-color device or file streams in configured OpenCV camera mode |
| `*.frames.jsonl` | Per-camera frame indices, capture times and byte offsets |

Align streams by host monotonic timestamps. The frame index is local to each video. If the receiver stops during a recording, the episode is marked interrupted. An abrupt process or disk failure can leave a partial stream. Replay checks the available files and reports inconsistencies.

## Implementation and tests

| File | Main functions |
| --- | --- |
| [app.js](quest-pose/app.js), [scene.js](quest-pose/scene.js) | `mouseInput`, `transmit`, `render` and `drawScene` generate the right-hand input, send the latest sample, and draw the handle, articulated arms and gripper. [index.html](quest-pose/index.html) provides the controls. |
| [xr.js](quest-pose/xr.js), [xr_robot.js](quest-pose/xr_robot.js) | `startQuest`, `readHands` and `createPanel` read poses and buttons and draw the selected camera in both eyes. Synthetic mode also draws the articulated arms. |
| [robot_sim.py](quest-pose/robot_sim.py) | `RobotArm.solve` applies bounded inverse kinematics. `forward` and `snapshot` produce seven joint origins, the achieved tool pose, finger positions and jaw opening. |
| [teleop.py](quest-pose/teleop.py) | `Teleop.apply`, `Teleop.capture`, `Hand.update` and `difference` manage references, calculate translation and orientation differences, and update each tool and jaw. |
| [trial.py](quest-pose/trial.py) | `Trial.begin` captures starting joints. `Trial.finish` and `Trial.advance` hold the endpoint or return the simulated joints after saving. |
| [live_cameras.py](quest-pose/live_cameras.py), [terminal_view.py](quest-pose/terminal_view.py) | `CameraBank` acquires the three configured sources. `TerminalMonitor` renders the selected camera, input and trial status in the workstation terminal. |
| [quest_pose_receiver.py](quest-pose/quest_pose_receiver.py), [station.py](quest-pose/station.py) | `make_server`, `Station.sample` and `Station.control` serve the UI, enforce one input owner, dispatch events and coordinate recording. |
| [recording.py](quest-pose/recording.py), [synthetic_cameras.py](quest-pose/synthetic_cameras.py), [replay.py](quest-pose/replay.py) | `Recorder.start/sample/finish`, `render` and `inspect_episode` write, generate and verify the episode data. |

Run the new offline regression suite from the project directory:

```sh
python -m unittest discover -s quest-pose -v
```

The optional [browser gesture test](quest-pose/browser_smoke.py) drives a real Chromium browser through translation, rotation, capture, re-clutch, trigger pickup and trial endings. With Chrome or Edge and the Python package `websocket-client` installed, run `python quest-pose/browser_smoke.py`. With OpenCV installed, run `python quest-pose/browser_smoke.py --stick-cameras` to also rehearse the joystick and verify the selected camera pixels in both eye views with recorded video inputs. By default the test writes to a temporary directory.

To repeat the separate [mock WebXR test](quest-pose/tests/webxr-smoke.html), run `python -m http.server 8765 --bind 127.0.0.1 --directory quest-pose` and open `http://127.0.0.1:8765/tests/webxr-smoke.html` in desktop Chrome. It exercises the production WebXR input code with generated poses, tracking loss, reference-space reset and both eye viewports in a real WebGL renderer.

The supplied OpenArm CLI is in [its original directory](openarm-cli-smooth-accel-20261015-155332/README.txt). All 66 of its offline tests passed: 62 POSIX-compatible tests on Ubuntu and four mocked CANable-query tests on Windows. To run them together on the lab Mac, use the CLI archive's virtual environment, dependencies and offline test command in its README.

The simulator sends no USB/CAN commands. Physical teleoperation requires confirming the arm revision, base and tool-center transforms, joint order, signs, zero offsets, mechanical limits, and measured J8 endpoints on each side. The archive's joint-limit and saved-pose maps are empty. Actual tracking, button order, headset playback, wireless timing and physical calibration need a lab check.
