# OpenArm teleoperation and data collection with Meta Quest 2

Use Quest Touch controllers to move an OpenArm and see the commanded robot model in the headset. The default mode is simulation. For hardware, run the receiver on the Windows, Linux or macOS computer cabled to the robot. Choose `robot-readonly` to preview movement and read the encoders, or `robot-drive` for locally armed J1–J7 motion and optional J8 control. Trials can record the robot state and one to three camera views. [Connect, record and return the physical robot](QUEST2_DATA_COLLECTION.md#connect-the-quest-to-a-physical-openarm)

**Taking this to a new lab computer?** Start with the [lab handoff](LAB_HANDOFF.md) for the fresh-clone procedure, real-Quest simulation check and physical gripper bench test.

On the Windows lab PC, use the [bm2roar reconnection steps](LAB_HANDOFF.md#reconnect-on-the-windows-lab-pc) to launch the simulator, reuse its local six-digit code, and open it from the headset by typing the PC's IP address.

## Run at the lab

On the computer you will use with the Quest, use a GitHub account with access to this private repository. Open a terminal and run these commands once.

```sh
git clone https://github.com/xzhu399-dotcom/openarm-quest-teleop.git
cd openarm-quest-teleop
```

Run the receiver from the directory containing this README. Put the Quest 2 and workstation on the same LAN and allow incoming connections to the receiver (TCP 8443 by default). The headset runs the existing Meta Quest Browser.

### Connect a real arm on Windows or a separate robot computer

1. Connect the arm's USB CANable adapter and camera(s) to the computer that will run the receiver. Run USB discovery there, match the serial to the physical side, and load that rig's measured calibration. On a separate Linux robot computer, use the Linux commands in the collection guide. [Robot computer setup](QUEST2_DATA_COLLECTION.md#prepare-the-robot-computer)
2. On Windows, start **read-only** preview from the checkout after setup. The command uses the local `quest-pose/robot-hardware.lab.json` and the real camera sources in `cameras.lab.json`.

   ```powershell
   powershell.exe -NoProfile -ExecutionPolicy RemoteSigned -File .\Start-QuestSimulator.ps1 -NetworkName bm2roar -Mode robot-readonly -Cameras .\cameras.lab.json
   ```

3. Open the **complete HTTPS link printed by this hardware session** in Meta Quest Browser and select **Start Quest**. Verify the encoder readings, movement directions and camera labels. To switch to drive, stop preview with Ctrl+C, relaunch with `-Mode robot-drive`, and enter the arming phrase at the robot computer. The Windows hardware launcher uses TCP 8444 and prints a fresh link on every start. The simulator uses its own address and PIN. [Controller buttons, physical return and saved data](QUEST2_DATA_COLLECTION.md#pair-preview-drive-and-record)

### Use a different Wi-Fi network

1. If you're collecting, save the trial and wait for **Trial saved**. Stop the running receiver with **Ctrl+C** in its terminal. Connect the PC and the headset to the new Wi-Fi. It must allow the two devices to reach each other.
2. On Windows, open PowerShell in the repository folder and list the active network profiles. Find the Wi-Fi adapter and replace `New Lab WiFi` with the **Name** shown for that adapter.

   ```powershell
   Get-NetConnectionProfile | Select-Object Name, InterfaceAlias
   powershell.exe -NoProfile -ExecutionPolicy RemoteSigned -File .\Start-QuestSimulator.ps1 -NetworkName "New Lab WiFi"
   ```

   If the launcher cannot select an address, run `ipconfig`. Find the PC's **IPv4 Address** under its active Wi-Fi adapter and substitute it for the example below.

   ```powershell
   powershell.exe -NoProfile -ExecutionPolicy RemoteSigned -File .\Start-QuestSimulator.ps1 -LabIP 192.168.1.42
   ```

3. Open the **new HTTPS address printed by the launcher** in Quest Browser. Enter the printed PIN if prompted, select **Open simulator**, then **Start Quest**. The same Windows user on the same PC keeps the same PIN across networks. A new workstation or Windows account generates its own PIN on first launch.

If the address opens on the PC but times out on the headset, check whether the Wi-Fi isolates devices. On a managed network, ask the administrator whether the headset can reach the PC on TCP 443 (pairing) and 8443 (simulator). The optional HTTP-to-HTTPS redirect uses TCP 80. On macOS, rerun the receiver command below with the active network interface and open its newly printed full HTTPS URL.

For hardware on another Wi-Fi, run the hardware launch command on the robot computer using its new `-NetworkName` or `-LabIP` (Python CLI: `--host`). Open the new full `https://…:8444/?key=…` link in Quest Browser. The headset needs direct access to TCP 8444 on that computer.

### Share this private repository with a labmate

1. As the owner, open the repository on GitHub and select **Settings → Collaborators → Add people**. Find the labmate's GitHub username and send the invitation. [Repository access settings](https://github.com/xzhu399-dotcom/openarm-quest-teleop/settings/access) · [GitHub invitation guide](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/repository-access-and-collaboration/inviting-collaborators-to-a-personal-repository)
2. Have the labmate accept the invitation while signed in to that account. They can then use the clone command above on their computer and follow the lab handoff to start their own simulator. [Repository](https://github.com/xzhu399-dotcom/openarm-quest-teleop) · [Lab handoff](LAB_HANDOFF.md)

Collaborators on a private personal repository can read and push code. If the lab needs read-only GitHub access, an organization-owned private repository supports the **Read** role. [Personal repository permissions](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/repository-access-and-collaboration/permission-levels-for-a-personal-account-repository) · [Organization repository roles](https://docs.github.com/en/organizations/managing-user-access-to-your-organizations-repositories/managing-repository-roles/repository-roles-for-an-organization)

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

Edit `cameras.lab.json`. Keep the attached views from `overview`, `left_wrist` and `right_wrist` and set their actual device indices. For one webcam, keep a single entry such as `"overview": {"device": 0}`. On macOS, grant Camera access to the app running Python. [Camera template](quest-pose/cameras.example.json)

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

**3. Pair and verify.** In Meta Quest Browser, open the **full HTTPS URL printed in the terminal**, including the port and `?key=...`. Select **Start Quest**. Open the same URL on the workstation to see the inspector. With both grips released, move each handle and check that its reported position changes. Use **left X** to check each configured camera label and confirm that the headset and terminal show the same feed.

**4. Record one trial.** With grips released, select **Capture initial pose** on the workstation. Tap **right A** to start and check for **REC**. Hold the side grip to move or rotate that arm. With the side grip held, tilt the thumbstick up to open or down to close and let it center to preserve the opening. Release the side grips to leave both arms in place and tap **right A** to save. All configured cameras record throughout the trial. Check the save status before starting the next trial. From the same Python environment, run `python quest-pose/replay.py episodes/EPISODE_ID` with the directory name shown by the station. Substitute the virtual environment’s Python path if necessary. [Full collection guide and button reference](QUEST2_DATA_COLLECTION.md)

## Which code runs where?

| Role | Folder and entry points | Responsibility |
| --- | --- | --- |
| Physical robot control on the Mac | [OpenArm hardware CLI](openarm-cli-smooth-accel-20261015-155332/), [openarm_cli.py](openarm-cli-smooth-accel-20261015-155332/openarm_cli.py), [openarm_motor_test.py](openarm-cli-smooth-accel-20261015-155332/openarm_motor_test.py) | The console controls J1–J7 and the J8 gripper through the arm’s USB CANable adapter. |
| Quest → physical arm on Windows, Linux or macOS | [Full-arm bridge](quest-pose/robot_hardware.py), [calibration template](quest-pose/robot-hardware.example.json) | Connects the configured sides, reads encoder feedback, bounds motor commands, and returns engaged arms to the measured startup pose. |
| Meta Quest input and display | [quest-pose/xr.js](quest-pose/xr.js), [app.js](quest-pose/app.js), [index.html](quest-pose/index.html) | Browser code reads headset and controller poses, triggers, face buttons and thumbsticks, sends the samples, and draws the selected camera in both eyes. |
| Quest ↔ workstation communication | [quest-pose/quest_pose_receiver.py](quest-pose/quest_pose_receiver.py), [station.py](quest-pose/station.py) | The HTTPS receiver accepts `POST /sample` and `POST /control`. The workstation sends state and the selected camera back through `GET /state` and `GET /camera`. |
| Workstation → simulated robot | [quest-pose/teleop.py](quest-pose/teleop.py), [robot_sim.py](quest-pose/robot_sim.py), [trial.py](quest-pose/trial.py) | The station feeds controller displacement and rotation into the teleoperation engine, which computes joint positions, jaw opening and the optional trial return. |
| Quest → physical J8 bench on the Mac | [Receiver](quest-pose/quest_pose_receiver.py) and [bench calibration template](quest-pose/gripper-bench.example.json) | The selected controller supplies the closure request. Shadow mode reads the configured gripper encoder. Execute mode adds supervised commands for that gripper. |
| Cameras, recording and review | [Camera capture](quest-pose/live_cameras.py), [recorder](quest-pose/recording.py), [replay](quest-pose/replay.py), [trial metrics](quest-pose/trial_metrics.py) | The workstation saves trials in `episodes/<episode-id>/`. Replay verifies the streams and the metrics command summarizes timing, simulated tracking and any J8 bench readbacks. The [examples](examples/) are recorded mouse demonstrations. |

### Check the physical robot connection on the Mac

Use a separate terminal in the hardware CLI folder to inspect the connected CANable adapters. This check reads USB capabilities and works with motor power off. Close tools using the selected adapter before opening it in the physical gripper bench.

```sh
cd openarm-cli-smooth-accel-20261015-155332
python3 -m venv .venv-can-diagnostics
.venv-can-diagnostics/bin/python -m pip install -r requirements.txt
.venv-can-diagnostics/bin/python canable_inspect.py
test -e openarm_cli_config.json || cp openarm_cli_config.template.json openarm_cli_config.json
```

On a fresh checkout, fill in the local `openarm_cli_config.json` with the verified left/right adapter serial numbers before using the console. The template has placeholder serial numbers and empty pose and limit maps. Use the hardware guide for serial identification, console preview, calibration prerequisites and supervised motor operation. The hardware CLI uses macOS terminal facilities. [Hardware setup](openarm-cli-smooth-accel-20261015-155332/README.txt) · [OpenArm console guide](openarm-cli-smooth-accel-20261015-155332/OPENARM_CLI_GUIDE.txt)

For a physical Quest/J8 trial, complete a local copy of the [bench calibration template](quest-pose/gripper-bench.example.json) with values verified on the selected gripper. Start the receiver with `--host`, `--pose-report` and `--gripper-bench FILE` for read-only shadow mode. Check the selected side, predicted target and measured encoder angle. After the shadow test passes, the lab operator can restart with `--execute-gripper` and complete the local arming procedure. Keep J1–J7 stationary throughout this test. [Bench procedure and stop checks](LAB_HANDOFF.md)

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

The simulator uses the published OpenArm v1.0 joint origins, axes, limits and finger travel. Damped inverse kinematics solves for J1-J7. Forward kinematics places the links, palm and fingertips. When a pose is unreachable, the arm stays at its last accepted configuration. With Quest side-grip control, move the handle back within reach while still squeezing and the arm follows again. With the mouse, release, reposition and re-clutch. The link graphics are schematic. This kinematic model omits contact, gravity, collision response and motor dynamics. The lab rig's revision and calibration still need verification. [Pinned OpenArm model](https://github.com/enactic/openarm_description/blob/624e3ddadba10304f1fb3bb5df1f8c65977e3b25/assets/robot/openarm_v1.0/urdf/example/v1.urdf)

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

With real Quest input to the simulator, each tracked hand drives its arm while its side grip is pressed, whether recording or idle. If tracking or sample delivery is interrupted, the affected arm stays at its last accepted pose. When fresh tracking returns with the grip squeezed, the simulator anchors at the current hand and tool poses so the next movement starts there. The mouse simulator and physical gripper bench use the software hold. The inspector's arrival age is time since the server received a sample. The reported source lag is additional delay relative to the best observed browser/host clock offset. Neither value measures physical motion-to-display latency. The solver rejects targets outside its joint limits or reach.

## Quest Touch controls

Only one input session controls the simulation at once. Use **Disconnect input** before switching between the mouse and headset. The workstation browser can observe a Quest session and displays **Grip engaged** when the selected hand drives its arm.

The page reads viewer pose and each controller's grip pose in one stationary `local-floor` or `local` space. Inspect the actual profile and raw buttons on the borrowed controllers. The named mapping below uses the Oculus Touch profile. [WebXR Device API](https://www.w3.org/TR/webxr/) · [Oculus Touch v3 input profile](https://raw.githubusercontent.com/immersive-web/webxr-input-profiles/main/packages/registry/profiles/oculus/oculus-touch-v3.json)

| Quest input | Simulator action |
| --- | --- |
| Left or right side grip | Clutch that side's tool |
| Index trigger while clutched, trigger mode | Squeeze to close continuously and ease off to open |
| Vertical thumbstick while clutched, stick mode | Up opens, down closes, center holds the opening |
| Rotate the clutched controller | Pitch, yaw and roll the gripper around its tool center |
| Right A | Start or save an episode. Held side grips continue to control the arms. |
| Left X | Cycle overview, left wrist and right wrist |
| Right A + left X together, with both side grips released | Return both simulated arms to the pose they had when the simulator started. Squeeze either side grip to cancel. |
| Left Y | Insert an episode marker |
| Headset | Look around without moving a tool |

**To return the robot home**, release both side grips, press right **A** and left **X** so their presses overlap, then let go of both buttons. The arms move back smoothly and keep their current jaw openings. You can do this before, during, or after a trial. An active recording continues through the motion and the selected camera stays in place. From the workstation, **Return arms to home** does the same thing with a live input session. **Capture initial pose** sets the reference for measuring controller displacement. The robot's home pose is set when the simulator starts.

Synthetic mode shows the robot in stereo and one selected synthetic camera panel. With `--cameras cameras.lab.json`, OpenCV acquires and records the configured physical cameras or local video files. The headset shows only the selected camera on its panel. Add `--terminal-view` to see the same feed and input telemetry inside the workstation terminal. Preview uses same-origin HTTPS image requests. End-to-end image latency still needs measurement on the lab rig.

The original terminal XYZ diagnostic remains available with `--legacy` on the same HTTPS command. Open its full pairing URL. This page uses the headset-relative viewer frame. Use the main simulator page for stationary-frame measurements and teleoperation.

## Inspect a demonstration

The [articulated robot demonstration](examples/mouse-robot/manifest.json) uses the mouse as a right Quest handle. Replay also supports the earlier [tool-target demonstration](examples/mouse-right/manifest.json). From the project directory run the commands below.

```sh
python quest-pose/replay.py examples/mouse-robot
python quest-pose/replay.py examples/mouse-robot --at 1 --frames replay-frames
python quest-pose/replay.py examples/mouse-robot --play
python quest-pose/trial_metrics.py examples/mouse-robot
```

The first command verifies and summarizes the saved streams. The second reports the poses and writes the last available frame from each camera at or before the requested time. The third prints the recorded tool trajectory at its recorded timing. For your own completed episodes, `--outcome success`, `failure` or `aborted` records the reviewed outcome in the manifest.

The metrics command reports median and 95th-percentile intervals between recorded input samples, sequence gaps, per-hand simulated tool tracking errors and observed IK rejections. For bench recordings it also summarizes distinct J8 encoder readbacks in degrees.

| Episode file | Contents |
| --- | --- |
| `manifest.json` | Simulation and calibration labels, units, clock information, outcome and dropped-record counts |
| `robot_io.jsonl`, `observations.jsonl` | Hardware runs record per-arm motor transactions, joint feedback and source Quest sequence numbers, plus time-aligned references to each saved camera view. |
| `input.jsonl` | Raw samples, browser monotonic time and receiver arrival timestamps |
| `actions.jsonl` | Per-hand initial and clutch references, pose differences, simulated tool targets, closure, control state and IK outputs |
| `feedback.jsonl` | Simulated joint angles in radians, achieved tool pose and jaw opening. A gripper bench recording adds `physical_gripper`, including target and measured J8 angles in degrees and the host readback timestamp. |
| `events.jsonl` | Record boundaries, markers, hold transitions, explicit captures and camera switches |
| `overview.png`, `left_wrist.png`, `right_wrist.png` | Independently readable animated PNG camera streams |
| `overview.mjpg`, `left_wrist.mjpg`, `right_wrist.mjpg` | Full-color device or file streams in configured OpenCV camera mode |
| `*.frames.jsonl` | Per-camera frame indices, capture times and byte offsets |

Align streams by host monotonic timestamps. The frame index is local to each video. If the receiver stops during a recording, the episode is marked interrupted. An abrupt process or disk failure can leave a partial stream. Replay checks the available files and reports inconsistencies.

## Implementation and tests

| File | Main functions |
| --- | --- |
| [app.js](quest-pose/app.js), [scene.js](quest-pose/scene.js) | `mouseInput`, `transmit`, `render` and `drawScene` generate the right-hand input, send the latest sample, and draw the handle, articulated arms and gripper. [index.html](quest-pose/index.html) provides the controls. |
| [xr.js](quest-pose/xr.js), [xr_robot.js](quest-pose/xr_robot.js) | `startQuest`, `readHands` and `createPanel` read poses and buttons and draw the selected camera in both eyes. Hardware mode and synthetic camera mode also draw the articulated arms. |
| [robot_sim.py](quest-pose/robot_sim.py) | `RobotArm.solve` applies bounded inverse kinematics. `forward` and `snapshot` produce seven joint origins, the achieved tool pose, finger positions and jaw opening. |
| [teleop.py](quest-pose/teleop.py) | `Teleop.apply`, `Teleop.capture`, `Hand.update` and `difference` manage references, calculate translation and orientation differences, and update each tool and jaw. |
| [trial.py](quest-pose/trial.py) | `Trial.begin` captures starting joints. `Trial.finish` and `Trial.advance` hold the endpoint or return the simulated joints after saving. |
| [live_cameras.py](quest-pose/live_cameras.py), [terminal_view.py](quest-pose/terminal_view.py) | `CameraBank` acquires the configured sources. `TerminalMonitor` renders the selected camera, input and trial status in the workstation terminal. |
| [quest_pose_receiver.py](quest-pose/quest_pose_receiver.py), [station.py](quest-pose/station.py) | `make_server`, `Station.sample` and `Station.control` serve the UI, enforce one input owner, dispatch events and coordinate recording. |
| [recording.py](quest-pose/recording.py), [synthetic_cameras.py](quest-pose/synthetic_cameras.py), [replay.py](quest-pose/replay.py) | `Recorder.start/sample/finish`, `render` and `inspect_episode` write, generate and verify the episode data. |

Run the new offline regression suite from the project directory:

```sh
python -m unittest discover -s quest-pose -v
```

The optional [browser gesture test](quest-pose/browser_smoke.py) drives a real Chromium browser through translation, rotation, capture, re-clutch, trigger pickup and trial endings. With Chrome or Edge and the Python package `websocket-client` installed, run `python quest-pose/browser_smoke.py`. With OpenCV installed, run `python quest-pose/browser_smoke.py --stick-cameras` to also rehearse the joystick and verify the selected camera pixels in both eye views with recorded video inputs. By default the test writes to a temporary directory.

To repeat the separate [mock WebXR test](quest-pose/tests/webxr-smoke.html), run `python -m http.server 8765 --bind 127.0.0.1 --directory quest-pose` and open `http://127.0.0.1:8765/tests/webxr-smoke.html` in desktop Chrome. It exercises the production WebXR input code with generated poses, tracking loss, reference-space reset and both eye viewports in a real WebGL renderer.

The supplied OpenArm CLI is in [its original directory](openarm-cli-smooth-accel-20261015-155332/README.txt). With the USB dependencies installed, run `python -m unittest discover -s openarm-cli-smooth-accel-20261015-155332 -p 'test_*offline.py' -q` from the checkout to exercise the motor session with mocked adapters.

Commission the physical bridge against the installed arm revision, base and tool-center transforms, joint order, encoder signs, offsets and measured limits. The templates leave rig-specific calibration values empty. [Hardware setup and first trial](QUEST2_DATA_COLLECTION.md#connect-the-quest-to-a-physical-openarm)
