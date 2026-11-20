# Collect a trial with Meta Quest 2 and OpenArm

One trial is one episode directory. The workstation receives the Quest poses and buttons, computes simulated OpenArm joint and gripper motion, acquires the three cameras, and writes the episode. The headset and terminal follow one shared camera selection. You can practice the entire procedure using a mouse and synthetic cameras.

**Choose the trial mode** on the computer running the receiver. In both robot modes the headset shows the commanded OpenArm model alongside the camera panel, and the workstation displays measured joint angles.

| Receiver option | What the robot does |
| --- | --- |
| `--mode simulate` (default) | Runs the kinematic simulator without opening a motor adapter. |
| `--mode robot-readonly` | Reads the configured arm encoders and previews commands. Motors stay disabled. |
| `--mode robot-drive` | After local arming, a fresh side-grip squeeze enables the corresponding calibrated arm. The Quest controls J1–J7 and, when configured, the J8 jaw. |

The separate `--gripper-bench` workflow is available for testing one J8 by itself. [J8 bench handoff](LAB_HANDOFF.md#3-real-quest-and-real-j8-read-only-first)

## Connect the Quest to a physical OpenArm

1. **Use the Mac connected to the robot.** Plug each participating arm's USB CANable adapter and the overview and wrist cameras into that Mac. Join the Quest and Mac to a network that lets the headset reach the Mac. The Quest sends controller data over Wi-Fi, the Mac sends motor commands over USB/CAN, and the Mac returns the model and selected camera view to the headset. A Windows PC can keep its separate simulation or open the Mac's pairing URL as an observer.
2. **Prepare the hardware on the Mac.** From the repo root, activate the project's Python environment. Install the hardware CLI requirements and OpenCV in that environment. Use the read-only USB inspection to match each physical side to its serial. Stop any other process using those adapters before starting the Quest receiver.

   ```sh
   python -m pip install -r openarm-cli-smooth-accel-20261015-155332/requirements.txt opencv-python
   python openarm-cli-smooth-accel-20261015-155332/canable_inspect.py
   ```

3. **Prepare the local configuration files.** The commands below copy templates only where a local file is missing. In the hardware CLI config, enter the lab's verified adapter serials and measured encoder limits in degrees for each connected joint. In the Quest calibration, configure `right`, `left`, or both. For J1 through J7, `encoder_zero_deg` gives the encoder reading at model zero, and `encoder_sign` gives the direction (+1 or −1). Set the lab-approved speed. Include `gripper` only for a calibrated J8. Check the model revision, mounting orientation, and tool frame against the actual robot. In the camera config assign the actual overview, left-wrist and right-wrist device IDs. [Arm calibration template](quest-pose/robot-hardware.example.json) · [Camera template](quest-pose/cameras.example.json)

   ```sh
   cp -n openarm-cli-smooth-accel-20261015-155332/openarm_cli_config.template.json openarm-cli-smooth-accel-20261015-155332/openarm_cli_config.json
   cp -n quest-pose/robot-hardware.example.json quest-pose/robot-hardware.lab.json
   cp -n quest-pose/cameras.example.json cameras.lab.json
   ```

   With the adapter serial configured, an arm with all eight motors installed can be checked independently using `python openarm-cli-smooth-accel-20261015-155332/openarm_cli.py --arm right status` (use `left` for the left arm). This reads the motor states before teleoperation.

4. **Start with encoder preview.** Support the robot in the starting pose you want to return to. The receiver reads its stationary encoders and initializes the model from those measurements. Use the Mac's Wi-Fi interface in place of `en0` if needed. In **Quest Browser**, open the **entire HTTPS URL printed on this Mac** and select **Start Quest**. Open the same URL on the Mac to inspect the hardware readings. Compare the model to the resting arm, move a controller with its grip held, and check that the proposed movement and signs agree with the lab calibration while the physical encoders remain stationary. Use **left X** to check each camera's identity.

   ```sh
   lab_ip="$(ipconfig getifaddr en0)"
   python quest-pose/quest_pose_receiver.py --host "$lab_ip" --mode robot-readonly --robot-calibration quest-pose/robot-hardware.lab.json --cameras cameras.lab.json --gripper-mode stick --pose-report
   ```

5. **Drive the calibrated arm under local supervision.** Save any trial and stop preview with Ctrl+C. Support the arm in the intended startup pose again, clear its workspace, and have the rig's physical stop within reach. Restart with the command below. Review the measured startup angles, enter the arming phrase in the Mac terminal, and open the newly printed URL in the headset. Keep the right controller tracked for **B**. Start with both grips released. Squeeze the configured side grip and move that controller a small distance, comparing the physical direction and encoder readings to the headset model. Releasing the grip holds the last motor reference. Right **B** requests a stop. A stopped or faulted drive session can require a fresh local launch.

   ```sh
   python quest-pose/quest_pose_receiver.py --host "$lab_ip" --mode robot-drive --robot-calibration quest-pose/robot-hardware.lab.json --cameras cameras.lab.json --gripper-mode stick --pose-report
   ```

6. **Collect and return.** Tap **right A** to start a trial. With the return path clear and the arm previously engaged, release both grips, then press **right A + left X together**. The Mac also has a **Return physical robot to startup pose** button. Each engaged arm returns J1–J7 to the encoder angles measured at the start of this drive session, preserving its jaw opening. Squeezing a grip cancels the return. To record the whole return, wait for the physical arm's **returned** status and check its encoders before tapping **A alone** to save. Wait for **Trial saved**, then replay the episode. Keep the arms mechanically supported for shutdown or a fault. The bridge bounds joint commands and checks encoder feedback. The operator must verify clearance along the path, including between the arms.

Physical timing uses the motor computer's monotonic clock. Each collected Quest sample has a session and sequence number. `robot_io.jsonl` identifies the source sample used in each motor cycle, the commands actually sent, and the measured encoder state and read times. `observations.jsonl` links each sample and motor cycle to the latest saved frame at or before its timestamp for **each** camera, with the frame's age or a missing value. Multiple commands can share a frame. Camera timestamps measure host delivery. The cameras are free-running and their exposure times can differ. Supply `--cameras` for lab images. Omitting it records synthetic views.

## Buttons on the Quest Touch controllers

The table describes the simulated arms. For the optional J8 bench, follow the physical clutch, arming and stop checks in the lab handoff.

| Operator action | Control | Result |
| --- | --- | --- |
| Take control of one arm | Hold that hand's **side grip** with your middle finger | Captures the current handle and tool poses. Subsequent handle displacement moves the arm. |
| Rotate the wrist | Keep the side grip held and turn the handle | Pitch, yaw and roll apply around the tool center between the fingers. Translation and rotation can be combined. |
| Reposition your hand | Release the side grip | That arm and its jaw stay where they are. The next squeeze anchors at the new hand position. |
| Set jaw opening in **trigger** mode | While clutched, squeeze or ease the **index trigger** | Continuous absolute closure. Fully squeezed closes, fully released opens. Each arm has its own trigger. |
| Adjust jaw opening in **stick** mode | While clutched, tilt that hand's **thumbstick up** to open or **down** to close | Tilt controls adjustment speed. Let the stick spring to center to retain the exact opening while moving or rotating the arm. |
| Start a trial | Tap **right A** while idle | Starts recording and captures both simulated arms' starting joint positions. A does not engage a clutch. |
| End the trial | Tap **right A** while recording | Saves the episode. Side-grip control continues. Release the grips to leave both arms at the endpoint. |
| Return the simulated arms home | Release both side grips. Press **right A + left X** together, then release both buttons. The workstation also has a **Return arms to home** button. | Both arms move to their simulator startup joint positions while keeping their jaw openings. A recording in progress continues. Squeeze either grip to cancel. |
| Switch the displayed camera | Tap **left X** | Cycles **Overview → Left wrist → Right wrist → Overview**. All three cameras continue recording. |
| Mark a moment | Tap **left Y** during recording | Writes a timestamped marker, for example contact or a slip. |

If tracking drops during simulated motion, the affected arm stays at its last pose. With its side grip still squeezed, fresh tracking anchors the handle there, and subsequent hand motion drives the arm again. Head movement changes the view. On the physical gripper bench, **right B** stops J8. The mapping is based on the Oculus Touch v3 WebXR profile. [Controller profile](https://raw.githubusercontent.com/immersive-web/webxr-input-profiles/main/packages/registry/profiles/oculus/oculus-touch-v3.json)

### How to maintain a grasp

- **Choose `--gripper-mode stick` for carrying an object with a relaxed thumb.** Start with the stick centered and hold the side grip. Tilt down until the opening reads 40 mm, then center the stick. Move or rotate the handle. The opening stays at 40 mm. Tilt up to open further. A small dead zone around center absorbs stick drift. The index trigger has no jaw action in this mode.
- **Choose `--gripper-mode trigger` for direct finger control.** This is the default. In the simulated 88 mm gripper, a 50% squeeze requests a 44 mm gap. Keeping the trigger steady maintains that gap while moving. Easing off while clutched opens the jaws. To relax the trigger without opening them, release the **side grip first**, then release the trigger. When you re-clutch, the jaw waits for the trigger to reach or cross the stored closure before following it again.

The displayed millimetres describe the nominal simulator geometry. A bench recording reports the selected J8 encoder angle in degrees. To measure the actual jaw gap or grasp force, use a separate physical measurement.

## 1. Prepare the workstation and cameras

**If you have only the Quest and a computer**, use the receiver's `--pose-report` mode. The Mac and Windows commands, HTTPS pairing steps and a short gripper test are in the [Quest-only quick start](README.md#just-a-quest-2-and-a-computer). You can skip the camera setup below. Move a handle with its side grip released to check its tracked XYZ and XYZW orientation, then clutch it to see the simulated tool target move. Tap **right A** twice, once to start and once to stop, and check the terminal for the episode ID and save result. This mode records synthetic camera views alongside your real Quest samples.

Run the commands from the folder containing this guide. For mouse practice with synthetic cameras and a desktop browser, the following command needs only Python 3.9 or later.

```sh
python3 quest-pose/quest_pose_receiver.py --sim --gripper-mode stick
```

Open the printed URL on the same computer and choose **Simulate Quest with mouse**. On Windows use `python` in place of `python3`. Stop the practice receiver with Ctrl+C when launching the lab configuration on the same port.

To rehearse one right-handed trial, click **Capture initial pose**, then tap the on-screen **A**. In **Move**, right-drag to move the virtual controller and scroll while holding the right button to change depth. In **Rotate**, right-drag for pitch and yaw and scroll for roll. In stick mode, focus the pad, hold the right button and use the up/down arrow keys to open/close. Release the arrow and continue dragging to see the opening hold. The two on-screen ending choices let you practice both outcomes with one mouse.

For physical cameras or the in-terminal picture, install OpenCV in the environment that will run the receiver. Use the following commands on the Mac.

```sh
python3 -m venv .venv-teleop
source .venv-teleop/bin/activate
python -m pip install opencv-python
test -e cameras.lab.json || cp quest-pose/cameras.example.json cameras.lab.json
```

Connect the three cameras to the workstation. Edit `cameras.lab.json` so `overview`, `left_wrist` and `right_wrist` refer to **three different** OpenCV device indices or supported absolute device paths. The example indices 0, 1 and 2 are placeholders. For a file-based rehearsal, replace `"device": 0` in an entry with `"file": "overview.mp4"`. File paths are relative to the config file. The example requests 640 × 480 at 15 fps. Check the actual views and delivered frames on your setup.

If macOS asks for camera access, allow the application running Python under **System Settings → Privacy & Security → Camera**. Close other applications that have exclusive access to the cameras. [Apple camera permissions](https://support.apple.com/guide/mac-help/control-access-to-the-camera-on-mac-mchlf6d108da/mac)

## 2. Connect the Quest 2 to the workstation

1. Connect the Quest 2 and the workstation to the same reachable LAN. The workstation can use Ethernet. On the Quest, open Settings or Quick Settings, select Wi-Fi and join the lab network. Guest-network client isolation can prevent the headset from reaching the workstation. [Meta Wi-Fi instructions](https://www.meta.com/help/quest/1816744325172615/)
2. On the Mac, run the command below. Replace `en0` with the active network interface when necessary. In `ipconfig getifaddr en0`, you should see the workstation's LAN IPv4 address. The receiver binds to that address and normally uses TCP port 8443.

   ```sh
   python quest-pose/quest_pose_receiver.py --host "$(ipconfig getifaddr en0)" --cameras cameras.lab.json --gripper-mode stick --terminal-view --episodes episodes
   ```

   To rehearse using the Quest before attaching cameras, omit `--cameras cameras.lab.json`. Omit `--terminal-view` to use only the browser previews. On Windows, run the following in PowerShell, pausing after the copy to edit the camera assignments.

   ```powershell
   python -m venv .venv-teleop
   .\.venv-teleop\Scripts\python.exe -m pip install opencv-python
   if (!(Test-Path cameras.lab.json)) { Copy-Item quest-pose/cameras.example.json cameras.lab.json }
   # Edit cameras.lab.json with your actual devices before continuing.
   ipconfig
   $labIP = Read-Host 'Enter the workstation LAN IPv4 address shown above'
   .\.venv-teleop\Scripts\python.exe quest-pose/quest_pose_receiver.py --host $labIP --cameras cameras.lab.json --gripper-mode stick --terminal-view --episodes episodes
   ```

3. Open **Meta Quest Browser** on the headset. Enter the **complete pairing URL printed by the receiver**, including `https://`, the port, and `?key=...`. Each launch generates a new pairing key. Use the computer's LAN address. `127.0.0.1` on the headset points back to the headset. This workflow uses the existing browser over Wi-Fi and does not require Quest Link, an APK or casting. [Meta WebXR overview](https://developers.meta.com/vr/documentation/web/webxr-overview/)
4. The default launch generates a short-lived local TLS certificate with OpenSSL. For the lab's known local receiver, proceed through its certificate prompt if the Quest browser permits it. If headset policy prevents this, use a lab-trusted certificate valid for the workstation LAN IP and add `--certfile lab-cert.pem --keyfile lab-key.pem` to the receiver command. OpenSSL is needed when the receiver generates the certificate. The key file stays on the computer. [Meta local WebXR setup](https://developers.meta.com/vr/documentation/web/webxr-first-steps/)
5. On the headset, select **Start Quest** and grant the XR session prompt if shown. Open the same pairing URL in the workstation browser as an observer. Leave **Simulate Quest with mouse** off while the headset owns input. Move each controller with the grips released. Confirm the corresponding numeric handle pose changes in the terminal or inspector while the simulated arms stay still. Then turn your head with the handles stationary and check the arms again.

If the page does not load, check the LAN IP, firewall access for the receiver on the lab network, port and full pairing URL. If the page loads but **Start Quest** stays disabled, verify the page is open over HTTPS in the headset's Browser. If another client owns input, disconnect it before starting the headset session.

## 3. Check the camera view

The workstation opens the three sources and serves one selected frame stream over the same authenticated HTTPS connection. The Quest draws that image on an immersive panel. In physical/file-camera mode the panel displays the chosen feed by itself, with camera and trial status. In synthetic mode the articulated robot also appears in stereo. The workstation browser and the `--terminal-view` picture follow the same selection. The terminal renders the actual image as colored character cells and reports the tracked handles. Enlarge the terminal for more detail.

Before recording, tap **left X** through all three views. Cover or wave at one lens at a time to verify its label, orientation, and wrist side. Check that the image updates on the Quest **and** in the workstation terminal. If a camera stalls, the controller can still drive the simulated arm. Check the episode's camera status and restore the feed before collecting another demonstration. Preview age is measured from workstation receipt. The headset display and USB cameras still need an on-device latency check for your task.

For a lab latency measurement, point the selected camera at a flashing LED. With all three cameras recording, use a separate high-speed camera to film both the LED and its image through a headset lens. Count the frames between each pair of flashes and divide by the high-speed camera's frame rate. Report the median and 95th percentile across repeated flashes for each view.

## 4. Collect and finish one trial

1. With the grips released, put the handles in a comfortable pose. **Capture initial pose** on the workstation records the handle reference in `episodes/initial_poses.json`. A new clutch squeeze captures a separate hand/tool reference for motion.
2. Set the **simulated arms** to the pose where you want the trial to start. Tap **right A** once. Check for **REC** and elapsed time in the headset. The trial-start snapshot records the current joints.
3. Clutch only the arm you intend to move. Move the handles, rotate your wrists, and adjust the jaws using the selected mode. Tap **left X** for another view or **left Y** for a marker.
4. At the endpoint, release both side grips and tap **right A** to save. Check the headset for **Trial saved** or **Incomplete trial**.
5. If an episode is incomplete, inspect its camera status and files before using it as a demonstration. Restore any missing feed before the next trial. Tap A to start the next trial after the previous action finishes. Use Ctrl+C in the receiver terminal when the session is over.

## 5. Verify and label the dataset

Run these commands from the project root using the episode directory name shown by the station.

```sh
python quest-pose/replay.py episodes/EPISODE_ID
python quest-pose/replay.py episodes/EPISODE_ID --at 1.0 --frames replay-frames
python quest-pose/trial_metrics.py episodes/EPISODE_ID
python quest-pose/replay.py episodes/EPISODE_ID --outcome success
```

The first command checks the manifest, structured streams, and all three videos and their frame tables. The second exports one PNG per camera at or before the selected time when a frame is available. The trial-ending button marks the end of collection. After reviewing a completed recording, label its outcome `success`, `failure` or `aborted`.

Use the [trial metrics command](quest-pose/trial_metrics.py) to review the observed controller profiles, median and 95th-percentile receive intervals in milliseconds, sequence gaps and recorder drops. Per-hand results include simulated tool tracking error on accepted clutched targets, observed IK rejections and jaw opening range. For bench data, inspect `recorded_j8_bench` for the number of distinct encoder readings and their range in degrees. Receiver intervals describe the cadence of the recorded samples. Optical latency requires the separate measurement above.

| Files in each episode | What to check |
| --- | --- |
| `manifest.json` | Save status, model, camera source types/IDs, frame counts, errors, drops, times and outcome |
| `input.jsonl`, `actions.jsonl`, `feedback.jsonl` | Raw Quest input, requested targets and simulated feedback. Bench feedback also carries a separate `physical_gripper` record with side, mode, target and measured J8 angles, and readback timestamp. |
| `events.jsonl` | Trial boundaries, starting configuration, end choice, markers and camera switches |
| `robot_io.jsonl`, `observations.jsonl` | Physical-arm bus cycles and the saved camera frame associated with each Quest input or bus cycle. See the physical-connection procedure above. |
| `overview.mjpg`, `left_wrist.mjpg`, `right_wrist.mjpg` | Three independent full-color Motion JPEG streams when using devices or files. Synthetic camera recordings use animated PNGs. |
| `*.frames.jsonl` | Stream-specific indices, host timestamps and frame offsets. Align streams by time. Device exposure times and hardware synchronization are unknown. |

The first lab run should verify the actual Touch button mapping, the camera-to-label assignment, successful replay of all three streams and an episode saved with **right A**. A simulation result establishes the software behavior. Physical grasp force, motor faults, collision clearance and the real robot's return path require separate commissioning.
