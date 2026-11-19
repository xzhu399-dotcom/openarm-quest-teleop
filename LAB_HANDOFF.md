# Lab handoff for the Quest 2 and OpenArm

Use a real Quest 2 to test the simulated arms first. The physical test uses **one gripper motor, J8**, on the lab Mac. J1–J7 stay outside the Quest hardware controller. The gripper's encoder reports degrees. The arm model's jaw display reports simulated millimetres.

## Prompt to paste into Codex on the lab computer

```text
Help me run the OpenArm + Meta Quest 2 lab handoff on this computer. Clone the private repository https://github.com/xzhu399-dotcom/openarm-quest-teleop.git, or update its existing checkout with git pull --ff-only after checking local changes. Read LAB_HANDOFF.md. Detect this computer's OS and LAN IP, prepare its Python environment, and run the offline checks. Preserve local calibration, recordings and other changes.

First run the real Quest receiver with --pose-report --gripper-mode stick and synthetic cameras. Give me the exact pairing URL and wait for me to open it in Meta Quest Browser and select Start Quest. Guide me through the released-hand, clutch, wrist rotation, jaw retention, and A start/stop checks. Record a trial and verify it with replay.py and trial_metrics.py. Report the observed controller profile, receipt timing, IK rejections, and simulated tracking error.

Next help me test one physical J8 gripper on the Mac. Start with read-only USB discovery and encoder feedback. Use the lab's verified adapter side, J8 encoder limits, measured open/close test angles, and test speed. Ask me for any missing measurements. Run the read-only gripper bench and compare requested and measured encoder degrees. Before physical motion, wait for my on-site go-ahead. I will run the --execute-gripper command and type the local confirmation myself. Keep J1–J7 out of the test. Help me record and verify a small opening, closing, and hold. At the end, give me the episode IDs, results, and anything still unverified. Keep hardware IDs and trials local.
```

## 1. Prepare a fresh checkout

Sign in to GitHub on the lab computer with an account that can read the private repository. From a parent directory run:

```sh
git clone https://github.com/xzhu399-dotcom/openarm-quest-teleop.git
cd openarm-quest-teleop
```

For an existing checkout, run `git status --short --branch` there and use `git pull --ff-only` if the checkout can be updated safely. Run subsequent commands from the repository root. On the Mac:

```sh
python3 -m venv .venv-lab
source .venv-lab/bin/activate
python -m unittest discover -s quest-pose -q
python quest-pose/replay.py examples/mouse-robot
python quest-pose/trial_metrics.py examples/mouse-robot
```

The receiver needs Python 3.9+ and OpenSSL for its default local HTTPS certificate. Optical cameras are optional. Install `opencv-python` in the environment only if using the real camera feeds, the image terminal, or their optional tests. Python reports skipped optional tests explicitly. Have the Quest Browser available and both Touch controllers awake. Windows can run the entire simulated-arm stage using the [PowerShell commands in the README](README.md#just-a-quest-2-and-a-computer). The supplied motor tools use POSIX facilities and the lab hardware procedure below uses the Mac.

## 2. Real Quest, simulated arms

Join the same reachable LAN on the headset and workstation. Use the Mac's active Wi-Fi or Ethernet interface in place of `en0` if necessary. Keep this process running.

```sh
lab_ip="$(ipconfig getifaddr en0)"
python quest-pose/quest_pose_receiver.py --host "$lab_ip" --pose-report --gripper-mode stick
```

Enter the **complete printed HTTPS URL** in the Quest Browser, including the port and `?key=...`. Click **Start Quest** on the headset. Optionally open the same address on the Mac as an observer. If the headset cannot load it, check TCP 8443, firewall permission on the lab LAN, and Wi-Fi client isolation. Every receiver restart prints a new key. Certificate options are in the [pairing guide](QUEST2_DATA_COLLECTION.md#2-connect-the-quest-2-to-the-workstation).

| Test | What to do | Evidence to look for |
| --- | --- | --- |
| Baseline and isolation | Release both grips, choose **Capture initial pose** in the workstation browser, tap **right A** to record. Keep the handles still as you turn your head. Then move only the released right handle about 5 cm beside a ruler. | Head and hand values change independently. Released hand motion changes the reported hand displacement while both tool targets hold. |
| Arm and wrist | Hold the right side grip, move the right handle a few centimetres, then turn your wrist in place. Release, reposition and re-clutch. | The right target follows reachable movement and rotation. The left arm holds. Re-clutching starts from the existing tool pose. |
| Gripper | Begin with the stick centered. While clutched, tilt the right stick down briefly, center it, and move the handle again. Tilt up to reopen. | The simulated aperture in mm decreases, holds during arm movement, and increases when requested. |
| Save | Release both grips and tap **right A** again. | The terminal reports `Recording STOP`, a saved result and the episode ID. In a second trial, left stick **click** + right A after releasing both side grips also tests the simulated post-save arm return. |

From a second terminal in the same checkout and Python environment, substitute the reported ID:

```sh
python quest-pose/replay.py episodes/EPISODE_ID
python quest-pose/trial_metrics.py episodes/EPISODE_ID
```

Check the reported Quest controller profiles, input/action/feedback counts, recorder drops, and all three camera streams. Write down median/p95 receiver inter-sample intervals, simulated tool position/orientation errors, and observed inverse-kinematics rejections. Excluding rejected moves can make residual errors look small, so inspect both. The intervals measure delivery cadence at the computer. Motion-to-photon latency and physical arm tracking require separate measurements.

## 3. Real Quest and real J8, read-only first

Save any active trial and stop the simulator receiver with Ctrl+C. Support the physical arm and leave the jaws empty with room for a small stroke. Identify the adapter and gripper on the chosen side with the lab operator and have the physical stop within reach. Close other CAN programs before starting a hardware session.

```sh
hw=openarm-cli-smooth-accel-20261015-155332
python -m pip install -r "$hw/requirements.txt"
python "$hw/canable_inspect.py"
test -e "$hw/openarm_cli_config.json" || cp "$hw/openarm_cli_config.template.json" "$hw/openarm_cli_config.json"
test -e gripper-bench.lab.json || cp quest-pose/gripper-bench.example.json gripper-bench.lab.json
```

USB discovery can run with motor power off. Identify the serial by physical side. Supply the verified local config before querying motors. With the rig powered per lab procedure, `python "$hw/openarm_cli.py" --arm right status` reads the selected arm's J1–J8 angles, speeds, temperatures, states and modes. Substitute `left` for a left-side bench. Check actual motor replies. Use an unobstructed, mechanically supported setup throughout.

| Local configuration | Values supplied by the lab |
| --- | --- |
| `openarm_cli_config.json` in the hardware folder | `arms.<side>.serial` and `arms.<side>.limits_deg["8"]` as an ordered pair of lower/upper safe **encoder degree** limits. Preserve existing settings. |
| `gripper-bench.lab.json` in the repository root | Chosen `arm`, matching `adapter_serial`, relative `hardware_config` path, measured `open_deg` and `closed_deg` for a small verified test segment, and the lab-approved `max_speed_deg_s` between 0.1 and 1.0 encoder degrees/s. The open angle may be larger or smaller than the closed angle. |

Use the lab's established gripper calibration or obtain these values with its operator and the existing [hardware procedure](openarm-cli-smooth-accel-20261015-155332/OPENARM_CLI_GUIDE.txt). The supplied example deliberately has empty measurements. Simulated mm and encoder degrees have different meanings. Begin with the actual encoder inside the chosen segment, the motor disabled and in position-plus-velocity mode 2.

```sh
python quest-pose/quest_pose_receiver.py --host "$lab_ip" --gripper-mode stick --pose-report --gripper-bench gripper-bench.lab.json
```

Reconnect the headset with the **new URL**. Keep the right controller tracked for its **B** stop button, including during a left-gripper test. Release the chosen side grip, center its stick, and start a trial with **right A**. Clutch and briefly tilt the stick. Check that the target goes toward the specified open/closed angle, the measured encoder stays at rest, and J8 remains disabled. Save the trial, note its ID, and stop the receiver. Resolve any mismatch in side, sign, readback or calibration before a movement test.

## 4. Supervised first movement of the physical gripper

The on-site operator runs the following command in an interactive terminal after reviewing the read-only result. Bringing up this receiver can enable **only J8 on the chosen arm**. The run starts with a local confirmation and a fresh controller engagement. Run this stage while watching the actual gripper.

```sh
python quest-pose/quest_pose_receiver.py --host "$lab_ip" --gripper-mode stick --pose-report --gripper-bench gripper-bench.lab.json --execute-gripper
```

1. Read the terminal prompt and type the requested confirmation locally. Pair the headset with the new URL. Start with both grips released and the stick centered. Tap **right A** to record, then deliberately squeeze the side grip for the configured hand.
2. Make a small closing request and observe the physical jaws, requested encoder reference, and measured encoder degrees. Center the stick and verify the jaws settle. Make a small opening request and verify the opposite direction. Release the grip and check the endpoint holds. If the jaws move in the wrong direction or feedback is lost, end the run. **Right B** requests a software stop. Use the rig's physical stop when needed.
3. Release both grips and tap **right A** to save. Stop the receiver with Ctrl+C while the arm is supported. Check the hardware CLI's disable result. If disable cannot be confirmed, use the lab's shutdown procedure. Keep the local CAN diagnostic log.
4. Replay the physical-test episode and run `trial_metrics.py` as above. A physical pass needs **visible jaw movement in the expected direction and distinct encoder readbacks** in execute mode. Merely launching the program or seeing the simulator change is insufficient. Record the before/after angles and whether the hold and stop behaved as expected.

To check **right B** on the real rig, first save the movement trial. With the supported rig supervised, the operator can press B after J8 has been enabled, then verify the bench exits and the driver confirms disable. Resume physical tests by starting a fresh bench session. An unconfirmed disable belongs in the lab report as a failed check.

The physical encoder snapshot lives in `feedback.jsonl` under `physical_gripper`, with its own host timestamp. The simulated tool, joints and jaw retain their original fields. The local hardware trace is in the motor tools' `motor-test-logs/` directory. Save-and-return restores simulated arm joints. This bench does not command a physical return path or measure real TCP pose. Full physical J1–J7 teleoperation is a further commissioning task requiring the rig's joint mapping, zero/signs, limits, base/tool calibration and a validated hardware controller.
