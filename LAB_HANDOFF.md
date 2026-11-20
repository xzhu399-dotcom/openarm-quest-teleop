# Lab handoff for the Quest 2 and OpenArm

Use a real Quest 2 to test the simulated arms first. For full-arm hardware, run the receiver on the Windows, Linux or macOS computer with the CANable adapters and cameras. [Robot connection and recording procedure](QUEST2_DATA_COLLECTION.md#connect-the-quest-to-a-physical-openarm)

Sections 3 and 4 cover the separate single-gripper bench test on the lab Mac. The gripper's encoder reports degrees and the model displays jaw opening in millimetres.

## Prompt to paste into Codex on the lab computer

```text
Help me run the OpenArm + Meta Quest 2 lab handoff on this computer. Clone the private repository https://github.com/xzhu399-dotcom/openarm-quest-teleop.git, or update its existing checkout with git pull --ff-only after checking local changes. Read LAB_HANDOFF.md. Detect this computer's OS and LAN IP, prepare its Python environment, and run the offline checks. Preserve local calibration, recordings and other changes.

First run the real Quest receiver with --pose-report --gripper-mode stick and synthetic cameras. On Windows, use Start-QuestSimulator.ps1 and give me the printed address and simulator PIN. It defaults to bm2roar. If we're on another Wi-Fi, select its active Windows network profile with -NetworkName or its local IPv4 address with -LabIP. On another OS, use the receiver's printed pairing URL. Wait for me to open it in Meta Quest Browser and select Start Quest. Guide me through the released-hand, clutch, wrist rotation, jaw retention, and A start/stop checks. Record a trial and verify it with replay.py and trial_metrics.py. Report the observed controller profile, receipt timing, IK rejections, and simulated tracking error.

Next help me test one physical J8 gripper on the Mac. Start with read-only USB discovery and encoder feedback. Use the lab's verified adapter side, J8 encoder limits, measured open/close test angles, and test speed. Ask me for any missing measurements. Run the read-only gripper bench and compare requested and measured encoder degrees. Before physical motion, wait for my on-site go-ahead. I will run the --execute-gripper command and type the local confirmation myself. Keep J1–J7 out of the test. Help me record and verify a small opening, closing, and hold. At the end, give me the episode IDs, results, and anything still unverified. Keep hardware IDs and trials local.
```

For a **full-arm hardware session** after the simulation check, use this prompt on the robot computer.

```text
Help me connect my Quest to the real OpenArm from the computer with the USB CANable adapters and cameras. Clone or update xzhu399-dotcom/openarm-quest-teleop and follow “Connect the Quest to a physical OpenArm” in QUEST2_DATA_COLLECTION.md. Identify the actual connected sides, adapter serials and available camera views, and use the rig's measured calibration. Start robot-readonly and give me its full HTTPS pairing URL. Help me check encoder readings and camera labels while the headset shows the model. Once I confirm the preview and rig are ready, give me the robot-drive command. I will arm it at the terminal. Guide me through a small side-grip movement, a right-A recording, an A+X return to the measured startup pose, and replay of the saved robot and camera data.
```

## 1. Prepare a fresh checkout

Sign in to GitHub on the lab computer with an account that can read the private repository. A new labmate can clone it after accepting the owner's invitation. [Private repository sharing instructions](README.md#share-this-private-repository-with-a-labmate)

From a parent directory, clone the repository and enter its folder.

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

In Windows PowerShell, use the installed Python 3 interpreter to create the environment. If `python` opens the Microsoft Store, use the full path to your installed `python.exe` in the first command (for example, `& 'C:\path\to\python.exe' -m venv .venv-lab`).

```powershell
python -m venv .venv-lab
.\.venv-lab\Scripts\python.exe -m unittest discover -s quest-pose -q
.\.venv-lab\Scripts\python.exe quest-pose/replay.py examples/mouse-robot
.\.venv-lab\Scripts\python.exe quest-pose/trial_metrics.py examples/mouse-robot
```

The receiver needs Python 3.9+ and OpenSSL for its default local HTTPS certificate. Install `opencv-python` for real camera feeds, the image terminal or their optional tests. Have Quest Browser available and both Touch controllers awake. Windows can run both the simulator and the full-arm receiver. Sections 3 and 4 use the Mac's single-gripper bench. [Windows and separate robot computer setup](QUEST2_DATA_COLLECTION.md#prepare-the-robot-computer)

## 2. Real Quest, simulated arms

### Reconnect on the Windows lab PC

For another Wi-Fi network, follow the steps to select the new profile or the PC's active IPv4 address. [Use a different Wi-Fi network](README.md#use-a-different-wi-fi-network)

1. Connect the PC and the headset to **bm2roar**. Open PowerShell in the checkout and run the [Windows launcher](Start-QuestSimulator.ps1). Leave its terminal open during collection.

   ```powershell
   powershell.exe -NoProfile -ExecutionPolicy RemoteSigned -File .\Start-QuestSimulator.ps1
   ```

2. In **Quest Browser**, enter the PC address printed by the launcher. You can type the bare IP or `http://<PC-IP>/`. The launcher redirects the browser to HTTPS before asking for a code. The PC's address can change when it rejoins Wi-Fi. If the browser shows a certificate warning, check that the address matches the one in PowerShell. The launcher generates a local certificate for that IP. For a managed headset, see the [trusted-certificate instructions](QUEST2_DATA_COLLECTION.md#2-connect-the-quest-2-to-the-workstation).
3. Enter the six-digit **simulator pairing code** printed in PowerShell, choose **Open simulator**, then **Start Quest**. The launcher remembers the code across restarts in `%USERPROFILE%\.openarm-quest\simulator-pin.txt` for this Windows user. A second browser on the PC can use the same code to observe. Five wrong entries pause new pairing attempts for five minutes.
4. Tap **A** on the right controller to start recording. Hold a side grip and move or turn that controller to drive the corresponding arm. Release the grip to leave the arm in place. Tap **A** again to save. Wait for **Trial saved** before shutting down. Stop the receiver with **Ctrl+C** in its terminal. After a reboot, repeat steps 1–3. If the PC only slept, rerun the launcher to see the address and PIN for the running simulator, then select **Start Quest** again in the headset.

The launcher starts the simulated receiver and its HTTP/HTTPS entry points together. The following commands cover launching individual components and the Mac workflow.

Join the same reachable LAN on the headset and workstation. Use the Mac's active Wi-Fi or Ethernet interface in place of `en0` if necessary. Keep this process running.

```sh
lab_ip="$(ipconfig getifaddr en0)"
python quest-pose/quest_pose_receiver.py --host "$lab_ip" --pose-report --gripper-mode stick
```

On Windows, find the IPv4 address of the adapter on the headset's network and launch from the same checkout:

```powershell
ipconfig
$labIP = Read-Host 'Enter the workstation IPv4 address on the Quest network'
.\.venv-lab\Scripts\python.exe quest-pose/quest_pose_receiver.py --host $labIP --pose-report --gripper-mode stick
```

Put on the Quest 2 and open **Browser** from the headset's app library. Select the browser address bar and enter the **complete printed HTTPS URL**, including the port and every character after `?key=`. The **Robot simulator** page should appear. Select **Start Quest** on the headset and allow the immersive session if prompted. Open the same full address on the workstation to observe or capture the initial pose. Each browser pairs separately.

**Optional short-code pairing.** Leave the receiver running. In a second workstation terminal at the repository root, run `python quest-pose/short_pairing.py` with the same Python used for the receiver (on Windows, `.\.venv-lab\Scripts\python.exe quest-pose/short_pairing.py`). At the prompt, paste the receiver's full pairing URL on the workstation. Open the short HTTPS address printed by the helper in the headset's Browser and enter the six-digit code within 15 minutes. Then select **Start Quest**. Once paired, that browser can bookmark `https://<workstation-ip>:<receiver-port>/` and reopen it while the receiver session and pairing cookie remain valid.

The helper listens on TCP 443 by default. If that port is unavailable, launch the helper with `--port 8444` and use the address it prints, including the port. If the receiver uses a lab-trusted certificate, supply the same `--certfile` and `--keyfile` paths to the helper.

If the workstation changes Wi-Fi, reconnect the headset to the same reachable LAN and check the workstation's current IPv4 address. If the address changed, stop the receiver with Ctrl+C, restart with `--host` set to the new address, and open the newly printed pairing URL. Each restart generates a fresh key.

To check headset reachability, open a fresh Quest Browser tab and enter just `https://<workstation-ip>:<port>/` using the IP and port printed by the running receiver. Its message asking for the complete pairing URL (HTTP 403), or the station page if already paired, confirms the headset reached the receiver. Use the full URL to pair. For a timeout, check the workstation's firewall permission for the receiver port (default TCP 8443) and Wi-Fi client isolation. For a certificate error, see the [pairing guide](QUEST2_DATA_COLLECTION.md#2-connect-the-quest-2-to-the-workstation).

| Test | What to do | Evidence to look for |
| --- | --- | --- |
| Baseline and isolation | Release both grips, choose **Capture initial pose** in the workstation browser, tap **right A** to record. Keep the handles still as you turn your head. Then move only the released right handle about 5 cm beside a ruler. | Head and hand values change independently. Released hand motion changes the reported hand displacement while both tool targets hold. |
| Arm and wrist | Hold the right side grip, move the right handle a few centimetres, then turn your wrist in place. Release, reposition and re-clutch. | The right target follows reachable movement and rotation. The left arm holds. Re-clutching starts from the existing tool pose. |
| Tracking recovery | While squeezing the right side grip, briefly move that controller out of tracking, then bring it back and move a few centimetres. | The arm stays in place during tracking loss. Once the controller is tracked again, **Grip engaged** returns and the new hand movement drives the arm from its last pose. |
| Gripper | Begin with the stick centered. While clutched, tilt the right stick down briefly, center it, and move the handle again. Tilt up to reopen. | The simulated aperture in mm decreases, holds during arm movement, and increases when requested. |
| Return home during collection | Release both side grips. Press **right A** and **left X** together, then release both. | Both arms return to their simulator startup poses. The jaw openings and camera selection stay fixed, and the same trial keeps recording. |
| Save | Release both grips and tap **right A** again. | The terminal reports `Recording STOP`, a saved result and the episode ID. The arms remain at the poses where you released the grips. |

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

The J8 bench records the encoder snapshot in `feedback.jsonl` under `physical_gripper`, with its own host timestamp. The simulated tool, joints and jaw retain their original fields. The motor tools also write a local hardware trace. For J1–J7 teleoperation, select the full-arm robot mode and follow the calibration and physical return checks. [Full-arm procedure](QUEST2_DATA_COLLECTION.md#connect-the-quest-to-a-physical-openarm)
