[CmdletBinding(DefaultParameterSetName = 'Network')]
param(
    [Parameter(ParameterSetName = 'Network')]
    [string]$NetworkName = 'bm2roar',

    [Parameter(Mandatory, ParameterSetName = 'Address')]
    [System.Net.IPAddress]$LabIP,

    [ValidateSet('simulate', 'robot-readonly', 'robot-drive')]
    [string]$Mode = 'simulate',
    [string]$RobotCalibration,
    [string]$Cameras
)

$ErrorActionPreference = 'Stop'
$robotMode = $Mode -ne 'simulate'
$receiverArgs = @('--mode', $Mode.ToLowerInvariant(), '--pose-report', '--gripper-mode', 'stick')
if ($robotMode) {
    if (-not $RobotCalibration) {
        $RobotCalibration = Join-Path $PSScriptRoot 'quest-pose\robot-hardware.lab.json'
    }
    if (-not (Test-Path -LiteralPath $RobotCalibration -PathType Leaf)) {
        throw "Supply -RobotCalibration with the measured configuration for the attached arms. File missing: $RobotCalibration"
    }
    $receiverArgs += @('--robot-calibration', (Resolve-Path -LiteralPath $RobotCalibration).Path, '--port', '8444')
} elseif ($RobotCalibration) {
    throw 'Select -Mode robot-readonly or robot-drive to use -RobotCalibration.'
}
if ($Cameras) {
    $receiverArgs += @('--cameras', (Resolve-Path -LiteralPath $Cameras).Path)
}
$addresses = @(Get-NetIPAddress -AddressFamily IPv4 -AddressState Preferred |
    Where-Object { -not $_.SkipAsSource })
if ($PSCmdlet.ParameterSetName -eq 'Network') {
    $profiles = @(Get-NetConnectionProfile | Where-Object { $_.Name -eq $NetworkName })
    if (-not $profiles) {
        throw "Connect the PC to '$NetworkName', or specify its Quest-network address with -LabIP."
    }
    $matching = @($addresses | Where-Object { $_.InterfaceIndex -in $profiles.InterfaceIndex })
    if ($matching.Count -ne 1) {
        throw "Couldn't select one IPv4 address for '$NetworkName'. Use ipconfig and pass -LabIP with the PC's address."
    }
    $LabIP = [System.Net.IPAddress]::Parse($matching[0].IPAddress)
} elseif ($LabIP.IPAddressToString -notin $addresses.IPAddress) {
    throw "$LabIP is not an active IPv4 address on this PC. Check ipconfig and retry."
}

$receiverScript = Join-Path $PSScriptRoot 'quest-pose\quest_pose_receiver.py'
# Store Python can redirect LocalAppData. Both processes see the same file under the user profile.
$codeFile = Join-Path $env:USERPROFILE '.openarm-quest\simulator-pin.txt'
$ports = if ($robotMode) { @(8444) } else { @(80, 443, 8443) }
$listeners = @(Get-NetTCPConnection -State Listen | Where-Object {
    $_.LocalAddress -in @($LabIP.IPAddressToString, '0.0.0.0') -and $_.LocalPort -in $ports
})
$busy = @($listeners | Select-Object -ExpandProperty LocalPort -Unique)
$owners = @($listeners | Select-Object -ExpandProperty OwningProcess -Unique)
if ($robotMode -and $busy) {
    throw "Port 8444 on $LabIP is already in use. Check the existing receiver terminal for its mode and pairing link. To switch modes, save the trial and stop that terminal with Ctrl+C."
}
if (-not $robotMode -and $busy.Count -eq $ports.Count -and $owners.Count -eq 1) {
    $process = Get-CimInstance Win32_Process -Filter ('ProcessId = ' + $owners[0])
    $command = [string]$process.CommandLine
    if ($command.IndexOf($receiverScript, [StringComparison]::OrdinalIgnoreCase) -ge 0 -and
        $command.IndexOf($codeFile, [StringComparison]::OrdinalIgnoreCase) -ge 0 -and
        $command -match '\s--pairing-code-file\s' -and $command -notmatch '\s(?:--gripper-bench\s|--mode[ =]robot-)') {
        if ($Cameras -and $command.IndexOf((Resolve-Path -LiteralPath $Cameras).Path, [StringComparison]::OrdinalIgnoreCase) -lt 0) {
            throw 'A simulator is running with a different camera configuration. Save its trial and stop its terminal with Ctrl+C before changing cameras.'
        }
        Write-Output "Simulator already running. Open https://$LabIP/ in Quest Browser."
        if (Test-Path -LiteralPath $codeFile -PathType Leaf) {
            $savedCode = (Get-Content -Raw -LiteralPath $codeFile).Trim()
            if ($savedCode -match '^[0-9]{6}$') { Write-Output "Simulator pairing code: $savedCode" }
        }
        exit 0
    }
}
if ($busy) {
    throw "Port(s) $($busy -join ', ') on $LabIP are already in use. If the simulator is already running, use its printed URL. To restart it, save any trial and stop its previous terminal with Ctrl+C."
}
if (-not $robotMode) {
    $receiverArgs += @('--pairing-code-file', $codeFile)
}

$parentDirectory = Split-Path -Parent $PSScriptRoot
$candidates = @(
    (Join-Path $PSScriptRoot '.venv-lab\Scripts\python.exe'),
    (Join-Path $PSScriptRoot '.venv-teleop\Scripts\python.exe'),
    (Join-Path $PSScriptRoot '.venv\Scripts\python.exe'),
    (Join-Path $parentDirectory '.venv\Scripts\python.exe')
)
$python = $candidates | Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } | Select-Object -First 1
$pythonArgs = @()
if (-not $python) {
    $launcher = Get-Command py.exe -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($launcher) {
        $python = $launcher.Source
        $pythonArgs = @('-3')
    } else {
        $python = (Get-Command python.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
    }
}

Push-Location -LiteralPath $PSScriptRoot
try {
    & $python @pythonArgs -u $receiverScript --host $LabIP.IPAddressToString @receiverArgs
    $result = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $result
