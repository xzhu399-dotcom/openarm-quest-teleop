[CmdletBinding(DefaultParameterSetName = 'Network')]
param(
    [Parameter(ParameterSetName = 'Network')]
    [string]$NetworkName = 'bm2roar',

    [Parameter(Mandatory, ParameterSetName = 'Address')]
    [System.Net.IPAddress]$LabIP
)

$ErrorActionPreference = 'Stop'
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
$ports = 80, 443, 8443
$listeners = @(Get-NetTCPConnection -State Listen | Where-Object {
    $_.LocalAddress -in @($LabIP.IPAddressToString, '0.0.0.0') -and $_.LocalPort -in $ports
})
$busy = @($listeners | Select-Object -ExpandProperty LocalPort -Unique)
$owners = @($listeners | Select-Object -ExpandProperty OwningProcess -Unique)
if ($busy.Count -eq $ports.Count -and $owners.Count -eq 1) {
    $process = Get-CimInstance Win32_Process -Filter ('ProcessId = ' + $owners[0])
    $command = [string]$process.CommandLine
    if ($command.IndexOf($receiverScript, [StringComparison]::OrdinalIgnoreCase) -ge 0 -and
        $command.IndexOf($codeFile, [StringComparison]::OrdinalIgnoreCase) -ge 0 -and
        $command -match '\s--pairing-code-file\s' -and $command -notmatch '\s--gripper-bench\s') {
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

$parentDirectory = Split-Path -Parent $PSScriptRoot
$candidates = @(
    (Join-Path $PSScriptRoot '.venv-lab\Scripts\python.exe'),
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
    & $python @pythonArgs -u $receiverScript --host $LabIP.IPAddressToString --pose-report --gripper-mode stick --pairing-code-file $codeFile
    $result = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $result
