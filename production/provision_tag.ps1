# provision_tag.ps1
#
# frtag3 production station: program, test, serialise and power-check one tag,
# and leave a secondary in production sleep.
#
# Station wiring
#   - ST-LINK (V3) on the tag's SWD header. Its virtual COM port carries the
#     tag's debug UART (USART2, 115200 8N1); the script finds that port on its
#     own unless -ComPort names one.
#   - Optionally an Otii Arc on the tag's battery input. Primary and
#     secondary run from the same battery type, so both are supplied at 3.8 V
#     (-SupplyV). With it the script powers the board itself, power-cycles it,
#     and measures current.
#     Without it (or without an Otii Automation Toolbox licence on this PC) the
#     board must be powered some other way; every step still runs except the
#     current measurements, and resets go through the ST-LINK instead.
#
# What one run does
#   1. Programs the firmware over SWD (newest combined bootloader+app hex in
#      ..\Debug unless -FlashHex names one; -NoFlash skips it).
#   2. Reads the chip's 96-bit UID and its OTP identity slots over SWD, derives
#      the radio id (same formula as LORARADIO_u32GetUniqueId) and checks the
#      unit database: a radio id or serial already owned by another chip stops
#      the run. A chip seen before keeps its serial.
#   3. With an Otii, secondary only: boots the board and waits for it to put
#      itself into production sleep (a unit not yet in service always does,
#      APP_BOOT_WINDOW_MS after boot), then measures the SLEEP CURRENT - the
#      first pass, before anything was switched on.
#   4. Resets the board and opens a kernel session on the debug UART inside the
#      boot window, then runs the production test (kernel 'u'): live flash,
#      accelerometer, GPS and radio checks, battery and solar readings, then
#      everything switched on for -HoldS seconds while the Otii measures the
#      ALL-ON CURRENT. With an Otii the tag's battery reading is also checked
#      against the voltage the Otii is supplying.
#   5. Writes the serial number (kernel 'N') if the unit has none - only after
#      the production test passed, so a failed board does not use a serial -
#      and reads it back (kernel 'n').
#   6. Secondary: puts the tag into production sleep (kernel 'P', persisted, so
#      the tag stays asleep through resets until its panel or a shake wakes it)
#      and measures the SLEEP CURRENT again - the final pass, which proves that
#      everything the test switched on was switched off again.
#   7. Resets once more and verifies the OTP record over SWD, independently of
#      what the firmware said. A secondary goes straight back to production
#      sleep after that reset.
#   8. Switches the Otii off and appends one row to the production test log.
#
# Every run is logged, including re-runs of a unit that went through before:
# the unit keeps its serial and the log gets another row with the run number.
# A full transcript of each run, UART traffic included, goes to -RunLogDir.
#
# The serial number is permanent (STM32WL OTP, see
# fr_app/settings/Identity_Config.h); a wrong one can only be superseded with
# -Fix, and each chip has 32 slots for that in total.
#
# Serial number format: 8 hex digits, 0xVRNNNNNN - V hardware version (3 for
# every frtag3; units with the same hardware version run the same code base),
# R hardware revision (1..F, -HwRevision, Tagskaap V3 RevA = 1), NNNNNN unit
# number within that revision (000001..FFFFFF). The first unit is 31000001.
# A new unit gets the next free number in its revision from the unit database;
# -Serial overrides that. The firmware refuses any other format.
#
# The unit database is this station's own file. Two stations each with their
# own database would hand out the same numbers: run one station, or share the
# database between them (one run at a time).
#
# Requirements
#   - STM32CubeProgrammer (STM32_Programmer_CLI.exe).
#   - Firmware built with FRKERNEL_INTERFACE_UART (build_config.h; on by
#     default) - the station talks to the tag through it.
#   - Read-out protection at level 0 (SWD must be able to read UID and OTP).
#   - For power tests: Otii desktop software with otiicli.exe and an Otii
#     Automation Toolbox licence logged in on this PC; production\otii_agent.lua
#     is the script it runs.
#
# Limits
#   The current limits are parameters because they belong to the board, not to
#   this script. Set them from a known-good unit before relying on them: run a
#   few golden boards, read SleepStart/AllOn/SleepEnd from the test log, and
#   put the limits a sensible margin outside what they measured.
#
# Examples
#   .\provision_tag.ps1 -Role secondary
#   .\provision_tag.ps1 -Role primary -NoOtii
#   .\provision_tag.ps1 -Role secondary -DryRun          (read and check only)
#   .\provision_tag.ps1 -Role secondary -Serial 31000123 -Fix
#   .\provision_tag.ps1 -Role secondary -HwRevision 2    (first RevB unit: 32000001)
#
# Exit code 0 when every check passed, 1 otherwise.

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('secondary', 'primary')]
    [string] $Role,

    [string] $ComPort = '',
    [string] $FlashHex = '',
    [switch] $NoFlash,

    [string] $Serial = '',          # 8 hex digits; empty = keep / next free
    [ValidateRange(1, 15)]
    [int]    $HwRevision = 1,       # revision digit for newly allocated serials
    [switch] $Fix,

    [string] $Database  = (Join-Path $PSScriptRoot 'provisioned_units.csv'),
    [string] $TestLog   = (Join-Path $PSScriptRoot 'production_tests.csv'),
    [string] $RunLogDir = (Join-Path $PSScriptRoot 'runs'),

    [switch] $NoOtii,
    [double] $SupplyV = 3.8,        # Otii output, both roles (same battery)
    [int]    $HoldS = 10,
    [double] $SleepMaxUA = 50,      # placeholder - set from golden units
    [double] $AllOnMinMA = 5,       # placeholder - set from golden units
    [double] $AllOnMaxMA = 80,      # placeholder - set from golden units
    [int]    $BatToleranceMV = 150,

    [switch] $DryRun,
    [int]    $Baud = 115200,
    [string] $Operator = $env:USERNAME,
    [string] $Programmer = '',
    [string] $OtiiCli = ''
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

# ---- Constants shared with the firmware ----------------------------------
# Change these only together with the firmware files named beside them.
$UID_ADDR          = 0x1FFF7590   # UID_BASE, stm32wle5xx.h
$OTP_ID_ADDR       = 0x1FFF7000   # IDENTITY_OTP_BASE_ADDR, Identity_Config.h
$OTP_ID_SLOTS      = 32           # IDENTITY_OTP_SLOTS
$ID_MAGIC          = 0x4E53       # IDENTITY_MAGIC
$ID_FORMAT         = 1            # IDENTITY_FORMAT
$LORA_ID_CRC8_POLY = 0x07         # LORA_ID_CRC8_POLY, LoraRadio_Config.h
$LORA_ID_HI_SUBST  = 0xFF00       # LORA_ID_HI_ZERO_SUBST
$BOOT_WINDOW_S     = 30           # APP_BOOT_WINDOW_MS, DeviceDiscovery.h
$SN_HW_VERSION     = 3            # IDENTITY_SERIAL_HW_VERSION, Identity_Config.h
$SN_FAMILY_MASK    = 0xFF000000L  # version + revision digits
$SN_UNIT_MASK      = 0x00FFFFFFL  # IDENTITY_SERIAL_UNIT_MASK
# Note for anyone editing: in Windows PowerShell a hex literal of 0x80000000
# or more is a NEGATIVE [int] (0xFFFFFFFF is -1). Every such constant below
# carries the L suffix to make it a positive [long].

# Station timing.
$BOOT_LINE_TIMEOUT_S  = 25    # power-up to the "Identity:" boot line
$SLEEP_SETTLE_S       = 4     # entering production sleep to a steady current
$SLEEP_MEASURE_MS     = 5000
$ALLON_SETTLE_S       = 2     # "PT hold" to a steady all-on current
$POWER_OFF_S          = 2     # off time for a power-cycle reset

# ---- Run state and logging ------------------------------------------------

$script:Fails  = New-Object System.Collections.Generic.List[string]
$script:Notes  = New-Object System.Collections.Generic.List[string]
$script:R      = [ordered]@{}   # the test-log row, filled as the run goes
$script:Otii   = $null
$script:Port   = $null
$script:Cli    = $null

if (-not (Test-Path $RunLogDir)) { New-Item -ItemType Directory -Path $RunLogDir | Out-Null }
$RunStamp = Get-Date
$RunLog   = Join-Path $RunLogDir ($RunStamp.ToString('yyyyMMdd_HHmmss') + '.log')

function Log([string] $msg, [string] $color = '') {
    $line = '{0} {1}' -f (Get-Date).ToString('HH:mm:ss.fff'), $msg
    Add-Content -Path $RunLog -Value $line
    if ($color) { Write-Host $msg -ForegroundColor $color } else { Write-Host $msg }
}
function Add-Fail([string] $msg) { $script:Fails.Add($msg); Log "FAIL: $msg" 'Red' }
function Add-Note([string] $msg) { $script:Notes.Add($msg); Log "note: $msg" 'Yellow' }
# Fatal: record, then unwind to the finally block that powers down and logs.
function Stop-Run([string] $msg) { Add-Fail $msg; throw 'run aborted' }

# ---- STM32CubeProgrammer --------------------------------------------------

function Find-Programmer {
    if ($Programmer -ne '') { return $Programmer }
    $cmd = Get-Command 'STM32_Programmer_CLI.exe' -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    $default = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    if (Test-Path $default) { return $default }
    return $null
}

function Invoke-Programmer([string[]] $cliArgs, [switch] $NoThrow) {
    $out = & $script:Cli @cliArgs 2>&1 | Out-String
    Add-Content -Path $RunLog -Value ("CLI> " + ($cliArgs -join ' ') + "`r`n" + $out)
    if ($LASTEXITCODE -ne 0) {
        if ($NoThrow) { return $null }
        Stop-Run "STM32_Programmer_CLI $($cliArgs -join ' ') exited with $LASTEXITCODE"
    }
    return $out
}

# Parse "-r32" dump lines ("0x1FFF7000 : 11111111 22222222 ...") into a
# hashtable address -> uint32. Keys are always [long]: a hashtable treats
# 4 as [int] and 4 as [uint32] as different keys.
function Read-Words([string] $dump) {
    $words = @{}
    foreach ($line in ($dump -split "`r?`n")) {
        if ($line -match '^\s*0x([0-9A-Fa-f]{8})\s*:\s*((?:[0-9A-Fa-f]{8}\s*)+)$') {
            [long] $addr = [Convert]::ToUInt32($Matches[1], 16)
            foreach ($w in ($Matches[2].Trim() -split '\s+')) {
                $words[$addr] = [Convert]::ToUInt32($w, 16)
                $addr += 4
            }
        }
    }
    return $words
}

function Get-LeBytes([uint32[]] $words) {
    $bytes = New-Object System.Collections.Generic.List[byte]
    foreach ($w in $words) { $bytes.AddRange([BitConverter]::GetBytes([uint32]$w)) }
    return ,$bytes.ToArray()
}

# CRC-32/ISO-HDLC, as crc32.h (zlib.crc32). Done in [uint64] so PowerShell's
# signed bitwise operators never see a negative number.
function Get-Crc32([byte[]] $data) {
    [uint64] $crc = 0xFFFFFFFFL
    foreach ($b in $data) {
        $crc = $crc -bxor [uint64]$b
        for ($i = 0; $i -lt 8; $i++) {
            if (($crc -band 1) -ne 0) { $crc = ($crc -shr 1) -bxor 0xEDB88320L }
            else                      { $crc = $crc -shr 1 }
        }
    }
    return [uint32](($crc -bxor 0xFFFFFFFFL) -band 0xFFFFFFFFL)
}

# CRC-8/SMBUS, as LORARADIO_u8IdCrc8 (LoraRadio.c).
function Get-Crc8([byte[]] $data) {
    [int] $crc = 0
    foreach ($b in $data) {
        $crc = $crc -bxor $b
        for ($i = 0; $i -lt 8; $i++) {
            if (($crc -band 0x80) -ne 0) { $crc = (($crc -shl 1) -bxor $LORA_ID_CRC8_POLY) -band 0xFF }
            else                         { $crc = ($crc -shl 1) -band 0xFF }
        }
    }
    return [byte]$crc
}

# The radio id, exactly as LORARADIO_u32GetUniqueId builds it.
function Get-RadioId([uint32[]] $uid) {
    [uint32] $w0 = $uid[0]; [uint32] $w1 = $uid[1]; [uint32] $w2 = $uid[2]
    [uint32] $lo = ((($w0 -shr 16) -band 0xFF) -shl 8) -bor ($w0 -band 0xFF)
    [byte[]] $lot = @(
        [byte](($w1 -shr 8) -band 0xFF), [byte](($w1 -shr 16) -band 0xFF), [byte](($w1 -shr 24) -band 0xFF),
        [byte]($w2 -band 0xFF), [byte](($w2 -shr 8) -band 0xFF), [byte](($w2 -shr 16) -band 0xFF),
        [byte](($w2 -shr 24) -band 0xFF))
    [uint32] $hi = (($w1 -band 0xFF) -shl 8) -bor (Get-Crc8 $lot)
    if ($hi -eq 0) { $hi = $LORA_ID_HI_SUBST }
    return [uint32](([uint64]$hi -shl 16) -bor $lo)
}

# ---- Serial numbers ---------------------------------------------------------
# Serials are [long] inside the script and 8 hex digits everywhere they are
# shown, logged or stored.

function Format-Serial($sn) { return '{0:X8}' -f [long]$sn }

# Text (8 hex digits, optional 0x) to [long]; $null when it is not that.
function ConvertFrom-SerialText([string] $text) {
    $t = $text.Trim()
    if ($t -match '^0[xX]') { $t = $t.Substring(2) }
    if ($t -notmatch '^[0-9A-Fa-f]{8}$') { return $null }
    return [long]::Parse($t, [Globalization.NumberStyles]::HexNumber)
}

# Same rule as IDENTITY_bSerialFormatOk: version 3, revision 1..F, unit 1..FFFFFF.
function Test-SerialFormat([long] $sn) {
    return ((($sn -shr 28) -band 0xF) -eq $SN_HW_VERSION) -and
           ((($sn -shr 24) -band 0xF) -ne 0) -and
           (($sn -band $SN_UNIT_MASK) -ne 0)
}

# Every serial in the unit database, parsed. A row that is not 8 hex digits
# stops the run: allocating around a row we cannot read could reuse a number.
function Get-DbSerial($row) {
    $sn = ConvertFrom-SerialText $row.Serial
    if ($null -eq $sn) { Stop-Run ("unit database row for chip {0} has serial '{1}', not 8 hex digits - fix $Database" -f $row.Uid, $row.Serial) }
    return $sn
}

# Decode the OTP identity slots. Returns the current record (highest valid
# slot) or $null, plus how many slots are still free.
function Get-OtpIdentity($words, [uint32[]] $uid) {
    $current = $null
    $lastUsed = -1
    for ($s = 0; $s -lt $OTP_ID_SLOTS; $s++) {
        $base = $OTP_ID_ADDR + 16 * $s
        $w = @(0..3 | ForEach-Object { $words[[long]($base + 4 * $_)] })
        if ($w -contains $null) { Stop-Run "OTP dump is missing slot $s" }
        if (@($w | Where-Object { $_ -ne 0xFFFFFFFFL }).Count -eq 0) { continue }
        $lastUsed = $s
        $magic  = $w[0] -band 0xFFFF
        $format = ($w[0] -shr 16) -band 0xFF
        $crc    = Get-Crc32 (Get-LeBytes (@($w[0], $w[1], $w[2]) + $uid))
        if (($magic -eq $ID_MAGIC) -and ($format -eq $ID_FORMAT) -and ($crc -eq $w[3]) -and
            ($w[1] -ge 1) -and ($w[1] -le 0xFFFFFFFEL)) {
            $current = [pscustomobject]@{ Slot = $s; Serial = [uint32]$w[1]; RadioId = [uint32]$w[2] }
        }
    }
    return [pscustomobject]@{ Current = $current; Free = $OTP_ID_SLOTS - ($lastUsed + 1) }
}

# A few attempts: right after a reset the tag may still be in a STOP2 nap of
# its init sequence, where a hot-plug SWD connect can fail. The boot window
# keeps it awake shortly after.
function Read-Target {
    $dump = $null
    for ($try = 1; $try -le 4 -and $null -eq $dump; $try++) {
        $dump = Invoke-Programmer @('-c', 'port=SWD', 'mode=HOTPLUG',
                                    '-r32', ('0x{0:X8}' -f $UID_ADDR), '12',
                                    '-r32', ('0x{0:X8}' -f $OTP_ID_ADDR), ($OTP_ID_SLOTS * 16).ToString()) -NoThrow
        if ($null -eq $dump) { Start-Sleep -Seconds 2 }
    }
    if ($null -eq $dump) { Stop-Run 'SWD read of UID/OTP failed - check the ST-LINK connection and that RDP is level 0' }
    $words = Read-Words $dump
    $uid = @(0..2 | ForEach-Object { $words[[long]($UID_ADDR + 4 * $_)] })
    if ($uid -contains $null) { Stop-Run 'could not parse the UID from the programmer output' }
    return [pscustomobject]@{ Uid = [uint32[]]$uid; Otp = (Get-OtpIdentity $words ([uint32[]]$uid)) }
}

function Format-Uid([uint32[]] $uid) { '{0:X8}{1:X8}{2:X8}' -f $uid[2], $uid[1], $uid[0] }

function Find-FlashHex {
    if ($FlashHex -ne '') { return $FlashHex }
    $dir = Join-Path $PSScriptRoot '..\Debug'
    $hex = Get-ChildItem -Path $dir -Filter 'frtag3_*_bl_*.hex' -ErrorAction SilentlyContinue |
           Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if ($hex) { return $hex.FullName }
    return $null
}

# ---- ST-LINK virtual COM port --------------------------------------------

function Find-ComPort {
    if ($ComPort -ne '') { return $ComPort }
    $ports = @(Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
               Where-Object { $_.Name -match 'STLink Virtual COM Port \((COM\d+)\)' } |
               ForEach-Object { [regex]::Match($_.Name, '\((COM\d+)\)').Groups[1].Value })
    if ($ports.Count -eq 1) { return $ports[0] }
    if ($ports.Count -eq 0) { Stop-Run 'no ST-LINK virtual COM port found - connect the ST-LINK or pass -ComPort' }
    Stop-Run ("several ST-LINK COM ports ({0}) - pass -ComPort" -f ($ports -join ', '))
}

function Open-Uart([string] $name) {
    $p = New-Object System.IO.Ports.SerialPort $name, $Baud, 'None', 8, 'One'
    $p.NewLine     = "`n"
    $p.ReadTimeout = 250
    $p.Open()
    $p.DiscardInBuffer()
    return $p
}

# Read UART lines until one matches $pattern or $timeoutS runs out. Every line
# goes to the run log. Returns the matching line (trimmed) or $null.
function Wait-Line([string] $pattern, [double] $timeoutS) {
    $deadline = (Get-Date).AddSeconds($timeoutS)
    while ((Get-Date) -lt $deadline) {
        try { $line = $script:Port.ReadLine() } catch [TimeoutException] { continue }
        $line = $line.TrimEnd("`r", "`n")
        Add-Content -Path $RunLog -Value ("UART< " + $line)
        if ($line -match $pattern) { return $line }
    }
    return $null
}

function Send-Line([string] $cmd) {
    Add-Content -Path $RunLog -Value ("UART> " + $cmd)
    $script:Port.Write("$cmd`r")
}

# ---- Otii (through otiicli + otii_agent.lua) -------------------------------

function Read-OtiiLine($o, [int] $timeoutMs) {
    if ($null -eq $o.Pending) { $o.Pending = $o.Proc.StandardOutput.ReadLineAsync() }
    if (-not $o.Pending.Wait($timeoutMs)) { return $null }
    $line = $o.Pending.Result
    $o.Pending = $null
    if ($null -ne $line) { Add-Content -Path $RunLog -Value ("OTII< " + $line) }
    return $line
}

function Start-Otii {
    if ($NoOtii) { Add-Note 'Otii disabled (-NoOtii) - no power tests'; return $null }
    $cli = $OtiiCli
    if ($cli -eq '') {
        $cmd = Get-Command 'otiicli.exe' -ErrorAction SilentlyContinue
        if ($cmd) { $cli = $cmd.Source } else { $cli = 'C:\Program Files\otii\otiicli.exe' }
    }
    if (-not (Test-Path $cli)) { Add-Note 'otiicli.exe not found - no power tests'; return $null }

    $agent = Join-Path $PSScriptRoot 'otii_agent.lua'
    $pipe  = 'frtag3-otii-' + $PID
    $psi = New-Object System.Diagnostics.ProcessStartInfo $cli
    $psi.Arguments              = ('-n "{0}" {1}' -f $agent, $pipe)
    $psi.UseShellExecute        = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError  = $true
    $psi.CreateNoWindow         = $true
    $proc = [System.Diagnostics.Process]::Start($psi)
    $o = @{ Proc = $proc; Pending = $null; Pipe = $null }

    # otiicli prints its own errors ("No available license", "No Otii Arc
    # found") on stdout and exits; anything but READY means no Otii this run.
    $line = $null
    $deadline = (Get-Date).AddSeconds(20)
    while ((Get-Date) -lt $deadline) {
        $line = Read-OtiiLine $o 1000
        if ($null -ne $line -and $line -match 'READY') { break }
        if ($proc.HasExited) { break }
        if ($null -ne $line -and $line -match 'licen[cs]e|No Otii|Cannot|error') {
            Add-Note "Otii not available ($line) - no power tests"
            if (-not $proc.HasExited) { $proc.Kill() }
            return $null
        }
    }
    if ($proc.HasExited -or ($null -eq $line) -or ($line -notmatch 'READY')) {
        $why = 'agent did not start'
        if ($proc.HasExited) {
            $err = ($proc.StandardError.ReadToEnd() + ' ' + $proc.StandardOutput.ReadToEnd()).Trim() -replace '\s+', ' '
            if ($err) { $why = $err }
        }
        Add-Note "Otii not available ($why) - no power tests"
        if (-not $proc.HasExited) { $proc.Kill() }
        return $null
    }

    $o.Pipe = New-Object System.IO.Pipes.NamedPipeClientStream '.', $pipe, ([System.IO.Pipes.PipeDirection]::InOut)
    try { $o.Pipe.Connect(5000) } catch {
        Add-Note "Otii agent pipe did not open ($_) - no power tests"
        $proc.Kill(); return $null
    }
    Log 'Otii Arc connected'
    return $o
}

# One command, one answer line (OK / MEAS / ERR). $null on timeout.
function Invoke-Otii([string] $cmd, [int] $timeoutMs = 10000) {
    $o = $script:Otii
    if ($null -eq $o) { return $null }
    Add-Content -Path $RunLog -Value ("OTII> " + $cmd)
    $bytes = [System.Text.Encoding]::ASCII.GetBytes($cmd)
    $o.Pipe.Write($bytes, 0, $bytes.Length)
    $o.Pipe.Flush()
    $deadline = (Get-Date).AddMilliseconds($timeoutMs)
    while ((Get-Date) -lt $deadline) {
        $line = Read-OtiiLine $o 500
        if ($null -ne $line -and $line -match '^(OK|MEAS|ERR)') { return $line }
        if ($o.Proc.HasExited) { break }
    }
    return $null
}

function Set-OtiiOrNote([string] $cmd) {
    $r = Invoke-Otii $cmd
    if ($r -ne 'OK') { Add-Fail "Otii '$cmd' answered '$r'" }
}

# Average current over $ms milliseconds, in amperes; $null if not measurable.
function Measure-Current([int] $ms) {
    $r = Invoke-Otii "meas $ms" ($ms + 20000)
    if ($null -eq $r -or $r -notmatch '^MEAS avg=(\S+) min=(\S+) max=(\S+) n=(\d+)') {
        Add-Fail "Otii measurement failed ('$r')"
        return $null
    }
    return [pscustomobject]@{
        Avg = [double]::Parse($Matches[1], [Globalization.CultureInfo]::InvariantCulture)
        Min = [double]::Parse($Matches[2], [Globalization.CultureInfo]::InvariantCulture)
        Max = [double]::Parse($Matches[3], [Globalization.CultureInfo]::InvariantCulture)
        N   = [int]$Matches[4]
    }
}

# ---- Target control --------------------------------------------------------

# Reset the tag. With an Otii it is a real power cycle, which also clears any
# debug-in-low-power bits an SWD session left set (those survive a system
# reset and would inflate the sleep current). Without one, NRST via ST-LINK.
function Reset-Target {
    if ($script:Otii) {
        Set-OtiiOrNote 'off'
        Start-Sleep -Seconds $POWER_OFF_S
        Set-OtiiOrNote 'on'
    } elseif ($null -eq (Invoke-Programmer @('-c', 'port=SWD', 'mode=HOTPLUG', '-hardRst') -NoThrow)) {
        Invoke-Programmer @('-c', 'port=SWD', 'mode=HOTPLUG', '-rst') | Out-Null
    }
}

# Open a kernel session: inside the boot window the tag answers any line, so
# send 'i' until its status line comes back. Returns the parsed status.
function Connect-Kernel {
    $deadline = (Get-Date).AddSeconds($BOOT_WINDOW_S - 2)
    while ((Get-Date) -lt $deadline) {
        Send-Line 'i'
        $l = Wait-Line '^[0-9A-F]{8} sn=[0-9A-F]{8} (PRIMARY|secondary) fw=v' 1.5
        if ($l) {
            if ($l -notmatch '^([0-9A-F]{8}) sn=([0-9A-F]{8}) (PRIMARY|secondary) fw=v(\S+) bl=v(\S+) bat=(\d+)mV sol=(\d+)mV state=(\S+)') {
                Stop-Run "unrecognised status line: $l"
            }
            return [pscustomobject]@{
                RadioId = $Matches[1]; Serial = $Matches[2]
                Role = $Matches[3].ToLower(); Fw = $Matches[4]; Bl = $Matches[5]
                BatMv = [int]$Matches[6]; SolMv = [int]$Matches[7]; State = $Matches[8]
            }
        }
    }
    Stop-Run 'the tag does not answer kernel commands on the debug UART - is the firmware built with FRKERNEL_INTERFACE_UART, and is the VCP wired?'
}

function Wait-Boot {
    if (-not (Wait-Line 'Identity: ' $BOOT_LINE_TIMEOUT_S)) {
        Stop-Run "no boot output on $($script:Port.PortName) within $BOOT_LINE_TIMEOUT_S s"
    }
}

# Wait for the tag to report production sleep, then measure. $label is the
# test-log column prefix.
function Measure-Sleep([string] $label) {
    Start-Sleep -Seconds $SLEEP_SETTLE_S
    $m = Measure-Current $SLEEP_MEASURE_MS
    if ($null -eq $m) { return }
    $ua = [math]::Round($m.Avg * 1e6, 2)
    $script:R["${label}_uA"] = $ua
    $script:R["${label}Max_uA"] = [math]::Round($m.Max * 1e6, 1)
    if ($ua -gt $SleepMaxUA) { Add-Fail ("{0} sleep current {1} uA > {2} uA" -f $label, $ua, $SleepMaxUA) }
    else { Log ("{0} sleep current {1} uA (limit {2})" -f $label, $ua, $SleepMaxUA) 'Green' }
}

# ---- Run --------------------------------------------------------------------

$exitCode = 1
try {
    Log ("frtag3 production run - role {0}, station {1}, operator {2}" -f $Role, $env:COMPUTERNAME, $Operator)

    $script:Cli = Find-Programmer
    if (-not $script:Cli) { Stop-Run 'STM32_Programmer_CLI.exe not found - install STM32CubeProgrammer or pass -Programmer' }

    $R.Timestamp  = $RunStamp.ToString('yyyy-MM-ddTHH:mm:ssK')
    $R.Station    = $env:COMPUTERNAME
    $R.Operator   = $Operator
    $R.RoleWanted = $Role

    # -- Power up --
    if (-not $DryRun) {
        $script:Otii = Start-Otii
        if ($script:Otii) {
            Set-OtiiOrNote ('volt {0}' -f $SupplyV.ToString([Globalization.CultureInfo]::InvariantCulture))
            Set-OtiiOrNote 'on'
            Start-Sleep -Seconds 1
            $R.SupplyV = $SupplyV
        }
    }
    $R.Otii = [bool]$script:Otii

    # -- 1. Program --
    if ($NoFlash -or $DryRun) {
        $R.Flashed = ''
    } else {
        $hex = Find-FlashHex
        if (-not $hex -or -not (Test-Path $hex)) { Stop-Run 'no firmware image - build first or pass -FlashHex' }
        Log "Programming $hex ..."
        Invoke-Programmer @('-c', 'port=SWD', 'mode=UR', '-w', $hex, '-v') | Out-Null
        $R.Flashed = Split-Path $hex -Leaf
    }

    # -- 2. Identify --
    $target  = Read-Target
    $uid     = $target.Uid
    $uidText = Format-Uid $uid
    $radioId = Get-RadioId $uid
    $idText  = '{0:X8}' -f $radioId
    $R.Uid = $uidText; $R.RadioId = $idText
    Log "UID      $uidText"
    Log "Radio id $idText  (legacy 16-bit id $('{0:X4}' -f ($radioId -band 0xFFFF)))"

    # The radio id assumes UID word 0 holds the die X/Y position in BCD
    # (RM0461). A hex digit above 9 there means the assumption is wrong for
    # this part, and the radio id is not what the firmware design intended.
    if (('{0:X8}' -f $uid[0]) -match '[A-F]') {
        Stop-Run ("UID word 0 ({0:X8}) is not BCD - the radio id layout in LoraRadio.c does not hold for this chip" -f $uid[0])
    }

    $otpCur = $target.Otp.Current
    if ($otpCur) { Log ("OTP      serial {0} in slot {1}, {2} slots free" -f (Format-Serial $otpCur.Serial), $otpCur.Slot, $target.Otp.Free) }
    else         { Log ("OTP      no serial, {0} slots free" -f $target.Otp.Free) }

    $db = @(); if (Test-Path $Database) { $db = @(Import-Csv $Database) }
    $runs = @(); if (Test-Path $TestLog) { $runs = @(Import-Csv $TestLog | Where-Object { $_.Uid -eq $uidText }) }
    $R.RunNumber = $runs.Count + 1
    if ($runs.Count -gt 0) { Log ("Unit seen before: this is production run {0}" -f $R.RunNumber) }

    $byChip = @($db | Where-Object { $_.Uid -eq $uidText })
    $byId   = @($db | Where-Object { $_.RadioId -eq $idText -and $_.Uid -ne $uidText })
    if ($byId.Count -gt 0) {
        Stop-Run (("radio id $idText already belongs to chip {0} (serial {1}). Two chips share an id: " +
                   "set this unit aside and report it - the id derivation needs attention.") -f $byId[0].Uid, $byId[0].Serial)
    }

    $dbSerials = @($db | ForEach-Object { Get-DbSerial $_ })
    if ($Serial -ne '') {
        $sn = ConvertFrom-SerialText $Serial
        if ($null -eq $sn) { Stop-Run "-Serial '$Serial' is not 8 hex digits (e.g. 31000001)" }
    } elseif ($byChip.Count -gt 0) {
        $sn = Get-DbSerial $byChip[-1]
    } elseif ($otpCur) {
        $sn = [long]$otpCur.Serial
    } else {
        # Next free unit number in this revision. The highest one issued plus
        # one, not the first gap: a gap may be a unit that failed after its
        # serial was written but before it reached the database.
        $family = ([long]$SN_HW_VERSION -shl 28) -bor ([long]$HwRevision -shl 24)
        $issued = @($dbSerials | Where-Object { ($_ -band $SN_FAMILY_MASK) -eq $family })
        if ($issued.Count -eq 0) {
            $sn = $family + 1
        } else {
            $top = [long]($issued | Measure-Object -Maximum).Maximum
            if (($top -band $SN_UNIT_MASK) -eq $SN_UNIT_MASK) {
                Stop-Run ("revision {0:X} has used all {1} unit numbers - the next units need a new -HwRevision" -f $HwRevision, $SN_UNIT_MASK)
            }
            $sn = [long]$top + 1
        }
    }
    $snText = Format-Serial $sn

    $bySerial = @($db | Where-Object { (Get-DbSerial $_) -eq $sn -and $_.Uid -ne $uidText })
    if ($bySerial.Count -gt 0) { Stop-Run ("serial $snText already belongs to chip {0} (radio id {1})" -f $bySerial[0].Uid, $bySerial[0].RadioId) }

    $needFix = $false
    if ($otpCur -and [long]$otpCur.Serial -ne $sn) {
        if (-not $Fix) { Stop-Run ("chip already holds serial {0}; to replace it with {1} re-run with -Fix (uses one of the {2} free OTP slots)" -f (Format-Serial $otpCur.Serial), $snText, $target.Otp.Free) }
        $needFix = $true
    }
    if ($byChip.Count -gt 0 -and (Get-DbSerial $byChip[-1]) -ne $sn -and -not $Fix) {
        Stop-Run ("database has this chip as serial {0}; pass -Fix to re-serialise it as {1}" -f $byChip[-1].Serial, $snText)
    }
    $needWrite = (-not $otpCur) -or $needFix
    if (-not (Test-SerialFormat $sn)) {
        # The firmware refuses to write such a serial, so stop before testing.
        # A unit that already holds one keeps it; it is only flagged.
        if ($needWrite) { Stop-Run "serial $snText is not in the 3Rnnnnnn format (version 3, revision 1-F, unit 000001-FFFFFF)" }
        Add-Note "serial $snText predates the 3Rnnnnnn format - re-serialise with -Serial <new> -Fix"
    }
    if ($needWrite -and $target.Otp.Free -eq 0) { Stop-Run 'no free OTP identity slot left on this chip' }
    $R.Serial = $snText
    Log "Serial   $snText$(if ($needFix) { '  (correction)' } elseif (-not $needWrite) { '  (already in OTP)' })"

    if ($DryRun) {
        Log 'DryRun: identity and database checks passed; nothing programmed, tested or written.' 'Yellow'
        $R.Result = 'DRYRUN'
        $exitCode = 0
        throw 'run done'
    }

    $script:Port = Open-Uart (Find-ComPort)
    Log "Debug UART on $($script:Port.PortName)"

    # -- 3. First-pass sleep current (secondary, Otii) --
    if ($Role -eq 'secondary' -and $script:Otii) {
        Reset-Target
        Wait-Boot
        Log "Waiting for the tag to enter production sleep (boot window $BOOT_WINDOW_S s) ..."
        if (Wait-Line 'ProductionSleep' ($BOOT_WINDOW_S + 15)) {
            Measure-Sleep 'SleepStart'
        } else {
            Add-Note 'tag did not go to production sleep by itself - it is marked in service; first-pass sleep current skipped'
        }
    }

    # -- 4. Session + production test --
    Reset-Target
    Wait-Boot
    $st = Connect-Kernel
    Log ("Tag      {0} {1} fw {2} bl {3} bat {4} mV sol {5} mV" -f $st.RadioId, $st.Role, $st.Fw, $st.Bl, $st.BatMv, $st.SolMv)
    $R.Role = $st.Role; $R.Fw = $st.Fw; $R.Bl = $st.Bl
    if ($st.Role -ne $Role) { Stop-Run "tag reports role '$($st.Role)' but the station is set up for '$Role' - check the board and -Role" }
    if ($st.RadioId -ne $idText) { Stop-Run "tag reports radio id $($st.RadioId), SWD derived $idText" }

    Send-Line ("u {0}" -f $HoldS)
    $pt = Wait-Line '^PT v1 ' 30
    if (-not $pt) { Stop-Run 'no production test result (kernel u) - firmware too old?' }
    $kv = @{}
    foreach ($m in [regex]::Matches($pt, '(\w+)=(\S+)')) { $kv[$m.Groups[1].Value] = $m.Groups[2].Value }
    foreach ($k in 'flash', 'acc', 'gps', 'radio', 'nf', 'bat', 'sol', 'flashdetail') {
        if ($kv.ContainsKey($k)) { $R["PT_$k"] = $kv[$k] }
    }
    foreach ($k in 'flash', 'acc', 'gps', 'radio') {
        if ($kv.ContainsKey($k) -and $kv[$k] -eq 'FAIL') {
            Add-Fail ("production test: {0} FAIL{1}" -f $k, $(if ($k -eq 'flash') { " ($($kv['flashdetail']))" } else { '' }))
        }
    }
    $ptPass = ($kv['result'] -eq 'PASS')
    if ($ptPass) { Log 'Production test checks PASS' 'Green' }

    if ($script:Otii -and $Role -eq 'secondary' -and $kv['bat'] -match '^\d+$') {
        $diff = [math]::Abs([int]$kv['bat'] - [int]($SupplyV * 1000))
        if ($diff -gt $BatToleranceMV) { Add-Fail ("battery reads {0} mV, Otii supplies {1} mV (tolerance {2})" -f $kv['bat'], [int]($SupplyV * 1000), $BatToleranceMV) }
    }

    if ($HoldS -gt 0) {
        if (-not (Wait-Line '^PT hold ' 10)) { Add-Fail 'production test did not enter its all-on hold' }
        elseif ($script:Otii) {
            Start-Sleep -Seconds $ALLON_SETTLE_S
            $ms = [math]::Max(1000, ($HoldS - $ALLON_SETTLE_S - 1) * 1000)
            $m = Measure-Current $ms
            if ($m) {
                $ma = [math]::Round($m.Avg * 1e3, 3)
                $R.AllOn_mA = $ma; $R.AllOnMax_mA = [math]::Round($m.Max * 1e3, 2)
                if ($ma -lt $AllOnMinMA -or $ma -gt $AllOnMaxMA) { Add-Fail ("all-on current {0} mA outside {1}..{2} mA" -f $ma, $AllOnMinMA, $AllOnMaxMA) }
                else { Log ("All-on current {0} mA (limits {1}..{2})" -f $ma, $AllOnMinMA, $AllOnMaxMA) 'Green' }
            }
        }
    }
    if (-not (Wait-Line '^PT done' ($HoldS + 15))) { Add-Fail 'production test did not finish' }

    # -- 5. Serial --
    if ($needWrite) {
        if (-not $ptPass) {
            Add-Fail 'serial not written: the production test failed'
        } else {
            $cmd = "N $snText $idText$(if ($needFix) { ' fix' })"
            Log "Writing serial: $cmd"
            Send-Line $cmd
            $reply = Wait-Line ('^N {0} {1}: ' -f $snText, $idText) 10
            if (-not $reply) { Add-Fail 'no reply to the N command' }
            elseif ($reply -notmatch ': (written|already set to this serial) ') { Add-Fail "the tag refused the serial: $reply" }
            else { Log $reply }
        }
    }
    Send-Line 'n'
    $nLine = Wait-Line '^serial: ' 5
    if (-not $nLine) { Add-Fail "no reply to 'n'" }
    elseif ($nLine -match '^serial: ([0-9A-F]{8}) ') { $R.SerialOnTag = $Matches[1] }
    else { $R.SerialOnTag = 'none' }

    # -- 6. Sleep and final-pass current --
    if ($Role -eq 'secondary') {
        Send-Line 'P'
        if (-not (Wait-Line 'production sleep - wakes' 5)) { Add-Fail "tag did not acknowledge 'P'" }
        if ($script:Otii) { Measure-Sleep 'SleepEnd' }
    } else {
        Send-Line 'x'
        [void](Wait-Line '^bye' 5)
    }

    # -- 7. Independent OTP check over SWD (inside the next boot window) --
    if ($R.Contains('SerialOnTag') -and $R.SerialOnTag -ne 'none') {
        Reset-Target
        Start-Sleep -Seconds 3
        $after = Read-Target
        $cur = $after.Otp.Current
        if (-not $cur)                    { Add-Fail 'OTP holds no valid identity record' }
        elseif ([long]$cur.Serial -ne $sn) { Add-Fail ("OTP holds serial {0}, expected {1}" -f (Format-Serial $cur.Serial), $snText) }
        elseif ($cur.RadioId -ne $radioId) { Add-Fail ("OTP record names radio id {0:X8}, expected {1}" -f $cur.RadioId, $idText) }
        else { Log ("OTP verified: slot {0}, serial {1}, {2} slots free" -f $cur.Slot, (Format-Serial $cur.Serial), $after.Otp.Free) 'Green' }

        $known = @($db | Where-Object { $_.Uid -eq $uidText -and (Get-DbSerial $_) -eq $sn }).Count -gt 0
        if ($cur -and [long]$cur.Serial -eq $sn -and -not $known) {
            [pscustomobject]@{
                Timestamp = $R.Timestamp; Serial = $snText; RadioId = $idText; Uid = $uidText
                OtpSlot = $cur.Slot; Role = $Role; Firmware = $R.Fw; Operator = $Operator
            } | Export-Csv -Path $Database -Append -NoTypeInformation
            Log "Unit recorded in $Database"
        }
    }

    if ($script:Fails.Count -eq 0) { $exitCode = 0 }
}
catch {
    if ($_.Exception.Message -notin @('run aborted', 'run done')) { Add-Fail ("script error: " + $_.Exception.Message) }
}
finally {
    if ($script:Port -and $script:Port.IsOpen) { $script:Port.Close() }
    if ($script:Otii) {
        [void](Invoke-Otii 'exit' 5000)
        if (-not $script:Otii.Proc.HasExited) { $script:Otii.Proc.Kill() }
    }

    if (-not $R.Contains('Result')) { $R.Result = $(if ($script:Fails.Count -eq 0) { 'PASS' } else { 'FAIL' }) }
    $R.Failures = ($script:Fails -join '; ')
    $R.Notes    = ($script:Notes -join '; ')
    $R.RunLog   = Split-Path $RunLog -Leaf

    if ($R.Result -ne 'DRYRUN') {
        # Fixed column set so rows from different runs line up whatever a run
        # managed to fill in.
        $cols = 'Timestamp', 'Result', 'Serial', 'SerialOnTag', 'RadioId', 'Uid', 'RunNumber', 'RoleWanted', 'Role',
                'Fw', 'Bl', 'Flashed', 'PT_flash', 'PT_flashdetail', 'PT_acc', 'PT_gps', 'PT_radio', 'PT_nf',
                'PT_bat', 'PT_sol', 'Otii', 'SupplyV', 'SleepStart_uA', 'SleepStartMax_uA', 'AllOn_mA',
                'AllOnMax_mA', 'SleepEnd_uA', 'SleepEndMax_uA', 'Failures', 'Notes', 'Station', 'Operator', 'RunLog'
        $row = [ordered]@{}
        foreach ($c in $cols) { $row[$c] = $(if ($R.Contains($c)) { $R[$c] } else { '' }) }
        [pscustomobject]$row | Export-Csv -Path $TestLog -Append -NoTypeInformation
    }

    Log ''
    if ($R.Result -eq 'PASS') {
        Log ("PASS   SN {0}    ID {1}" -f $R.Serial, $R.RadioId) 'Green'
    } elseif ($R.Result -eq 'FAIL') {
        Log ("FAIL   {0}" -f $R.Failures) 'Red'
    }
}
exit $exitCode
