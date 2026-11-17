# provision_tag.ps1
#
# frtag3 production step: give one tag its serial number.
#
# What it does, in order, for the one tag on the ST-LINK and debug UART:
#
#   1. Optionally programs a firmware image (-FlashHex, normally the combined
#      frtag3_<M>_<m>_<p>_bl_<b>.hex from Debug\).
#   2. Reads the MCU's 96-bit unique ID and the OTP identity area over SWD.
#   3. Computes the tag's 32-bit radio id from the UID, with exactly the
#      formula the firmware uses (LORARADIO_u32GetUniqueId, LoraRadio.c).
#   4. Checks the production database (a CSV file):
#        - the radio id must not belong to a different chip. Radio ids are
#          unique within a wafer lot but only very probably unique across
#          lots; this check is what turns that into a guarantee.
#        - the serial must not belong to a different chip.
#        - a chip that is already in the database gets its recorded serial
#          again, so re-running the script on a unit is harmless.
#   5. Resets the tag, waits for it to boot, and sends the FrKernel command
#        N <serial> <radio id>
#      on the debug UART. The firmware refuses the write unless the radio id
#      is its own, so the wrong unit can never be stamped.
#   6. Reads OTP back over SWD and verifies the record independently of what
#      the firmware replied: magic, CRC (which folds in the UID) and serial.
#   7. Appends the unit to the database and prints the label text.
#
# The serial number is permanent. It goes into the STM32WL OTP area, which
# nothing can erase; a wrong serial can only be superseded (-Fix), and each
# chip has 32 slots for that in total. See fr_app/settings/Identity_Config.h.
#
# Requirements
#   - STM32CubeProgrammer (STM32_Programmer_CLI.exe) on PATH or in its default
#     install folder.
#   - The firmware on the tag must be built with FRKERNEL_INTERFACE_UART
#     enabled in fr_app/inc/config/build_config.h. Without it the tag ignores
#     the debug UART and step 5 times out; the script says so.
#   - Read-out protection must still be at level 0: at level 1 SWD cannot read
#     the UID or OTP. Provision before setting RDP.
#
# Serial numbers
#   -Serial gives the number explicitly. Without it the script allocates the
#   next one: the highest serial in the database plus one, or -FirstSerial
#   when the database is empty. The digit scheme is yours; the firmware only
#   requires 1..4294967294.
#
# Examples
#   .\provision_tag.ps1 -ComPort COM7 -FirstSerial 2611000001 -DryRun
#   .\provision_tag.ps1 -ComPort COM7 -FlashHex ..\Debug\frtag3_3_1_1_bl_2.hex
#   .\provision_tag.ps1 -ComPort COM7 -Serial 2611000123 -Fix
#
# Exit code 0 on success (including "already provisioned with this serial"),
# 1 on any refusal or failure. Nothing is written to OTP or the database
# before every check has passed, and -DryRun stops before anything is written
# at all.

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string] $ComPort,

    [uint32] $Serial = 0,
    [uint32] $FirstSerial = 0,
    [string] $Database = (Join-Path $PSScriptRoot 'provisioned_units.csv'),
    [string] $FlashHex = '',
    [switch] $Fix,
    [switch] $DryRun,
    [int]    $Baud = 115200,
    [int]    $BootTimeoutS = 45,
    [string] $Operator = $env:USERNAME,
    [string] $Programmer = ''
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
# Note for anyone editing: in Windows PowerShell a hex literal of 0x80000000
# or more is a NEGATIVE [int] (0xFFFFFFFF is -1). Every such constant below
# carries the L suffix to make it a positive [long].

# ---- Helpers --------------------------------------------------------------

function Fail([string] $msg) {
    Write-Host "FAIL: $msg" -ForegroundColor Red
    exit 1
}

function Find-Programmer {
    if ($Programmer -ne '') { return $Programmer }
    $cmd = Get-Command 'STM32_Programmer_CLI.exe' -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    $default = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    if (Test-Path $default) { return $default }
    Fail 'STM32_Programmer_CLI.exe not found - install STM32CubeProgrammer or pass -Programmer'
}

function Invoke-Programmer([string[]] $cliArgs) {
    $out = & $script:Cli @cliArgs 2>&1 | Out-String
    if ($LASTEXITCODE -ne 0) {
        Write-Host $out
        Fail "STM32_Programmer_CLI $($cliArgs -join ' ') exited with $LASTEXITCODE"
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

# Decode the OTP identity slots. Returns the current record (highest valid
# slot) or $null, plus how many slots are still free.
function Get-OtpIdentity($words, [uint32[]] $uid) {
    $current = $null
    $lastUsed = -1
    for ($s = 0; $s -lt $OTP_ID_SLOTS; $s++) {
        $base = $OTP_ID_ADDR + 16 * $s
        $w = @(0..3 | ForEach-Object { $words[[long]($base + 4 * $_)] })
        if ($w -contains $null) { Fail "OTP dump is missing slot $s" }
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

function Read-Target {
    $dump = Invoke-Programmer @('-c', 'port=SWD', 'mode=HOTPLUG',
                                '-r32', ('0x{0:X8}' -f $UID_ADDR), '12',
                                '-r32', ('0x{0:X8}' -f $OTP_ID_ADDR), ($OTP_ID_SLOTS * 16).ToString())
    $words = Read-Words $dump
    $uid = @(0..2 | ForEach-Object { $words[[long]($UID_ADDR + 4 * $_)] })
    if ($uid -contains $null) { Write-Host $dump; Fail 'could not parse the UID from the programmer output' }
    return [pscustomobject]@{ Uid = [uint32[]]$uid; Otp = (Get-OtpIdentity $words ([uint32[]]$uid)) }
}

function Format-Uid([uint32[]] $uid) { '{0:X8}{1:X8}{2:X8}' -f $uid[2], $uid[1], $uid[0] }

# ---- 1. Optional programming ----------------------------------------------

$script:Cli = Find-Programmer

if ($FlashHex -ne '') {
    if (-not (Test-Path $FlashHex)) { Fail "firmware image not found: $FlashHex" }
    if ($DryRun) {
        Write-Host "DryRun: would program $FlashHex"
    } else {
        Write-Host "Programming $FlashHex ..."
        Invoke-Programmer @('-c', 'port=SWD', 'mode=UR', '-w', $FlashHex, '-v', '-rst') | Out-Null
    }
}

# ---- 2./3. Read the chip ---------------------------------------------------

$target  = Read-Target
$uid     = $target.Uid
$uidText = Format-Uid $uid
$radioId = Get-RadioId $uid
$idText  = '{0:X8}' -f $radioId

Write-Host "UID      $uidText"
Write-Host "Radio id $idText  (legacy 16-bit id $('{0:X4}' -f ($radioId -band 0xFFFF)))"

# The radio id assumes UID word 0 holds the die X/Y position in BCD (RM0461).
# A hex digit above 9 there means that assumption is wrong for this part and
# the id is not what the firmware design intended - stop and look.
if (('{0:X8}' -f $uid[0]) -match '[A-F]') {
    Fail ("UID word 0 ({0:X8}) is not BCD - the radio id layout in LoraRadio.c does not hold for this chip" -f $uid[0])
}

$otpCur = $target.Otp.Current
if ($otpCur) {
    Write-Host ("OTP      serial {0} in slot {1}, {2} slots free" -f $otpCur.Serial, $otpCur.Slot, $target.Otp.Free)
} else {
    Write-Host ("OTP      no serial, {0} slots free" -f $target.Otp.Free)
}

# ---- 4. Database checks ----------------------------------------------------

$db = @()
if (Test-Path $Database) { $db = @(Import-Csv $Database) }

$byChip  = @($db | Where-Object { $_.Uid -eq $uidText })
$byId    = @($db | Where-Object { $_.RadioId -eq $idText -and $_.Uid -ne $uidText })
if ($byId.Count -gt 0) {
    Fail (("radio id $idText already belongs to chip {0} (serial {1}). Two chips share an id: " +
           "set this unit aside and report it - the id derivation needs attention.") -f $byId[0].Uid, $byId[0].Serial)
}

# Pick the serial.
if ($Serial -eq 0) {
    if ($byChip.Count -gt 0)  { $Serial = [uint32]$byChip[-1].Serial }
    elseif ($otpCur)          { $Serial = $otpCur.Serial }
    elseif ($db.Count -gt 0)  { $Serial = [uint32](($db | ForEach-Object { [uint32]$_.Serial } | Measure-Object -Maximum).Maximum + 1) }
    elseif ($FirstSerial -ne 0) { $Serial = $FirstSerial }
    else { Fail 'database is empty: pass -Serial or -FirstSerial for the first unit' }
}
if (($Serial -lt 1) -or ($Serial -gt 0xFFFFFFFEL)) { Fail "serial $Serial is out of range 1..4294967294" }

$bySerial = @($db | Where-Object { [uint32]$_.Serial -eq $Serial -and $_.Uid -ne $uidText })
if ($bySerial.Count -gt 0) { Fail ("serial $Serial already belongs to chip {0} (radio id {1})" -f $bySerial[0].Uid, $bySerial[0].RadioId) }

$needFix = $false
if ($otpCur -and $otpCur.Serial -ne $Serial) {
    if (-not $Fix) { Fail ("chip already holds serial {0}; to replace it with {1} re-run with -Fix (uses one of the {2} free OTP slots)" -f $otpCur.Serial, $Serial, $target.Otp.Free) }
    $needFix = $true
}
if ($byChip.Count -gt 0 -and [uint32]$byChip[-1].Serial -ne $Serial -and -not $Fix) {
    Fail ("database has this chip as serial {0}; pass -Fix to re-serialise it as {1}" -f $byChip[-1].Serial, $Serial)
}
if (-not $otpCur -or $needFix) {
    if ($target.Otp.Free -eq 0) { Fail 'no free OTP identity slot left on this chip' }
}

Write-Host "Serial   $Serial$(if ($needFix) { '  (correction)' })"

if ($DryRun) {
    Write-Host 'DryRun: all checks passed, nothing written.' -ForegroundColor Yellow
    exit 0
}

# ---- 5. Write through the firmware ------------------------------------------

if ($otpCur -and -not $needFix) {
    Write-Host 'OTP already holds this serial - skipping the write.'
} else {
    $port = New-Object System.IO.Ports.SerialPort $ComPort, $Baud, 'None', 8, 'One'
    $port.NewLine     = "`r`n"
    $port.ReadTimeout = 500
    try {
        $port.Open()
        $port.DiscardInBuffer()

        # Reset the tag so it boots into the application, and wait for the
        # identity line every boot logs (IDENTITY_vInit). Boot can take
        # seconds: the LSE start alone is allowed that long.
        Invoke-Programmer @('-c', 'port=SWD', 'mode=HOTPLUG', '-rst') | Out-Null

        function Wait-Line([string] $pattern, [int] $timeoutS) {
            $deadline = (Get-Date).AddSeconds($timeoutS)
            while ((Get-Date) -lt $deadline) {
                try { $line = $port.ReadLine() } catch [TimeoutException] { continue }
                Write-Verbose "UART< $line"
                if ($line -match $pattern) { return $line }
            }
            return $null
        }

        if (-not (Wait-Line '^.*Identity: ' $BootTimeoutS)) {
            Fail "no boot output on $ComPort within $BootTimeoutS s - check the UART cable and port"
        }
        Start-Sleep -Seconds 2   # let the kernel task reach its loop

        # Probe first, so a build without the UART kernel fails clearly.
        $port.Write("n`r")
        if (-not (Wait-Line '^serial: ' 5)) {
            Fail 'the tag does not answer kernel commands on the debug UART - build the firmware with FRKERNEL_INTERFACE_UART enabled'
        }

        $cmd = "N $Serial $idText$(if ($needFix) { ' fix' })"
        Write-Host "UART>    $cmd"
        $port.Write("$cmd`r")
        $reply = Wait-Line ('^N {0} {1}: ' -f $Serial, $idText) 10
        if (-not $reply) { Fail 'no reply to the N command' }
        Write-Host "UART<    $reply"
        if ($reply -notmatch ': (written|already set to this serial) ') { Fail "the tag refused: $reply" }

        $port.Write("x`r")   # end the kernel session; the tag resumes its schedule
    }
    finally {
        if ($port.IsOpen) { $port.Close() }
    }
}

# ---- 6. Independent verification over SWD ---------------------------------

$after = Read-Target
$cur   = $after.Otp.Current
if (-not $cur)                 { Fail 'OTP holds no valid identity record after the write' }
if ($cur.Serial -ne $Serial)   { Fail ("OTP holds serial {0}, expected {1}" -f $cur.Serial, $Serial) }
if ($cur.RadioId -ne $radioId) { Fail ("OTP record names radio id {0:X8}, expected {1}" -f $cur.RadioId, $idText) }
Write-Host ("Verified OTP slot {0}: serial {1}, radio id {2}, {3} slots free" -f $cur.Slot, $cur.Serial, $idText, $after.Otp.Free) -ForegroundColor Green

# ---- 7. Record and label ----------------------------------------------------

$known = @($db | Where-Object { $_.Uid -eq $uidText -and [uint32]$_.Serial -eq $Serial }).Count -gt 0
if (-not $known) {
    [pscustomobject]@{
        Timestamp = (Get-Date).ToString('yyyy-MM-ddTHH:mm:ssK')
        Serial    = $Serial
        RadioId   = $idText
        Uid       = $uidText
        OtpSlot   = $cur.Slot
        Firmware  = $(if ($FlashHex -ne '') { Split-Path $FlashHex -Leaf } else { '' })
        Operator  = $Operator
    } | Export-Csv -Path $Database -Append -NoTypeInformation
    Write-Host "Recorded in $Database"
}

Write-Host ''
Write-Host "LABEL    SN $Serial    ID $idText" -ForegroundColor Cyan
exit 0
