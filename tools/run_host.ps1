<#
.SYNOPSIS
    Launch the OrcaSlicer nightly plugin host from this repository, always isolated.

.DESCRIPTION
    Any regular OrcaSlicer installation and its user data should stay
    untouched. This script always passes an explicit --datadir inside this
    repository (host\data), so the host never attaches to another
    installation's configuration.

    It refuses to run if the data directory would resolve outside the repo.

.PARAMETER Fixture
    Optional model to open on launch. Defaults to the 20x20x60 mm coupon fixture.

.PARAMETER Short
    Open the 15 mm gap-sweep coupon instead of the 60 mm reference coupon.

.PARAMETER Airfoil
    Open the constant-section airfoil fixture instead of the coupon.

.PARAMETER NoFixture
    Launch with an empty plate.
#>
[CmdletBinding()]
param(
    [string] $Fixture,
    [switch] $Short,
    [switch] $Airfoil,
    [switch] $NoFixture
)

$ErrorActionPreference = 'Stop'

$repo    = Split-Path -Parent $PSScriptRoot
$exe     = Join-Path $repo 'host\orca-nightly\OrcaSlicer\orca-slicer.exe'
$dataDir = Join-Path $repo 'host\data'

if (-not (Test-Path $exe)) {
    throw "Host executable not found: $exe`nSee INSTALL.md for how to obtain the host."
}

New-Item -ItemType Directory -Force -Path $dataDir | Out-Null
$resolvedData = (Resolve-Path $dataDir).Path
$resolvedRepo = (Resolve-Path $repo).Path
if (-not $resolvedData.StartsWith($resolvedRepo, [StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to launch: data directory $resolvedData is outside the repository."
}

if (-not $NoFixture -and -not $Fixture) {
    $Fixture = if ($Airfoil) {
        Join-Path $repo 'tests\fixtures\airfoil\naca2412_c60_s60.stl'
    } elseif ($Short) {
        Join-Path $repo 'tests\fixtures\coupon\coupon_20x20x15.stl'
    } else {
        Join-Path $repo 'tests\fixtures\coupon\coupon_20x20x60.stl'
    }
}

$arguments = @("--datadir=$resolvedData")
if (-not $NoFixture) {
    if (-not (Test-Path $Fixture)) { throw "Fixture not found: $Fixture" }
    $arguments += (Resolve-Path $Fixture).Path
}

# The Plugins dialog installs a local .py into a folder named after the file.
$marker = Join-Path $resolvedData 'orca_plugins\aircraft_structures.py\m1_enabled'
$armed  = Test-Path $marker

# WebView2 derives its user-data folder from the application identity, NOT from
# --datadir. The stable OrcaSlicer install therefore shares that folder with this
# nightly, and whichever process starts second fails to create its WebView2
# environment ("the group or resource is not in the correct state") — which shows
# up as a completely blank config wizard and blank Plugins window.
# Give this host its own folder inside the repo so the two never contend.
$webView2Dir = Join-Path $resolvedData 'EBWebView'
New-Item -ItemType Directory -Force -Path $webView2Dir | Out-Null
$env:WEBVIEW2_USER_DATA_FOLDER = $webView2Dir
Write-Host "WebView2: $webView2Dir"

Write-Host "Host:     $exe"
Write-Host "Data dir: $resolvedData"
Write-Host "Probe:    $(if ($armed) { 'ARMED (m1_enabled present)' } else { 'not armed - slices will be untouched' })"
if (-not $NoFixture) { Write-Host "Fixture:  $Fixture" }

$proc = Start-Process -FilePath $exe -ArgumentList $arguments -PassThru
Write-Host "Launched PID $($proc.Id). Stop only this PID when finished:"
Write-Host "    Stop-Process -Id $($proc.Id)"
