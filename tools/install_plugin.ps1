<#
.SYNOPSIS
    Updates an EXISTING installation of the aircraft_structures plugin from its
    source-of-truth location in this repository.

.DESCRIPTION
    Copies plugin\aircraft_structures\aircraft_structures.py over the installed
    copy at <DataDir>\orca_plugins\aircraft_structures.py\aircraft_structures.py.

    This script does NOT perform a first installation. The host only loads a
    plugin that was installed through the OrcaSlicer Plugins window; a folder
    copied into orca_plugins by hand is discovered but never loaded. Install
    the plugin once through the Plugins window (which creates the plugin folder
    and its .install_state.json), then use this script to refresh it after
    rebuilding. If the plugin folder or its .install_state.json is missing, the
    script exits with an error and creates nothing.

    Prints the SHA-256 of both the source and destination files so the
    installed copy can be proven identical to the source of truth.

    Refuses to run if the resolved destination path would fall outside the
    repository root (a safety guard against a misconfigured -DataDir).

.PARAMETER RepoRoot
    Root of the aircraft-orca-structures repository. Defaults to the parent
    of the directory containing this script (tools\..).

.PARAMETER DataDir
    OrcaSlicer data directory holding the existing installation. Defaults to
    $RepoRoot\host\data (a portable host started with --datadir pointing
    there, e.g. through tools\run_host.ps1). It never defaults to the user's
    %APPDATA% or %LOCALAPPDATA% OrcaSlicer folders.
#>

[CmdletBinding()]
param(
    [string]$RepoRoot = $null,
    [string]$DataDir = $null
)

$ErrorActionPreference = "Stop"

if (-not $PSScriptRoot) {
    $ScriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
} else {
    $ScriptRoot = $PSScriptRoot
}

if (-not $RepoRoot) {
    $RepoRoot = Split-Path -Parent $ScriptRoot
}

$RepoRoot = (Resolve-Path -LiteralPath $RepoRoot).ProviderPath

if (-not $DataDir) {
    $DataDir = Join-Path $RepoRoot "host\data"
}

$SourcePath = Join-Path $RepoRoot "plugin\aircraft_structures\aircraft_structures.py"

if (-not (Test-Path -LiteralPath $SourcePath -PathType Leaf)) {
    throw "Plugin source not found: $SourcePath"
}
$SourcePath = (Resolve-Path -LiteralPath $SourcePath).ProviderPath

# Resolve the destination directory without requiring it to exist yet.
function Resolve-PathNoExist {
    param([string]$Path)
    if (-not [System.IO.Path]::IsPathRooted($Path)) {
        $Path = Join-Path (Get-Location).ProviderPath $Path
    }
    return [System.IO.Path]::GetFullPath($Path)
}

$DataDir = Resolve-PathNoExist $DataDir
# The Plugins dialog installs a local .py plugin into a folder named after the
# FILE, so the real install directory is "aircraft_structures.py", not
# "aircraft_structures". This script only refreshes that existing install; first
# installation must go through the Plugins window.
$DestDir = Resolve-PathNoExist (Join-Path $DataDir "orca_plugins\aircraft_structures.py")
$DestPath = Join-Path $DestDir "aircraft_structures.py"

# Safety guard: refuse to write outside the repository root.
$RepoRootNorm = $RepoRoot.TrimEnd('\') + '\'
if (-not $DestPath.StartsWith($RepoRootNorm, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to install: destination '$DestPath' is outside repo root '$RepoRoot'"
}

# Refuse to create anything: only an installation made through the Plugins
# window (plugin folder plus .install_state.json) may be updated.
$StatePath = Join-Path $DestDir ".install_state.json"
if (-not (Test-Path -LiteralPath $DestDir -PathType Container) -or
    -not (Test-Path -LiteralPath $StatePath -PathType Leaf)) {
    throw ("No existing installation found at '$DestDir' (the plugin folder or " +
           "its .install_state.json is missing). This script only updates an " +
           "installed plugin. Install it first through the OrcaSlicer Plugins " +
           "window, then re-run this script to refresh it. Nothing was changed.")
}

Copy-Item -LiteralPath $SourcePath -Destination $DestPath -Force

$SourceHash = (Get-FileHash -LiteralPath $SourcePath -Algorithm SHA256).Hash
$DestHash = (Get-FileHash -LiteralPath $DestPath -Algorithm SHA256).Hash

Write-Host "Source:      $SourcePath"
Write-Host "Source SHA256:      $SourceHash"
Write-Host "Destination: $DestPath"
Write-Host "Destination SHA256: $DestHash"

if ($SourceHash -ne $DestHash) {
    throw "Hash mismatch after install: installed copy does not match source of truth"
}

Write-Host "OK: installed copy matches source of truth."
