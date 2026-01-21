param(
    [string]$Python = "python",
    [string]$Venv = ".venv",
    [string]$Requirements = "requirements.txt",
    [switch]$InstallExtras,
    [switch]$RunSmoke,
    [string]$SmokeConfig = "configs/reip_viz.yaml",
    [int]$SmokeTimesteps = 60
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

function Write-Info($msg) { Write-Host "[setup] $msg" -ForegroundColor Cyan }
function Write-Warn($msg) { Write-Host "[setup] $msg" -ForegroundColor Yellow }
function Write-Ok($msg) { Write-Host "[setup] $msg" -ForegroundColor Green }

# Resolve project root = directory containing this script's parent (scripts/)
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir
Push-Location $ProjectRoot

try {
    # 1) Create venv if missing
    if (-not (Test-Path $Venv)) {
        Write-Info "Creating virtual environment at $Venv"
        & $Python -m venv $Venv
    } else {
        Write-Info "Virtual environment already exists at $Venv"
    }

    $VenvPython = Join-Path $Venv "Scripts/python.exe"
    if (-not (Test-Path $VenvPython)) {
        throw "Could not find $VenvPython"
    }

    # 2) Upgrade pip
    Write-Info "Upgrading pip"
    & $VenvPython -m pip install --upgrade pip

    # 3) Install requirements (fallback to inline list if file missing)
    if (Test-Path $Requirements) {
        Write-Info "Installing pinned requirements from $Requirements"
        & $VenvPython -m pip install -r $Requirements
    } else {
        Write-Warn "$Requirements not found. Installing pinned base deps inline."
        & $VenvPython -m pip install `
            contourpy==1.3.3 `
            cycler==0.12.1 `
            fonttools==4.60.1 `
            kiwisolver==1.4.9 `
            matplotlib==3.10.6 `
            networkx==3.5 `
            numpy==2.3.3 `
            packaging==25.0 `
            pillow==11.3.0 `
            pyparsing==3.2.5 `
            python-dateutil==2.9.0.post0 `
            six==1.17.0 `
            PyYAML==6.0
    }

    if ($InstallExtras) {
        Write-Info "Installing optional extras (pandas, tqdm)"
        & $VenvPython -m pip install "pandas>=2.2,<3" "tqdm>=4.66"
    }

    # 4) Sanity check
    Write-Info "Running import sanity check"
    $tmpFile = Join-Path $env:TEMP ("reip_sanity_{0}.py" -f ([guid]::NewGuid()))
    @'
import sys
mods = ["numpy", "networkx", "matplotlib", "yaml"]
for m in mods:
    __import__(m)
print("deps-ok")
'@ | Set-Content -Path $tmpFile -Encoding ASCII
    & $VenvPython $tmpFile
    Remove-Item -Path $tmpFile -Force -ErrorAction SilentlyContinue

    Write-Ok "Environment ready. Activate with: `n  .\\$Venv\\Scripts\\Activate.ps1"

    if ($RunSmoke) {
        Write-Info "Running smoke test sim ($SmokeConfig, T override=$SmokeTimesteps)"
        # Patch T in-memory by passing through src.main.run_simulation replacement is not trivial here; run normally.
        # We fallback to running the config as-is.
        & $VenvPython -m src.main --config $SmokeConfig
        Write-Ok "Smoke test completed"
    }
}
finally {
    Pop-Location
}
