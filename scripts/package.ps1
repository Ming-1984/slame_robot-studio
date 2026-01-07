param(
  [ValidateSet('Debug','Release')]
  [string]$Configuration = 'Release',

  [string]$Runtime = 'win-x64',

  [string]$OutDir,

  [switch]$NoZip,

  [switch]$SkipWebView2Download
)

$ErrorActionPreference = 'Stop'

function Assert-Command([string]$Name) {
  if (-not (Get-Command $Name -ErrorAction SilentlyContinue)) {
    throw "Required command not found in PATH: $Name"
  }
}

$root = Resolve-Path (Join-Path $PSScriptRoot '..')

$project = Join-Path $root 'windows_control_center\RobotStudioControlCenter\RobotStudioControlCenter.csproj'
if (-not (Test-Path $project)) {
  throw "Project not found: $project"
}

$auroraSrc = Join-Path $root 'aurora_app'
if (-not (Test-Path $auroraSrc)) {
  throw "Aurora app folder not found: $auroraSrc"
}

if (-not $OutDir) {
  $OutDir = Join-Path $root 'dist\RobotStudio'
}

Write-Host "Root: $root"
Write-Host "OutDir: $OutDir"

if (Test-Path $OutDir) {
  Write-Host "Cleaning: $OutDir"
  Remove-Item -Recurse -Force $OutDir
}

Assert-Command dotnet
Write-Host "Publishing Robot Studio..."
dotnet publish $project `
  -c $Configuration `
  -r $Runtime `
  --self-contained true `
  -p:DebugType=None `
  -p:DebugSymbols=false `
  -o $OutDir

Write-Host "Copying aurora_app..."
$auroraDst = Join-Path $OutDir 'aurora_app'
Assert-Command robocopy
New-Item -ItemType Directory -Force -Path $auroraDst | Out-Null
$roboArgs = @(
  '/E'
  '/NFL'
  '/NDL'
  '/NJH'
  '/NJS'
  '/NP'
  '/R:1'
  '/W:1'
  '/XF'
  'TODO_*.md'
)
& robocopy $auroraSrc $auroraDst @roboArgs | Out-Null
$code = $LASTEXITCODE
# Robocopy: 0-7 are success codes; >=8 are failures.
if ($code -ge 8) {
  throw "robocopy failed with exit code $code"
}

Write-Host "Preparing installers..."
$installersDir = Join-Path $OutDir 'installers'
New-Item -ItemType Directory -Force -Path $installersDir | Out-Null

$vcRedistSrc = Join-Path $auroraSrc 'VC_redist.x64.exe'
if (Test-Path $vcRedistSrc) {
  Copy-Item -Force $vcRedistSrc (Join-Path $installersDir 'VC_redist.x64.exe')
}

$webView2Installer = Join-Path $installersDir 'MicrosoftEdgeWebView2RuntimeInstallerX64.exe'
if (-not $SkipWebView2Download) {
  $cacheDir = Join-Path $root 'dist\.cache\installers'
  New-Item -ItemType Directory -Force -Path $cacheDir | Out-Null
  $webView2Cache = Join-Path $cacheDir 'MicrosoftEdgeWebView2RuntimeInstallerX64.exe'

  if (-not (Test-Path $webView2Cache)) {
    Assert-Command curl.exe
    Write-Host "Downloading WebView2 Runtime installer (x64) to cache..."
    & curl.exe -L -o $webView2Cache 'https://go.microsoft.com/fwlink/p/?LinkId=2124701'
  }

  Copy-Item -Force $webView2Cache $webView2Installer
}

if (-not $NoZip) {
  $zipPath = Join-Path (Split-Path -Parent $OutDir) 'RobotStudio.zip'
  if (Test-Path $zipPath) {
    Remove-Item -Force $zipPath
  }

  Write-Host "Creating zip: $zipPath (this may take a while)"
  Compress-Archive -Path (Join-Path $OutDir '*') -DestinationPath $zipPath -Force
}

Write-Host "Done."
Write-Host "Run: `"$OutDir\\RobotStudio.exe`""
