# Regenerates js/build-info.js GIT_COMMIT from the current HEAD short SHA.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $root

$sha = (git rev-parse --short HEAD).Trim()
if (-not $sha) {
  throw 'Failed to read git short SHA.'
}

$outPath = Join-Path $root 'js\build-info.js'
$version = '1.0.0'
if (Test-Path $outPath) {
  $existing = Get-Content -Raw $outPath
  if ($existing -match "APP_VERSION\s*=\s*'([^']+)'") {
    $version = $Matches[1]
  }
}

$content = @"
/** App version and git commit — update commit via scripts/update-build-info.ps1 */
export const APP_VERSION = '$version';
export const GIT_COMMIT = '$sha';
"@

Set-Content -Path $outPath -Value $content -NoNewline
Write-Host "Wrote $outPath (APP_VERSION=$version, GIT_COMMIT=$sha)"
