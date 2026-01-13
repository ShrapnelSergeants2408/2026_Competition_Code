param(
  [string]$RepoRoot = (Resolve-Path ".").Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$dateStamp = Get-Date -Format "yyyy-MM-dd"
$logFile = Join-Path $RepoRoot "logs\\daily\\$dateStamp.md"

if (-not (Test-Path $logFile)) {
  $reviewPath = Join-Path $RepoRoot "scripts\\daily_codex_review.ps1"
  $reviewScript = Get-Content -Raw -Path $reviewPath
  & ([scriptblock]::Create($reviewScript)) -RepoRoot $RepoRoot
}
