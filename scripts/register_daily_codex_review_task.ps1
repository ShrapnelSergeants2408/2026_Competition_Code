param(
  [string]$RepoRoot = (Resolve-Path ".").Path,
  [string]$TaskName = "CodexDailyReview_2026_Competition_Code",
  [string]$RunTime = "05:00"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$scriptPath = Join-Path $RepoRoot "scripts\\daily_codex_review.ps1"
if (-not (Test-Path $scriptPath)) {
  throw "Missing review script at $scriptPath"
}

$escapedScriptPath = $scriptPath.Replace("'", "''")
$escapedRepoRoot = $RepoRoot.Replace("'", "''")
$inlineCommand = "`$path='$escapedScriptPath'; & ([scriptblock]::Create((Get-Content -Raw -Path `$path))) -RepoRoot '$escapedRepoRoot'"
$action = New-ScheduledTaskAction -Execute "powershell.exe" -Argument "-NoProfile -ExecutionPolicy Bypass -Command `"$inlineCommand`"" -WorkingDirectory $RepoRoot
$trigger = New-ScheduledTaskTrigger -Daily -At $RunTime
$principal = New-ScheduledTaskPrincipal -UserId $env:USERNAME -LogonType Interactive -RunLevel Limited

Register-ScheduledTask -TaskName $TaskName -Action $action -Trigger $trigger -Principal $principal -Force | Out-Null
