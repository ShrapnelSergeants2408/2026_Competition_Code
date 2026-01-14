param(
  [string]$RepoRoot = (Resolve-Path ".").Path,
  [string]$CodexPath = $env:CODEX_PATH
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$outputDir = Join-Path $RepoRoot "logs\\daily"
$dateStamp = Get-Date -Format "yyyy-MM-dd"
$timeStamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
$logFile = Join-Path $outputDir "$dateStamp.md"

New-Item -ItemType Directory -Force -Path $outputDir | Out-Null

$repoUrl = ""
try {
  $repoUrl = git -C $RepoRoot remote get-url origin
} catch {
  $repoUrl = "(unknown)"
}

$codexExe = $null
if ($CodexPath -and (Test-Path $CodexPath)) {
  $codexExe = $CodexPath
} else {
  $codexCmd = Get-Command codex -ErrorAction SilentlyContinue
  if ($codexCmd) {
    $codexExe = $codexCmd.Path
  } else {
    $fallback = Get-ChildItem -Path "C:\\Users\\Public\\wpilib\\2026\\vscode\\data\\extensions\\openai.chatgpt-*\\bin\\windows-x86_64\\codex.exe" -ErrorAction SilentlyContinue |
      Sort-Object FullName -Descending |
      Select-Object -First 1 -ExpandProperty FullName
    if ($fallback) {
      $codexExe = $fallback
    }
  }
}

@"
# Daily Codex Review ($dateStamp)
Generated: $timeStamp
Repo: $repoUrl
"@ | Set-Content -Path $logFile -Encoding ASCII

if (-not $codexExe) {
  Add-Content -Path $logFile -Encoding ASCII -Value "`n### Summary`n- Codex run failed: codex.exe not found. Set CODEX_PATH or add codex to PATH.`n### Suggestions`n- No suggestions."
  exit 0
}

git -C $RepoRoot fetch --all --prune --tags | Out-Null

$branches = git -C $RepoRoot for-each-ref refs/remotes/origin --format="%(refname:short)" |
  Where-Object { $_ -and $_ -like "origin/*" -and $_ -ne "origin/HEAD" } |
  Sort-Object

if (-not $branches) {
  Add-Content -Path $logFile -Encoding ASCII -Value "`n_No remote branches found under origin._"
  exit 0
}

foreach ($branch in $branches) {
  Add-Content -Path $logFile -Encoding ASCII -Value "`n## $branch`n"

  $safeBranch = ($branch -replace "[^A-Za-z0-9._-]", "_")
  $worktreePath = Join-Path $env:TEMP ("codex_review_{0}_{1}" -f $safeBranch, $dateStamp)
  $outputFile = Join-Path $env:TEMP ("codex_review_output_{0}.txt" -f $safeBranch)

  if (Test-Path $worktreePath) {
    Remove-Item -Recurse -Force $worktreePath
  }
  if (Test-Path $outputFile) {
    Remove-Item -Force $outputFile
  }

  git -C $RepoRoot worktree add --detach $worktreePath $branch | Out-Null

  try {
    if ($branch -ne "origin/main") {
      $diffCount = git -C $RepoRoot diff --name-only origin/main...$branch -- src | Measure-Object | Select-Object -ExpandProperty Count
      if ($diffCount -eq 0) {
        Add-Content -Path $logFile -Encoding ASCII -Value "### Summary`n- No differences in src/ compared to origin/main.`n### Suggestions`n- No suggestions."
        continue
      }
    }

    if ($branch -eq "origin/main") {
      $prompt = @"
You are generating a branch summary and improvement suggestions.
Only read files under src/.
Output markdown exactly with:
### Summary
- ...
### Suggestions
- ...
Keep it concise. Use ASCII only and '-' bullets.
If src/ is missing or empty, say so in Summary and put "No suggestions." in Suggestions.
Do not propose code changes or diffs.
"@
    } else {
      $prompt = @"
You are generating a branch summary and improvement suggestions.
Only consider differences in src/ between this branch (HEAD) and origin/main.
Use git diff origin/main...HEAD -- src to identify changes.
Output markdown exactly with:
### Summary
- ...
### Suggestions
- ...
Keep it concise. Use ASCII only and '-' bullets.
If there are no src/ differences vs origin/main, say so in Summary and put "No suggestions." in Suggestions.
Do not propose code changes or diffs.
"@
    }

    $null = $prompt | & $codexExe -a never -s read-only exec -C $worktreePath --output-last-message $outputFile -

    if (Test-Path $outputFile) {
      $content = Get-Content -Path $outputFile -Raw
      if ([string]::IsNullOrWhiteSpace($content)) {
        $content = "### Summary`n- No output from codex.`n### Suggestions`n- No suggestions."
      } elseif ($content -notmatch "### Summary" -or $content -notmatch "### Suggestions") {
        $content = "### Summary`n- Codex output was malformed.`n### Suggestions`n- No suggestions."
      }
    } else {
      $content = "### Summary`n- No output file from codex.`n### Suggestions`n- No suggestions."
    }

    Add-Content -Path $logFile -Encoding ASCII -Value $content
  } catch {
    $message = $_.Exception.Message
    Add-Content -Path $logFile -Encoding ASCII -Value "### Summary`n- Codex run failed: $message`n### Suggestions`n- No suggestions."
  } finally {
    git -C $RepoRoot worktree remove $worktreePath --force | Out-Null
    if (Test-Path $worktreePath) {
      Remove-Item -Recurse -Force $worktreePath
    }
    if (Test-Path $outputFile) {
      Remove-Item -Force $outputFile
    }
  }
}
