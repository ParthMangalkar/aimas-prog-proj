[CmdletBinding()]
param(
    [string]$CppClient = "",
    [string]$ServerExe = "",
    [string[]]$Algorithms = @("-smart-wastar 3"),
    [string[]]$LevelRoots = @("levels", "new_comp_levels"),
    [string]$LevelList = "",
    [int]$TimeoutSeconds = 180,
    [int]$MaxJointActions = 20000,
    [int]$Limit = 0,
    [string]$OutputName = "all-levels-latest"
)

$ErrorActionPreference = "Stop"

function Resolve-RepoPath {
    param(
        [string]$RepoRoot,
        [string]$Path
    )

    if ([System.IO.Path]::IsPathRooted($Path)) {
        return (Resolve-Path $Path).Path
    }
    return (Resolve-Path (Join-Path $RepoRoot $Path)).Path
}

function Resolve-CppClientPath {
    param([string]$RepoRoot, [string]$Provided)

    if ($Provided) {
        return (Resolve-RepoPath -RepoRoot $RepoRoot -Path $Provided)
    }

    $candidates = @(
        "searchclient_cpp\build\searchclient_cpp\Release\searchclient_cpp.exe",
        "searchclient_cpp\build\searchclient_cpp\Debug\searchclient_cpp.exe",
        "direct-tests\bin\searchclient_cpp.exe"
    )
    foreach ($candidate in $candidates) {
        $path = Join-Path $RepoRoot $candidate
        if (Test-Path $path) {
            return (Resolve-Path $path).Path
        }
    }
    throw "Could not find searchclient_cpp.exe. Pass -CppClient explicitly."
}

function Resolve-ServerPath {
    param([string]$RepoRoot, [string]$Provided)

    if ($Provided) {
        return (Resolve-RepoPath -RepoRoot $RepoRoot -Path $Provided)
    }

    $candidates = @(
        "searchclient_cpp\build\server_cpp\Release\server_cpp.exe",
        "searchclient_cpp\build\server_cpp\Debug\server_cpp.exe",
        "direct-tests\bin\server_cpp.exe"
    )
    foreach ($candidate in $candidates) {
        $path = Join-Path $RepoRoot $candidate
        if (Test-Path $path) {
            return (Resolve-Path $path).Path
        }
    }
    throw "Could not find server_cpp.exe. Pass -ServerExe explicitly."
}

function Quote-Argument {
    param([string]$Value)

    if ($Value -match '[\s"]') {
        return '"' + ($Value -replace '"', '\"') + '"'
    }
    return $Value
}

function Normalize-LevelFile {
    param(
        [string]$OriginalPath,
        [string]$TempPath
    )

    $content = [System.IO.File]::ReadAllText($OriginalPath)
    $lines = $content -split "`r?`n"

    $filteredLines = New-Object System.Collections.Generic.List[string]

    foreach ($rawLine in $lines) {
        $line = $rawLine.TrimEnd()
        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }
        $filteredLines.Add($line)
    }

    $normalized = $filteredLines -join "`r`n"
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($TempPath, $normalized, $utf8NoBom)
}

function Get-LevelPaths {
    param(
        [string]$RepoRoot,
        [string[]]$Roots,
        [string]$ListPath,
        [int]$MaxCount
    )

    if ($ListPath) {
        $resolvedList = Resolve-RepoPath -RepoRoot $RepoRoot -Path $ListPath
        $levels = Get-Content $resolvedList |
            Where-Object { -not [string]::IsNullOrWhiteSpace($_) -and -not $_.Trim().StartsWith("#") } |
            ForEach-Object { Resolve-RepoPath -RepoRoot $RepoRoot -Path $_.Trim() }
    } else {
        $levels = foreach ($root in $Roots) {
            $resolvedRoot = Resolve-RepoPath -RepoRoot $RepoRoot -Path $root
            Get-ChildItem -Path $resolvedRoot -Filter *.lvl -File -Recurse |
                Sort-Object FullName |
                ForEach-Object { $_.FullName }
        }
    }

    $unique = $levels | Sort-Object -Unique
    if ($MaxCount -gt 0) {
        return @($unique | Select-Object -First $MaxCount)
    }
    return @($unique)
}

function Convert-ToRelativePath {
    param(
        [string]$RepoRoot,
        [string]$Path
    )

    $root = (Resolve-Path $RepoRoot).Path.TrimEnd('\', '/')
    $full = (Resolve-Path $Path).Path
    if ($full.StartsWith($root, [System.StringComparison]::OrdinalIgnoreCase)) {
        return $full.Substring($root.Length).TrimStart('\', '/')
    }
    return $full
}

function Parse-RunOutput {
    param([string]$Text)

    $solved = $false
    $solutionLength = $null
    $clientSolutionLength = $null
    $expanded = $null
    $frontier = $null
    $generated = $null
    $searchSeconds = $null
    $memoryMb = $null

    $serverSolved = [regex]::Match($Text, "Solved in\s+([0-9,]+)\s+steps\.")
    if ($serverSolved.Success) {
        $solved = $true
        $solutionLength = [int64](($serverSolved.Groups[1].Value) -replace ",", "")
    }

    $clientSolved = [regex]::Match($Text, "Found solution of length\s+([0-9,]+)\.")
    if ($clientSolved.Success) {
        $clientSolutionLength = [int64](($clientSolved.Groups[1].Value) -replace ",", "")
    }

    $statusMatches = [regex]::Matches($Text, "#Expanded:\s*([0-9,]+),\s*#Frontier:\s*([0-9,]+),\s*#Generated:\s*([0-9,]+),\s*Time:\s*([0-9.]+)\s*s")
    if ($statusMatches.Count -gt 0) {
        $last = $statusMatches[$statusMatches.Count - 1]
        $expanded = [int64](($last.Groups[1].Value) -replace ",", "")
        $frontier = [int64](($last.Groups[2].Value) -replace ",", "")
        $generated = [int64](($last.Groups[3].Value) -replace ",", "")
        $searchSeconds = [double]::Parse($last.Groups[4].Value, [System.Globalization.CultureInfo]::InvariantCulture)
    }

    $memoryMatch = [regex]::Match($Text, "\[Used:\s*([0-9.]+)\s*MB\]")
    if ($memoryMatch.Success) {
        $memoryMb = [double]::Parse($memoryMatch.Groups[1].Value, [System.Globalization.CultureInfo]::InvariantCulture)
    }

    return [pscustomobject]@{
        solved = $solved
        solution_length = $solutionLength
        client_solution_length = $clientSolutionLength
        expanded = $expanded
        frontier = $frontier
        generated = $generated
        search_seconds = $searchSeconds
        memory_mb = $memoryMb
    }
}

function Stop-ProcessTree {
    param([int]$ProcessId)

    if ($ProcessId -le 0) {
        return
    }

    try {
        & taskkill.exe /PID $ProcessId /T /F | Out-Null
    } catch {
        try {
            Stop-Process -Id $ProcessId -Force -ErrorAction SilentlyContinue
        } catch {
        }
    }
}

function Invoke-LevelRun {
    param(
        [string]$ServerPath,
        [string]$ClientPath,
        [string]$LevelPath,
        [string]$Algorithm,
        [int]$TimeoutSeconds,
        [int]$MaxJointActions,
        [string]$LogPath
    )

    # Create normalized temporary copy of level file
    $tempDir = Join-Path $env:TEMP "aimas-levels"
    New-Item -ItemType Directory -Force -Path $tempDir | Out-Null
    $levelHashBytes = [System.Text.Encoding]::UTF8.GetBytes((Resolve-Path $LevelPath).Path)
    $levelHash = [BitConverter]::ToString(
        [System.Security.Cryptography.SHA1]::Create().ComputeHash($levelHashBytes)
    ).Replace("-", "").Substring(0, 10)
    $tempLevelName = "{0}-{1}{2}" -f `
        [System.IO.Path]::GetFileNameWithoutExtension($LevelPath), `
        $levelHash, `
        [System.IO.Path]::GetExtension($LevelPath)
    $tempLevelPath = Join-Path $tempDir $tempLevelName
    Normalize-LevelFile -OriginalPath $LevelPath -TempPath $tempLevelPath

    $clientCommand = Quote-Argument $ClientPath
    if (-not [string]::IsNullOrWhiteSpace($Algorithm)) {
        $clientCommand = "$clientCommand $Algorithm"
    }

    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.UseShellExecute = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.CreateNoWindow = $true
    $psi.FileName = $ServerPath
    $psi.Arguments = "-l $(Quote-Argument $tempLevelPath) -c $(Quote-Argument $clientCommand) -t $TimeoutSeconds"
    $psi.WorkingDirectory = Split-Path -Parent $tempLevelPath

    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $psi

    $stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    $null = $process.Start()
    $stdoutTask = $process.StandardOutput.ReadToEndAsync()
    $stderrTask = $process.StandardError.ReadToEndAsync()

    $waitMs = $TimeoutSeconds * 1000
    $forcedTimeout = $false
    $finished = $process.WaitForExit($waitMs)
    if (-not $finished) {
        $forcedTimeout = $true
        Stop-ProcessTree -ProcessId $process.Id
        $process.WaitForExit(5000) | Out-Null
        $finished = $process.HasExited
    }
    if ($finished) {
        $process.WaitForExit()
    }
    $stopwatch.Stop()

    $stdout = ""
    $stderr = ""
    try {
        $stdout = $stdoutTask.GetAwaiter().GetResult()
    } catch {
    }
    try {
        $stderr = $stderrTask.GetAwaiter().GetResult()
    } catch {
    }
    $combined = ($stdout.TrimEnd() + [Environment]::NewLine + $stderr.TrimEnd()).Trim()
    Set-Content -Path $LogPath -Value $combined -Encoding utf8

    $parsed = Parse-RunOutput -Text $combined
    $timedOut = $forcedTimeout -or -not $finished
    if (-not $timedOut -and $combined -match "timeout|timed out|Timeout") {
        $timedOut = $true
    }
    $exceededJointActionLimit =
        $null -ne $parsed.solution_length -and
        [int64]$parsed.solution_length -gt [int64]$MaxJointActions
    $countsAsSolved = $parsed.solved -and -not $exceededJointActionLimit

    return [pscustomobject]@{
        algorithm = $Algorithm
        solved = $countsAsSolved
        timeout = $timedOut
        exceeded_joint_action_limit = $exceededJointActionLimit
        solution_length = $parsed.solution_length
        expanded = $parsed.expanded
        frontier = $parsed.frontier
        generated = $parsed.generated
        search_seconds = $parsed.search_seconds
        client_solution_length = $parsed.client_solution_length
        wall_seconds = [Math]::Round($stopwatch.Elapsed.TotalSeconds, 3)
        memory_mb = $parsed.memory_mb
        exit_code = if ($finished) { $process.ExitCode } else { $null }
        log = $LogPath
    }
}

function Write-MarkdownSummary {
    param(
        [string]$Path,
        [object[]]$Rows,
        [string[]]$Algorithms,
        [string[]]$Levels,
        [int]$TimeoutSeconds,
        [int]$MaxJointActions,
        [string]$ClientPath,
        [string]$ServerPath
    )

    $lines = New-Object System.Collections.Generic.List[string]
    $lines.Add("# All-Level Benchmark")
    $lines.Add("")
    $lines.Add("Generated: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")
    $lines.Add("")
    $lines.Add("- Server: $ServerPath")
    $lines.Add("- Client: $ClientPath")
    $lines.Add("- Timeout per run: $TimeoutSeconds seconds")
    $lines.Add("- Joint-action limit: $MaxJointActions")
    $lines.Add("- Levels: $($Levels.Count)")
    $lines.Add("- Algorithms: $($Algorithms -join ', ')")
    $lines.Add("")

    $lines.Add("## Summary")
    $lines.Add("")
    $lines.Add("| Algorithm | Solved | Timeout | Over 20k | Failed | Avg Wall s | Avg Solution | Avg Expanded |")
    $lines.Add("|---|---:|---:|---:|---:|---:|---:|---:|")
    foreach ($algorithm in $Algorithms) {
        $subset = @($Rows | Where-Object { $_.algorithm -eq $algorithm })
        $solvedRows = @($subset | Where-Object { $_.solved })
        $timeoutCount = @($subset | Where-Object { $_.timeout }).Count
        $overLimitCount = @($subset | Where-Object { $_.exceeded_joint_action_limit }).Count
        $failedCount = $subset.Count - $solvedRows.Count - $timeoutCount - $overLimitCount
        $avgWall = if ($subset.Count) { [Math]::Round((($subset | Measure-Object wall_seconds -Average).Average), 3) } else { $null }
        $avgLen = if ($solvedRows.Count) { [Math]::Round((($solvedRows | Measure-Object solution_length -Average).Average), 2) } else { $null }
        $expandedRows = @($subset | Where-Object { $null -ne $_.expanded })
        $avgExpanded = if ($expandedRows.Count) { [Math]::Round((($expandedRows | Measure-Object expanded -Average).Average), 0) } else { $null }
        $lines.Add("| $algorithm | $($solvedRows.Count) / $($subset.Count) | $timeoutCount | $overLimitCount | $failedCount | $avgWall | $avgLen | $avgExpanded |")
    }
    $lines.Add("")

    $lines.Add("## Unsolved Or Timed Out")
    $lines.Add("")
    $lines.Add("| Level | Algorithm | Timeout | Over 20k | Server Len | Client Len | Wall s | Exit | Log |")
    $lines.Add("|---|---|---:|---:|---:|---:|---:|---:|---|")
    $badRows = @($Rows | Where-Object { -not $_.solved -or $_.timeout -or $_.exceeded_joint_action_limit } | Sort-Object level, algorithm)
    if ($badRows.Count -eq 0) {
        $lines.Add("| none |  |  |  |  |  |  |  |  |")
    } else {
        foreach ($row in $badRows) {
            $lines.Add("| $($row.level) | $($row.algorithm) | $($row.timeout) | $($row.exceeded_joint_action_limit) | $($row.solution_length) | $($row.client_solution_length) | $($row.wall_seconds) | $($row.exit_code) | $($row.log) |")
        }
    }
    $lines.Add("")

    $lines.Add("## Raw Runs")
    $lines.Add("")
    $lines.Add("| Level | Algorithm | Solved | Timeout | Over 20k | Server Len | Client Len | Expanded | Generated | Search s | Wall s | Mem MB |")
    $lines.Add("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    foreach ($row in ($Rows | Sort-Object algorithm, level)) {
        $lines.Add("| $($row.level) | $($row.algorithm) | $($row.solved) | $($row.timeout) | $($row.exceeded_joint_action_limit) | $($row.solution_length) | $($row.client_solution_length) | $($row.expanded) | $($row.generated) | $($row.search_seconds) | $($row.wall_seconds) | $($row.memory_mb) |")
    }

    Set-Content -Path $Path -Value $lines -Encoding utf8
}

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptRoot
$clientPath = Resolve-CppClientPath -RepoRoot $repoRoot -Provided $CppClient
$serverPath = Resolve-ServerPath -RepoRoot $repoRoot -Provided $ServerExe
$levelPaths = Get-LevelPaths -RepoRoot $repoRoot -Roots $LevelRoots -ListPath $LevelList -MaxCount $Limit

if ($levelPaths.Count -eq 0) {
    throw "No .lvl files found."
}

$resultsDir = Join-Path $scriptRoot "results"
$logsDir = Join-Path $resultsDir "$OutputName-logs"
New-Item -ItemType Directory -Force -Path $resultsDir | Out-Null
New-Item -ItemType Directory -Force -Path $logsDir | Out-Null

$rows = New-Object System.Collections.Generic.List[object]
$totalRuns = $levelPaths.Count * $Algorithms.Count
$runIndex = 0

foreach ($algorithm in $Algorithms) {
    foreach ($levelPath in $levelPaths) {
        ++$runIndex
        $relativeLevel = Convert-ToRelativePath -RepoRoot $repoRoot -Path $levelPath
        $safeAlgorithm = if ([string]::IsNullOrWhiteSpace($algorithm)) { "default" } else { $algorithm }
        $safeName = (($relativeLevel -replace '[\\/:*?"<>| ]', '_') + "__" + ($safeAlgorithm -replace '[\\/:*?"<>| ]', '_'))
        $logPath = Join-Path $logsDir "$safeName.log"

        Write-Host "[$runIndex/$totalRuns] $relativeLevel :: $safeAlgorithm"
        $run = Invoke-LevelRun -ServerPath $serverPath `
                               -ClientPath $clientPath `
                               -LevelPath $levelPath `
                               -Algorithm $algorithm `
                               -TimeoutSeconds $TimeoutSeconds `
                               -MaxJointActions $MaxJointActions `
                               -LogPath $logPath

        $rows.Add([pscustomobject]@{
            level = $relativeLevel
            algorithm = $algorithm
            solved = $run.solved
            timeout = $run.timeout
            exceeded_joint_action_limit = $run.exceeded_joint_action_limit
            solution_length = $run.solution_length
            expanded = $run.expanded
            frontier = $run.frontier
            generated = $run.generated
            search_seconds = $run.search_seconds
            client_solution_length = $run.client_solution_length
            wall_seconds = $run.wall_seconds
            memory_mb = $run.memory_mb
            exit_code = $run.exit_code
            log = Convert-ToRelativePath -RepoRoot $repoRoot -Path $run.log
        })
    }
}

$csvPath = Join-Path $resultsDir "$OutputName.csv"
$mdPath = Join-Path $resultsDir "$OutputName.md"

$rows |
    Select-Object level, algorithm, solved, timeout, exceeded_joint_action_limit, solution_length, client_solution_length, expanded, frontier, generated, search_seconds, wall_seconds, memory_mb, exit_code, log |
    Export-Csv -Path $csvPath -NoTypeInformation

Write-MarkdownSummary -Path $mdPath `
                      -Rows $rows `
                      -Algorithms $Algorithms `
                      -Levels $levelPaths `
                      -TimeoutSeconds $TimeoutSeconds `
                      -MaxJointActions $MaxJointActions `
                      -ClientPath $clientPath `
                      -ServerPath $serverPath

Write-Host ""
Write-Host "Wrote CSV summary to $csvPath"
Write-Host "Wrote Markdown summary to $mdPath"
Write-Host "Wrote logs to $logsDir"
