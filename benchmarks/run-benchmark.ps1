[CmdletBinding()]
param(
    [string]$CppClient = "",
    [string]$LevelList = "",
    [int]$TimeoutSeconds = 60,
    [string]$JavaExe = "",
    [string]$JavacExe = "",
    [string]$JavaHeap = "4g",
    [switch]$ForceRebuildReference
)

$ErrorActionPreference = "Stop"

function Resolve-ToolPath {
    param(
        [string]$Provided,
        [string]$CommandName
    )

    if ($Provided) {
        return (Resolve-Path $Provided).Path
    }

    $command = Get-Command $CommandName -ErrorAction SilentlyContinue
    if (-not $command) {
        throw "Could not find '$CommandName' on PATH."
    }
    return $command.Source
}

function Resolve-CppClientPath {
    param([string]$RepoRoot, [string]$Provided)

    if ($Provided) {
        return (Resolve-Path $Provided).Path
    }

    $candidates = @(
        (Join-Path $RepoRoot "direct-tests\bin\searchclient_cpp.exe"),
        (Join-Path $RepoRoot "searchclient_cpp\build\searchclient_cpp\Release\searchclient_cpp.exe")
    )

    foreach ($candidate in $candidates) {
        if (Test-Path $candidate) {
            return (Resolve-Path $candidate).Path
        }
    }

    throw "Could not find searchclient_cpp.exe. Pass -CppClient explicitly."
}

function Get-ReferenceJavaFiles {
    return @(
        "Action.java",
        "Color.java",
        "Frontier.java",
        "GraphSearch.java",
        "Heuristic.java",
        "Memory.java",
        "NotImplementedException.java",
        "SearchClient.java",
        "State.java"
    )
}

function Restore-ReferenceJavaClient {
    param(
        [string]$RepoRoot,
        [string]$JavacPath,
        [switch]$Force
    )

    $commit = "60ada586fbb961e0683e4acff6784c760c74f6c7"
    $generatedRoot = Join-Path $RepoRoot "benchmarks\.generated\reference-java"
    $sourceRoot = Join-Path $generatedRoot "searchclient"
    $files = Get-ReferenceJavaFiles

    New-Item -ItemType Directory -Force -Path $sourceRoot | Out-Null

    foreach ($file in $files) {
        $target = Join-Path $sourceRoot $file
        if ($Force -or -not (Test-Path $target)) {
            $spec = "${commit}:searchclient_java/searchclient/$file"
            $content = git -C $RepoRoot show $spec
            if ($LASTEXITCODE -ne 0) {
                throw "Failed to restore $file from git history."
            }
            Set-Content -Path $target -Value $content -Encoding ascii
        }
    }

    $needsCompile = $Force
    if (-not $needsCompile) {
        foreach ($file in $files) {
            $javaPath = Join-Path $sourceRoot $file
            $classPath = Join-Path $sourceRoot ($file -replace "\.java$", ".class")
            if (-not (Test-Path $classPath) -or (Get-Item $javaPath).LastWriteTimeUtc -gt (Get-Item $classPath).LastWriteTimeUtc) {
                $needsCompile = $true
                break
            }
        }
    }

    if ($needsCompile) {
        $sources = Get-ChildItem -Path $sourceRoot -Filter *.java | ForEach-Object { $_.FullName }
        & $JavacPath $sources
        if ($LASTEXITCODE -ne 0) {
            throw "javac failed while compiling the Java reference client."
        }
    }

    return $generatedRoot
}

function Split-AlgorithmArgument {
    param([string]$Algorithm)

    if ([string]::IsNullOrWhiteSpace($Algorithm)) {
        return @()
    }

    return $Algorithm.Trim().Split(" ", [System.StringSplitOptions]::RemoveEmptyEntries)
}

function Quote-Argument {
    param([string]$Value)

    if ($Value -match '[\s"]') {
        return '"' + ($Value -replace '"', '\"') + '"'
    }
    return $Value
}

function Join-Arguments {
    param([string[]]$Arguments)

    return (($Arguments | ForEach-Object { Quote-Argument $_ }) -join " ")
}

function Parse-BenchmarkOutput {
    param([string]$Text)

    $result = [ordered]@{
        solved = $false
        solution_length = $null
        expanded = $null
        frontier = $null
        generated = $null
        search_seconds = $null
    }

    $statusMatches = [regex]::Matches($Text, "#Expanded:\s*([0-9,]+),\s*#Frontier:\s*([0-9,]+),\s*#Generated:\s*([0-9,]+),\s*Time:\s*([0-9.]+)\s*s")
    if ($statusMatches.Count -gt 0) {
        $last = $statusMatches[$statusMatches.Count - 1]
        $result.expanded = [int64](($last.Groups[1].Value) -replace ",", "")
        $result.frontier = [int64](($last.Groups[2].Value) -replace ",", "")
        $result.generated = [int64](($last.Groups[3].Value) -replace ",", "")
        $result.search_seconds = [double]::Parse($last.Groups[4].Value, [System.Globalization.CultureInfo]::InvariantCulture)
    }

    $solutionMatch = [regex]::Match($Text, "Found solution of length\s*([0-9,]+)\.")
    if ($solutionMatch.Success) {
        $result.solved = $true
        $result.solution_length = [int64](($solutionMatch.Groups[1].Value) -replace ",", "")
    }

    return [pscustomobject]$result
}

function Invoke-BenchmarkRun {
    param(
        [string]$ClientKind,
        [string]$LevelPath,
        [string]$Algorithm,
        [int]$TimeoutSeconds,
        [string]$JavaPath,
        [string]$JavaClassPath,
        [string]$JavaHeap,
        [string]$CppPath
    )

    $algorithmArgs = Split-AlgorithmArgument $Algorithm
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.UseShellExecute = $false
    $psi.RedirectStandardInput = $true
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.CreateNoWindow = $true
    $psi.WorkingDirectory = Split-Path -Parent $LevelPath

    if ($ClientKind -eq "java") {
        $psi.FileName = $JavaPath
        $psi.Arguments = Join-Arguments (@("-Xmx$JavaHeap", "-cp", $JavaClassPath, "searchclient.SearchClient") + $algorithmArgs)
    } elseif ($ClientKind -eq "cpp") {
        $psi.FileName = $CppPath
        $psi.Arguments = Join-Arguments $algorithmArgs
    } else {
        throw "Unknown client kind '$ClientKind'."
    }

    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $psi

    $null = $process.Start()
    $stdoutTask = $process.StandardOutput.ReadToEndAsync()
    $stderrTask = $process.StandardError.ReadToEndAsync()

    [System.IO.File]::ReadLines($LevelPath) | ForEach-Object {
        $process.StandardInput.WriteLine($_)
    }
    $process.StandardInput.Close()

    $stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    $finished = $process.WaitForExit($TimeoutSeconds * 1000)
    if (-not $finished) {
        try {
            $process.Kill($true)
        } catch {
        }
        $stopwatch.Stop()
        return [pscustomobject]@{
            client = $ClientKind
            level = $LevelPath
            algorithm = $Algorithm
            timeout = $true
            solved = $false
            solution_length = $null
            expanded = $null
            frontier = $null
            generated = $null
            search_seconds = $null
            wall_seconds = [Math]::Round($stopwatch.Elapsed.TotalSeconds, 3)
            exit_code = $null
            output = ""
        }
    }

    $process.WaitForExit()
    $stopwatch.Stop()

    $stdout = $stdoutTask.GetAwaiter().GetResult()
    $stderr = $stderrTask.GetAwaiter().GetResult()
    $combined = ($stdout.TrimEnd() + [Environment]::NewLine + $stderr.TrimEnd()).Trim()
    $parsed = Parse-BenchmarkOutput $combined

    return [pscustomobject]@{
        client = $ClientKind
        level = $LevelPath
        algorithm = $Algorithm
        timeout = $false
        solved = $parsed.solved
        solution_length = $parsed.solution_length
        expanded = $parsed.expanded
        frontier = $parsed.frontier
        generated = $parsed.generated
        search_seconds = $parsed.search_seconds
        wall_seconds = [Math]::Round($stopwatch.Elapsed.TotalSeconds, 3)
        exit_code = $process.ExitCode
        output = $combined
    }
}

function New-ComparisonRows {
    param([object[]]$Rows)

    $javaRows = @{}
    $cppRows = @{}
    foreach ($row in $Rows) {
        $key = "$($row.level)|$($row.algorithm)"
        if ($row.client -eq "java") {
            $javaRows[$key] = $row
        } elseif ($row.client -eq "cpp") {
            $cppRows[$key] = $row
        }
    }

    $keys = ($javaRows.Keys + $cppRows.Keys | Sort-Object -Unique)
    $comparisons = foreach ($key in $keys) {
        $javaRow = $javaRows[$key]
        $cppRow = $cppRows[$key]
        if (-not $javaRow -or -not $cppRow) {
            continue
        }

        $deltaExpanded = $null
        if ($null -ne $javaRow.expanded -and $null -ne $cppRow.expanded) {
            $deltaExpanded = [int64]$cppRow.expanded - [int64]$javaRow.expanded
        }

        [pscustomobject]@{
            level = $javaRow.level
            algorithm = $javaRow.algorithm
            java_solved = $javaRow.solved
            cpp_solved = $cppRow.solved
            java_expanded = $javaRow.expanded
            cpp_expanded = $cppRow.expanded
            delta_expanded = $deltaExpanded
            java_solution_length = $javaRow.solution_length
            cpp_solution_length = $cppRow.solution_length
            java_search_seconds = $javaRow.search_seconds
            cpp_search_seconds = $cppRow.search_seconds
            java_timeout = $javaRow.timeout
            cpp_timeout = $cppRow.timeout
        }
    }

    return $comparisons
}

function Write-MarkdownSummary {
    param(
        [string]$Path,
        [object[]]$Rows,
        [object[]]$Comparisons,
        [string[]]$Levels,
        [string[]]$Algorithms
    )

    $exactMatches = ($Comparisons | Where-Object {
        -not $_.java_timeout -and -not $_.cpp_timeout -and
        $_.java_solved -eq $_.cpp_solved -and
        $_.java_expanded -eq $_.cpp_expanded -and
        $_.java_solution_length -eq $_.cpp_solution_length
    }).Count

    $mismatches = ($Comparisons | Where-Object {
        $_.java_timeout -or $_.cpp_timeout -or
        $_.java_solved -ne $_.cpp_solved -or
        $_.java_expanded -ne $_.cpp_expanded -or
        $_.java_solution_length -ne $_.cpp_solution_length
    }).Count

    $lines = New-Object System.Collections.Generic.List[string]
    $lines.Add("# Benchmark Results")
    $lines.Add("")
    $generatedAt = Get-Date -Format 'yyyy-MM-dd HH:mm:ss'
    $lines.Add("Generated: $generatedAt")
    $lines.Add("")
    $lines.Add("Levels:")
    foreach ($level in $Levels) {
        $lines.Add("- $level")
    }
    $lines.Add("")
    $lines.Add("Algorithms:")
    foreach ($algorithm in $Algorithms) {
        $lines.Add("- $algorithm")
    }
    $lines.Add("")
    $lines.Add("- Comparison rows: $($Comparisons.Count)")
    $lines.Add("- Exact Java/C++ matches: $exactMatches")
    $lines.Add("- Mismatches: $mismatches")
    $lines.Add("")
    $lines.Add("## Java vs C++")
    $lines.Add("")
    $lines.Add("| Level | Algorithm | Java Expanded | C++ Expanded | Delta | Java Len | C++ Len | Java Time | C++ Time |")
    $lines.Add("|---|---|---:|---:|---:|---:|---:|---:|---:|")
    foreach ($row in ($Comparisons | Sort-Object @{Expression = {
        if ($null -eq $_.delta_expanded) { return 0 }
        return [Math]::Abs([int64]$_.delta_expanded)
    }; Descending = $true }, level, algorithm)) {
        $lines.Add("| $($row.level) | $($row.algorithm) | $($row.java_expanded) | $($row.cpp_expanded) | $($row.delta_expanded) | $($row.java_solution_length) | $($row.cpp_solution_length) | $($row.java_search_seconds) | $($row.cpp_search_seconds) |")
    }
    $lines.Add("")
    $lines.Add("## Raw Runs")
    $lines.Add("")
    $lines.Add("| Client | Level | Algorithm | Solved | Timeout | Expanded | Frontier | Generated | Solution Len | Search Time | Wall Time |")
    $lines.Add("|---|---|---|---|---|---:|---:|---:|---:|---:|---:|")
    foreach ($row in ($Rows | Sort-Object client, level, algorithm)) {
        $lines.Add("| $($row.client) | $($row.level) | $($row.algorithm) | $($row.solved) | $($row.timeout) | $($row.expanded) | $($row.frontier) | $($row.generated) | $($row.solution_length) | $($row.search_seconds) | $($row.wall_seconds) |")
    }

    Set-Content -Path $Path -Value $lines -Encoding utf8
}

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptRoot
$javaPath = Resolve-ToolPath -Provided $JavaExe -CommandName "java"
$javacPath = Resolve-ToolPath -Provided $JavacExe -CommandName "javac"
$cppClientPath = Resolve-CppClientPath -RepoRoot $repoRoot -Provided $CppClient

if (-not $LevelList) {
    $LevelList = Join-Path $scriptRoot "default-levels.txt"
}

$referenceJavaRoot = Restore-ReferenceJavaClient -RepoRoot $repoRoot -JavacPath $javacPath -Force:$ForceRebuildReference

$levels = Get-Content $LevelList | Where-Object { -not [string]::IsNullOrWhiteSpace($_) } | ForEach-Object { $_.Trim() }
$algorithms = @(
    "-bfs",
    "-dfs",
    "-astar",
    "-wastar 5",
    "-smart",
    "-smart-wastar 3",
    "-greedy",
    "-greedy-goalcount",
    "-astar-goalcount"
)

$results = New-Object System.Collections.Generic.List[object]
foreach ($level in $levels) {
    $levelPath = Join-Path $repoRoot $level
    if (-not (Test-Path $levelPath)) {
        throw "Level not found: $levelPath"
    }

    foreach ($algorithm in $algorithms) {
        Write-Host "Running java on $level with $algorithm"
        $results.Add((Invoke-BenchmarkRun -ClientKind "java" -LevelPath $levelPath -Algorithm $algorithm -TimeoutSeconds $TimeoutSeconds -JavaPath $javaPath -JavaClassPath $referenceJavaRoot -JavaHeap $JavaHeap -CppPath $cppClientPath))

        Write-Host "Running cpp on $level with $algorithm"
        $results.Add((Invoke-BenchmarkRun -ClientKind "cpp" -LevelPath $levelPath -Algorithm $algorithm -TimeoutSeconds $TimeoutSeconds -JavaPath $javaPath -JavaClassPath $referenceJavaRoot -JavaHeap $JavaHeap -CppPath $cppClientPath))
    }
}

$resultsDir = Join-Path $scriptRoot "results"
New-Item -ItemType Directory -Force -Path $resultsDir | Out-Null

$csvPath = Join-Path $resultsDir "latest.csv"
$mdPath = Join-Path $resultsDir "latest.md"

$results |
    Select-Object client, level, algorithm, solved, timeout, solution_length, expanded, frontier, generated, search_seconds, wall_seconds, exit_code |
    Export-Csv -Path $csvPath -NoTypeInformation

$comparisons = New-ComparisonRows -Rows $results
Write-MarkdownSummary -Path $mdPath -Rows $results -Comparisons $comparisons -Levels $levels -Algorithms $algorithms

Write-Host ""
Write-Host "Wrote CSV summary to $csvPath"
Write-Host "Wrote Markdown summary to $mdPath"
