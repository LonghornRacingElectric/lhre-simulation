$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath($PSScriptRoot)
$exportScript = Join-Path $root 'tools\export_optimumlap_event_suite.ps1'
$powershell32 = 'C:\Windows\SysWOW64\WindowsPowerShell\v1.0\powershell.exe'

Push-Location -LiteralPath $root
try {
    & $powershell32 -NoProfile -ExecutionPolicy Bypass -File $exportScript -OutputRoot $root
    if ($LASTEXITCODE -ne 0) {
        throw "OptimumLap event export failed with exit code $LASTEXITCODE"
    }

    foreach ($command in @(
        @('src\build_openlap_inputs.py'),
        @('src\build_event_tracks.py'),
        @('src\openlap_solver.py'),
        @('src\run_event_suite.py'),
        @('src\compare_results.py'),
        @('src\compare_event_suite.py'),
        @('-m', 'unittest', 'discover', '-s', 'tests', '-v')
    )) {
        & python @command
        if ($LASTEXITCODE -ne 0) {
            throw "Python command failed: python $($command -join ' ')"
        }
    }
} finally {
    Pop-Location
}
