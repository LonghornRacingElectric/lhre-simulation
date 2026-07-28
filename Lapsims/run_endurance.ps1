$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath($PSScriptRoot)

Push-Location -LiteralPath $root
try {
    & python 'src\run_endurance.py' @args
    if ($LASTEXITCODE -ne 0) {
        throw "Battery-aware endurance simulation failed with exit code $LASTEXITCODE"
    }
    & python -m unittest discover -s tests -v
    if ($LASTEXITCODE -ne 0) {
        throw "Regression tests failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}
