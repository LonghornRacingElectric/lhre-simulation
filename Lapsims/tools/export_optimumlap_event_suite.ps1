param(
    [string]$VehiclePath = 'C:\Users\Abishek\Desktop\Aero Sweeps\Project\Vehicle\FSAE Aero.OLVeh',
    [string]$TrackRoot = 'C:\Users\Abishek\Desktop\Aero Sweeps\proof_sweep\tracks',
    [string]$OutputRoot = (Join-Path $PSScriptRoot '..')
)

$ErrorActionPreference = 'Stop'
$resolvedOutputRoot = [IO.Path]::GetFullPath($OutputRoot)
$exporter = Join-Path $PSScriptRoot 'export_optimumlap_baseline.ps1'
$eventInputDirectory = Join-Path $resolvedOutputRoot 'inputs\events'
$eventOutputDirectory = Join-Path $resolvedOutputRoot 'outputs\events'
$workDirectory = Join-Path $resolvedOutputRoot '.work\event_exports'
New-Item -ItemType Directory -Path $eventInputDirectory, $eventOutputDirectory, $workDirectory -Force | Out-Null

$events = @(
    [ordered]@{
        Slug = 'acceleration'
        File = 'FSAE Acceleration 75m.OLTra'
        ExpectedConfiguration = 'Open Circuit'
    },
    [ordered]@{
        Slug = 'autocross'
        File = 'FSAE Autocross Nebraska 2013.OLTra'
        ExpectedConfiguration = 'Open Circuit'
    },
    [ordered]@{
        Slug = 'skidpad'
        File = 'FSAE Skidpad 9.125m Radius.OLTra'
        ExpectedConfiguration = 'Closed Circuit'
    },
    [ordered]@{
        Slug = 'michigan_endurance'
        File = 'FSAE Michigan Endurance 2014.OLTra'
        ExpectedConfiguration = 'Closed Circuit'
    }
)

$manifestEvents = [System.Collections.Generic.List[object]]::new()
foreach ($event in $events) {
    $trackPath = Join-Path $TrackRoot $event.File
    if (-not (Test-Path -LiteralPath $trackPath -PathType Leaf)) {
        throw "Track not found: $trackPath"
    }
    $temporaryRoot = Join-Path $workDirectory $event.Slug
    New-Item -ItemType Directory -Path $temporaryRoot -Force | Out-Null

    & $exporter -VehiclePath $VehiclePath -TrackPath $trackPath -OutputRoot $temporaryRoot | Out-Null

    $baseline = Get-Content -Raw -LiteralPath (Join-Path $temporaryRoot 'inputs\optimumlap_baseline.json') | ConvertFrom-Json
    if ([string]$baseline.Track.Configuration -ne [string]$event.ExpectedConfiguration) {
        throw "Unexpected track configuration for $($event.Slug): $($baseline.Track.Configuration)"
    }

    $segmentDestination = Join-Path $eventInputDirectory "$($event.Slug)_optimumlap_segments.csv"
    $baselineDestination = Join-Path $eventInputDirectory "$($event.Slug)_optimumlap_baseline.json"
    $traceDestination = Join-Path $eventOutputDirectory "$($event.Slug)_optimumlap_trace.csv"
    $summaryDestination = Join-Path $eventOutputDirectory "$($event.Slug)_optimumlap_summary.json"
    Copy-Item -LiteralPath (Join-Path $temporaryRoot 'inputs\michigan_optimumlap_segments.csv') -Destination $segmentDestination -Force
    Copy-Item -LiteralPath (Join-Path $temporaryRoot 'inputs\optimumlap_baseline.json') -Destination $baselineDestination -Force
    Copy-Item -LiteralPath (Join-Path $temporaryRoot 'outputs\optimumlap_trace.csv') -Destination $traceDestination -Force
    Copy-Item -LiteralPath (Join-Path $temporaryRoot 'outputs\optimumlap_summary.json') -Destination $summaryDestination -Force

    $summary = Get-Content -Raw -LiteralPath $summaryDestination | ConvertFrom-Json
    $manifestEvents.Add([pscustomobject][ordered]@{
        Slug = $event.Slug
        TrackFile = $event.File
        SourcePath = $trackPath
        SourceSha256 = $baseline.Track.SourceSha256
        Name = $baseline.Track.Name
        Configuration = $baseline.Track.Configuration
        IsClosed = [bool]$baseline.Track.IsClosed
        TrackLength_m = [double]$baseline.Track.TotalLength_m
        SegmentSize_m = [double]$baseline.Track.SegmentSize_m
        SegmentCount = [int]$baseline.Track.SegmentCount
        OptimumLapTime_s = [double]$summary.LapTime_s
    })
}

$manifest = [pscustomobject][ordered]@{
    VehiclePath = $VehiclePath
    VehicleSha256 = (Get-FileHash -LiteralPath $VehiclePath -Algorithm SHA256).Hash
    Solver = 'OptimumLap 1.5.5 native OptimumLap_Simple.Solver'
    Events = @($manifestEvents)
}
$manifest | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $eventInputDirectory 'event_suite_manifest.json') -Encoding UTF8
$manifest | ConvertTo-Json -Depth 6
