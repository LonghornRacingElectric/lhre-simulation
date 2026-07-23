param(
    [string]$VehiclePath = 'C:\Users\Abishek\Desktop\Aero Sweeps\Project\Vehicle\FSAE Aero.OLVeh',
    [string]$TrackPath = 'C:\Users\Abishek\Desktop\Aero Sweeps\proof_sweep\tracks\FSAE Michigan Endurance 2014.OLTra',
    [string]$OutputRoot = (Join-Path $PSScriptRoot '..')
)

$ErrorActionPreference = 'Stop'
$installDir = 'C:\Program Files (x86)\OptimumG\OptimumLap'
$resolvedOutputRoot = [IO.Path]::GetFullPath($OutputRoot)
$inputDirectory = Join-Path $resolvedOutputRoot 'inputs'
$outputDirectory = Join-Path $resolvedOutputRoot 'outputs'
New-Item -ItemType Directory -Path $inputDirectory, $outputDirectory -Force | Out-Null

Set-Location -LiteralPath $installDir
Get-ChildItem -LiteralPath $installDir -Filter *.dll | ForEach-Object {
    try { [Reflection.Assembly]::LoadFrom($_.FullName) | Out-Null } catch {}
}

$assembly = [Reflection.Assembly]::LoadFrom((Join-Path $installDir 'OptimumLap.exe'))
$vehicleType = $assembly.GetType('OptimumLap_Simple.Vehicle', $true)
$trackType = $assembly.GetType('OptimumLap_Simple.Track', $true)
$solverType = $assembly.GetType('OptimumLap_Simple.Solver', $true)
$metricFlags = [Reflection.BindingFlags]'Public,NonPublic,Instance'

function Get-PropertyValue($object, [Type]$type, [string]$name) {
    return $type.GetProperty($name).GetValue($object, $null)
}

function Set-PropertyValue($object, [Type]$type, [string]$name, [double]$value) {
    $property = $type.GetProperty($name)
    if ($null -eq $property -or -not $property.CanWrite) {
        throw "Writable property not found: $name"
    }
    $property.SetValue($object, $value, $null)
}

function Get-OutputValue($parameters, [string]$key) {
    $output = $parameters[$key]
    if ($null -eq $output) {
        throw "Vehicle parameter not found: $key"
    }
    return [double]$output.GetType().GetProperty('Value').GetValue($output, $null)
}

function Invoke-SolverMetric($solver, [string]$name, $result) {
    $method = $solverType.GetMethod($name, $metricFlags)
    if ($null -eq $method) {
        throw "Solver metric method not found: $name"
    }
    return [double]$method.Invoke($solver, [object[]]@($result))
}

function Convert-OutputDictionary($dictionary) {
    $converted = [ordered]@{}
    foreach ($key in ($dictionary.Keys | Sort-Object)) {
        $output = $dictionary[$key]
        $outputType = $output.GetType()
        $values = $outputType.GetProperty('Values').GetValue($output, $null)
        $value = $outputType.GetProperty('Value').GetValue($output, $null)
        $converted[$key] = [ordered]@{
            Name = [string]$outputType.GetProperty('Name').GetValue($output, $null)
            FullName = [string]$outputType.GetProperty('FullName').GetValue($output, $null)
            Unit = [string]$outputType.GetProperty('Unit').GetValue($output, $null)
            OutputType = [int]$outputType.GetProperty('OutputType').GetValue($output, $null)
            Value = [double]$value
            Values = if ($null -eq $values) { @() } else { @($values) }
        }
    }
    return $converted
}

$vehicle = $vehicleType.GetConstructor([Type[]]@([string])).Invoke([object[]]@('OpenLAP comparison baseline'))
$vehicleLoad = $vehicleType.GetMethod('Load', [Type[]]@([string]))
if (-not $vehicleLoad.Invoke($vehicle, [object[]]@([string]$VehiclePath))) {
    throw "Could not load $VehiclePath"
}
if (-not [bool](Get-PropertyValue $vehicle $vehicleType 'AllValuesValid')) {
    throw 'Loaded OptimumLap vehicle is invalid.'
}

$vehicleParameters = Get-PropertyValue $vehicle $vehicleType 'VehicleParameters'
foreach ($setting in @(
    @('LatFriction', (Get-OutputValue $vehicleParameters 'latfriction')),
    @('LongFriction', (Get-OutputValue $vehicleParameters 'longfriction')),
    @('MassLatFriction', (Get-OutputValue $vehicleParameters 'masslatfriction')),
    @('MassLongFriction', (Get-OutputValue $vehicleParameters 'masslongfriction')),
    @('LatFrictionSensitivity', (Get-OutputValue $vehicleParameters 'sensitivitylatfriction')),
    @('LongFrictionSensitivity', (Get-OutputValue $vehicleParameters 'sensitivitylongfriction'))
)) {
    Set-PropertyValue $vehicle $vehicleType $setting[0] ([double]$setting[1])
}

$runtimeVehicle = [ordered]@{}
foreach ($property in ($vehicleType.GetProperties() | Sort-Object Name)) {
    if ($property.PropertyType -eq [double] -or
        $property.PropertyType -eq [bool] -or
        $property.PropertyType -eq [string] -or
        $property.PropertyType -eq [double[]]) {
        try {
            $raw = $property.GetValue($vehicle, $null)
            $runtimeVehicle[$property.Name] = if ($property.PropertyType -eq [double[]]) {
                if ($null -eq $raw) { @() } else { @($raw) }
            } else {
                $raw
            }
        } catch {}
    }
}

$track = $trackType.GetConstructor([Type[]]@([string])).Invoke([object[]]@('OpenLAP comparison track'))
$trackLoad = $trackType.GetMethod('Load', [Type[]]@([string]))
if (-not $trackLoad.Invoke($track, [object[]]@([string]$TrackPath))) {
    throw "Could not load $TrackPath"
}
if (-not [bool](Get-PropertyValue $track $trackType 'AllValuesValid')) {
    throw 'Loaded OptimumLap track is invalid.'
}

$segments = @(Get-PropertyValue $track $trackType 'Segments')
$segmentRows = foreach ($segment in $segments) {
    [pscustomobject][ordered]@{
        Index = [int]$segment.index
        Length_m = [double]$segment.length
        TotalLength_m = [double]$segment.totallength
        Radius_m = [double]$segment.radius
        Direction = [double]$segment.direction
        Grade = [double]$segment.grade
        X_m = [double]$segment.globalx
        Y_m = [double]$segment.globaly
        Z_m = [double]$segment.globalz
        CarAttitude = [double]$segment.carattitude
        Sector = [int]$segment.sector
    }
}
$segmentRows | Export-Csv -LiteralPath (Join-Path $inputDirectory 'michigan_optimumlap_segments.csv') -NoTypeInformation

$trackMetadata = [ordered]@{
    SourcePath = $TrackPath
    SourceSha256 = (Get-FileHash -LiteralPath $TrackPath -Algorithm SHA256).Hash
    Name = [string](Get-PropertyValue $track $trackType 'Name')
    Type = [string](Get-PropertyValue $track $trackType 'Type')
    Configuration = [string](Get-PropertyValue $track $trackType 'Configuration')
    Direction = [string](Get-PropertyValue $track $trackType 'Direction')
    IsClosed = [bool](Get-PropertyValue $track $trackType 'IsClosed')
    TotalLength_m = [double](Get-PropertyValue $track $trackType 'TotalTrackLength')
    SegmentSize_m = [double](Get-PropertyValue $track $trackType 'SegmentSize')
    SegmentCount = $segments.Count
}

$vehicleMetadata = [ordered]@{
    SourcePath = $VehiclePath
    SourceSha256 = (Get-FileHash -LiteralPath $VehiclePath -Algorithm SHA256).Hash
    OptimumLapVersion = [string]$assembly.GetName().Version
    Runtime = $runtimeVehicle
    ParameterDictionary = Convert-OutputDictionary $vehicleParameters
}

[pscustomobject][ordered]@{
    Vehicle = $vehicleMetadata
    Track = $trackMetadata
} | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $inputDirectory 'optimumlap_baseline.json') -Encoding UTF8

$solver = [Activator]::CreateInstance($solverType)
$worker = [System.ComponentModel.BackgroundWorker]::new()
$worker.WorkerReportsProgress = $true
$worker.WorkerSupportsCancellation = $true
$solverRun = $solverType.GetMethod('Solver')
$result = $solverRun.Invoke($solver, [object[]]@($vehicle, $track, $worker))
if ($null -eq $result) {
    throw 'OptimumLap solver returned null.'
}

$resultOutputs = Get-PropertyValue $result $result.GetType() 'Outputs'
$convertedResults = Convert-OutputDictionary $resultOutputs
$vectorKeys = @($convertedResults.Keys | Where-Object {
    $convertedResults[$_].Values.Count -eq $segments.Count
})

$wideRows = for ($i = 0; $i -lt $segments.Count; $i++) {
    $row = [ordered]@{ Index = $i }
    foreach ($key in $vectorKeys) {
        $row[$key] = [double]$convertedResults[$key].Values[$i]
    }
    [pscustomobject]$row
}
$wideRows | Export-Csv -LiteralPath (Join-Path $outputDirectory 'optimumlap_trace.csv') -NoTypeInformation

$summary = [ordered]@{
    Solver = 'OptimumLap native OptimumLap_Simple.Solver'
    Vehicle = [string](Get-PropertyValue $vehicle $vehicleType 'Name')
    Track = [string](Get-PropertyValue $track $trackType 'Name')
    TrackLength_m = [double](Get-PropertyValue $track $trackType 'TotalTrackLength')
    SegmentCount = $segments.Count
    LapTime_s = Invoke-SolverMetric $solver 'KpiLapTime' $result
    EnergySpent_J = Invoke-SolverMetric $solver 'KpiEnergySpent' $result
    LowestSpeed_mps = Invoke-SolverMetric $solver 'KpiLowestSpeed' $result
    HighestSpeed_mps = Invoke-SolverMetric $solver 'KpiHighestSpeed' $result
    AverageSpeed_mps = Invoke-SolverMetric $solver 'KpiAverageSpeed' $result
    MaxLatAccel_mps2 = Invoke-SolverMetric $solver 'MaxLat' $result
    MaxLongAccel_mps2 = Invoke-SolverMetric $solver 'MaxLongAccel' $result
    MaxLongDecel_mps2 = Invoke-SolverMetric $solver 'MaxLongDecel' $result
    ResultVectorKeys = $vectorKeys
}
$summary | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $outputDirectory 'optimumlap_summary.json') -Encoding UTF8
[pscustomobject]$summary | Select-Object Solver,Vehicle,Track,TrackLength_m,SegmentCount,LapTime_s,EnergySpent_J,LowestSpeed_mps,HighestSpeed_mps,AverageSpeed_mps,MaxLatAccel_mps2,MaxLongAccel_mps2,MaxLongDecel_mps2 | ConvertTo-Json -Depth 4
