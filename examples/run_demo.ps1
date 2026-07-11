# Build and run the feature tour (requires g++ on PATH).
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$build = Join-Path $root "build"
New-Item -ItemType Directory -Force $build | Out-Null

$core = Get-ChildItem (Join-Path $root "..\Enlighten\src\core\*.cpp") | ForEach-Object FullName
$out = Join-Path $build "feature_tour.exe"

& g++ -std=c++14 -O2 -Wall -Wextra -Werror `
    -I (Join-Path $root "..\Enlighten\src") -I (Join-Path $root "..\test") `
    @core (Join-Path $root "..\test\sim.cpp") (Join-Path $root "feature_tour.cpp") `
    -o $out
if ($LASTEXITCODE -ne 0) { exit 1 }

& $out
exit $LASTEXITCODE
