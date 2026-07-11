# Build the sequence generator and show player (requires g++ on PATH).
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$build = Join-Path $root "build"
New-Item -ItemType Directory -Force $build | Out-Null

$core = Get-ChildItem (Join-Path $root "..\Enlighten\src\core\*.cpp") | ForEach-Object FullName
$inc = @("-I", (Join-Path $root "..\Enlighten\src"), "-I", (Join-Path $root "..\test"))

& g++ -std=c++14 -O2 -Wall -Wextra -Werror @inc @core (Join-Path $root "..\test\sim.cpp") `
    (Join-Path $root "seqgen.cpp") -o (Join-Path $build "seqgen.exe")
if ($LASTEXITCODE -ne 0) { exit 1 }

& g++ -std=c++14 -O2 -Wall -Wextra -Werror @inc @core (Join-Path $root "..\test\sim.cpp") `
    (Join-Path $root "showplay.cpp") -o (Join-Path $build "showplay.exe")
if ($LASTEXITCODE -ne 0) { exit 1 }

& g++ -std=c++14 -O2 -Wall -Wextra -Werror @inc @core (Join-Path $root "..\test\sim.cpp") `
    (Join-Path $root "showviz.cpp") -o (Join-Path $build "showviz.exe")
if ($LASTEXITCODE -ne 0) { exit 1 }

Write-Host "built: $build\seqgen.exe, $build\showplay.exe, $build\showviz.exe"
