# Build and run the host-side test suite (requires g++ on PATH).
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$build = Join-Path $root "build"
New-Item -ItemType Directory -Force $build | Out-Null

$core = Get-ChildItem (Join-Path $root "..\Enlighten\src\core\*.cpp") | ForEach-Object FullName
$tests = Get-ChildItem (Join-Path $root "*.cpp") | ForEach-Object FullName
$out = Join-Path $build "tests.exe"

& g++ -std=c++14 -O2 -Wall -Wextra -Werror `
    -I (Join-Path $root "..\Enlighten\src") -I $root `
    @core @tests -o $out
if ($LASTEXITCODE -ne 0) { exit 1 }

& $out
exit $LASTEXITCODE
