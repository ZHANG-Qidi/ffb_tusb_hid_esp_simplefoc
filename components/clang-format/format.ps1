$root = Split-Path -Parent (Get-Location)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$config = Join-Path $scriptDir ".clang-format"
$configArg = "--style=""file:$config"""

$buildDir = Join-Path $root "build"

$files = Get-ChildItem -Path $root -Recurse -File -Include *.c, *.h, *.cpp, *.hpp |
    Where-Object { $_.FullName -notlike "$buildDir\*" }

foreach ($f in $files) {
    Write-Host "Formatting $($f.FullName)"
    clang-format -i $configArg --assume-filename=foo.cpp $f.FullName
}
