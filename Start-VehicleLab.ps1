$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$candidates = @(
    (Join-Path $PSScriptRoot '.venv\Scripts\python.exe'),
    (Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe')
)
foreach ($candidate in $candidates) {
    if (Test-Path -LiteralPath $candidate) {
        & $candidate (Join-Path $PSScriptRoot 'run.py')
        exit $LASTEXITCODE
    }
}
foreach ($name in @('py', 'python', 'python3')) {
    $command = Get-Command $name -ErrorAction SilentlyContinue
    if ($command) {
        try {
            & $command.Source --version
            if ($LASTEXITCODE -eq 0) {
                & $command.Source (Join-Path $PSScriptRoot 'run.py')
                exit $LASTEXITCODE
            }
        } catch { }
    }
}
Write-Host 'Python 3.10+ with Tkinter is required. Install Python, then run: python run.py'
Read-Host 'Press Enter to close'
exit 1
