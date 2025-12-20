param(
  [ValidateSet("Debug", "Release", "RelWithDebInfo")]
  [string]$Config = "RelWithDebInfo",
  [string]$Python = "python"
)

$script_dir = Split-Path -Parent $MyInvocation.MyCommand.Path
$py = Join-Path $script_dir "compile_shaders.py"

& $Python $py --config $Config
