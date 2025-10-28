$COMMON = @("--target-env=vulkan1.3", "-O", "-g", "-Werror", "-I", "shaders")

Get-ChildItem -Path "shaders" -Include *.frag,*.vert,*.comp,*.geom,*.tesc,*.tese,*.mesh,*.task,*.rgen,*.rint,*.rahit,*.rchit,*.rmiss,*.rcall -Recurse |
        ForEach-Object {
          $extra = @()
          if ($_.Extension -eq ".mesh") { $extra += "-fshader-stage=mesh" }
          elseif ($_.Extension -eq ".task") { $extra += "-fshader-stage=task" }
          glslc $_.FullName @COMMON @extra -o "$($_.FullName).spv"
        }
