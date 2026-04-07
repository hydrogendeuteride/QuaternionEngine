@echo off

set "VSWHERE=C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
    echo [clion-clangcl-env] vswhere.exe not found: "%VSWHERE%"
    exit /b 1
)

set "VS_INSTALL="
for /f "usebackq delims=" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
    set "VS_INSTALL=%%I"
)

if not defined VS_INSTALL (
    echo [clion-clangcl-env] Visual Studio C++ toolchain was not found.
    exit /b 1
)

call "%VS_INSTALL%\VC\Auxiliary\Build\vcvars64.bat"
if errorlevel 1 (
    echo [clion-clangcl-env] Failed to initialize the Visual Studio x64 toolchain.
    exit /b 1
)

if exist "C:\Program Files\LLVM\bin\clang-cl.exe" (
    set "PATH=C:\Program Files\LLVM\bin;%PATH%"
    set "CC=C:\Program Files\LLVM\bin\clang-cl.exe"
    set "CXX=C:\Program Files\LLVM\bin\clang-cl.exe"
)

if defined VULKAN_SDK if exist "%VULKAN_SDK%\Bin" (
    set "PATH=%VULKAN_SDK%\Bin;%PATH%"
)

where rc >nul 2>nul
if errorlevel 1 (
    echo [clion-clangcl-env] rc.exe is still missing after vcvars64.bat.
    exit /b 1
)

exit /b 0
