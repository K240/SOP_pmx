cmake_minimum_required( VERSION 3.18 )

project( SOP_pmx )

list( APPEND CMAKE_PREFIX_PATH "$ENV{HFS}/toolkit/cmake" )

find_package( Houdini REQUIRED )

set( library_name SOP_pmx )

# macOS (Houdini 22+): houdini_configure_target() runs hconfig to resolve the
# user dso path; hconfig can fail during CMake if HOUDINI_* env is incomplete
# (e.g. KeyError: 'HHP'). Set INSTDIR explicitly from the SDK version file.
if( APPLE )
    if( NOT DEFINED ENV{HFS} OR "$ENV{HFS}" STREQUAL "" )
        message( FATAL_ERROR "HFS must be set to the Houdini Resources directory on macOS." )
    endif()
    set( _houdini_version_cmake "$ENV{HFS}/toolkit/cmake/HoudiniConfigVersion.cmake" )
    if( EXISTS "${_houdini_version_cmake}" )
        file( READ "${_houdini_version_cmake}" _houdini_ver_file )
        string( REGEX MATCH "set\\( version_major ([0-9]+)" _ "${_houdini_ver_file}" )
        set( _houdini_ver_major ${CMAKE_MATCH_1} )
        string( REGEX MATCH "set\\( version_minor ([0-9]+)" _ "${_houdini_ver_file}" )
        set( _houdini_ver_minor ${CMAKE_MATCH_1} )
        set( _houdini_dso_installdir "$ENV{HOME}/Library/Preferences/houdini/${_houdini_ver_major}.${_houdini_ver_minor}/dso" )
    endif()
    if( NOT _houdini_dso_installdir )
        message( FATAL_ERROR "Could not derive macOS dso path from ${_houdini_version_cmake}. Check HFS." )
    endif()
endif()

# Windows: hython prints hou.homeHoudiniDirectory() in UTF-8 (for example a localized OneDrive Documents folder).
# CMake must decode stdout as UTF-8; otherwise the path mojibakes and dso lands in the wrong tree.
# MSVC .vcxproj also mishandles non-ASCII OutDir, so we pass an 8.3 short path (ASCII) as INSTDIR.
if( WIN32 )
    set( _hython_exe "$ENV{HFS}/bin/hython${CMAKE_EXECUTABLE_SUFFIX}" )
    execute_process(
        COMMAND "${_hython_exe}" -c "import hou; print(hou.homeHoudiniDirectory())"
        OUTPUT_VARIABLE _houdini_user_pref
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ENCODING UTF-8
        RESULT_VARIABLE _houdini_home_rc
    )
    if( NOT _houdini_home_rc EQUAL 0 )
        message( FATAL_ERROR "Could not run ${_hython_exe} to get hou.homeHoudiniDirectory() (exit ${_houdini_home_rc}). Is HFS set correctly?" )
    endif()
    set( _houdini_dso_long "${_houdini_user_pref}/dso" )
    file( MAKE_DIRECTORY "${_houdini_dso_long}" )
    file( TO_NATIVE_PATH "${_houdini_dso_long}" _houdini_dso_native )
    set( _houdini_ps_shortpath [=[& { $p = [System.IO.Path]::GetFullPath($args[0]); if (-not (Test-Path -LiteralPath $p)) { New-Item -ItemType Directory -Force -Path $p | Out-Null }; (New-Object -ComObject Scripting.FileSystemObject).GetFolder($p).ShortPath }]=] )
    execute_process(
        COMMAND powershell -NoProfile -Sta -ExecutionPolicy Bypass -Command "${_houdini_ps_shortpath}" "${_houdini_dso_native}"
        OUTPUT_VARIABLE _houdini_win_dso_installdir
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _houdini_win_shortpath_rc
    )
    if( NOT _houdini_win_shortpath_rc EQUAL 0 OR "${_houdini_win_dso_installdir}" STREQUAL "" )
        message( FATAL_ERROR "Could not resolve Windows DSO output path to a short (ASCII) path. hython pref dir: ${_houdini_dso_long}. PowerShell exit: ${_houdini_win_shortpath_rc}" )
    endif()
endif()

# Code generation for the embedded DS file.
houdini_generate_proto_headers( FILES
    SOP_pmx_reader.C
)

# Add a library and its source files.
add_library( ${library_name} SHARED
    SOP_pmx_reader.C
    SOP_pmx_reader.h
    SOP_pmx_common.h
    pmx_parser.cpp
    pmx_parser.h
)

# Link against the Houdini libraries, and add required include directories and
# compile definitions.
target_link_libraries( ${library_name} Houdini )

# Include ${CMAKE_CURRENT_BINARY_DIR} for the generated header.
target_include_directories( ${library_name} PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Sets several common target properties, such as the library's output directory.
if( APPLE AND _houdini_dso_installdir )
    houdini_configure_target( ${library_name} INSTDIR "${_houdini_dso_installdir}" )
elseif( WIN32 AND _houdini_win_dso_installdir )
    houdini_configure_target( ${library_name} INSTDIR "${_houdini_win_dso_installdir}" )
else()
    houdini_configure_target( ${library_name} )
endif()
