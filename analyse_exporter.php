<?php

// capture and compare the output/s of both ./waf and cmake builds, using VERBOSE mode in both

//was any parameter passed?
$arg1 = isset($argv[1]) ? $argv[1] : '';
if ($arg1 == 'clean' ) { 
    //clean up previous files
    @unlink('waf_build_debug.txt');
    @unlink('cmake_build_debug.txt');
    print("Cleaned previous debug output files.\n");
    exit(0);
}

function _shell_exec($cmd) {
    print("Executing command: $cmd\n");
    return shell_exec($cmd);
}
// use proc_open to capture output in real-time
function _proc_open($cmd ) { 
    $descriptorspec = array(
        1 => array("pipe", "w"),  // stdout is a pipe that the child will write to
        2 => array("pipe", "w")   // stderr is a pipe that the child will write to
    );
    $process = proc_open($cmd, $descriptorspec, $pipes);
    if (is_resource($process)) {
        $output = '';
        while ($line = fgets($pipes[1])) {
            echo $line; // real-time output
            $output .= $line;
        }
        while ($line = fgets($pipes[2])) {
            echo $line; // real-time error output
            $output .= $line;
        }
        fclose($pipes[1]);
        fclose($pipes[2]);
        proc_close($process);
        return $output;
    }
    return false;
}

chdir('/home/buzz2/ardupilot');

print("Starting build debug capture...\n");

// only do this if waf_build_debug.txt doesnt exist.
if (!file_exists('waf_build_debug.txt')) {
    $bld_waf_debug = _shell_exec('rm -rf build ;./waf configure  ; ./waf copter -v 2>&1');
    print("Waf build complete.\n");
    file_put_contents('waf_build_debug.txt', $bld_waf_debug);
    print("Waf build debug output captured to waf_build_debug.txt\n");
}
if (!file_exists('cmake_build_debug.txt')) {
    //$bld_cmake_debug = _shell_exec('rm -rf build2 ; ./waf cmake_exporter 2>/dev/null >/dev/null ; cd build2 ; cmake .. ; make VERBOSE=9 bin_arducopter 2>&1');
    $bld_cmake_debug = _shell_exec('rm -rf build2 ; ./waf cmake_exporter 2>/dev/null >/dev/null ; cd build2 ; cmake .. ; make VERBOSE=9 -i -j 16 2>&1');
    print("CMake build complete.\n");
    file_put_contents('cmake_build_debug.txt', $bld_cmake_debug);
    print("CMake build debug output captured to cmake_build_debug.txt\n");
}

function given_long_compile_command_work_out_source_file_name_and_path($cmd_line) {
    // find the last argument that ends with .c or .cpp
    $parts = explode(' ', $cmd_line);
    $source_file = '';
    foreach ($parts as $part) {
        if (preg_match('/\.(c|cpp)$/', $part)) {
            $source_file = $part;
        }
    }
    //strip /home/buzz2/ardupilot/ prefix if it exists
    $source_file = preg_replace('/^\/home\/buzz2\/ardupilot\//', '', $source_file);
    return $source_file;
}
function given_long_compile_command_work_out_object_file_name_and_path($cmd_line) {
    // find the argument that comes after the -o
    $parts = explode(' ', $cmd_line);
    $obj_file = '';
    $next_is_obj = false;
    foreach ($parts as $part) {
        if ($next_is_obj) {
            $obj_file = $part;
            break;
        }
        if ($part == '-o') {
            $next_is_obj = true;
        }
    }
    return $obj_file;
}

function given_relative_source_file_path_work_out_working_directory($rel_path) {
    // if its empty, just return.
    if (empty($rel_path)) {
        return '';
    }
    // if starts with ../../modules
    if (preg_match('/^\.\.\/\.\.\/modules\//', $rel_path)) {
        return 'build/sitl';
    }
    // if starts with ../../libraries
    if (preg_match('/^\.\.\/\.\.\/libraries\//', $rel_path)) {
        return 'build/sitl';
    }
    if (preg_match('/^libraries\/AP_Scripting\/lua_generated_bindings.cpp/', $rel_path)) {
        return 'build/sitl';
    }
    if (preg_match('/dsdlc_generated/', $rel_path)) {
        return 'build/sitl';
    }
    if (preg_match('/^..\/..\/ArduCopter\//', $rel_path)) {
        return 'build/sitl';
    }
    // add plane and rover etc , todo

    // eg libraries/AC_AttitudeControl/AC_CommandModel.cpp
    $parts = explode('/', $rel_path);
    array_pop($parts); // remove last part (the file name)
    $dir_path = implode('/', $parts);
    return $dir_path;
    // eg ../../modules/littlefs/lfs_util.c

}

# test the above two.


print("Analyzing build debug outputs...\n");
//parse all build and link commands from both and compare them.
// build a list of commands for each

$waf_cmds = array();
// WAF 
$bld_debug = file_get_contents('waf_build_debug.txt');
print "WAF linecount: " . strlen($bld_debug) . "\n";
$lines = explode("\n", $bld_debug);
print "WAF lines: " . count($lines) . "\n";
//
//[  13/1396] Compiling libraries/AC_AttitudeControl/AC_CommandModel.cpp
//09:58:59 runner ['/usr/lib/ccache/g++', '-std=gnu++11', '-fdata-sections', '-ffunction-sections', '-fno-exceptions', '-fsigned-char', '-Wall', '-Wextra', '-Wpointer-arith', '-Wno-unused-parameter', '-Wno-missing-field-initializers', '-Wno-redundant-decls', '-Wno-unknown-pragmas', '-Wno-expansion-to-defined', '-Werror=reorder', '-Werror=cast-align', '-Werror=attributes', '-Werror=format-security', '-Werror=format-extra-args', '-Werror=enum-compare', '-Werror=format', '-Werror=array-bounds', '-Werror=uninitialized', '-Werror=init-self', '-Werror=narrowing', '-Werror=return-type', '-Werror=switch', '-Werror=sign-compare', '-Werror=type-limits', '-Werror=undef', '-Werror=unused-result', '-Werror=shadow', '-Werror=unused-value', '-Werror=unused-variable', '-Werror=delete-non-virtual-dtor', '-Wfatal-errors', '-Wno-trigraphs', '-Werror=parentheses', '-DARDUPILOT_BUILD', '-Wuninitialized', '-Warray-bounds', '-Wno-format-contains-nul', '-Werror=unused-but-set-variable', '-fsingle-precision-constant', '-Wno-psabi', '-Werror=suggest-override', '-Werror=implicit-fallthrough', '-Werror=maybe-uninitialized', '-Werror=duplicated-cond', '-Werror=sizeof-pointer-div', '-Werror=use-after-free', '-D__AP_LINE__=__LINE__', '-DAP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED=HAL_GCS_ENABLED&&HAL_RALLY_ENABLED', '-DAC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT=HAL_GCS_ENABLED&&AP_FENCE_ENABLED', '-DHAL_NAVEKF2_AVAILABLE=1', '-DCANARD_64_BIT=1', '-Werror=float-equal', '-Werror=missing-declarations', '-DAP_NETWORKING_ENABLED=1', '-O3', '-MMD', '-ffile-prefix-map=../..=../..', '-include', 'ap_config.h', '-DHAL_HAVE_AP_ROMFS_EMBEDDED_H', '-include', 'ap_config.h', '-Ilibraries', '-Ilibraries/GCS_MAVLink', '-Imodules/DroneCAN/libcanard/dsdlc_generated/include', '-I../../modules/DroneCAN/libcanard', '-I../../libraries/AP_DroneCAN/canard', '-I.', '-I../../libraries', '-I../../modules/littlefs', '-I../../libraries/AP_Common/missing', '-I../../modules/lwip/src/include', '-I../../libraries/AP_Networking/config', '-I../../libraries/AP_Networking/lwip_hal/include', '-DAP_BARO_PROBE_EXTERNAL_I2C_BUSES=1', '-DAP_CUSTOMCONTROL_ENABLED=1', '-DAP_DDS_ENABLED=0', '-DAP_SCRIPTING_CHECKS=1', '-DCANARD_ALLOCATE_SEM=1', '-DCANARD_ENABLE_ASSERTS=1', '-DCANARD_ENABLE_CANFD=1', '-DCANARD_ENABLE_DEADLINE=1', '-DCANARD_IFACE_ALL=3', '-DCANARD_MULTI_IFACE=1', '-DCONFIG_HAL_BOARD=HAL_BOARD_SITL', '-DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE', '-DDRONECAN_CXX_WRAPPERS=1', '-DENABLE_ONVIF=0', '-DLUA_32BITS=1', '-DUSE_USER_HELPERS=1', '../../libraries/AC_AttitudeControl/AC_CommandModel.cpp', '-c', '-olibraries/AC_AttitudeControl/AC_CommandModel.cpp.0.o']
//[  14/1396] Compiling libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
//
//[4/4] Linking build/sitl/liblittlefs.a
//09:58:54 runner ['/usr/bin/ar', 'rcs', 'liblittlefs.a', 'modules/littlefs/lfs.c.1.o', 'modules/littlefs/lfs_util.c.1.o', 'modules/littlefs/bd/lfs_filebd.c.1.o']
$is_compile = false;
$is_link = false;
foreach ($lines as $line) {
    if (preg_match('/Compiling (.*)$/', $line, $match)) {
        $is_compile = $match[1]; $is_link = false;
        $src_path_c = $is_compile;
        //print "WDebug: Found compile for " . $src_path_c . "\n";
    }
    if (preg_match('/Linking (.*)$/', $line, $match)) {
        $is_compile = false; $is_link = $match[1];
        $is_link = preg_replace('/^build\/sitl\//', '', $is_link); #before eg: build/sitl/xxxxxx.a
        $is_link = preg_replace('/lib\//', '', $is_link); #before eg: lib/libArduCopter_libs.a
        $is_link = preg_replace('/bin\//', 'bin_', $is_link); #before eg: bin/arducopter
        //print "WDebug: Found link for " . $is_link . "\n"; 
    }
    // 'runner' line occurs right after the compiling and linking line.
    if (preg_match('/runner \[(.*)\]$/', $line, $match)) {
        $remove_quotes_and_commas = str_replace(array("'", ","), "", $match[1]);
        if ($is_compile) {
            $waf_cmds[] = ["COMPILE", $is_compile,$remove_quotes_and_commas];
            $is_compile = false;
        } elseif ($is_link) {
            $waf_cmds[] = ["LINK"   , $is_link,   $remove_quotes_and_commas];
            $is_link = false;
        }
    } 
}
// count commands in each
print("WAF build commands captured: " . count($waf_cmds) . "\n");
foreach ($waf_cmds as $cmd) {
    // work out the pwd for the compile command
    if ($cmd[0] == 'COMPILE') {
        $src_file = given_long_compile_command_work_out_source_file_name_and_path($cmd[2]); 
        $pwd = given_relative_source_file_path_work_out_working_directory($src_file);
        //print "Debug: Compile source file: " . $src_file . " in dir: " . $pwd . "\n";
        $cmd[] = $pwd; 
    }
    // print "WAF:";
    // print_r($cmd);
    // print "\n";
    print("WAF0:".$cmd[0]."\n");
    print("WAF1:".$cmd[1]."\n");
    print("WAF2:".$cmd[2]."\n");
    // if ( $cmd[3]??false) print("WAF3:".$cmd[3]."\n");
}
//exit;
print "---------------------------------------------------------------------------------\n";
// CMAKE 


function given_cmake_build__workout_current_dir($cmd_line) {
    // doesit have ' ArduCopter/' in it, precisely?
    if (preg_match('/ ArduCopter\//', $cmd_line)) {
        return 'build2/'; # build2/ArduCopter
    }
    // CMakeFiles/littlefs.dir/modules/littlefs/lfs.c.o
    if (preg_match('/ CMakeFiles\/littlefs.dir\/modules\/littlefs\//', $cmd_line)) {
        return 'build2/';
    }
    // CMakeFiles/Blimp_libs.dir
    if (preg_match('/ CMakeFiles\/Blimp_libs.dir\//', $cmd_line)) {
        return 'build2/Blimp';
    }
    $src_file = given_long_compile_command_work_out_source_file_name_and_path($cmd_line);

    $obj_file = given_long_compile_command_work_out_object_file_name_and_path($cmd_line);
    $short_obj_file = basename($obj_file);


    // find the cd /some/path && part
    if (preg_match('/cd (.*) &&/', $cmd_line, $match)) {
        return $match[1];
    }
    return '';
}


$cmake_cmds = array();
$bld_debug = file_get_contents('cmake_build_debug.txt');
print "CMAKE linecount: " . strlen($bld_debug) . "\n";
$lines = explode("\n", $bld_debug);
print "CMAKE lines: " . count($lines) . "\n";
//[100%] Building CXX object ArduCopter/CMakeFiles/bin_arducopter.dir/takeoff_check.cpp.o
//cd /home/buzz2/ardupilot/build2/ArduCopter && /usr/lib/ccache/c++ -DAPM_BUILD_DIRECTORY=APM_BUILD_ArduCopter -DAP_BARO_PROBE_EXTERNAL_I2C_BUSES=1 -DAP_BUILD_TARGET_NAME=\"ArduCopter\" -DAP_CUSTOMCONTROL_ENABLED=1 -DAP_DDS_ENABLED=0 -DAP_SCRIPTING_CHECKS=1 -DCANARD_ALLOCATE_SEM=1 -DCANARD_ENABLE_ASSERTS=1 -DCANARD_ENABLE_CANFD=1 -DCANARD_ENABLE_DEADLINE=1 -DCANARD_IFACE_ALL=3 -DCANARD_MULTI_IFACE=1 -DCONFIG_HAL_BOARD=HAL_BOARD_SITL -DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE -DDRONECAN_CXX_WRAPPERS=1 -DENABLE_ONVIF=0 -DFRAME_CONFIG=MULTICOPTER_FRAME -DLUA_32BITS=1 -DUSE_USER_HELPERS=1 -I/home/buzz2/ardupilot/. -I/home/buzz2/ardupilot/.. -I/home/buzz2/ardupilot/../.. -I/home/buzz2/ardupilot/../../.. -I/home/buzz2/ardupilot/ArduCopter/../build/sitl/libraries -I/home/buzz2/ardupilot/ArduCopter/../build/sitl/libraries/GCS_MAVLink -I/home/buzz2/ardupilot/ArduCopter/../build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include -I/home/buzz2/ardupilot/ArduCopter/../modules/DroneCAN/libcanard -I/home/buzz2/ardupilot/ArduCopter/../libraries/AP_DroneCAN/canard -I/home/buzz2/ardupilot/build/sitl -I/home/buzz2/ardupilot/libraries -I/home/buzz2/ardupilot/modules/littlefs -I/home/buzz2/ardupilot/libraries/AP_Common/missing -I/home/buzz2/ardupilot/modules/lwip/src/include -I/home/buzz2/ardupilot/libraries/AP_Networking/config -I/home/buzz2/ardupilot/libraries/AP_Networking/lwip_hal/include -I/home/buzz2/ardupilot/build/sitl/libraries -I/home/buzz2/ardupilot/build/sitl/libraries/GCS_MAVLink -I/home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated/include -I/home/buzz2/ardupilot/modules/DroneCAN/libcanard -I/home/buzz2/ardupilot/libraries/AP_DroneCAN/canard -std=gnu++11 -fdata-sections -ffunction-sections -fno-exceptions -fsigned-char -Wall -Wextra -Wpointer-arith -Wno-unused-parameter -Wno-missing-field-initializers -Wno-redundant-decls -Wno-unknown-pragmas -Wno-expansion-to-defined -Werror=reorder -Werror=cast-align -Werror=attributes -Werror=format-security -Werror=format-extra-args -Werror=enum-compare -Werror=format -Werror=array-bounds -Werror=uninitialized -Werror=init-self -Werror=narrowing -Werror=return-type -Werror=switch -Werror=sign-compare -Werror=type-limits -Werror=undef -Werror=unused-result -Werror=unused-value -Werror=unused-variable -Werror=delete-non-virtual-dtor -Wfatal-errors -Wno-trigraphs -Werror=parentheses -DARDUPILOT_BUILD -Wuninitialized -Warray-bounds -Wno-format-contains-nul -Werror=unused-but-set-variable -fsingle-precision-constant -Wno-psabi -Werror=suggest-override -Werror=implicit-fallthrough -Werror=maybe-uninitialized -Werror=duplicated-cond -Werror=sizeof-pointer-div -Werror=use-after-free -D__AP_LINE__=__LINE__ -DAP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED=HAL_GCS_ENABLED -DAC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT=HAL_GCS_ENABLED -DHAL_NAVEKF2_AVAILABLE=1 -DCANARD_64_BIT=1 -Werror=float-equal -Werror=missing-declarations -DAP_NETWORKING_ENABLED=1 -O3 -MMD -ffile-prefix-map=../..=../.. -include ap_config.h -DHAL_HAVE_AP_ROMFS_EMBEDDED_H -MD -MT ArduCopter/CMakeFiles/bin_arducopter.dir/takeoff_check.cpp.o -MF CMakeFiles/bin_arducopter.dir/takeoff_check.cpp.o.d -o CMakeFiles/bin_arducopter.dir/takeoff_check.cpp.o -c /home/buzz2/ardupilot/ArduCopter/takeoff_check.cpp


//[ 77%] Linking CXX executable bin_arducopter
// cd /home/buzz2/ardupilot/build2/ArduCopter && /usr/local/bin/cmake -E cmake_link_script CMakeFiles/bin_arducopter.dir/link.txt --verbose=9
// /usr/lib/ccache/c++ -std=gnu++11 -fdata-sections -ffunction-sections -fno-exceptions -fsigned-char -Wall -Wextra -Wpointer-arith -Wno-unused-parameter -Wno-missing-field-initializers -Wno-redundant-decls -Wno-unknown-pragmas -Wno-expansion-to-defined -Werror=reorder -Werror=cast-align -Werror=attributes -Werror=format-security -Werror=format-extra-args -Werror=enum-compare -Werror=format -Werror=array-bounds -Werror=uninitialized -Werror=init-self -Werror=narrowing -Werror=return-type -Werror=switch -Werror=sign-compare -Werror=type-limits -Werror=undef -Werror=unused-re...
$is_compile = false;
$is_link = false;
foreach ($lines as $line) {
    if (preg_match('/Building (CXX|C|ASM) object (.*)$/', $line, $match)) {
        $is_compile = $match[2]; $is_link = false;
        $is_cxx = $match[1]=='CXX'?true:false;
        
        $path_stripped_obj = basename($is_compile);
        // strip everything up to the double underscore and slash __/
        $path_stripped = preg_replace('/^.*__\//', '', $is_compile);
        // remove a .o suffix if its there.
        $path_stripped = preg_replace('/\.o$/', '', $path_stripped);
        $is_compile = $path_stripped;
        // before ArduCopter/CMakeFiles/bin_arducopter.dir/AP_Arming_Copter.cpp   after ArduCopter/AP_Arming_Copter.cpp
        $is_compile = preg_replace('/CMakeFiles\//', '', $is_compile);
        $is_compile = preg_replace('/bin[\w_]+.dir\//', '', $is_compile);
        $is_compile = preg_replace('/littlefs.dir\//', '', $is_compile);
        $is_compile = preg_replace('/objs[\w_]+.dir\//', '', $is_compile);
        $is_compile = preg_replace('/[\w_]+.dir\//', '', $is_compile);

        // doesit start with libraries?
        $is_library = false;
        if (preg_match('/^libraries\//', $is_compile)) {
            $is_library = true;
        }
        if ($is_compile === 'ArduCopter/landing_gear.cpp') {
            $x = 1; // break here
        }
        if ($is_compile === 'ArduCopter/UserCode.cpp') {
            $x = 1; // break here
        }
        if ($is_compile === 'ArduPlane/AP_ExternalControl_Plane.cpp') {
            $x = 1; // break here
        }  
        //print "CDebug: Found compile for " . $is_compile . "\n";
        continue;
    }
    if (preg_match('/Linking CXX executable (.*)$/', $line, $match)) {
        $is_compile = false; $is_link = $match[1];
        //print "Debug: Found link for " . $is_link . "\n";
        continue;
    }
    if (preg_match('/Linking C static library (.*)$/', $line, $match)) {
        $is_compile = false; $is_link = $match[1];
        $is_link = str_replace('../', '', $is_link); // remove ../ prefix if it exists
        //print "Debug: Found link for " . $is_link . "\n";
        continue;
    }
    // the actual compile/link command line occurs after the above lines
    if ($is_compile && preg_match('/^(.*)$/', $line, $match)) {
        $foo = $match[1];
        if (preg_match('/^\/usr\/lib\/ccache/', $foo)) {
            $cmake_cmds[] = ["COMPILE", $is_compile, trim($foo)];
            $is_compile = false;
        }
        // "/usr/bin/ranlib liblittlefs.a" is skipped ok.
        if (preg_match('/^\/usr\/bin\/ranlib/', $foo)) {
            $is_compile = false;
        }
        // "/usr/local/bin/cmake" is skipped ok.
        if (preg_match('/^\/usr\/local\/bin\/cmake/', $foo)) {
            $is_compile = false;
        }
        // cd /x/x/x && /usr/lib/ccache/c++ ...
        if (preg_match('/^cd ([\w\d\/]+) && \/usr\/lib\/ccache\//', $foo)) {
            //$is_compile = false;
            $src_file = given_long_compile_command_work_out_source_file_name_and_path($foo);
            if ($src_file === $is_compile) {
                $cmake_cmds[] = ["COMPILE", $is_compile, trim($foo)];
                $is_compile = false;// we said we were compiling it, and it matches.
            } else { 
                //register both $src_file and $is_compile as sometimes this happens.
                $cmake_cmds[] = ["COMPILE", $src_file, trim($foo)];
                //$cmake_cmds[] = ["COMPILE", $is_compile, trim($foo)];
                $is_compile = false;
            }
        }

        if ($is_compile && preg_match('/^\/usr\//', $foo)) {
            $x = 1; // break here
        }

    } 
    if ($is_link && preg_match('/^(.*)$/', $line, $match)) {
        $foo = $match[1];
        //$is_link = false;
        // if $foo starts with '/usr/bin/ar ' then $is_link = false;
        if (preg_match('/^\/usr\/bin\/ar /', $foo)) {
            $cmake_cmds[] = ["LINK"   , $is_link,    trim($foo)];
            $is_link = false;
        }
        if ($is_link && preg_match('/^\/usr\/bin\/ld/', $foo)) {
            $cmake_cmds[] = ["LINK"   , $is_link,    trim($foo)];
            $is_link = false;
        }
        if ($is_link && preg_match('/^\/usr\/bin\//', $foo)) {
            $x = 1; // break here
        }
    } 
}
// count commands in each
foreach ($cmake_cmds as $idx =>$cmd) {
    // skip ones that start with build
    // if (preg_match('/^build/', $cmd[1])) {
    //     continue;
    // }
    if ($cmd[0] == 'COMPILE') {
        //if $cmd[1] starts with copter.dir/ at the start, strip that off, as cmake uses that and waf doersnt.
        if (preg_match('/^[\w\d_]+.dir\//i', $cmd[1])) {
            $cmake_cmds[$idx][1] = preg_replace('/^[\w\d_]+.dir\//i', '', $cmd[1]);
        }// makes them compatible with waf.
       

        $src_file = given_long_compile_command_work_out_source_file_name_and_path($cmd[2]);
        $pwd = given_cmake_build__workout_current_dir($cmd[2]);
        //print "Debug: Compile source file: " . $src_file . " in dir: " . $pwd . "\n";

        if ($src_file === 'ArduCopter/UserCode.cpp') {
            $x = 1; // break here
        }
        if ($src_file === 'ArduPlane/AP_ExternalControl_Plane.cpp') {
            $x = 1; // break here
        }  

        $cmd[] = $pwd; 
    }
    //print "CMAKE:";
    //print_r($cmd);
    //print "\n";
    // print with CMAKE prefix $cmd[0]
    print("CMAKE0:".$cmd[0]."\n");
    print("CMAKE1:".$cmd[1]."\n");
    print("CMAKE2:".$cmd[2]."\n");
    // if ( $cmd[3]??false) print("CMAKE3:".$cmd[3]."\n");
}
print("CMAKE build commands captured: " . count($cmake_cmds) . "\n");
print("... analysing done\n");

// $find = 'ArduCopter/UserParameters.cpp';
// print("\n-----------------------------------------------------------------------\nSearching for $find in both builds...\n");
// // search in waf cmds
// foreach ($waf_cmds as $cmd) {
//     if ($cmd[1] == $find) {
//         print("Found in WAF build (exact): " . $cmd[0] ." ". $cmd[1] ." ". $cmd[2] ."\n");
//         break;
//     }
// }
// foreach ($cmake_cmds as $cmd) {
//     if ($cmd[1] == $find) {
//         print("Found in CMAKE build (exact): " . $cmd[0] ." ". $cmd[1] ." ". $cmd[2] ."\n");
//         break;
//     }
//     // regex find it in [0] or [1] or [2], then print it.
//     $find_regex = preg_quote($find, '/');
//     if (preg_match('/'.$find_regex.'/', $cmd[0]) || preg_match('/'.$find_regex.'/', $cmd[1])|| preg_match('/'.$find_regex.'/', $cmd[2])) {
//         print("Found in CMAKE build (regex): " . $cmd[0] ." ". $cmd[1] ." ". $cmd[2] ."\n");
//         break;
//     }
// }
// exit;

//now do a simple compare , using waf list as the master list.
print("Comparing WAF and CMAKE build commands...\n");
$missing_in_cmake = array();
foreach ($waf_cmds as $waf_cmd) {
    $found = false;
    foreach ($cmake_cmds as $cmake_cmd) {
        if ($waf_cmd[0] == $cmake_cmd[0]) {
            if ($waf_cmd[1] == $cmake_cmd[1]) {
                // match found
                $found = true;
                print("MATCHED: " . $waf_cmd[0] . " " . $waf_cmd[1] . "\n");
                break;
            }
        }
    }
    if (!$found) {
        //$missing_in_cmake[] = $waf_cmd;
        print("CMAKE MISSING:" . $waf_cmd[0] . " " . $waf_cmd[1] . "\n");
    }
}




