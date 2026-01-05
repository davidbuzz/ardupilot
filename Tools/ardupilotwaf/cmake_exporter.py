#! /usr/bin/env python
# -*- encoding: utf-8 -*-
# Original Author: Michel Mooij, michel.mooij7@gmail.com
# Michael licensed the orignal version of this file as MIT , see https://bitbucket.org/Moo7/waftools/src/master/LICENSE
# slightly tweaked by buzz to fit ardupilot waf system, but not enough to change the license.

# buzz cherry picked  just this file by buzz from here and renamed and did a few search/replace:
# https://bitbucket.org/Moo7/waftools/src/master/waftools/cmake.py
# turned a few 'cmake' references into 'cmake_exporter' to be more specific.
# use: 
# ./waf configure --board=sitl
# ./waf configure --board=CubeOrange
# ./waf cmake_exporter
# ./waf build plane

'''
Summary
-------
Generate *cmake* files of all C/C++ programs, static- and shared libraries
that have been defined within a *waf* build environment.
Once exported to *cmake*, all exported (C/C++) tasks can be build without 
any further need for, or dependency, to the *waf* build system itself.

**cmake** is an open source cross-platform build system designed to build, test 
and package software. It is available for all major Desktop Operating Systems 
(MS Windows, all major Linux distributions and Macintosh OS-X).
See http://www.cmake.org for a more detailed description on how to install
and use it for your particular Desktop environment.


Description
-----------
When exporting this *waf* project data, a single top level **CMakeLists.txt** file
will be exported in the top level directory of your *waf* build environment. 
This *cmake* build file will contain references to all exported *cmake*
build files of each individual C/C++ build task. It will also contain generic 
variables and settings (e.g compiler to use, global preprocessor defines, link
options and so on).

Example below presents an overview of an environment in which *cmake* 
build files already have been exported::

        .
        ├── components
        │   └── clib
        │       ├── program
        │       │   └── wscript
        │       ├── shared
        │       │   └── wscript
        │       └── static
        │           └── wscript
        │
        ├── CMakeLists.txt
        └── wscript


Usage
-----
Tasks can be exported to *cmake* using the command, as shown in the
example below::

        $ waf cmake_exporter

All exported *cmake* build files can be removed in 'one go' using the *cmake*
*cleanup* option::

        $ waf cmake_exporter --cmake_exporter_clean

Tasks generators to be excluded can be marked with the *skipme* option 
as shown below::

    def build(bld):
        bld.program(name='foo', src='foobar.c', cmake_skip=True)

'''


import os
from modules.waf.waflib import TaskGen
from waflib.Build import BuildContext
from waflib import Utils, Logs, Context, Errors
import waftools
from waftools import deps

# quick utility function to run a shell command and print its output to stdout
def run_shell_cmd(cmd):
    import subprocess
    print('Running shell command: %s ' % cmd)
    process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    if process.returncode != 0:
        print('Error running command: %s ' % cmd)
        print('stderr: %s ' % stderr.decode())
    else:
        print('stdout: %s ' % stdout.decode())
        print('stderr: %s ' % stderr.decode())
    return process.returncode, stdout.decode(), stderr.decode()

# like the above, but capture output and return as string, NO PRINTING.
def run_shell_cmd_capture_output(cmd):
    import subprocess
    process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    return process.returncode, stdout.decode(), stderr.decode()


def options(opt):
    '''Adds command line options for the CMakeExporter *waftool*.

    :param opt: Options context from the *waf* build environment.
    :type opt: waflib.Options.OptionsContext
    '''
    opt.add_option('--cmake_exporter', dest='cmake_exporter', default=False, action='store_true', help='select cmake for export/import actions')
    opt.add_option('--cmake_exporter_clean', dest='clean', default=False, action='store_true', help='delete exported files')


def configure(conf):
    '''Method that will be invoked by *waf* when configuring the build 
    environment.
    
    :param conf: Configuration context from the *waf* build environment.
    :type conf: waflib.Configure.ConfigurationContext
    '''
    conf.find_program('cmake_exporter', var='CMAKE_EXPORTER', mandatory=False)

# this data is used in both the parent and subdirs so we dont do objs multiple times.
lib_register = {}
def is_lib_registered(libname):
    global lib_register
    return libname in lib_register
def register_lib(libname):
    global lib_register
    lib_register[libname] = True

class CMakeExporterContext(BuildContext):
    '''export C/C++ tasks to CMakeExporter.'''
    cmd = 'cmake_exporter'
    BOARDNAME = '' #sitl, CubeOrange, etc

    # rip off of BuildContext.__call__ in waflib/Build.py
    def __call__(self, *k, **kw):
        #kw['bld'] = self
        #ret = TaskGen.task_gen(*k, **kw)
        #self.task_gen_cache_names = {} # reset the cache, each time
        #self.add_to_group(ret, group=kw.get('group'))
        #return ret

        name = kw.get('name', None)
        features = kw.get('features', [])
        source = kw.get('source', [])
        output_dir = kw.get('output_dir', None)
        export_includes = kw.get('export_includes', True)
        xgroup = kw.get('group', None) #rare, for dynamic_sources

        # uncomment this block and run './waf make_exporter' to see the flow of all taskgens
        # we hook into the relevant ones.
        known=False
        if 'ap_library_object' in features:
            #print ('ap_library_object name: %s ' % name)
            known=True
        if 'cxx' in features:
            #print ('cxx name: %s ' % name)
            known=True
        if 'c' in features:
            print ('c name: %s ' % name)
            known=True
        if 'cstdlib' in features:
            print ('cstdlib name: %s ' % name)
            known=True
        if 'droncangen' in features:
            print ('droncangen name: %s ' % name)
            known=True
        if 'git_submodule' in features:
            print ('git_submodule name: %s ' % name)
            known=True
        if 'mavgen' in features:
            print ('mavgen name: %s ' % name)
            known=True
        if 'ap_version' in features or name=='ap_version':
            print ('ap_version name: %s ' % name)
            known=True
        if xgroup == 'dynamic_sources':
            #print ('dynamic_sources name: %s ' % name)
            known=True
        if known==False:
            print ('unknown: name: %s features: %s ' % (name, features))

        import os
        from pathlib import Path

        waf_bld_dir = self.bldnode.abspath() # eg /home/buzz2/ardupilot/build/sitl/ from waf
        # btreakup waf path into 3 rihgt-most parts and discard hte rest.
        self.bld_root_dir,part2,BOARDNAME = waf_bld_dir.rsplit('/',2)
        # bld_root_dir = '/home/buzz2/ardupilot',
        # part2 = 'build',
        # BOARDNAME = 'CubeOrange'  or 'sitl' etc
        self.BOARDNAME = BOARDNAME # not sure its needed.
        self.cmake_build_dir = 'build2/' + BOARDNAME # rel to bld_root_dir, eg build2/CubeOrange or build2/sitl
        self.cmake_full_build_dir = self.bld_root_dir + '/build2/' + BOARDNAME # eg /home/buzz2/ardupilot/build2/CubeOrange
        #pwd = os.getcwd()
        #print("pwd: %s " % pwd_output)
        #actual_root_path = Path(bld_root_dir).resolve()
        #print("root: %s " % actual_root_path)
        pwd = os.getcwd()
        if pwd != self.bld_root_dir:
            os.chdir(self.bld_root_dir)
            pwd = os.getcwd()
            print('Changed working dir to: %s ' % (pwd))

        namelist = ['mavlink','dronecan','ap_version','ap_config','mavgen']
        if name in namelist:
            print('Debug: CMakeExporterContext __call__ for name: %s ' % name)
            if name == 'mavlink':
                shell_cmd1 = 'mkdir -p '+self.cmake_build_dir+' ; cd '+self.cmake_build_dir+' ; gcc -std=c99 -Wno-error=missing-field-initializers -Wall -Werror -Wextra -o gen-bindings ../../libraries/AP_Scripting/generator/src/main.c; file gen-bindings ; cd '+pwd
                run_shell_cmd(shell_cmd1)
                # needs to be run from build2/sitl/ folder as thats where gen-bindings wants to run from.
                shell_cmd2 = 'cd '+self.cmake_build_dir+' ; mkdir -p libraries/AP_Scripting/lua_generated_bindings ;./gen-bindings -o libraries/AP_Scripting/lua_generated_bindings -i ../../libraries/AP_Scripting/generator/description/bindings.desc; ls -l libraries/AP_Scripting/lua_generated_bindings/* ; cd '+pwd
                run_shell_cmd(shell_cmd2)
                #source='modules/mavlink/message_definitions/v1.0/all.xml',
                #output_dir='libraries/GCS_MAVLink/include/mavlink/v2.0/',
                shell_cmd3 = 'mkdir -p '+self.cmake_build_dir+'/libraries/GCS_MAVLink/include/mavlink/v2.0/ ; /usr/bin/python3 ./modules/mavlink/pymavlink/tools/mavgen.py --lang C ./modules/mavlink/message_definitions/v1.0/all.xml -o '+self.cmake_build_dir+'/libraries/GCS_MAVLink/include/mavlink/v2.0/ --wire-protocol=2.0; cd '+pwd
                run_shell_cmd(shell_cmd3)

            if name == 'dronecan':
                #blddir = self.bldnode.abspath() #eg build/sitl from waf, its wrong for cmake, thats build2/sitl    
                #blddir = self.bldnode.abspath() + '/../../build2/sitl' # build2/sitl for cmake
                shell_cmd = '/usr/bin/python3 '+self.bld_root_dir+'/modules/DroneCAN/dronecan_dsdlc/dronecan_dsdlc.py -O'+self.cmake_full_build_dir+'/modules/DroneCAN/libcanard/dsdlc_generated '+self.bld_root_dir+'/modules/DroneCAN/DSDL/ardupilot '+self.bld_root_dir+'/modules/DroneCAN/DSDL/com '+self.bld_root_dir+'/modules/DroneCAN/DSDL/cuav '+self.bld_root_dir+'/modules/DroneCAN/DSDL/dronecan '+self.bld_root_dir+'/modules/DroneCAN/DSDL/mppt '+self.bld_root_dir+'/modules/DroneCAN/DSDL/tests '+self.bld_root_dir+'/modules/DroneCAN/DSDL/uavcan >/dev/null 2>&1'
                run_shell_cmd(shell_cmd)

            if name == 'ap_version':
                #builddir = self.bldnode.abspath() #waf build dir, not cmake . build/sitl
                # shell. `mkdir -p build2/sitl`
                shell_cmd = 'mkdir -p '+self.cmake_build_dir
                print('ap_version: Running shell command: %s ' % shell_cmd)
                run_shell_cmd(shell_cmd)
                # write ap_config.h
                tgt = self.cmake_build_dir+'/ap_config.h'  # eg build2/sitl/ap_config.h
                print('Debug: Writing ap_config.h to: %s ' % tgt)
                with open(tgt, 'w') as f:
                    print('''/* WARNING! */\n#ifndef _AP_CONFIG_H_\n#define _AP_CONFIG_H_\n''',file=f)
                    for strv in self.env['DEFINES']:
                        #value = the bit after the equals sign.
                        k,sep,v = strv.partition('=')
                        print('#define {} {}'.format(k, v), file=f)
                    print('''#endif \n''',file=f)

                # write ap_version.h
                tgt = self.cmake_build_dir+'/ap_version.h' # eg build2/sitl/ap_version.h
                print('Debug: Writing ap_version.h to: %s ' % tgt)
                git_head_version_cmd = 'git rev-parse --short=8 HEAD'
                retcode, git_short_hash, dummy = run_shell_cmd_capture_output(git_head_version_cmd)
                git_short_hash = git_short_hash.strip()
                git_head_version_cmd = 'git rev-parse --short=16 HEAD'
                retcode, git_full_hash, dummy = run_shell_cmd_capture_output(git_head_version_cmd)
                git_full_hash = git_full_hash.strip()
                build_root = self.bld_root_dir.replace('"','\\"') #escape any quotes
                with open(tgt, 'w') as f:
                    print(
f'''// auto-generated 
#pragma once
#ifndef FORCE_VERSION_H_INCLUDE
#error ap_version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif
#define GIT_VERSION "{git_short_hash}"
#define GIT_VERSION_EXTENDED "{git_full_hash}"
#define GIT_VERSION_INT 42
#define AP_BUILD_ROOT "{build_root}"
''', file=f)

                # write hwdef.h from hwdef.dat with 'gen_from_hwdef_dat.py' code
                shell_cmd = '/usr/bin/python3 '+self.bld_root_dir+'/Tools/ardupilotwaf/gen_from_hwdef_dat.py --board='+self.BOARDNAME
                print('ap_version: Running shell command: %s ' % shell_cmd)
                run_shell_cmd(shell_cmd)
                tgt = self.cmake_build_dir+'/hwdef.h' # eg build2/sitl/hwdef.h
                print('Debug: Writing hwdef.h to: %s ' % tgt)



        # call the parent class __call__
        return super().__call__(*k, **kw)

    def execute(self):
        '''Will be invoked when issuing the *cmake_exporter* command.'''
        self.restore()
        if not self.all_envs:
            self.load_envs()
        self.recurse([self.run_dir])
        self.pre_build()

        for group in self.groups:
            for tgen in group:
                try:
                    f = tgen.post
                except AttributeError:
                    pass
                else:
                    f()
        try:
            self.get_tgen_by_name('')
        except Exception:
            pass

        self.cmake = True
        # need self.options to have the clean attribute
        if self.options and hasattr(self.options, 'clean') and self.options.clean:
            cleanup(self)
        else:
            export(self)
            pass
        self.timer = Utils.Timer()


def export(bld):
    '''Exports all C and C++ task generators to cmake.

    :param bld: a *waf* build instance from the top level *wscript*.
    :type bld: waflib.Build.BuildContext
    '''
    boardname = bld.BOARDNAME # aka self.BOARDNAME #sitl, CubeOrange, etc

    cmakes = {}
    loc = bld.path.relpath().replace('\\', '/')  # eg loc = '.'
    #print('ZZCreating new CMakeExporter for location: %s ' % (loc))
    top = CMakeExporter(bld, loc)
    #cmakes[loc] = top
    targets = waftools.deps.get_targets(bld)
    _cmakes = [] # tmp storage to avoid dupes, not  same as self.cmakes
    for tgen in bld.task_gen_cache_names.values():
        if targets and tgen.get_name() not in targets:
            #print('skipping tgen:', tgen.get_name()) # examples, tests, replay, etcetc
            continue
        if getattr(tgen, 'cmake_skipme', False):
            continue
        if set(('c', 'cxx')) & set(getattr(tgen, 'features', [])):
            loc = tgen.path.relpath().replace('\\', '/') # eg loc = 'Rover' or 'ArduPlane' etc
            top.add_tgen(tgen)
            if loc not in _cmakes:
                print('Exporting location: %s ' % (loc))
                _cmakes.append(loc) # to avoid dupes

    top.export() # just export the top one.


def cleanup(bld):
    '''Removes all generated makefiles from the *waf* build environment.
    
    :param bld: a *waf* build instance from the top level *wscript*.
    :type bld: waflib.Build.BuildContext
    '''
    if not bld.options.clean:
        return

    loc = bld.path.relpath().replace('\\', '/')
    CMakeExporter(bld, loc).cleanup()
    targets = waftools.deps.get_targets(bld)

    for tgen in bld.task_gen_cache_names.values():
        if targets and tgen.get_name() not in targets:
            continue
        if getattr(tgen, 'cmake_skipme', False):
            continue
        if set(('c', 'cxx')) & set(getattr(tgen, 'features', [])):
            loc = tgen.path.relpath().replace('\\', '/')
            CMakeExporter(bld, loc).cleanup()


class CMakeExporter(object):
    def __init__(self, bld, location):
        self.bld = bld
        self.location = location # relative path from top level dir and 'ArduPlane' or 'Rover' etc
        self.cmakes = []
        self.tgens = []

        waf_bld_dir = bld.bldnode.abspath() # eg /home/buzz2/ardupilot/build/sitl/ from waf
        # breakup waf path into 3 rihgt-most parts and discard hte rest.
        self.bld_root_dir,part2,BOARDNAME = waf_bld_dir.rsplit('/',2)
        # bld_root_dir = '/home/buzz2/ardupilot',
        # part2 = 'build',
        # BOARDNAME = 'CubeOrange'  or 'sitl' etc
        self.BOARDNAME = BOARDNAME # not sure its needed.
        self.cmake_build_dir = 'build2/' + BOARDNAME # rel to bld_root_dir, eg build2/CubeOrange or build2/sitl
        self.cmake_full_build_dir = self.bld_root_dir + '/build2/' + BOARDNAME # eg /home/buzz2/ardupilot/build2/CubeOrange

        pwd = os.getcwd()
        if pwd != self.bld_root_dir:
            os.chdir(self.bld_root_dir)
            pwd = os.getcwd()
            print('Changed working dir to: %s ' % (pwd))

    def export(self):
        content = self.get_content()
        if not content:
            return ''

        node = self.make_node()
        if not node:
            return ''
        node.write(content)
        loc = self.location
        return content

    def cleanup(self):
        node = self.find_node()
        if node:
            node.delete()
            Logs.pprint('YELLOW', 'removed: %s' % node.abspath())

    def add_child(self, cmake):
        self.cmakes.append(cmake)

    def add_tgen(self, tgen):
        self.tgens.append(tgen)

    def find_node(self):
        name = '%s/CMakeLists.txt' % (self.location)
        if not name:
            return None    
        return self.bld.srcnode.find_node(name)

    def make_node(self):
        # only make one at the top.
        if self.location != ".":
            return None
        name = '%s/CMakeLists.txt' % (self.location)
        if not name:
            return None    
        return self.bld.srcnode.make_node(name)

    def get_content(self):
        is_top = (self.location == self.bld.path.relpath())

        content = ''
        if is_top and self.location == ".":

            #pre_run_2 = 'todo copy.. cp ../build/sitl/ap_config.h .'  to build2 dir
            if is_top == "." :
                # test if ap_config.h exists in the build dir and if not copy it there.
                ap_config_h_src = self.bld.bldnode.abspath() + '/ap_config.h'
                ap_config_h_dst = self.bld.srcnode.abspath() + '/ap_config.h'
                if not os.path.isfile(ap_config_h_dst):
                    import shutil
                    shutil.copyfile(ap_config_h_src, ap_config_h_dst)
                    print ('copied: %s to %s' % (ap_config_h_src, ap_config_h_dst))
            
            # 3.13 need to enable the NEW behavior of policy CMP0079 in your CMake project
            # 3.1 also introduced target_sources
            # 3.23 introduced File Sets, we dont use those.
            # 3.13.0 changed the default behavior to treating relative paths as being relative to the current source directory
            content += 'cmake_minimum_required (VERSION 3.13.0)\n'
            #content += 'project (%s)\n' % (getattr(Context.g_module, Context.APPNAME))
            content += 'project (ArduPilot)\n'
            content += '\n'

            # env is from ... and has 
            env = self.bld.env

            tmpblob='''
# Read the entire JSON file into a string variable
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/configure_env.json" CONFIG_JSON_STRING)
# Extract values using the GET subcommand and a path
string(JSON APJ_BOARD_ID GET "${CONFIG_JSON_STRING}" env APJ_BOARD_ID)
string(JSON AR GET "${CONFIG_JSON_STRING}" env AR)
string(JSON ARFLAGS GET "${CONFIG_JSON_STRING}" env ARFLAGS)
# things that end in '0' are a list that is just choosing the first element from, as there's only one.
string(JSON CMAKE_C_COMPILER GET "${CONFIG_JSON_STRING}" env CC 0)
string(JSON CFLAGS GET "${CONFIG_JSON_STRING}" env CFLAGS)
string(JSON CMAKE_CXX_COMPILER GET "${CONFIG_JSON_STRING}" env CXX 0)
string(JSON CXXFLAGS GET "${CONFIG_JSON_STRING}" env CXXFLAGS)
string(JSON CPU_FLAGS GET "${CONFIG_JSON_STRING}" env CPU_FLAGS)
string(JSON DEBUG GET "${CONFIG_JSON_STRING}" env DEBUG)
string(JSON DEFINES GET "${CONFIG_JSON_STRING}" env DEFINES)
string(JSON DOUBLE_PRECISION_LIBRARIES GET "${CONFIG_JSON_STRING}" env DOUBLE_PRECISION_LIBRARIES)
string(JSON DOUBLE_PRECISION_SOURCES GET "${CONFIG_JSON_STRING}" env DOUBLE_PRECISION_SOURCES)
string(JSON BOARD GET "${CONFIG_JSON_STRING}" env BOARD)
string(JSON BOARD_CLASS GET "${CONFIG_JSON_STRING}" env BOARD_CLASS)
string(JSON BUILDROOT GET "${CONFIG_JSON_STRING}" env BUILDROOT)
string(JSON HWDEF GET "${CONFIG_JSON_STRING}" env HWDEF)
string(JSON HWDEF_EXTRA GET "${CONFIG_JSON_STRING}" env HWDEF_EXTRA)
string(JSON INCLUDES GET "${CONFIG_JSON_STRING}" env INCLUDES)
string(JSON LINKFLAGS GET "${CONFIG_JSON_STRING}" env LINKFLAGS)
string(JSON LINK_CC GET "${CONFIG_JSON_STRING}" env LINK_CC)
string(JSON LINK_CXX GET "${CONFIG_JSON_STRING}" env LINK_CXX)
string(JSON NM GET "${CONFIG_JSON_STRING}" env NM)
string(JSON OBJCOPY GET "${CONFIG_JSON_STRING}" env OBJCOPY)
string(JSON TOOLCHAIN GET "${CONFIG_JSON_STRING}" env TOOLCHAIN)
string(JSON ENV_cfg_files GET "${CONFIG_JSON_STRING}" env cfg_files)
string(JSON ROMFS_FILES GET "${CONFIG_JSON_STRING}" env ROMFS_FILES)
            '''
            content += tmpblob
            content += '\n'

            AP_CONFIG_FLAGS = ' -D_AP_CONFIG_H_=1 -DWAF_BUILD=1 -D__STDC_FORMAT_MACROS=1 -DAP_SIM_ENABLED=1 -DHAL_WITH_SPI=1 -DHAL_WITH_RAMTRON=1 -DAP_OPENDRONEID_ENABLED=1 -DAP_SIGNED_FIRMWARE=0 -DAP_NOTIFY_LP5562_BUS=2 -DAP_NOTIFY_LP5562_ADDR=48 -DHAL_NUM_CAN_IFACES=2 -DHAL_CAN_WITH_SOCKETCAN=1 -DHAVE_FEENABLEEXCEPT=1 -DHAVE_CMATH_ISFINITE=1 -DHAVE_CMATH_ISINF=1 -DHAVE_CMATH_ISNAN=1 -DNEED_CMATH_ISFINITE_STD_NAMESPACE=1 -DNEED_CMATH_ISINF_STD_NAMESPACE=1 -DNEED_CMATH_ISNAN_STD_NAMESPACE=1 -D_GNU_SOURCE=1 '
            # we're fully excluding these three from chibios builds.
            SITL_ONLY_FLAGS = ' -DHAVE_ENDIAN_H=1 -DHAVE_BYTESWAP_H=1 -DHAVE_MEMRCHR=1 '

            # by doing this early, we can use BOARD variable in generated output later.
            BOARD = self.bld.env.BOARD
            content += '#set(BOARD %s)\n' % self.bld.env.BOARD

            if 'sitl' in BOARD.lower():
                AP_CONFIG_FLAGS += SITL_ONLY_FLAGS

            content += 'include_directories(.)\n'
            content += 'include_directories(..)\n'
            content += 'include_directories(../..)\n'
            content += 'include_directories(../../..)\n'

            # fatal error: AP_HAL/AP_HAL.h: No such file or directory
            content += 'include_directories(\n'
            content += '${CMAKE_CURRENT_SOURCE_DIR}\n'
            content += '${CMAKE_CURRENT_SOURCE_DIR}/AP_Common\n'
            content += '${CMAKE_CURRENT_SOURCE_DIR}/AP_HAL\n'
            content += '${CMAKE_CURRENT_SOURCE_DIR}/AP_HAL/board\n'
            content += '${CMAKE_CURRENT_SOURCE_DIR}/libraries\n'
            # generated mavlink needs these:
            #bld.bldnode.make_node('libraries').abspath(),
            #bld.bldnode.make_node('libraries/GCS_MAVLink').abspath(),
            content += '"${CMAKE_BINARY_DIR}/${BOARD}/libraries"\n'
            content += '"${CMAKE_BINARY_DIR}/${BOARD}/libraries/GCS_MAVLink"\n'
            # ap_version.h and ap_config.h
            content += '"${CMAKE_BINARY_DIR}/${BOARD}"\n'

            # dronecan needs canard/interface.h
            content += '${CMAKE_CURRENT_SOURCE_DIR}/modules/DroneCAN/libcanard\n'
            # droncan need canard_helpers_user.h
            content += '${CMAKE_CURRENT_SOURCE_DIR}/libraries/AP_DroneCAN/canard \n'
            # dronecan needs dronecan_msgs.h
            content += '"${CMAKE_BINARY_DIR}/${BOARD}/modules/DroneCAN/libcanard/dsdlc_generated/include" \n'

            # byteswap.h etc on non-sitl/chibios builds
            content += '"${CMAKE_CURRENT_SOURCE_DIR}/libraries/AP_Common/missing/"\n'
            content += ')\n\n'

            #set(ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
            content += 'set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")\n'
            content += '\n'

            env = self.bld.env
            env.DEFINES = [f for f in env.DEFINES if f != 'AP_SIM_ENABLED=0'] # remove this one, its set to 1 in ap_config.h
            defines = env.DEFINES 

            foo = [f for f in env.DEFINES if f.startswith('CONFIG_HAL_BOARD=')]
            #trim = 'CONFIG_HAL_BOARD=' off the front of foo
            if len(foo):
                foo = foo[0].split('=')[1]
                self.CONFIG_HAL_BOARD = foo # eg 'HAL_BOARD_SITL' or 'HAL_BOARD_CHIBIOS'
            
            if len(defines):
                content += 'add_definitions(-D%s) #1\n' % (' -D'.join(defines))
                content += '\n'

            #env has a lot of stuff in it.
            tmpE = env.table.keys()
            print('Debug: env keys: %s ' % (tmpE))
            apj_board_id = env['APJ_BOARD_ID']
            print('Debug: APJ_BOARD_ID: %s ' % (apj_board_id))
            apj_board_type = env['APJ_BOARD_TYPE']
            print('Debug: APJ_BOARD_TYPE: %s ' % (apj_board_type))
            BOARD = env['BOARD']
            print('Debug: BOARD: %s ' % (BOARD))
            board_class = env['BOARD_CLASS']
            print('Debug: BOARD_CLASS: %s ' % (board_class))
            waf_build_root = env['BUILDROOT']
            print('Debug: BUILDROOT: %s ' % (waf_build_root))
            CC = env['CC'] # full path to something ending in gcc
            print('Debug: CC: %s ' % (CC))
            CXX = env['CXX'] # full path to something ending in g++
            print('Debug: CXX: %s ' % (CXX))
            HWDEF = env['HWDEF']
            print('Debug: HWDEF: %s ' % (HWDEF)) # full path to a hwdef.dat file
            TOOLCHAIN = env['TOOLCHAIN']
            print('Debug: TOOLCHAIN: %s ' % (TOOLCHAIN)) # eg 'arm-none=eabi' prefix

            # drop '-Werror=shadow' from CFLAGS and CXXFLAGS
            env.CFLAGS = [f for f in env.CFLAGS if f != '-Werror=shadow']
            env.CXXFLAGS = [f for f in env.CXXFLAGS if f != '-Werror=shadow']
            env.CXXFLAGS = [f for f in env.CXXFLAGS if f != '-include ap_config.h']
            # convert 'HAL_GCS_ENABLED&&HAL_RALLY_ENABLED' to 'HAL_GCS_ENABLED' in CFLAGS and CXXFLAGS
            def clean_flags(flags):
                new_flags = []
                FRAME_CONFIG_seen = False
                for f in flags:
                    #print('Debug: original flag: %s' % f)
                    if '&&' in f:
                        parts = f.split('&&')
                        # keep only the first part
                        new_flags.append(parts[0])
                        print('Debug: fixed flag: %s' % parts[0])
                        continue
                    if 'FRAME_CONFIG=' in f:
                        # skip FRAME_CONFIG=... second and further occurrences
                        if not FRAME_CONFIG_seen:
                            new_flags.append(f)
                            FRAME_CONFIG_seen = True
                        continue
                    if 'ap_config.h' in f:
                        # skip this flag
                        continue
                    if '-include' in f:
                        # skip this flag
                        continue
                    #-DHAVE_ENDIAN_H=1 -DHAVE_BYTESWAP_H=1 -DHAVE_MEMRCHR=1
                    if f == '-DHAVE_ENDIAN_H=1':
                        continue
                    # or f.startswith('-DHAVE_BYTESWAP_H')
                    if f == '-DHAVE_BYTESWAP_H=1':
                        continue
                    # or f.startswith('-DHAVE_MEMRCHR'):
                    if f == '-DHAVE_MEMRCHR=1':
                        continue
                    new_flags.append(f)
                return new_flags
            
            content += '#set(CMAKE_C_COMPILER "%s")\n' % (' '.join(CC))
            content += '#set(CMAKE_CXX_COMPILER "%s")\n' % (' '.join(CXX))
            content += '#set($ENV{CC} "%s")\n' % (' '.join(CC))
            content += '#set($ENV{CXX} "%s")\n' % (' '.join(CXX))
            content += 'message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")\n'
            content += 'message(STATUS "C Compiler Version: ${CMAKE_C_COMPILER_VERSION}")\n'
            content += 'message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER}")\n'
            content += 'message(STATUS "CXX Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")\n'
            content += 'message(STATUS "BOARD: %s")\n' % BOARD

            flags1 = clean_flags(env.CFLAGS)
            if len(flags1):
                content += 'set(CMAKE_C_FLAGS "%s")\n\n' % (' '.join(flags1))
            flags2 = clean_flags(env.CXXFLAGS)
            flags2 += AP_CONFIG_FLAGS.split(' ')
            if len(flags2):
                content += 'set(CMAKE_CXX_FLAGS "%s")\n' % (' '.join(flags2))
                content += '\n# Build the gen-bindings tool\n'
                content += 'add_executable(gen-bindings libraries/AP_Scripting/generator/src/main.c)\n'
                content += 'target_compile_options(gen-bindings PRIVATE -std=c99 -Wno-error=missing-field-initializers -Wall -Wextra -Wno-error=maybe-uninitialized)\n'
                content += '# Generate lua_generated_bindings files\n'
                content += 'add_custom_command(\n'
                content += '    OUTPUT ${CMAKE_BINARY_DIR}/libraries/AP_Scripting/lua_generated_bindings.cpp ${CMAKE_BINARY_DIR}/libraries/AP_Scripting/lua_generated_bindings.h\n'
                content += '    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/libraries/AP_Scripting\n'
                content += '    COMMAND gen-bindings -o ${CMAKE_BINARY_DIR}/libraries/AP_Scripting/lua_generated_bindings -i ${CMAKE_SOURCE_DIR}/libraries/AP_Scripting/generator/description/bindings.desc\n'
                content += '    DEPENDS gen-bindings ${CMAKE_SOURCE_DIR}/libraries/AP_Scripting/generator/description/bindings.desc\n'
                content += '    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}\n'
                content += '    COMMENT "Generating Lua bindings"\n'
                content += ')\n\n'
        else:
            pass # not top level


        # before the tgens we need to add a couple of libraries that arent autogenerated.
        # 'mavlink', 'dronecan', 'AP_Navigation'
        mavlink_generated_sources = []
        mavlink_generated_includes = []
        # read all files in build2/sitl/libraries/GCS_MAVLink/include/mavlink/v2.0/
        #mavlink_gen_folder = self.bld.bldnode.make_node('../../build2/sitl/libraries/GCS_MAVLink/include/mavlink/v2.0/')
        mavlink_gen_folder = self.cmake_build_dir+'/libraries/GCS_MAVLink/include/mavlink/v2.0/'
        _mavlink_gen_folder = self.bld.bldnode.make_node('../../'+mavlink_gen_folder) # convert from waf to cmake
        print ('Debug: mavlink_gen_folder:', _mavlink_gen_folder)
        if _mavlink_gen_folder:
            for f in _mavlink_gen_folder.ant_glob('*.h'):
                relpath = f.path_from(self.bld.bldnode).replace('\\','/')
                mavlink_generated_sources.append(relpath)
            # also add the include folder itself
            mavlink_generated_includes.append(_mavlink_gen_folder.abspath())
        # now build the mavlink library content
        if len(mavlink_generated_sources):
            content += '\n#------------------------------------------------\n\n'
            content += 'set(mavlink_INCLUDES\n'
            # now add generated includes
            for inc in mavlink_generated_includes:
                # remove ../../ at front, if present
                if inc.startswith('../../'):
                    inc = inc.replace('../../', '')
                # dont harcode as build2/sitl or build2/CubeOrange, we use ${BOARD} instead.
                inc = inc.replace('build2/sitl', 'build2/${BOARD}')
                inc = inc.replace('build2/CubeOrange', 'build2/${BOARD}') 
                # to do more board types lke above. 
                content += '    %s\n' % (inc)
            content += ')\n\n'
            # INTERFACE is for header-only libraries
            content += 'add_library(mavlink INTERFACE ${mavlink_SOURCES}) #4\n'
            content += 'target_compile_definitions(mavlink INTERFACE -DLFS_NO_DEBUG -DLFS_NO_WARN -DLFS_NO_ERROR -DLFS_NO_ASSERT) #14\n'
            content += 'target_include_directories(mavlink INTERFACE ${mavlink_INCLUDES})\n'
            content += 'set_target_properties(mavlink PROPERTIES\n'
            content += '    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"\n'
            content += '    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"\n'
            # PROPERTIES LINKER_LANGUAGE CXX
            content += '    LINKER_LANGUAGE CXX\n'
            content += ')\n'

            register_lib('mavlink') # make sure its registered for linking later.


        #  dronecan libcanard.
        # - `canard.c` - the only translation unit; add it to your build or compile it into a separate static library;
        # - `canard.h` - the API header; include it in your application;
        # - `canard_internals.h` - internal definitions of the library;
        canard_sources = ['modules/DroneCAN/libcanard/canard.c',
                          'modules/DroneCAN/libcanard/canard.h',
                          'modules/DroneCAN/libcanard/canard_internals.h']
        canard_includes = ['modules/DroneCAN/libcanard']
        # we convert the above with self.bld.srcnode.make_node('xxx').abspath() below.
        print ('Debug: DroneCAN canard includes:', canard_includes)
        print ('Debug: DroneCAN canard sources:', canard_sources)
        if len(canard_sources):
            content += '\n#------------------------------------------------\n\n'
            content += 'set(canard_SOURCES #3\n'
            # add hardcoded sources first
            for src in canard_sources:
                # get full abspath
                src_abspath = self.bld.srcnode.make_node(src).abspath()
                content += '    %s\n' % (src_abspath)
            content += ')\n\n'
            content += 'set(canard_INCLUDES\n'
            # add hardcoded includes first
            for inc in canard_includes:
                # get full abspath
                inc_abspath = self.bld.srcnode.make_node(inc).abspath()
                # dont harcode as build2/sitl or build2/CubeOrange, we use ${BOARD} instead.
                inc = inc.replace('build2/sitl', 'build2/${BOARD}')
                inc = inc.replace('build2/CubeOrange', 'build2/${BOARD}') 
                content += '    %s\n' % (inc_abspath)
            content += ')\n\n'
            content += 'add_library(canard STATIC ${canard_SOURCES}) #5\n'
            content += 'target_include_directories(canard PUBLIC ${canard_INCLUDES})\n'
            content += 'set_target_properties(canard PROPERTIES\n'
            content += '    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"\n'
            content += '    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"\n'
            # PROPERTIES LINKER_LANGUAGE CXX
            content += '    LINKER_LANGUAGE CXX\n'
            content += ')\n'

            register_lib('canard') # make sure its registered for linking later.


        # emit dronecan_dsdlc_generated content here as per above comment, but generated like mavlink and canard.
        dsdlc_generated_folder = self.cmake_build_dir+'/modules/DroneCAN/libcanard/dsdlc_generated/src'
        dsdlc_generated_include_folder = self.cmake_build_dir+'/modules/DroneCAN/libcanard/dsdlc_generated/include'
        _dsdlc_generated_folder = self.bld.bldnode.make_node('../../'+dsdlc_generated_folder)
        _dsdlc_generated_include_folder = self.bld.bldnode.make_node('../../'+dsdlc_generated_include_folder)
        print ('Debug: dsdlc_generated_folder:', _dsdlc_generated_folder)
        print ('Debug: dsdlc_generated_include_folder:', _dsdlc_generated_include_folder)
        dsdlc_generated_sources = []
        for f in _dsdlc_generated_folder.ant_glob('*.c'):
            src_path = f.path_from(self.bld.bldnode).replace('\\','/')
            dsdlc_generated_sources.append(src_path)
        if len(dsdlc_generated_sources):
            content += '\n#------------------------------------------------\n\n'
            content += 'set(dronecan_dsdlc_generated_SOURCES #12\n'
            # add generated sources
            for src in dsdlc_generated_sources:
                # strip ../../ at front, if present
                if src.startswith('../../'):
                    src = src.replace('../../', '')
                # dont harcode as build2/sitl or build2/CubeOrange, we use ${BOARD} instead.
                src = src.replace('build2/sitl', 'build2/${BOARD}')
                src = src.replace('build2/CubeOrange', 'build2/${BOARD}') 
                content += '    %s\n' % (src)
            content += ')\n\n'
            content += 'set(dronecan_dsdlc_generated_INCLUDES\n'
            # add hardcoded includes first
            inc1 = _dsdlc_generated_include_folder.abspath()
            inc2 = self.bld.srcnode.make_node('modules/DroneCAN/libcanard').abspath()
            # strip /home/buzz2/ardupilot/ at front, if present
            if inc1.startswith(self.bld_root_dir+'/'):
                inc1 = inc1.replace(self.bld_root_dir+'/', '')
                # dont harcode as build2/sitl or build2/CubeOrange, we use ${BOARD} instead.
                inc1 = inc1.replace('build2/sitl', 'build2/${BOARD}')
                inc1 = inc1.replace('build2/CubeOrange', 'build2/${BOARD}') 
            if inc2.startswith(self.bld_root_dir+'/'):
                inc2 = inc2.replace(self.bld_root_dir+'/', '')
                # dont harcode as build2/sitl or build2/CubeOrange, we use ${BOARD} instead.
                inc2 = inc2.replace('build2/sitl', 'build2/${BOARD}')
                inc2 = inc2.replace('build2/CubeOrange', 'build2/${BOARD}') 
            
            content += '    %s\n' % (inc1)
            content += '    %s\n' % (inc2)
            content += ')\n\n'
            content += 'add_library(dronecan_dsdlc_generated STATIC ${dronecan_dsdlc_generated_SOURCES}) #6\n'
            content += 'target_include_directories(dronecan_dsdlc_generated PUBLIC ${dronecan_dsdlc_generated_INCLUDES})\n'
            content += 'set_target_properties(dronecan_dsdlc_generated PROPERTIES\n'
            content += '    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"\n'
            content += '    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"\n'
            # PROPERTIES LINKER_LANGUAGE CXX
            content += '    LINKER_LANGUAGE C\n'
            content += ')\n'

            #register_lib(cleanedname)
            register_lib('dronecan_dsdlc_generated') # make sure its registered for linking later.

        #-------------------
        # now process all task generators
        if len(self.tgens):
            content += '\n'
            for tgen in self.tgens:
                t = tgen.get_name()
                # list of libraries for ardupilot
                ap_libraries = tgen.ap_libraries if hasattr(tgen, 'ap_vehicle') else None
                # eg 'ArduCopter'
                ap_vehicle = tgen.ap_vehicle if hasattr(tgen, 'ap_vehicle') else None
                # list of source files, may include generated .c/cpp file from 'build' dir
                ap_sources = tgen.source if hasattr(tgen, 'source') else None
                # list of includes
                ap_includes = tgen.includes if hasattr(tgen, 'includes') else None
                if ap_vehicle is not None:
                    print('AP Vehicle: %s %s' % (ap_vehicle, t))

                #print('Debug: Generating cmake content for tgen: %s ' % t)
                content += self.get_tgen_content(tgen)
                

        if len(self.cmakes):
            content += '\n'
            for cmake in self.cmakes:
                t = cmake.export()
                content += t

        return content

    def get_tgen_content(self, tgen):
        content = ''
        name = tgen.get_name()

        # if name has two /'s in it, is a mid-level target, break it into pieces and remove the lastm which should be self.location
        # eg objs/AP_Scripting/ArduCopter  becomes objs/AP_Scripting
        strip_list= ['Rover', 'ArduPlane', 'ArduCopter', 'AntennaTracker', 'Blimp', 'ArduSub']
        parts = name.split('/')
        if len(parts) >= 3 and parts[-1]  in strip_list:
            parts = parts[:-1] # remove last part
            name = '/'.join(parts)

        cleanedname = name.replace('/', '_')
        # top level:
        # eg objs/AP_Scripting
        # next level:
        # eg objs/AP_Scripting/ArduCopter

        # does name start with bin/ ? if so print debug statement
        if name.startswith('bin'):  # name like 'bin/arducopter'
            print('Debug: Task generator name starts with bin: %s' % name)
        
        _dir = ''
        if cleanedname.startswith('bin_arduplane'):
            _dir = 'ArduPlane'
            cleanedname='plane'
        if cleanedname.startswith('bin_arducopter'):
            _dir = 'ArduCopter'
            # strip 'bin_ardu' from front on copters due to -heli
            _c = cleanedname[8:]
            cleanedname=_c
        if cleanedname.startswith('bin_ardusub'):
            _dir = 'ArduSub'
            cleanedname='sub'
        if cleanedname.startswith('bin_antennatracker'):
            _dir = 'AntennaTracker'
            cleanedname='tracker'
        if cleanedname.startswith('bin_blimp'):
            _dir = 'Blimp'
            cleanedname='blimp'
        if cleanedname.startswith('bin_ardurover'):
            _dir = 'Rover'
            cleanedname='rover'

        # libs also need to know the dir sometimes
        if cleanedname == 'ArduPlane_libs':
            _dir = 'ArduPlane'
        if cleanedname == 'ArduCopter_libs':
            _dir = 'ArduCopter'
        if cleanedname == 'ArduSub_libs':
            _dir = 'ArduSub'
        if cleanedname == 'AntennaTracker_libs':
            _dir = 'AntennaTracker'
        if cleanedname == 'Blimp_libs':
            _dir = 'Blimp'
        if cleanedname == 'Rover_libs':
            _dir = 'Rover'

        # check if 'name' starts with an *, as in '*.c' or similar.
        if '*' in name:
            print('Warning: Skipping task generator with wildcard name: %s' % name)
            return ''
        
        # obj/whatever is simplified to whatrver.
        if cleanedname.startswith('objs_'):
            cleanedname = cleanedname[5:] # remove 'objs_'
        
        reg_status = is_lib_registered(cleanedname)
        if reg_status:
            #print('Warning: Skipping already registered library: %s' % cleanedname)
            return ''
        #print('Registering Library: %s' % cleanedname)
        register_lib(cleanedname)

        #-------------------
        content += '#------------------------------------------------\n\n'
        uniq_src_list = []
        content += 'set(%s_SOURCES #13\n' % (cleanedname)
        for src in tgen.source:
            x = src.path_from(tgen.path).replace('\\','/') # a path to a source file
            xdir = os.path.dirname(x) # get folder that this src file is in
            # ls all the .h files in this dir and add them too, if not alreadty added.
            src_folder_node = tgen.path.find_node(xdir)
            if src_folder_node:
                for f in src_folder_node.ant_glob('*.h'):
                    hfile = f.path_from(tgen.path).replace('\\','/')
                    if hfile not in tgen.source: # can cause dupes
                        if _dir != '':
                            hfile = _dir + '/' + hfile
                        #content += '\n    %s' % (hfile)
                        if hfile not in uniq_src_list:
                            uniq_src_list.append(hfile)
                # repeat for .cpp
                for f in src_folder_node.ant_glob('*.cpp'):
                    cppfile = f.path_from(tgen.path).replace('\\','/')
                    if cppfile not in tgen.source: # can cause dupes
                        if _dir != '':
                            cppfile = _dir + '/' + cppfile
                        #content += '\n    %s' % (cppfile)
                        if cppfile not in uniq_src_list:
                            uniq_src_list.append(cppfile)
            
            if _dir != '':
                x = _dir + '/' + x
            if x not in uniq_src_list:
                uniq_src_list.append(x)
        exclude_these_files = [
            'libraries/GCS_MAVLink/GCS_Dummy.cpp',
            'libraries/AP_Scripting/lua/src/lua.c',
            'libraries/AP_Scripting/lua/src/luac.c',
        ]
        # key is a known file that exists, value is the file we include after it.
        include_these_files_by_other = {
            'libraries/AP_Scripting/lua_scripts.cpp': 'build2/${BOARD}/libraries/AP_Scripting/lua_generated_bindings.cpp',
        }
        for usrc in uniq_src_list: #dedupe
            if usrc in exclude_these_files:
                print('Debug: Excluding source file from cmake target %s: %s' % (cleanedname, usrc))
                continue
            usrc_str =str(usrc)
            # convert build/x waf paths to cmake style paths under build2/x
            usrc_str = usrc_str.replace('build/', 'build2/')
            # dont harcode as build2/sitl or build2/CubeOrange, we use ${BOARD} instead.
            usrc_str = usrc_str.replace('build2/sitl', 'build2/${BOARD}')
            usrc_str = usrc_str.replace('build2/CubeOrange', 'build2/${BOARD}') 
            content += '    %s\n' % (usrc_str)
            # check if usrc is in include_these_files_by_other keys
            for key, value in include_these_files_by_other.items():
                if usrc == key:
                    print('Debug: Including extra source file for cmake target %s: %s' % (cleanedname, value))
                    content += '    %s #extra\n' % (value)

        content += ')\n\n'

        #break here 'ArduCopter_libs'
        #if cleanedname == 'ArduCopter_libs':
        #    print('Debug: reached ArduCopter_libs source listing')

        includes = self.get_includes(tgen)
        includes.extend(tgen.env.INCLUDES)
        if len(includes):
            content += 'set(%s_INCLUDES' % (cleanedname)
            for include in includes:
                inc_as_str = str(include)
                # convert build/x waf paths to cmake style paths under build2/x
                inc_as_str = inc_as_str.replace('build/', 'build2/')
                # dont harcode as build2/sitl or build2/CubeOrange, we use ${BOARD} instead.
                inc_as_str = inc_as_str.replace('build2/sitl', 'build2/${BOARD}')
                inc_as_str = inc_as_str.replace('build2/CubeOrange', 'build2/${BOARD}') 
                content += '\n    %s' % inc_as_str
            content += ')\n\n'

        defines = self.get_genlist(tgen, 'defines')

        _interface = ''
        if set(('cprogram', 'cxxprogram')) & set(tgen.features):
            content += 'add_executable(%s ${%s_SOURCES}) #9\n' % (cleanedname, cleanedname)
            if len(defines):
                content += 'target_compile_definitions(%s PUBLIC -D%s) #10\n' % (cleanedname, ' -D'.join(defines))
            # link stuff
            #these will get /home/buzz2/ardupilot/ prefixed on them by cmake and become -Lxxx
            # because LIBRARY_OUTPUT_DIRECTORY=CMAKE_BINARY_DIR, ie build2, we need to add that to the link directories.
            content += 'target_link_directories(%s PUBLIC build2) #11\n' % (cleanedname)

            if len(includes):
                content += '#target_include_directories(%s PUBLIC ${%s_INCLUDES})\n' % (cleanedname, cleanedname)
            content += '\n'

        elif set(('cshlib', 'cxxshlib')) & set(tgen.features):
            content += 'add_library(%s SHARED ${%s_SOURCES}) #1\n' % (cleanedname, cleanedname)
            if len(defines):
                content += 'target_compile_definitions(%s PUBLIC -D%s) #3\n' % (cleanedname, ' -D'.join(defines))
            if len(includes):
                content += 'target_include_directories(%s PUBLIC ${%s_INCLUDES})\n' % (cleanedname, cleanedname)
            content += '\n'

            #set_target_properties(JE3D PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/out/library)
            content += 'set_target_properties(%s PROPERTIES\n' % (cleanedname)
            content += '    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/"\n'
            content += '    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"\n'
            content += ')\n\n'

        else: # cstlib, cxxstlib or objects
            if cleanedname.endswith('_libs'):
                _interface = 'INTERFACE '
            else :
                _interface = 'PUBLIC '
            prior_content = content #keep copy of content up to this point.
            # libs needing multi-target: ( eg needing -DAPM_BUILD_DIRECTORY=xx )
            # error: token "@" is not valid in preprocessor expressions | #define APM_BUILD_TYPE(type) @Invalid_use_of_APM_BUILD_TYPE
            needs_multi_target = [ 'SITL', 'AP_HAL_SITL', 'AC_AttitudeControl', 'AP_Vehicle', 'AP_Scripting', 
                         'AP_Avoidance','GCS_MAVLink','AP_InertialSensor','AP_NavEKF3','AC_Avoidance',
                         'AP_Filesystem' ,'AC_Fence','AC_WPNav','AP_IBus_Telem','AP_Logger','AP_AHRS','AP_Relay',
                         'AP_Frsky_Telem','AC_PrecLand','AP_ESC_Telem','AP_AIS','AP_TemperatureSensor','AP_BoardConfig',
                         'RC_Channel','AP_LandingGear','AP_DAL','AP_Baro','AP_Airspeed','AP_ExternalAHRS','AP_Math',
                         'AP_GyroFFT','AP_ADSB','AP_Follow','AP_Rally','AP_RCProtocol','AP_Landing','AP_LandingGear',
                         'AP_Motors','AP_OSD','AP_RCTelemetry','AP_Mission','AC_AutoTune','Filter','AC_AutoTune',
                         'AP_BattMonitor','AP_Terrain','AP_Proximity','AP_NavEKF2','AP_Scheduler','AP_HAL','AP_Arming',
                         'AP_Proximity','StorageManager','AP_HAL_ChibiOS', 'AP_BLHeli',
                         ]
            if True:
                content = '' # reset content to just the library part.
                content += 'add_library(%s %s${%s_SOURCES}) #2\n' % (cleanedname, _interface, cleanedname)
                _defines = [d for d in defines if not d.startswith('APM_BUILD_DIRECTORY')]
                _defines = [d for d in _defines if not d.startswith('AP_BUILD_TARGET_NAME')]
                _defines = [d for d in _defines if not d.startswith('CONFIG_HAL_BOARD')]
                if len(defines) != len(_defines):
                    #print(' APM_BUILD_DIRECTORY and/or AP_BUILD_TARGET_NAME ')
                    defines = _defines
                    if _dir != '':
                        defines.append('APM_BUILD_DIRECTORY=APM_BUILD_%s' % _dir) # buzz hack to define it.
                        #defines.append('AP_BUILD_TARGET_NAME=%s' % _dir)
                if cleanedname in needs_multi_target:
                    # assume copter and fix it later with regex
                    defines.append('AP_BUILD_TARGET_NAME="ArduCopter"') 
                    defines.append('APM_BUILD_DIRECTORY=APM_BUILD_ArduCopter')
                    defines.append('CONFIG_HAL_BOARD=%s' % str(self.CONFIG_HAL_BOARD)) # todo dont hardcode sitl
                    pass
                if len(defines):
                    content += 'target_compile_definitions(%s PUBLIC -D%s) #1\n' % (cleanedname, ' -D'.join(defines))
                if len(includes):
                    content += 'target_include_directories(%s %s${%s_INCLUDES})\n' % (cleanedname, _interface, cleanedname)
                content += '\n'
                content += 'set_target_properties(%s PROPERTIES\n' % (cleanedname)
                content += '    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"\n'
                content += '    CMAKE_LINK_LIBRARY_WHOLE_ARCHIVE_ATTRIBUTES "TRUE"\n'
                content += ')\n\n'
            # append the previous content
            new_content = content
            #if 'AC_AttitudeControl' in content:
            if cleanedname in needs_multi_target:
                #preg_replace from 'AC_AttitudeControl' to 'AC_AttitudeControl_Plane'
                new_content = new_content.replace(cleanedname+' ', cleanedname+'_Plane ') #space important
                new_content = new_content.replace('ArduCopter', 'ArduPlane')
                new_content = content + new_content
            content = prior_content + new_content
        #-------------------

        #libs = getattr(tgen, 'use', []) + getattr(tgen, 'lib', [])
        # TypeError: can only concatenate str (not "list") to str
        libs = []
        if hasattr(tgen, 'use'):
            foo = tgen.use # might be a list or a string.
            typeof_foo = type(foo)
            if typeof_foo == list:
                for u in getattr(tgen, 'use', []): #doing this on a string would get us letters.
                    libs.append(u)
            elif typeof_foo == str:
                libs.append(foo)
        for l in getattr(tgen, 'lib', []):
            libs.append(l)
        #for l in getattr(tgen, 'uselib', []): todo?
        #    libs.append(l)

        if len(libs):
            content += '\n'
            for lib in libs:
                # if lib is one or zero characters long, print warning.
                if len(lib) <= 1:
                    print('Warning: Skipping linking to library with too short name: "%s" in target: %s' % (lib, name))
                    continue
                if lib.startswith('objs/'):
                    strip_lib = lib[5:] # remove 'objs/' prefix
                    # does strip_lib end in /ArduCopter, ie has a / ?
                    if '/' in strip_lib:
                        continue
                    # exclude from this list:
                    if strip_lib in ['AP_Navigation', 'AP_Avoidance']:
                        continue
                    content += 'target_link_libraries(%s %s%s) #stripped\n' % (cleanedname, _interface, strip_lib)
                else:
                    # binary linking: for cmake, we're gonna rename 'dronecan' to 'canard' here , last minute.
                    _interface = ''
                    if lib == 'dronecan':
                        lib = 'canard'
                        _interface = ''
                    if lib.endswith('_libs'):
                        _interface = ''

                    content += 'target_link_libraries(%s %s%s) #clean\n' % (cleanedname,  _interface, lib)
                    if cleanedname  in ['copter','copter-heli', 'plane', 'rover', 'tracker', 'blimp', 'sub']:
                        global lib_register
                        for r in lib_register:
                            skiplist = [   'copter','copter-heli', 'plane', 'rover', 'tracker', 'blimp', 'sub', 
                                               'ArduCopter_libs',
                                               'ArduPlane_libs',
                                               'Rover_libs', 
                                               'AntennaTracker_libs', 
                                               'Blimp_libs', 
                                               'ArduSub_libs',
                                            ]
                            if r in skiplist:
                                continue # not-lib stuff.
                            per_target_skiplist = {
                                'copter': [],
                                'rover': ['AP_Avoidance'],
                                'sub': ['AP_AdvancedFailsafe','AP_LTM_Telem','AP_Avoidance'],
                                'plane': ['AP_Navigation'],
                                'tracker': ['AP_LTM_Telem','AP_AdvancedFailsafe'],
                            }
                            per_target_replace_list = { 
                                'copter': {
                                },
                                'plane': {
                                    'AC_AttitudeControl': 'AC_AttitudeControl_Plane',
                                    'SITL': 'SITL_Plane',
                                    'AP_HAL_SITL': 'AP_HAL_SITL_Plane',
                                    'AP_Vehicle': 'AP_Vehicle_Plane',
                                }
                            }
                            if cleanedname in per_target_skiplist:
                                if r in per_target_skiplist[cleanedname]:
                                    continue
                            if cleanedname in per_target_replace_list:
                                if r in per_target_replace_list[cleanedname]:
                                    r = per_target_replace_list[cleanedname][r]

                            content += 'target_link_libraries(%s %s-Wl,--whole-archive %s -Wl,--no-whole-archive) #clean2\n' % (cleanedname, _interface, r)
                        # Add system libraries and linker flags to match WAF build
                        content += 'target_link_libraries(%s %sm)  # Math library\n' % (cleanedname, _interface)
                        content += 'target_link_libraries(%s %spthread)  # POSIX threads\n' % (cleanedname, _interface)

                        # Add custom linker flags to match WAF build 
                        # set_target_properties(copter PROPERTIES 
                        #     LINK_FLAGS "-Wl,--gc-sections -Wl,--wrap,malloc" )
                        content += 'set_target_properties(%s PROPERTIES \n' % (cleanedname)
                        content += '    LINK_FLAGS "-Wl,--gc-sections -Wl,--wrap,malloc"\n'
                        content += ')\n'

            content += '\n'

        return content


    def get_includes(self, tgen):
        '''returns the include paths for the given task generator.
        '''
        includes = self.get_genlist(tgen, 'includes')
        for use in getattr(tgen, 'use', []):
            key = 'INCLUDES_%s' % use
            try:
                tg = self.bld.get_tgen_by_name(use)
                if 'fake_lib' in tg.features:
                    if key in tgen.env:
                        includes.extend(tgen.env[key])
            except Errors.WafError:
                if key in tgen.env:
                    includes.extend(tgen.env[key])
        return includes

    def get_genlist(self, tgen, name):
        lst = Utils.to_list(getattr(tgen, name, []))
        lst = [str(l.path_from(tgen.path)) if hasattr(l, 'path_from') else l for l in lst]
        return [l.replace('\\', '/') for l in lst]

