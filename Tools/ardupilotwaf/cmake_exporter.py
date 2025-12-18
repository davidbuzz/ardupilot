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
When exporting *waf* project data, a single top level **CMakeLists.txt** file
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
        │       │   ├── CMakeLists.txt
        │       │   └── wscript
        │       ├── shared
        │       │   ├── CMakeLists.txt
        │       │   └── wscript
        │       └── static
        │           ├── CMakeLists.txt
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


from modules.waf.waflib import TaskGen
from waflib.Build import BuildContext
from waflib import Utils, Logs, Context, Errors
import waftools
from waftools import deps


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

		import os

		namelist = ['mavlink','dronecan','ap_version','ap_config']
		if name in namelist:
			print('Debug: CMakeExporterContext __call__ for name: %s ' % name)
			if name == 'mavlink':
				shell_cmd1 = 'gcc -std=c99 -Wno-error=missing-field-initializers -Wall -Werror -Wextra -o gen-bindings ../../libraries/AP_Scripting/generator/src/main.c'
				print('mavlink: Running shell command: %s ' % shell_cmd1)
				os.system(shell_cmd1)

			if name == 'dronecan':
				shell_cmd = '/usr/bin/python3 /home/buzz2/ardupilot/modules/DroneCAN/dronecan_dsdlc/dronecan_dsdlc.py -O/home/buzz2/ardupilot/build/sitl/modules/DroneCAN/libcanard/dsdlc_generated /home/buzz2/ardupilot/modules/DroneCAN/DSDL/ardupilot /home/buzz2/ardupilot/modules/DroneCAN/DSDL/com /home/buzz2/ardupilot/modules/DroneCAN/DSDL/cuav /home/buzz2/ardupilot/modules/DroneCAN/DSDL/dronecan /home/buzz2/ardupilot/modules/DroneCAN/DSDL/mppt /home/buzz2/ardupilot/modules/DroneCAN/DSDL/tests /home/buzz2/ardupilot/modules/DroneCAN/DSDL/uavcan >/dev/null 2>&1'
				print('dronecan: Running shell command: %s ' % shell_cmd)
				os.system(shell_cmd)

			if name == 'ap_version':
				builddir = self.bldnode.abspath() #eg build/sitl
				# shell. `mkdir -p build2/sitl`
				shell_cmd = 'mkdir -p build2/sitl'
				print('ap_version: Running shell command: %s ' % shell_cmd)
				os.system(shell_cmd)
				# write ap_version.h
				tgt = 'build2/sitl/ap_config.h'
				print('Debug: Writing ap_config.h to: %s ' % tgt)
				with open(tgt, 'w') as f:
        				print(
'''/* WARNING! All changes made to this file will be lost! */

#ifndef _AP_CONFIG_H_
#define _AP_CONFIG_H_

#define WAF_BUILD 1
#define PYTHONDIR "/usr/lib/python3/dist-packages"
#define PYTHONARCHDIR "/usr/lib/python3/dist-packages"
#define __STDC_FORMAT_MACROS 1
#define AP_SIM_ENABLED 1
#define HAL_WITH_SPI 1
#define HAL_WITH_RAMTRON 1
#define AP_OPENDRONEID_ENABLED 1
#define AP_SIGNED_FIRMWARE 0
#define AP_NOTIFY_LP5562_BUS 2
#define AP_NOTIFY_LP5562_ADDR 48
#define HAL_NUM_CAN_IFACES 2
#define HAL_CAN_WITH_SOCKETCAN 1
#define HAVE_FEENABLEEXCEPT 1
#define HAVE_CMATH_ISFINITE 1
#define HAVE_CMATH_ISINF 1
#define HAVE_CMATH_ISNAN 1
#define NEED_CMATH_ISFINITE_STD_NAMESPACE 1
#define NEED_CMATH_ISINF_STD_NAMESPACE 1
#define NEED_CMATH_ISNAN_STD_NAMESPACE 1
#define HAVE_ENDIAN_H 1
#define HAVE_BYTESWAP_H 1
#define HAVE_MEMRCHR 1
#define _GNU_SOURCE 1

#endif /* _AP_CONFIG_H_ */
''', file=f)

        # for k, v in ctx.env['AP_VERSION_ITEMS']:
        #     print('#define {} {}'.format(k, v), file=f)	

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
		self.timer = Utils.Timer()


def export(bld):
	'''Exports all C and C++ task generators to cmake.

	:param bld: a *waf* build instance from the top level *wscript*.
	:type bld: waflib.Build.BuildContext
	'''
	#if bld.options and not hasattr(bld.options, 'cmake_exporter'):
	#	print('cmake_exporter option not selected, skipping cmake export')
	#	return

	cmakes = {}
	loc = bld.path.relpath().replace('\\', '/')  # eg loc = '.'
	#print('ZZCreating new CMakeExporter for location: %s ' % (loc))
	top = CMakeExporter(bld, loc)
	cmakes[loc] = top
	targets = waftools.deps.get_targets(bld)
	#unique_targets = set()
	#for t in targets:
	#	unique_targets.add(t)
	#print('unique_targets:', unique_targets)
	
	for tgen in bld.task_gen_cache_names.values():
		if targets and tgen.get_name() not in targets:
			#print('skipping tgen:', tgen.get_name())
			# examples, tests, etc
			continue
		if getattr(tgen, 'cmake_skipme', False):
			continue
		if set(('c', 'cxx')) & set(getattr(tgen, 'features', [])):
			loc = tgen.path.relpath().replace('\\', '/') # eg loc = 'Rover' or 'ArduPlane' etc
			if loc not in cmakes:
				#print('Creating new CMakeExporter for location: %s ' % (loc))
				cmake = CMakeExporter(bld, loc)
				cmakes[loc] = cmake
				top.add_child(cmake)
			cmakes[loc].add_tgen(tgen)

	for cmake in cmakes.values():
		cmake.export()


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
		print('CMakeExporter initialized for location: %s' % (self.location))

	def export(self):
		content = self.get_content()
		if not content:
			return

		node = self.make_node()
		if not node:
			return
		node.write(content)
		Logs.pprint('YELLOW', 'exported: %s' % node.abspath())

	def cleanup(self):
		node = self.find_node()
		if node:
			node.delete()
			Logs.pprint('YELLOW', 'removed: %s' % node.abspath())

	def add_child(self, cmake):
		self.cmakes.append(cmake)

	def add_tgen(self, tgen):
		self.tgens.append(tgen)

	def get_location(self):
		return self.location

	def get_fname(self):
		name = '%s/CMakeLists.txt' % (self.location)
		return name

	def find_node(self):
		name = self.get_fname()
		if not name:
			return None    
		return self.bld.srcnode.find_node(name)

	def make_node(self):
		name = self.get_fname()
		if not name:
			return None    
		return self.bld.srcnode.make_node(name)

	def get_content(self):
		is_top = (self.location == self.bld.path.relpath())

		content = ''
		if is_top:
			pre_run_ = './gen-bindings -o libraries/AP_Scripting/lua_generated_bindings -i ../../libraries/AP_Scripting/generator/description/bindings.desc'
			pre_run_dir = self.bld.srcnode.abspath() + '/build/sitl'
			# set cwd to ./build/sitl first.
			pre_run_1 = 'cd %s && %s' % (pre_run_dir, pre_run_)
			# run the above now with ``os.system`` so that the generated bindings are present for cmake build
			import os
			x = os.system(pre_run_1)
			print ('ran: %s , returned: %s dir: %s' % (pre_run_1, x, pre_run_dir))

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
			content += 'cmake_minimum_required (VERSION 3.13)\n'
			#content += 'project (%s)\n' % (getattr(Context.g_module, Context.APPNAME))
			content += 'project (ArduPilot)\n'
			content += '\n'

			# add_definitions(-I.)
			# add_definitions(-I..)
			# add_definitions(-I../..)
			# add_definitions(-I../../..)
			content += 'include_directories(.)\n'
			content += 'include_directories(..)\n'
			content += 'include_directories(../..)\n'
			content += 'include_directories(../../..)\n'

			#set(ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
			content += 'set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")\n'
			content += '\n'

			env = self.bld.env
			defines = env.DEFINES
			if len(defines):
				content += 'add_definitions(-D%s) #1\n' % (' -D'.join(defines))
				content += '\n'

			# drop '-Werror=shadow' from CFLAGS and CXXFLAGS
			env.CFLAGS = [f for f in env.CFLAGS if f != '-Werror=shadow']
			env.CXXFLAGS = [f for f in env.CXXFLAGS if f != '-Werror=shadow']
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
					new_flags.append(f)
				return new_flags

			flags1 = clean_flags(env.CFLAGS)
			if len(flags1):
				content += 'set(CMAKE_C_FLAGS "%s")\n' % (' '.join(flags1))

			flags2 = clean_flags(env.CXXFLAGS)
			if len(flags2):
				content += 'set(CMAKE_CXX_FLAGS "%s")\n' % (' '.join(flags2))
		else:
			pass # not top level

		if len(self.tgens):
			content += '\n'
			for tgen in self.tgens:
				t = tgen.get_name()
				#print('Debug: Generating cmake content for tgen: %s ' % t)
				content += self.get_tgen_content(tgen)

		if len(self.cmakes):
			content += '\n'
			for cmake in self.cmakes:
				content += 'add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/%s)\n' % (cmake.get_location())

		return content

	def get_tgen_content(self, tgen):
		content = ''
		name = tgen.get_name()

		# if name has two /'s in it, is a mid-level target, break it into pieces and remove the lastm which should be self.location
		# eg objs/AP_Scripting/ArduCopter  becomes objs/AP_Scripting
		parts = name.split('/')
		if len(parts) == 3:
			parts = parts[:-1] # remove last part
			name = '/'.join(parts)

		#CMP0037_name = name.replace('-', '_')
		cleanedname = name.replace('/', '_')

		# top level:
		# eg objs/AP_Scripting
		# next level:
		# eg objs/AP_Scripting/ArduCopter

		# does name start with bin/ ? if so print debug statement
		if name.startswith('bin'):
			print('Debug: Task generator name starts with bin: %s' % name)

		# check if 'name' starts with an *, as in '*.c' or similar.
		if '*' in name:
			print('Warning: Skipping task generator with wildcard name: %s' % name)
			return ''
		
		reg_status = is_lib_registered(cleanedname)
		if reg_status:
			print('Warning: Skipping already registered library: %s' % cleanedname)
			return ''
		print('Registering Library: %s' % cleanedname)
		register_lib(cleanedname)

		content += 'set(%s_SOURCES' % (cleanedname)
		for src in tgen.source:
			content += '\n    %s' % (src.path_from(tgen.path).replace('\\','/'))
		content += ')\n\n'

		includes = self.get_includes(tgen)
		includes.extend(tgen.env.INCLUDES)
		if len(includes):
			content += 'set(%s_INCLUDES' % (cleanedname)
			for include in includes:
				content += '\n    %s' % include
			content += ')\n\n'
			content += 'include_directories(${%s_INCLUDES})\n' % (cleanedname)
			content += '\n'

		defines = self.get_genlist(tgen, 'defines')
		#if len(defines):
		#	content += 'add_definitions(-D%s) #2\n' % (' -D'.join(defines))
		#	content += '\n'


		if set(('cprogram', 'cxxprogram')) & set(tgen.features):
			content += 'add_executable(%s ${%s_SOURCES})\n' % (cleanedname, cleanedname)
			if len(defines):
				content += 'target_compile_definitions(%s PRIVATE -D%s) #2\n' % (cleanedname, ' -D'.join(defines))
			content += '\n'

		elif set(('cshlib', 'cxxshlib')) & set(tgen.features):
			content += 'add_library(%s SHARED ${%s_SOURCES}) #1\n' % (cleanedname, cleanedname)
			if len(defines):
				content += 'target_compile_definitions(%s PRIVATE -D%s) #2\n' % (cleanedname, ' -D'.join(defines))
			content += '\n'

			#set_target_properties(JE3D PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/out/library)
			content += 'set_target_properties(%s PROPERTIES\n' % (cleanedname)
			content += '    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/"\n'
			content += ')\n\n'

		else: # cstlib, cxxstlib or objects
			content += 'add_library(%s ${%s_SOURCES}) #2\n' % (cleanedname, cleanedname)
			if len(defines):
				content += 'target_compile_definitions(%s PRIVATE -D%s) #2\n' % (cleanedname, ' -D'.join(defines))
			content += '\n'

			#set_target_properties(JE3D PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/out/library)
			content += 'set_target_properties(%s PROPERTIES\n' % (cleanedname)
			content += '    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"\n'
			content += ')\n\n'

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
		#	libs.append(l)

		if len(libs):
			content += '\n'
			for lib in libs:
				# if lib is one or zero characters long, print warning.
				if len(lib) <= 1:
					print('Warning: Skipping linking to library with too short name: "%s" in target: %s' % (lib, name))
					continue
				if lib.startswith('objs/'):
					#print('Debug: Skipping linking to library for objs/ target: %s lib: %s' % (name, lib))
					#continue
					strip_lib = lib[5:] # remove 'objs/' prefix
					# does strip_lib end in /ArduCopter, ie has a / ?
					if '/' in strip_lib:
						continue
					cleaned_name_bits = cleanedname.split('_')
					is_lib = cleaned_name_bits[1]
					is_loc = cleaned_name_bits[0]
					if is_lib == 'libs'  and self.location == is_loc:
						#content += 'target_link_libraries(%s ../libobjs_%s_%s.a) #stripped2\n' % (cleanedname, strip_lib, is_loc)
						content += 'target_link_libraries(%s ../libobjs_%s.a) #stripped2\n' % (cleanedname, strip_lib)
					else:
						content += 'target_link_libraries(%s objs_%s_%s) #stripped\n' % (cleanedname, strip_lib,)
				else:
					content += 'target_link_libraries(%s %s) #clean\n' % (cleanedname, lib)
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

