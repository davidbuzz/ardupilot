#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for ESP32 build
"""

from waflib import Errors, Logs, Task, Utils
from waflib.TaskGen import after_method, before_method, feature

import os
import shutil
import sys
import re
import pickle
import subprocess

def parse_inc_dir(lines):
    for line in lines.splitlines():
        if line.startswith('INCLUDES: '):
            return line.replace('INCLUDES: ', '').split()



def configure(cfg):

    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()
    def bldpath(path):
        return srcpath('build')

    cfg.find_program('make', var='MAKE')
    env = cfg.env
    env.AP_HAL_PLANE = srcpath('libraries/AP_HAL_ESP32/targets/plane')
    env.AP_HAL_COPTER = srcpath('libraries/AP_HAL_ESP32/targets/copter')
    env.AP_HAL_ROVER = srcpath('libraries/AP_HAL_ESP32/targets/rover')
    env.AP_HAL_SUB = srcpath('libraries/AP_HAL_ESP32/targets/sub')
    env.AP_PROGRAM_FEATURES += ['esp32_ap_program']

    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')
    env.APJ_TOOL = srcpath('Tools/scripts/apj_tool.py')

    try:
        env.IDF = os.environ['IDF_PATH']
    except:
        env.IDF = cfg.srcnode.abspath()+"/modules/esp_idf"
    print("USING EXPRESSIF IDF:"+str(env.IDF))

    cmd = "export IDF_PATH=\"{3}\"; cd {0}&&echo '{2}' > ../board.txt&&{1} defconfig BATCH_BUILD=1&&{1} showinc BATCH_BUILD=1".format(env.AP_HAL_PLANE, env.MAKE[0], env.BOARD, env.IDF)
    print(cmd)
    result = subprocess.check_output(cmd, shell=True)
    if not isinstance(result, str):
        result = result.decode()
    #env.INCLUDES = parse_inc_dir(result) + env.INCLUDES
    cfg.env.prepend_value('INCLUDES', parse_inc_dir(result))

    try:
        env.DEFAULT_PARAMETERS = os.environ['DEFAULT_PARAMETERS']
    except:
        env.DEFAULT_PARAMETERS = cfg.srcnode.abspath()+"/libraries/AP_HAL_ESP32/boards/defaults.parm"
    print("USING DEFAULT_PARAMETERS:"+str(env.DEFAULT_PARAMETERS))


class set_default_parameters(Task.Task):
    color='CYAN'
    always_run = True
    def keyword(self):
        return "setting default params"
    def run(self):

        # TODO: disabled this task outright as apjtool appears to destroy checksums and/or the esp32 partition table
        # TIP:  if u do try this, afterwards, be sure to 'rm -rf build/esp32buzz/idf-plane/*.bin' and re-run waf
        return

        # (752) esp_image: Checksum failed. Calculated 0xd3 read 0xa3
        # (752) boot: OTA app partition slot 0 is not bootable
        # (753) esp_image: image at 0x200000 has invalid magic byte
        # (759) boot_comm: mismatch chip ID, expected 0, found 65535
        # (766) boot_comm: can't run on lower chip revision, expected 1, found 255
        # (773) esp_image: image at 0x200000 has invalid SPI mode 255
        # (779) esp_image: image at 0x200000 has invalid SPI size 15
        # (786) boot: OTA app partition slot 1 is not bootable
        # (792) boot: No bootable app partitions in the partition table


        # skip task if nothing to do.
        if not self.env.DEFAULT_PARAMETERS:
            return

        default_parameters = self.env.get_flat('DEFAULT_PARAMETERS').replace("'", "")
        #print("apj defaults file:"+str(default_parameters))

        _bin = str(self.inputs[0])

        # paranoia check  before and after apj_tool to see if file hash has changed... 
        cmd = "shasum -b {0}".format( _bin )
        result = subprocess.check_output(cmd, shell=True)
        prehash = str(result).split(' ')[0][2:]
        
        cmd = "{1} {2} --set-file {3}".format(self.env.SRCROOT, self.env.APJ_TOOL, _bin, default_parameters )
        print(cmd)
        result = subprocess.check_output(cmd, shell=True)
        if not isinstance(result, str):
            result = result.decode()
        for i in str(result).split('\n'):
            print("\t"+i)
        
        # paranoia check  before and after apj_tool to see if file hash has changed... 
        cmd = "shasum -b {0}".format( _bin )
        result = subprocess.check_output(cmd, shell=True)
        posthash = str(result).split(' ')[0][2:]

        # display --show output, helpful.
        cmd = "{1} {2} --show ".format(self.env.SRCROOT, self.env.APJ_TOOL, _bin )
        print(cmd)
        result = subprocess.check_output(cmd, shell=True)
        if not isinstance(result, str):
            result = result.decode()
        for i in str(result).split('\n'):
            print("\t"+i)

        # were embedded params updated in .bin?
        if prehash == posthash:
            print("Embedded params in .bin unchanged (probably already up-to-date)")
        else:
            print("Embedded params in .bin UPDATED")


class build_esp32_image_plane(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="export IDF_PATH=\"${IDF}\"; cd ${AP_HAL_PLANE}&&'${MAKE}' BATCH_BUILD=1 2>/dev/null >/dev/null"
    always_run = True
    def keyword(self):
        return "Generating (and building IDF)"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_esp32_image_copter(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="export IDF_PATH=\"${IDF}\"; cd ${AP_HAL_COPTER}&&'${MAKE}' BATCH_BUILD=1"
    always_run = True
    def keyword(self):
        return "Generating (and building IDF)"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_esp32_image_rover(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="export IDF_PATH=\"${IDF}\"; cd ${AP_HAL_ROVER}&&'${MAKE}' V=1"
    always_run = True
    def keyword(self):
        return "Generating (and building IDF)"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_esp32_image_sub(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="export IDF_PATH=\"${IDF}\"; cd ${AP_HAL_SUB}&&'${MAKE}' V=1"
    always_run = True
    def keyword(self):
        return "Generating (and building IDF)"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class upload_fw(Task.Task):
    color='BLUE'
    always_run = True
    def keyword(self):
        return "Uploading"
    def run(self):
        upload_tools = self.env.get_flat('UPLOAD_TOOLS')
        upload_port = self.generator.bld.options.upload_port
        src = self.inputs[0]

        idf_type = str(src).split('/')[-2]; # the /idf-plane/ part of src .. or copter, or sub etc

        _et = str(self.get_cwd()) + "/../../modules/esp_idf/components/esptool_py/esptool/esptool.py"
        _otai = str(self.get_cwd()) + "/" + idf_type + "/ota_data_initial.bin"
        _bl = str(self.get_cwd()) + "/" + idf_type + "/bootloader/bootloader.bin"
        #_bin = "../../build/esp32buzz/idf-plane/arduplane.bin"
        _prt = str(self.get_cwd()) + "/" + idf_type + "/partitions.bin"

        # taken from esp_idf's 'make flash' output and lowered the baud rate a bit for reliability.
        # we left out --port /dev/ttyUSB0  because it can autodetect
        cmd = "{} {} --chip esp32 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0xf000 {} 0x1000 {} 0x20000 {} 0x8000 {}".format(self.env.get_flat('PYTHON'),_et,  _otai, _bl, src, _prt)
        if upload_port is not None:
            cmd = cmd.replace("esp32 ","esp32 --port %s " % upload_port,1)
        print("\n"+cmd)
        return self.exec_command(cmd)

    def exec_command(self, cmd, **kw):
        kw['stdout'] = sys.stdout
        return super(upload_fw, self).exec_command(cmd, **kw)




@feature('esp32_ap_program')
@after_method('process_source')
def esp32_firmware(self):

    link_output = self.link_task.outputs[0]
    bin_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.bin').name)
    apj_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.apj').name)


    if str(self.link_task.outputs[0]).endswith('libarduplane.a'):
        #build final image
        src_in = [self.bld.bldnode.find_or_declare('lib/libArduPlane_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libarduplane.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-plane/arduplane.elf')
        img_out2 = self.bld.bldnode.find_or_declare('idf-plane/arduplane.bin')
        self.generate_bin_task = self.create_task('build_esp32_image_plane', src=src_in, tgt=img_out)
        self.generate_bin_task.set_run_after(self.link_task)


    if str(self.link_task.outputs[0]).endswith('libarducopter.a'):
        #build final image
        src_in = [self.bld.bldnode.find_or_declare('lib/libArduCopter_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libarducopter.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-copter/arducopter.elf')
        img_out2 = self.bld.bldnode.find_or_declare('idf-copter/arducopter.bin')
        self.generate_bin_task = self.create_task('build_esp32_image_copter', src=src_in, tgt=img_out)
        self.generate_bin_task.set_run_after(self.link_task)

    if str(self.link_task.outputs[0]).endswith('libardurover.a'):
        #build final image
        src_in = [self.bld.bldnode.find_or_declare('lib/libRover_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libardurover.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-rover/ardurover.elf')
        img_out2 = self.bld.bldnode.find_or_declare('idf-rover/ardurover.bin')
        self.generate_bin_task = self.create_task('build_esp32_image_rover', src=src_in, tgt=img_out)
        self.generate_bin_task.set_run_after(self.link_task)

    if str(self.link_task.outputs[0]).endswith('libardusub.a'):
        #build final image
        src_in = [self.bld.bldnode.find_or_declare('lib/libArduSub_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libardusub.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-sub/ardusub.elf')
        img_out2 = self.bld.bldnode.find_or_declare('idf-sub/ardusub.bin')
        self.generate_bin_task = self.create_task('build_esp32_image_sub', src=src_in, tgt=img_out)
        self.generate_bin_task.set_run_after(self.link_task)


    # tool that can update the default params in a .bin or .apj
    self.default_params_task = self.create_task('set_default_parameters',
                                               src=img_out2)
    self.default_params_task.set_run_after(self.generate_bin_task)

    # optional upload is last
    if self.bld.options.upload:
        _upload_task = self.create_task('upload_fw', src=img_out2 )
        _upload_task.set_run_after(self.default_params_task)


