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
    cfg.find_program('make', var='MAKE')
    env = cfg.env
    env.AP_HAL_PLANE = srcpath('libraries/AP_HAL_ESP32/targets/plane')
    env.AP_HAL_COPTER = srcpath('libraries/AP_HAL_ESP32/targets/copter')
    env.AP_HAL_ROVER = srcpath('libraries/AP_HAL_ESP32/targets/rover')
    env.AP_HAL_SUB = srcpath('libraries/AP_HAL_ESP32/targets/sub')
    env.AP_PROGRAM_FEATURES += ['esp32_ap_program']

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
        return "apj_tool"
    def run(self):
        rel_default_parameters = self.env.get_flat('DEFAULT_PARAMETERS').replace("'", "")
        abs_default_parameters = os.path.join(self.env.SRCROOT, rel_default_parameters)
        apj_tool = self.env.APJ_TOOL
        sys.path.append(os.path.dirname(apj_tool))
        from apj_tool import embedded_defaults
        defaults = embedded_defaults(self.inputs[0].abspath())
        print("QQQQQQQQQ"+defaults)
        if defaults.find():
            defaults.set_file(abs_default_parameters)
            defaults.save()

class build_esp32_image_plane(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="export IDF_PATH=\"${IDF}\"; cd ${AP_HAL_PLANE}&&'${MAKE}' BATCH_BUILD=1 2>/dev/null >/dev/null"
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_esp32_image_copter(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="export IDF_PATH=\"${IDF}\"; cd ${AP_HAL_COPTER}&&'${MAKE}' BATCH_BUILD=1"
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_esp32_image_rover(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="export IDF_PATH=\"${IDF}\"; cd ${AP_HAL_ROVER}&&'${MAKE}' V=1"
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_esp32_image_sub(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="export IDF_PATH=\"${IDF}\"; cd ${AP_HAL_SUB}&&'${MAKE}' V=1"
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class generate_apj(Task.Task):
    '''generate an apj firmware file'''
    color='CYAN'
    always_run = True
    def keyword(self):
        return "apj_gen"
    def run(self):
        import json, time, base64, zlib
        img = open(self.inputs[0].abspath(),'rb').read()
        d = {
            "board_id": int(self.env.APJ_BOARD_ID),
            "magic": "APJFWv1",
            "description": "Firmware for a %s board" % self.env.APJ_BOARD_TYPE,
            "image": base64.b64encode(zlib.compress(img,9)).decode('utf-8'),
            "summary": self.env.BOARD,
            "version": "0.1",
            "image_size": len(img),
            "flash_total": int(self.env.FLASH_TOTAL),
            "flash_free": int(self.env.FLASH_TOTAL) - len(img),
            "git_identity": self.generator.bld.git_head_hash(short=True),
            "board_revision": 0,
            "USBID": self.env.USBID
        }
        if self.env.build_dates:
            # we omit build_time when we don't have build_dates so that apj
            # file is idential for same git hash and compiler
            d["build_time"] = int(time.time())
        apj_file = self.outputs[0].abspath()
        print("apj..."+str(apj_file))
        #f = open(apj_file, "w")
        #f.write(json.dumps(d, indent=4))
        #f.close()

#python ./modules/esp_idf/components/esptool_py/esptool/esptool.py 
#--chip esp32 --port /dev/ttyUSB0 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio 
#--flash_freq 80m --flash_size detect 
#0xf000 ./build/esp32buzz/idf-plane/ota_data_initial.bin 
#0x1000 ./build/esp32buzz/idf-plane/bootloader/bootloader.bin 
#0x20000 ./build/esp32buzz/idf-plane/arduplane.bin 
#0x8000 ./build/esp32buzz/idf-plane/partitions.bin
class upload_fw(Task.Task):
    color='BLUE'
    always_run = True
    def run(self):
        upload_tools = self.env.get_flat('UPLOAD_TOOLS')
        upload_port = self.generator.bld.options.upload_port
        src = self.inputs[0]
  
        _et = "../../modules/esp_idf/components/esptool_py/esptool/esptool.py"
        _otai = "../../build/esp32buzz/idf-plane/ota_data_initial.bin"
        _bl = "../../build/esp32buzz/idf-plane/bootloader/bootloader.bin"
        #_bin = "../../build/esp32buzz/idf-plane/arduplane.bin"
        _prt = "../../build/esp32buzz/idf-plane/partitions.bin"

        idf_type = str(src).split('/')[-2]; # the /idf-plane/ part of src .. or copter, or sub etc

        # we only get passed one path+file so this adjusts the above defines to match what is passed.
        _bl = _bl.split('/'); _bl[4] = idf_type; _bl = '/'.join(_bl);
        _otai = _otai.split('/'); _otai[4] = idf_type; _otai = '/'.join(_otai);
        _prt = _prt.split('/'); _prt[4] = idf_type; _prt = '/'.join(_prt);
        
        # taken from esp_idf's 'make flash' output and lowered the baud rate a bit for reliability.
        # we left out --port /dev/ttyUSB0  because it can autodetect
        cmd = "{} {} --chip esp32 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0xf000 {} 0x1000 {} 0x20000 {} 0x8000 {}".format(self.env.get_flat('PYTHON'),_et,  _otai, _bl, src, _prt)
        if upload_port is not None:
            cmd = cmd.replace("esp32 ","esp32 --port %s " % upload_port,1)
        print(cmd)
        return self.exec_command(cmd)

    def exec_command(self, cmd, **kw):
        kw['stdout'] = sys.stdout
        return super(upload_fw, self).exec_command(cmd, **kw)

    def keyword(self):
        return "Uploading"



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


    # not quite working yet.
    #if self.env.DEFAULT_PARAMETERS:
    #    default_params_task = self.create_task('set_default_parameters',
    #                                           src=img_out2)
    #    default_params_task.set_run_after(self.generate_bin_task)


    if self.bld.options.upload:
        _upload_task = self.create_task('upload_fw', src=img_out2 )
        _upload_task.set_run_after(self.generate_bin_task)


