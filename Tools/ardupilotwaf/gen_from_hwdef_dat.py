import os
import subprocess
import sys
import fnmatch
import shutil
import argparse

# this writes a hwdef.h from hwdef.dat for a given chibios/esp32/linux etc board, AND 
#  and writes common.ld , hwdef.dat, ldscript.ld, etc to the same build dir.
# without usijng waf.  its a standalone script thats designed to be also useful outside fo waf.
# author: buzz. davidbuzz@gmail.com
# this is designed to be used without waf and perhaps useful for the ardupilot cmake build.

board_list = [
    'sitl' 
]
_board_classes = {
    'sitl': 'sitl'
}
_board_hwdef_all = {
    'sitl': 'Tools/ardupilotwaf/hwdef/sitl_hwdef.dat'
}

def add_dynamic_boards_chibios():
    '''add boards based on existence of hwdef.dat in subdirectories for ChibiOS'''
    add_dynamic_boards_from_hwdef_dir('ChibiOS', 'libraries/AP_HAL_ChibiOS/hwdef')

def add_dynamic_boards_linux():
    '''add boards based on existence of hwdef.dat in subdirectories for '''
    add_dynamic_boards_from_hwdef_dir('Linux', 'libraries/AP_HAL_Linux/hwdef')

def add_dynamic_boards_from_hwdef_dir(base_type, hwdef_dir):
    '''add boards based on existence of hwdef.dat in subdirectory'''
    dirname, dirlist, filenames = next(os.walk(hwdef_dir))
    for d in dirlist:
        if d in _board_classes.keys():
            continue
        hwdef = os.path.join(dirname, d, 'hwdef.dat')
        hwdef_bl = os.path.join(dirname, d, 'hwdef-bl.dat')
        if os.path.exists(hwdef):
            _board_classes[d] = base_type
            _board_hwdef_all[d] = hwdef
        elif os.path.exists(hwdef_bl):
            _board_classes[d] = base_type
            _board_hwdef_all[d] = hwdef_bl

def add_dynamic_boards_esp32():
    '''add boards based on existence of hwdef.dat in subdirectories for ESP32'''
    dirname, dirlist, filenames = next(os.walk('libraries/AP_HAL_ESP32/hwdef'))
    for d in dirlist:
        if d in _board_classes.keys():
            continue
        hwdef = os.path.join(dirname, d, 'hwdef.dat')
        if os.path.exists(hwdef):
            mcu_esp32s3 = True if (d[0:7] == "esp32s3") else False
            if mcu_esp32s3:
                #newclass = type(d, (esp32s3,), {'name': d})
                base_type = 'esp32s3'
                _board_classes[d] = base_type
                _board_hwdef_all[d] = hwdef
            else:
                #newclass = type(d, (esp32,), {'name': d})
                base_type = 'esp32'
                _board_classes[d] = base_type
                _board_hwdef_all[d] = hwdef
def get_boards_names():
    add_dynamic_boards_chibios()
    add_dynamic_boards_esp32()
    add_dynamic_boards_linux()

    return sorted(list(_board_classes.keys()), key=str.lower)

get_all_board_names = get_boards_names()
# for b in get_all_board_names:
#     print(b,' => ',_board_classes[b], ' => ',_board_hwdef_all[b])
    #   b is the boardname, 
    #   _board_classes tells us if its ChibiOS, ESP32, Linux, etc
    #   _board_hwdef_all[b] tells us the path to its hwdef.dat file
# exit(0)


# modify our search path:
sys.path.append(
    os.path.join(os.path.dirname(os.path.realpath(__file__)),
                 '../../libraries/AP_HAL_ChibiOS/hwdef/scripts'),
    )
sys.path.append(
    os.path.join(os.path.dirname(os.path.realpath(__file__)),
                 '../../libraries/AP_HAL_ESP32/hwdef/scripts'),
    )
sys.path.append(
    os.path.join(os.path.dirname(os.path.realpath(__file__)),
                 '../../libraries/AP_HAL_Linux/hwdef/scripts'),
    )

# uses:'libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py'
# uses:'libraries/AP_HAL_ESP32/hwdef/scripts/esp32_hwdef.py'
# uses:'libraries/AP_HAL_Linux/hwdef/scripts/linux_hwdef.py'
chibios_hwdef = __import__('chibios_hwdef')
esp32_hwdef = __import__('esp32_hwdef')
linux_hwdef = __import__('linux_hwdef')


def write_config_header(configfile='', guard='', env={}, top=False, defines=True, headers=False, remove=True, define_prefix=''):
    """
    Writes a configuration header containing defines and includes, inspired by a similar one in waf
    """
    guard = guard or '_%s_CONFIG_H_' % configfile.replace('.', '_').upper()
    lst = ['/* WARNING! BUZZ WAS HERE. All changes made to this file will be lost! */\n']
    if top:
        lst.append('/* %s */\n' % top)
    lst.append('#ifndef %s\n#define %s\n' % (guard, guard))
    # if headers:
    #     for x in env[INCKEYS]:
    #         lst.append('#include <%s>' % x)

    if defines:
        tbl = {}
        for k in env.DEFINES:
            a, _, b = k.partition('=')
            tbl[a] = b

        # for k in env[DEFKEYS]:
        #     caption = get_define_comment(k)
        #     if caption:
        #         caption = ' /* %s */' % caption
        #     try:
        txt = '#define %s%s %s' % (define_prefix, k, tbl[k])
        #     except KeyError:
        #         txt = '/* #undef %s%s */%s' % (define_prefix, k, caption)
        #     lst.append(txt)
    lst.append("\n".join(lst))
    lst.append('\n#endif /* %s */\n' % guard)

    with open(configfile, 'w') as f:
        f.writelines('\n'.join(lst))
    return True

if __name__ == '__main__':

    #-------------------------------------------------
    board_name_from_cmdline = None
    parser = argparse.ArgumentParser(description='Generate hwdef.h and related files from hwdef.dat')
    parser.add_argument('--board', default=None, help='board name to generate hwdef for a board in %s' % ','.join(get_all_board_names))
    parser.add_argument('--targetdir', default=None, help='output target directory (overrides default build2/BOARDNAME)')
    args = parser.parse_args()
    board_name_from_cmdline = args.board
    targetdir_from_cmdline = args.targetdir
    # ensure its valid and in our list
    if board_name_from_cmdline is None:
        print("Error: --board argument is required. Valid boards are: %s" % ','.join(get_all_board_names))
        exit(1)
    if board_name_from_cmdline not in get_all_board_names:
        print("Error: --board argument '%s' is not valid. Valid boards are: %s" % (board_name_from_cmdline, ','.join(get_all_board_names)))
        exit(1)
    if targetdir_from_cmdline is not None:
        # override buildroot
        _buildroot = targetdir_from_cmdline
    else:
        _buildroot = './build2/%s' % board_name_from_cmdline
    #-------------------------------------------------

    # env, as an object 
    env = type('env', (object,), {
        'SRCROOT': '.',
        'BOARD': board_name_from_cmdline,
        'BUILDROOT': _buildroot,
        #'BUILDROOT': './build2/%s' % board_name_from_cmdline, # where it will output generated files - cmake uses build2/ to be different from waf build that uses build/
        #'HWDEF_EXTRA': '',
        #'DEFAULT_PARAMETERS': './libraries/AP_HAL_ChibiOS/hwdef/CubeOrange/default-parameters.dat',
        'BOOTLOADER': False,
        'AP_SIGNED_FIRMWARE': False,# too dumb for this
    })
    board = env.BOARD 

    board_class_ = _board_classes[board] # eg ChibiOS, ESP32, Linux, etc

    #hwdef = os.path.join('libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat' % board)
    hwdef = _board_hwdef_all[board]
    env.HWDEF = hwdef 
    # ensure target directory exists first
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    if hasattr(env, 'HWDEF_EXTRA'):
        hwdef = [hwdef, env.HWDEF_EXTRA]
    if hasattr(env, 'DEFAULT_PARAMETERS'):
        default_params_filepath = os.path.abspath(env.DEFAULT_PARAMETERS)
    else:
        # use hwdef path, but change hwdef.dat to defaults.parm with regex
        tmp = str(hwdef)
        default_params_filepath = fnmatch.filter([tmp], '*hwdef.dat')[0].replace('hwdef.dat', 'defaults.parm')
    if hasattr(env, 'BOOTLOADER') and env.BOOTLOADER:
        bootloader_flag = True
    else:
        bootloader_flag = False
    if hasattr(env, 'AP_SIGNED_FIRMWARE') and env.AP_SIGNED_FIRMWARE:
        signed_fw_flag = True
    else:
        signed_fw_flag = False

    if board_class_ == 'ChibiOS':
        hwdef_obj = chibios_hwdef.ChibiOSHWDef(
            outdir=hwdef_out,
            bootloader=bootloader_flag,
            signed_fw=signed_fw_flag,
            hwdef=[hwdef],# list of hwdef paths, eg hwdef.inc,sdcard,inc, etc
            # stringify like old subprocess based invocation. note that no error is
            # generated if this path is missing!
            default_params_filepath=default_params_filepath,
            quiet=False,
        )
        hwdef_obj.run()
    elif board_class_ == 'esp32' or board_class_ == 'esp32s3':
        hwdef_obj = esp32_hwdef.ESP32HWDef(
            outdir=hwdef_out,
            hwdef=[hwdef],# list of hwdef paths, eg hwdef.inc,sdcard,inc, etc
            quiet=False,
        )
        hwdef_obj.run()
    elif board_class_ == 'Linux':
        hwdef_obj = linux_hwdef.LinuxHWDef(
            outdir=hwdef_out,
            hwdef=[hwdef],# list of hwdef paths, eg hwdef.inc,sdcard,inc, etc
            quiet=False,
        )
        hwdef_obj.run()
    elif board_class_ == 'sitl':
        # sitl does not need hwdef generation
        print("Info: board class 'sitl' does not require hwdef generation")
    else:
        print("Error: board class '%s' not supported" % board_class_)
        exit(1)


    # now ewe do ap_config.h generation too, since its related to hwdef.h

# not done here any more, see cmake_exporter.py
#write_dir = os.path.join(env.BUILDROOT)
#write_config_header(os.path.join(write_dir, 'ap_config.h'), guard='_AP_CONFIG_H_', env=env)
