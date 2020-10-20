// nearlly from https://github.com/chjchoi/kai/tree/master/project/linux-cortexm-1.9.0/A2F/gdb-2011.03/cs-cygwin-wrapper

// maybe a better one would be... https://github.com/ChristianVisintin/termiWin right now only supports one serial port at a time.
//
//
/* Replacement termios.h for MinGW32.

   Copyright (C) 2008 CodeSourcery, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#ifndef MINGW_TERMIOS_H
#define MINGW_TERMIOS_H

//termios2 and speed_t and a few extra bits added from https://elixir.bootlin.com/linux/v3.4/source/arch/arm/include/asm/termbits.h#L18
typedef unsigned char	cc_t;
typedef unsigned int	speed_t;
typedef unsigned int	tcflag_t;

#define NCCS 19
struct termios {
	tcflag_t c_iflag;		/* input mode flags */
	tcflag_t c_oflag;		/* output mode flags */
	tcflag_t c_cflag;		/* control mode flags */
	tcflag_t c_lflag;		/* local mode flags */
	cc_t c_line;			/* line discipline */
	cc_t c_cc[NCCS];		/* control characters */
};

struct termios2 {
	tcflag_t c_iflag;		/* input mode flags */
	tcflag_t c_oflag;		/* output mode flags */
	tcflag_t c_cflag;		/* control mode flags */
	tcflag_t c_lflag;		/* local mode flags */
	cc_t c_line;			/* line discipline */
	cc_t c_cc[NCCS];		/* control characters */
	speed_t c_ispeed;		/* input speed */
	speed_t c_ospeed;		/* output speed */
};


#define VEOF 0
#define VEOL 1
#define VERASE 2
#define VINTR 3
#define VKILL 4
#define VMIN 5
#define VQUIT 6
#define VSTART 7
#define VSTOP 8
#define VSUSP 9
#define VTIME 10
#define VEOL2 11
#define VWERASE 12
#define VREPRINT 13
#define VLNEXT 15
#define VDISCARD 16

#define _POSIX_VDISABLE '\0'

#define BRKINT 0x1
#define ICRNL 0x2
#define IGNBRK 0x4
#define IGNCR 0x8
#define IGNPAR 0x10
#define INLCR 0x20
#define INPCK 0x40
#define ISTRIP 0x80
#define IXANY 0x100
#define IXOFF 0x200
#define IXON 0x400
#define PARMRK 0x800

// extras
#define ECHOCTL	0x01000
#define ECHOKE	0x04000
#define IMAXBEL	0x20000
#define CRTSCTS	  020000000000		/* flow control */

#define OPOST 0x1
#define ONLCR 0x2
#define OCRNL 0x4
#define ONOCR 0x8
#define ONLRET 0x10
#define OFILL 0x20

#define CSIZE 0x3
#define CS5 0x0
#define CS6 0x1
#define CS7 0x2
#define CS8 0x3
#define CSTOPB 0x4
#define CREAD 0x8
#define PARENB 0x10
#define PARODD 0x20
#define HUPCL 0x40
#define CLOCAL 0x80

#define ECHO 0x1
#define ECHOE 0x2
#define ECHOK 0x4
#define ECHONL 0x8
#define ICANON 0x10
#define IEXTEN 0x20
#define ISIG 0x40
#define NOFLSH 0x80
#define TOSTOP 0x100
#define FLUSHO 0x200

#define TCSANOW 0
#define TCSADRAIN 1
#define TCSAFLUSH 2

#define TCIOFF 0
#define TCION 1
#define TCOOFF 2
#define TCOON 3

int tcgetattr (int fd, struct termios2 *buf);
int tcsetattr (int fd, int actions, const struct termios2 *buf);
int tcdrain (int fd);
int tcflow (int fd, int action);

/* We want to intercept TIOCGWINSZ, but not FIONREAD.  No need to forward
   TIOCSWINSZ; readline only uses it to suspend if in the background.
   Readline doesn't make any other ioctl calls on mingw.  */

#include <winsock.h>

struct winsize
{
  unsigned short ws_row;
  unsigned short ws_col;
};

int mingw_getwinsize (struct winsize *window_size);
#define TIOCGWINSZ 0x42424240
#define TIOCSWINSZ 0x42424241
#define ioctl(fd, op, arg)				\
  (((op) == TIOCGWINSZ) ? mingw_getwinsize (arg)	\
   : ((op) == TIOCSWINSZ) ? -1				\
   : ioctlsocket (fd, op, arg))

#endif /* MINGW_TERMIOS_H */
