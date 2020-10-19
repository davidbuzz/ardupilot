# Makefile.in generated by automake 1.16.1 from Makefile.am.
# Makefile.  Generated from Makefile.in by configure.

# Copyright (C) 1994-2018 Free Software Foundation, Inc.

# This Makefile.in is free software; the Free Software Foundation
# gives unlimited permission to copy and/or distribute it,
# with or without modifications, as long as this notice is preserved.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY, to the extent permitted by law; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE.



am__is_gnu_make = { \
  if test -z '$(MAKELEVEL)'; then \
    false; \
  elif test -n '$(MAKE_HOST)'; then \
    true; \
  elif test -n '$(MAKE_VERSION)' && test -n '$(CURDIR)'; then \
    true; \
  else \
    false; \
  fi; \
}
am__make_running_with_option = \
  case $${target_option-} in \
      ?) ;; \
      *) echo "am__make_running_with_option: internal error: invalid" \
              "target option '$${target_option-}' specified" >&2; \
         exit 1;; \
  esac; \
  has_opt=no; \
  sane_makeflags=$$MAKEFLAGS; \
  if $(am__is_gnu_make); then \
    sane_makeflags=$$MFLAGS; \
  else \
    case $$MAKEFLAGS in \
      *\\[\ \	]*) \
        bs=\\; \
        sane_makeflags=`printf '%s\n' "$$MAKEFLAGS" \
          | sed "s/$$bs$$bs[$$bs $$bs	]*//g"`;; \
    esac; \
  fi; \
  skip_next=no; \
  strip_trailopt () \
  { \
    flg=`printf '%s\n' "$$flg" | sed "s/$$1.*$$//"`; \
  }; \
  for flg in $$sane_makeflags; do \
    test $$skip_next = yes && { skip_next=no; continue; }; \
    case $$flg in \
      *=*|--*) continue;; \
        -*I) strip_trailopt 'I'; skip_next=yes;; \
      -*I?*) strip_trailopt 'I';; \
        -*O) strip_trailopt 'O'; skip_next=yes;; \
      -*O?*) strip_trailopt 'O';; \
        -*l) strip_trailopt 'l'; skip_next=yes;; \
      -*l?*) strip_trailopt 'l';; \
      -[dEDm]) skip_next=yes;; \
      -[JT]) skip_next=yes;; \
    esac; \
    case $$flg in \
      *$$target_option*) has_opt=yes; break;; \
    esac; \
  done; \
  test $$has_opt = yes
am__make_dryrun = (target_option=n; $(am__make_running_with_option))
am__make_keepgoing = (target_option=k; $(am__make_running_with_option))
pkgdatadir = $(datadir)/ardumingw
pkgincludedir = $(includedir)/ardumingw
pkglibdir = $(libdir)/ardumingw
pkglibexecdir = $(libexecdir)/ardumingw
am__cd = CDPATH="$${ZSH_VERSION+.}$(PATH_SEPARATOR)" && cd
install_sh_DATA = $(install_sh) -c -m 644
install_sh_PROGRAM = $(install_sh) -c
install_sh_SCRIPT = $(install_sh) -c
INSTALL_HEADER = $(INSTALL_DATA)
transform = $(program_transform_name)
NORMAL_INSTALL = :
PRE_INSTALL = :
POST_INSTALL = :
NORMAL_UNINSTALL = :
PRE_UNINSTALL = :
POST_UNINSTALL = :
build_triplet = x86_64-pc-linux-gnu
host_triplet = x86_64-w64-mingw32
subdir = .
ACLOCAL_M4 = $(top_srcdir)/aclocal.m4
am__aclocal_m4_deps = $(top_srcdir)/m4/00gnulib.m4 \
	$(top_srcdir)/m4/absolute-header.m4 $(top_srcdir)/m4/alloca.m4 \
	$(top_srcdir)/m4/dup2.m4 $(top_srcdir)/m4/errno_h.m4 \
	$(top_srcdir)/m4/extensions.m4 \
	$(top_srcdir)/m4/extern-inline.m4 \
	$(top_srcdir)/m4/gnulib-common.m4 \
	$(top_srcdir)/m4/gnulib-comp.m4 \
	$(top_srcdir)/m4/include_next.m4 \
	$(top_srcdir)/m4/msvc-inval.m4 \
	$(top_srcdir)/m4/msvc-nothrow.m4 $(top_srcdir)/m4/off_t.m4 \
	$(top_srcdir)/m4/pid_t.m4 $(top_srcdir)/m4/select.m4 \
	$(top_srcdir)/m4/signal_h.m4 $(top_srcdir)/m4/socketlib.m4 \
	$(top_srcdir)/m4/sockets.m4 $(top_srcdir)/m4/socklen.m4 \
	$(top_srcdir)/m4/ssize_t.m4 $(top_srcdir)/m4/stdalign.m4 \
	$(top_srcdir)/m4/stddef_h.m4 $(top_srcdir)/m4/sys_select_h.m4 \
	$(top_srcdir)/m4/sys_socket_h.m4 \
	$(top_srcdir)/m4/sys_time_h.m4 $(top_srcdir)/m4/sys_types_h.m4 \
	$(top_srcdir)/m4/sys_uio_h.m4 $(top_srcdir)/m4/unistd_h.m4 \
	$(top_srcdir)/m4/warn-on-use.m4 $(top_srcdir)/m4/wchar_t.m4 \
	$(top_srcdir)/m4/zzgnulib.m4 $(top_srcdir)/configure.ac
am__configure_deps = $(am__aclocal_m4_deps) $(CONFIGURE_DEPENDENCIES) \
	$(ACLOCAL_M4)
DIST_COMMON = $(srcdir)/Makefile.am $(top_srcdir)/configure \
	$(am__configure_deps) $(am__DIST_COMMON)
am__CONFIG_DISTCLEAN_FILES = config.status config.cache config.log \
 configure.lineno config.status.lineno
mkinstalldirs = $(install_sh) -d
CONFIG_HEADER = $(top_builddir)/lib/config.h
CONFIG_CLEAN_FILES =
CONFIG_CLEAN_VPATH_FILES =
AM_V_P = $(am__v_P_$(V))
am__v_P_ = $(am__v_P_$(AM_DEFAULT_VERBOSITY))
am__v_P_0 = false
am__v_P_1 = :
AM_V_GEN = $(am__v_GEN_$(V))
am__v_GEN_ = $(am__v_GEN_$(AM_DEFAULT_VERBOSITY))
am__v_GEN_0 = @echo "  GEN     " $@;
am__v_GEN_1 = 
AM_V_at = $(am__v_at_$(V))
am__v_at_ = $(am__v_at_$(AM_DEFAULT_VERBOSITY))
am__v_at_0 = @
am__v_at_1 = 
SOURCES =
DIST_SOURCES =
RECURSIVE_TARGETS = all-recursive check-recursive cscopelist-recursive \
	ctags-recursive dvi-recursive html-recursive info-recursive \
	install-data-recursive install-dvi-recursive \
	install-exec-recursive install-html-recursive \
	install-info-recursive install-pdf-recursive \
	install-ps-recursive install-recursive installcheck-recursive \
	installdirs-recursive pdf-recursive ps-recursive \
	tags-recursive uninstall-recursive
am__can_run_installinfo = \
  case $$AM_UPDATE_INFO_DIR in \
    n|no|NO) false;; \
    *) (install-info --version) >/dev/null 2>&1;; \
  esac
RECURSIVE_CLEAN_TARGETS = mostlyclean-recursive clean-recursive	\
  distclean-recursive maintainer-clean-recursive
am__recursive_targets = \
  $(RECURSIVE_TARGETS) \
  $(RECURSIVE_CLEAN_TARGETS) \
  $(am__extra_recursive_targets)
AM_RECURSIVE_TARGETS = $(am__recursive_targets:-recursive=) TAGS CTAGS \
	cscope distdir distdir-am dist dist-all distcheck
am__tagged_files = $(HEADERS) $(SOURCES) $(TAGS_FILES) $(LISP)
# Read a list of newline-separated strings from the standard input,
# and print each of them once, without duplicates.  Input order is
# *not* preserved.
am__uniquify_input = $(AWK) '\
  BEGIN { nonempty = 0; } \
  { items[$$0] = 1; nonempty = 1; } \
  END { if (nonempty) { for (i in items) print i; }; } \
'
# Make sure the list of sources is unique.  This is necessary because,
# e.g., the same source file might be shared among _SOURCES variables
# for different programs/libraries.
am__define_uniq_tagged_files = \
  list='$(am__tagged_files)'; \
  unique=`for i in $$list; do \
    if test -f "$$i"; then echo $$i; else echo $(srcdir)/$$i; fi; \
  done | $(am__uniquify_input)`
ETAGS = etags
CTAGS = ctags
CSCOPE = cscope
DIST_SUBDIRS = $(SUBDIRS)
am__DIST_COMMON = $(srcdir)/Makefile.in AUTHORS COPYING ChangeLog \
	INSTALL NEWS README compile config.guess config.sub depcomp \
	install-sh missing
DISTFILES = $(DIST_COMMON) $(DIST_SOURCES) $(TEXINFOS) $(EXTRA_DIST)
distdir = $(PACKAGE)-$(VERSION)
top_distdir = $(distdir)
am__remove_distdir = \
  if test -d "$(distdir)"; then \
    find "$(distdir)" -type d ! -perm -200 -exec chmod u+w {} ';' \
      && rm -rf "$(distdir)" \
      || { sleep 5 && rm -rf "$(distdir)"; }; \
  else :; fi
am__post_remove_distdir = $(am__remove_distdir)
am__relativize = \
  dir0=`pwd`; \
  sed_first='s,^\([^/]*\)/.*$$,\1,'; \
  sed_rest='s,^[^/]*/*,,'; \
  sed_last='s,^.*/\([^/]*\)$$,\1,'; \
  sed_butlast='s,/*[^/]*$$,,'; \
  while test -n "$$dir1"; do \
    first=`echo "$$dir1" | sed -e "$$sed_first"`; \
    if test "$$first" != "."; then \
      if test "$$first" = ".."; then \
        dir2=`echo "$$dir0" | sed -e "$$sed_last"`/"$$dir2"; \
        dir0=`echo "$$dir0" | sed -e "$$sed_butlast"`; \
      else \
        first2=`echo "$$dir2" | sed -e "$$sed_first"`; \
        if test "$$first2" = "$$first"; then \
          dir2=`echo "$$dir2" | sed -e "$$sed_rest"`; \
        else \
          dir2="../$$dir2"; \
        fi; \
        dir0="$$dir0"/"$$first"; \
      fi; \
    fi; \
    dir1=`echo "$$dir1" | sed -e "$$sed_rest"`; \
  done; \
  reldir="$$dir2"
DIST_ARCHIVES = $(distdir).tar.gz
GZIP_ENV = --best
DIST_TARGETS = dist-gzip
distuninstallcheck_listfiles = find . -type f -print
am__distuninstallcheck_listfiles = $(distuninstallcheck_listfiles) \
  | sed 's|^\./|$(prefix)/|' | grep -v '$(infodir)/dir$$'
distcleancheck_listfiles = find . -type f -print
ACLOCAL = ${SHELL} /home/buzz/ardupilot/missing aclocal-1.16
ALLOCA = 
ALLOCA_H = alloca.h
AMTAR = $${TAR-tar}
AM_DEFAULT_VERBOSITY = 1
AR = x86_64-w64-mingw32-ar
ARFLAGS = cr
AUTOCONF = ${SHELL} /home/buzz/ardupilot/missing autoconf
AUTOHEADER = ${SHELL} /home/buzz/ardupilot/missing autoheader
AUTOMAKE = ${SHELL} /home/buzz/ardupilot/missing automake-1.16
AWK = gawk
CC = x86_64-w64-mingw32-gcc
CCDEPMODE = depmode=gcc3
CFLAGS = -g -O2
CPP = x86_64-w64-mingw32-gcc -E
CPPFLAGS = 
CYGPATH_W = echo
DEFS = -DHAVE_CONFIG_H
DEPDIR = .deps
ECHO_C = 
ECHO_N = -n
ECHO_T = 
EGREP = /bin/grep -E
EMULTIHOP_HIDDEN = 0
EMULTIHOP_VALUE = 
ENOLINK_HIDDEN = 0
ENOLINK_VALUE = 
EOVERFLOW_HIDDEN = 0
EOVERFLOW_VALUE = 
ERRNO_H = errno.h
EXEEXT = .exe
GNULIB_ACCEPT = 1
GNULIB_ACCEPT4 = 0
GNULIB_ACCESS = 0
GNULIB_BIND = 1
GNULIB_CHDIR = 0
GNULIB_CHOWN = 0
GNULIB_CLOSE = 0
GNULIB_CONNECT = 1
GNULIB_COPY_FILE_RANGE = 0
GNULIB_DUP = 0
GNULIB_DUP2 = 1
GNULIB_DUP3 = 0
GNULIB_ENVIRON = 0
GNULIB_EUIDACCESS = 0
GNULIB_FACCESSAT = 0
GNULIB_FCHDIR = 0
GNULIB_FCHOWNAT = 0
GNULIB_FDATASYNC = 0
GNULIB_FSYNC = 0
GNULIB_FTRUNCATE = 0
GNULIB_GETCWD = 0
GNULIB_GETDOMAINNAME = 0
GNULIB_GETDTABLESIZE = 0
GNULIB_GETENTROPY = 0
GNULIB_GETGROUPS = 0
GNULIB_GETHOSTNAME = 0
GNULIB_GETLOGIN = 0
GNULIB_GETLOGIN_R = 0
GNULIB_GETOPT_POSIX = 0
GNULIB_GETPAGESIZE = 0
GNULIB_GETPASS = 0
GNULIB_GETPEERNAME = 1
GNULIB_GETSOCKNAME = 0
GNULIB_GETSOCKOPT = 0
GNULIB_GETTIMEOFDAY = 0
GNULIB_GETUSERSHELL = 0
GNULIB_GROUP_MEMBER = 0
GNULIB_ISATTY = 0
GNULIB_LCHOWN = 0
GNULIB_LINK = 0
GNULIB_LINKAT = 0
GNULIB_LISTEN = 1
GNULIB_LSEEK = 0
GNULIB_PIPE = 0
GNULIB_PIPE2 = 0
GNULIB_PREAD = 0
GNULIB_PSELECT = 0
GNULIB_PTHREAD_SIGMASK = 0
GNULIB_PWRITE = 0
GNULIB_RAISE = 0
GNULIB_READ = 0
GNULIB_READLINK = 0
GNULIB_READLINKAT = 0
GNULIB_RECV = 1
GNULIB_RECVFROM = 1
GNULIB_RMDIR = 0
GNULIB_SELECT = 1
GNULIB_SEND = 1
GNULIB_SENDTO = 1
GNULIB_SETHOSTNAME = 0
GNULIB_SETSOCKOPT = 1
GNULIB_SHUTDOWN = 0
GNULIB_SIGACTION = 0
GNULIB_SIGNAL_H_SIGPIPE = 0
GNULIB_SIGPROCMASK = 0
GNULIB_SLEEP = 0
GNULIB_SOCKET = 1
GNULIB_SYMLINK = 0
GNULIB_SYMLINKAT = 0
GNULIB_TRUNCATE = 0
GNULIB_TTYNAME_R = 0
GNULIB_UNISTD_H_NONBLOCKING = 0
GNULIB_UNISTD_H_SIGPIPE = 0
GNULIB_UNLINK = 0
GNULIB_UNLINKAT = 0
GNULIB_USLEEP = 0
GNULIB_WRITE = 0
GREP = /bin/grep
HAVE_ACCEPT4 = 1
HAVE_ALLOCA_H = 0
HAVE_CHOWN = 1
HAVE_COPY_FILE_RANGE = 1
HAVE_DECL_ENVIRON = 1
HAVE_DECL_FCHDIR = 1
HAVE_DECL_FDATASYNC = 1
HAVE_DECL_GETDOMAINNAME = 1
HAVE_DECL_GETLOGIN = 1
HAVE_DECL_GETLOGIN_R = 1
HAVE_DECL_GETPAGESIZE = 1
HAVE_DECL_GETUSERSHELL = 1
HAVE_DECL_SETHOSTNAME = 1
HAVE_DECL_TRUNCATE = 1
HAVE_DECL_TTYNAME_R = 1
HAVE_DUP3 = 1
HAVE_EUIDACCESS = 1
HAVE_FACCESSAT = 1
HAVE_FCHDIR = 1
HAVE_FCHOWNAT = 1
HAVE_FDATASYNC = 1
HAVE_FSYNC = 1
HAVE_FTRUNCATE = 1
HAVE_GETDTABLESIZE = 1
HAVE_GETENTROPY = 1
HAVE_GETGROUPS = 1
HAVE_GETHOSTNAME = 1
HAVE_GETLOGIN = 1
HAVE_GETPAGESIZE = 1
HAVE_GETPASS = 1
HAVE_GETTIMEOFDAY = 1
HAVE_GROUP_MEMBER = 1
HAVE_LCHOWN = 1
HAVE_LINK = 1
HAVE_LINKAT = 1
HAVE_MAX_ALIGN_T = 1
HAVE_MSVC_INVALID_PARAMETER_HANDLER = 1
HAVE_OS_H = 0
HAVE_PIPE = 1
HAVE_PIPE2 = 1
HAVE_POSIX_SIGNALBLOCKING = 1
HAVE_PREAD = 1
HAVE_PSELECT = 1
HAVE_PTHREAD_SIGMASK = 1
HAVE_PWRITE = 1
HAVE_RAISE = 1
HAVE_READLINK = 1
HAVE_READLINKAT = 1
HAVE_SA_FAMILY_T = 0
HAVE_SETHOSTNAME = 1
HAVE_SIGACTION = 1
HAVE_SIGHANDLER_T = 0
HAVE_SIGINFO_T = 1
HAVE_SIGSET_T = 0
HAVE_SLEEP = 1
HAVE_STRUCT_SIGACTION_SA_SIGACTION = 1
HAVE_STRUCT_SOCKADDR_STORAGE = 1
HAVE_STRUCT_SOCKADDR_STORAGE_SS_FAMILY = 1
HAVE_STRUCT_TIMEVAL = 1
HAVE_SYMLINK = 1
HAVE_SYMLINKAT = 1
HAVE_SYS_PARAM_H = 0
HAVE_SYS_SELECT_H = 0
HAVE_SYS_SOCKET_H = 0
HAVE_SYS_TIME_H = 1
HAVE_SYS_UIO_H = 0
HAVE_TYPE_VOLATILE_SIG_ATOMIC_T = 1
HAVE_UNISTD_H = 1
HAVE_UNLINKAT = 1
HAVE_USLEEP = 1
HAVE_WCHAR_T = 1
HAVE_WINSOCK2_H = 1
HAVE_WS2TCPIP_H = 1
INCLUDE_NEXT = include_next
INCLUDE_NEXT_AS_FIRST_DIRECTIVE = include_next
INSTALL = /usr/bin/install -c
INSTALL_DATA = ${INSTALL} -m 644
INSTALL_PROGRAM = ${INSTALL}
INSTALL_SCRIPT = ${INSTALL}
INSTALL_STRIP_PROGRAM = $(install_sh) -c -s
LDFLAGS = 
LIBGNU_LIBDEPS = 
LIBGNU_LTLIBDEPS = 
LIBOBJS = 
LIBS = 
LIBSOCKET = -lws2_32
LIB_SELECT = -lws2_32
LTLIBOBJS = 
MAKEINFO = ${SHELL} /home/buzz/ardupilot/missing makeinfo
MKDIR_P = /bin/mkdir -p
NEXT_AS_FIRST_DIRECTIVE_ERRNO_H = <errno.h>
NEXT_AS_FIRST_DIRECTIVE_SIGNAL_H = <signal.h>
NEXT_AS_FIRST_DIRECTIVE_STDDEF_H = 
NEXT_AS_FIRST_DIRECTIVE_SYS_SELECT_H = <sys/select.h>
NEXT_AS_FIRST_DIRECTIVE_SYS_SOCKET_H = <sys/socket.h>
NEXT_AS_FIRST_DIRECTIVE_SYS_TIME_H = <sys/time.h>
NEXT_AS_FIRST_DIRECTIVE_SYS_TYPES_H = <sys/types.h>
NEXT_AS_FIRST_DIRECTIVE_SYS_UIO_H = <sys/uio.h>
NEXT_AS_FIRST_DIRECTIVE_UNISTD_H = <unistd.h>
NEXT_ERRNO_H = <errno.h>
NEXT_SIGNAL_H = <signal.h>
NEXT_STDDEF_H = 
NEXT_SYS_SELECT_H = <sys/select.h>
NEXT_SYS_SOCKET_H = <sys/socket.h>
NEXT_SYS_TIME_H = <sys/time.h>
NEXT_SYS_TYPES_H = <sys/types.h>
NEXT_SYS_UIO_H = <sys/uio.h>
NEXT_UNISTD_H = <unistd.h>
OBJEXT = o
PACKAGE = ardumingw
PACKAGE_BUGREPORT = 
PACKAGE_NAME = ardumingw
PACKAGE_STRING = ardumingw 0.1
PACKAGE_TARNAME = ardumingw
PACKAGE_URL = 
PACKAGE_VERSION = 0.1
PATH_SEPARATOR = :
PRAGMA_COLUMNS = 
PRAGMA_SYSTEM_HEADER = #pragma GCC system_header
RANLIB = x86_64-w64-mingw32-ranlib
REPLACE_ACCESS = 0
REPLACE_CHOWN = 0
REPLACE_CLOSE = 0
REPLACE_DUP = 0
REPLACE_DUP2 = 1
REPLACE_FACCESSAT = 0
REPLACE_FCHOWNAT = 0
REPLACE_FTRUNCATE = 0
REPLACE_GETCWD = 0
REPLACE_GETDOMAINNAME = 0
REPLACE_GETDTABLESIZE = 0
REPLACE_GETGROUPS = 0
REPLACE_GETLOGIN_R = 0
REPLACE_GETPAGESIZE = 0
REPLACE_GETPASS = 0
REPLACE_GETTIMEOFDAY = 0
REPLACE_ISATTY = 0
REPLACE_LCHOWN = 0
REPLACE_LINK = 0
REPLACE_LINKAT = 0
REPLACE_LSEEK = 0
REPLACE_NULL = 0
REPLACE_PREAD = 0
REPLACE_PSELECT = 0
REPLACE_PTHREAD_SIGMASK = 0
REPLACE_PWRITE = 0
REPLACE_RAISE = 0
REPLACE_READ = 0
REPLACE_READLINK = 0
REPLACE_READLINKAT = 0
REPLACE_RMDIR = 0
REPLACE_SELECT = 1
REPLACE_SLEEP = 0
REPLACE_STRUCT_TIMEVAL = 1
REPLACE_SYMLINK = 0
REPLACE_SYMLINKAT = 0
REPLACE_TRUNCATE = 0
REPLACE_TTYNAME_R = 0
REPLACE_UNLINK = 0
REPLACE_UNLINKAT = 0
REPLACE_USLEEP = 0
REPLACE_WRITE = 0
SET_MAKE = 
SHELL = /bin/bash
STDALIGN_H = 
STDDEF_H = 
STRIP = x86_64-w64-mingw32-strip
UNISTD_H_HAVE_SYS_RANDOM_H = 0
UNISTD_H_HAVE_WINSOCK2_H = 1
UNISTD_H_HAVE_WINSOCK2_H_AND_USE_SOCKETS = 1
VERSION = 0.1
WINDOWS_64_BIT_OFF_T = 0
WINDOWS_STAT_INODES = 0
abs_builddir = /home/buzz/ardupilot
abs_srcdir = /home/buzz/ardupilot
abs_top_builddir = /home/buzz/ardupilot
abs_top_srcdir = /home/buzz/ardupilot
ac_ct_CC = 
am__include = include
am__leading_dot = .
am__quote = 
am__tar = $${TAR-tar} chof - "$$tardir"
am__untar = $${TAR-tar} xf -
bindir = ${exec_prefix}/bin
build = x86_64-pc-linux-gnu
build_alias = 
build_cpu = x86_64
build_os = linux-gnu
build_vendor = pc
builddir = .
datadir = ${datarootdir}
datarootdir = ${prefix}/share
docdir = ${datarootdir}/doc/${PACKAGE_TARNAME}
dvidir = ${docdir}
exec_prefix = ${prefix}
gl_LIBOBJS =  accept.o bind.o connect.o dup2.o getpeername.o listen.o msvc-inval.o msvc-nothrow.o recv.o recvfrom.o select.o send.o sendto.o setsockopt.o socket.o
gl_LTLIBOBJS =  accept.lo bind.lo connect.lo dup2.lo getpeername.lo listen.lo msvc-inval.lo msvc-nothrow.lo recv.lo recvfrom.lo select.lo send.lo sendto.lo setsockopt.lo socket.lo
gltests_LIBOBJS = 
gltests_LTLIBOBJS = 
gltests_WITNESS = IN_ARDUMINGW_GNULIB_TESTS
host = x86_64-w64-mingw32
host_alias = x86_64-w64-mingw32
host_cpu = x86_64
host_os = mingw32
host_vendor = w64
htmldir = ${docdir}
includedir = ${prefix}/include
infodir = ${datarootdir}/info
install_sh = ${SHELL} /home/buzz/ardupilot/install-sh
libdir = ${exec_prefix}/lib
libexecdir = ${exec_prefix}/libexec
localedir = ${datarootdir}/locale
localstatedir = ${prefix}/var
mandir = ${datarootdir}/man
mkdir_p = $(MKDIR_P)
oldincludedir = /usr/include
pdfdir = ${docdir}
prefix = /usr/local
program_transform_name = s,x,x,
psdir = ${docdir}
runstatedir = ${localstatedir}/run
sbindir = ${exec_prefix}/sbin
sharedstatedir = ${prefix}/com
srcdir = .
sysconfdir = ${prefix}/etc
target_alias = 
top_build_prefix = 
top_builddir = .
top_srcdir = .
ACLOCAL_AMFLAGS = -I m4
AC_CONFIG_FILES = lib/Makefile
SUBDIRS = lib
EXTRA_DIST = m4/gnulib-cache.m4
AM_CONDITIONAL = GL_GENERATE_ERRNO_H GL_GENERATE_STDALIGN_H
AM_CPPFLAGS = -I$(top_builddir)/lib -I$(top_srcdir)/lib
LDADD = lib/libgnu.a
all: all-recursive

.SUFFIXES:
am--refresh: Makefile
	@:
$(srcdir)/Makefile.in:  $(srcdir)/Makefile.am  $(am__configure_deps)
	@for dep in $?; do \
	  case '$(am__configure_deps)' in \
	    *$$dep*) \
	      echo ' cd $(srcdir) && $(AUTOMAKE) --gnu'; \
	      $(am__cd) $(srcdir) && $(AUTOMAKE) --gnu \
		&& exit 0; \
	      exit 1;; \
	  esac; \
	done; \
	echo ' cd $(top_srcdir) && $(AUTOMAKE) --gnu Makefile'; \
	$(am__cd) $(top_srcdir) && \
	  $(AUTOMAKE) --gnu Makefile
Makefile: $(srcdir)/Makefile.in $(top_builddir)/config.status
	@case '$?' in \
	  *config.status*) \
	    echo ' $(SHELL) ./config.status'; \
	    $(SHELL) ./config.status;; \
	  *) \
	    echo ' cd $(top_builddir) && $(SHELL) ./config.status $@ $(am__maybe_remake_depfiles)'; \
	    cd $(top_builddir) && $(SHELL) ./config.status $@ $(am__maybe_remake_depfiles);; \
	esac;

$(top_builddir)/config.status: $(top_srcdir)/configure $(CONFIG_STATUS_DEPENDENCIES)
	$(SHELL) ./config.status --recheck

$(top_srcdir)/configure:  $(am__configure_deps)
	$(am__cd) $(srcdir) && $(AUTOCONF)
$(ACLOCAL_M4):  $(am__aclocal_m4_deps)
	$(am__cd) $(srcdir) && $(ACLOCAL) $(ACLOCAL_AMFLAGS)
$(am__aclocal_m4_deps):

# This directory's subdirectories are mostly independent; you can cd
# into them and run 'make' without going through this Makefile.
# To change the values of 'make' variables: instead of editing Makefiles,
# (1) if the variable is set in 'config.status', edit 'config.status'
#     (which will cause the Makefiles to be regenerated when you run 'make');
# (2) otherwise, pass the desired values on the 'make' command line.
$(am__recursive_targets):
	@fail=; \
	if $(am__make_keepgoing); then \
	  failcom='fail=yes'; \
	else \
	  failcom='exit 1'; \
	fi; \
	dot_seen=no; \
	target=`echo $@ | sed s/-recursive//`; \
	case "$@" in \
	  distclean-* | maintainer-clean-*) list='$(DIST_SUBDIRS)' ;; \
	  *) list='$(SUBDIRS)' ;; \
	esac; \
	for subdir in $$list; do \
	  echo "Making $$target in $$subdir"; \
	  if test "$$subdir" = "."; then \
	    dot_seen=yes; \
	    local_target="$$target-am"; \
	  else \
	    local_target="$$target"; \
	  fi; \
	  ($(am__cd) $$subdir && $(MAKE) $(AM_MAKEFLAGS) $$local_target) \
	  || eval $$failcom; \
	done; \
	if test "$$dot_seen" = "no"; then \
	  $(MAKE) $(AM_MAKEFLAGS) "$$target-am" || exit 1; \
	fi; test -z "$$fail"

ID: $(am__tagged_files)
	$(am__define_uniq_tagged_files); mkid -fID $$unique
tags: tags-recursive
TAGS: tags

tags-am: $(TAGS_DEPENDENCIES) $(am__tagged_files)
	set x; \
	here=`pwd`; \
	if ($(ETAGS) --etags-include --version) >/dev/null 2>&1; then \
	  include_option=--etags-include; \
	  empty_fix=.; \
	else \
	  include_option=--include; \
	  empty_fix=; \
	fi; \
	list='$(SUBDIRS)'; for subdir in $$list; do \
	  if test "$$subdir" = .; then :; else \
	    test ! -f $$subdir/TAGS || \
	      set "$$@" "$$include_option=$$here/$$subdir/TAGS"; \
	  fi; \
	done; \
	$(am__define_uniq_tagged_files); \
	shift; \
	if test -z "$(ETAGS_ARGS)$$*$$unique"; then :; else \
	  test -n "$$unique" || unique=$$empty_fix; \
	  if test $$# -gt 0; then \
	    $(ETAGS) $(ETAGSFLAGS) $(AM_ETAGSFLAGS) $(ETAGS_ARGS) \
	      "$$@" $$unique; \
	  else \
	    $(ETAGS) $(ETAGSFLAGS) $(AM_ETAGSFLAGS) $(ETAGS_ARGS) \
	      $$unique; \
	  fi; \
	fi
ctags: ctags-recursive

CTAGS: ctags
ctags-am: $(TAGS_DEPENDENCIES) $(am__tagged_files)
	$(am__define_uniq_tagged_files); \
	test -z "$(CTAGS_ARGS)$$unique" \
	  || $(CTAGS) $(CTAGSFLAGS) $(AM_CTAGSFLAGS) $(CTAGS_ARGS) \
	     $$unique

GTAGS:
	here=`$(am__cd) $(top_builddir) && pwd` \
	  && $(am__cd) $(top_srcdir) \
	  && gtags -i $(GTAGS_ARGS) "$$here"
cscope: cscope.files
	test ! -s cscope.files \
	  || $(CSCOPE) -b -q $(AM_CSCOPEFLAGS) $(CSCOPEFLAGS) -i cscope.files $(CSCOPE_ARGS)
clean-cscope:
	-rm -f cscope.files
cscope.files: clean-cscope cscopelist
cscopelist: cscopelist-recursive

cscopelist-am: $(am__tagged_files)
	list='$(am__tagged_files)'; \
	case "$(srcdir)" in \
	  [\\/]* | ?:[\\/]*) sdir="$(srcdir)" ;; \
	  *) sdir=$(subdir)/$(srcdir) ;; \
	esac; \
	for i in $$list; do \
	  if test -f "$$i"; then \
	    echo "$(subdir)/$$i"; \
	  else \
	    echo "$$sdir/$$i"; \
	  fi; \
	done >> $(top_builddir)/cscope.files

distclean-tags:
	-rm -f TAGS ID GTAGS GRTAGS GSYMS GPATH tags
	-rm -f cscope.out cscope.in.out cscope.po.out cscope.files

distdir: $(BUILT_SOURCES)
	$(MAKE) $(AM_MAKEFLAGS) distdir-am

distdir-am: $(DISTFILES)
	$(am__remove_distdir)
	test -d "$(distdir)" || mkdir "$(distdir)"
	@srcdirstrip=`echo "$(srcdir)" | sed 's/[].[^$$\\*]/\\\\&/g'`; \
	topsrcdirstrip=`echo "$(top_srcdir)" | sed 's/[].[^$$\\*]/\\\\&/g'`; \
	list='$(DISTFILES)'; \
	  dist_files=`for file in $$list; do echo $$file; done | \
	  sed -e "s|^$$srcdirstrip/||;t" \
	      -e "s|^$$topsrcdirstrip/|$(top_builddir)/|;t"`; \
	case $$dist_files in \
	  */*) $(MKDIR_P) `echo "$$dist_files" | \
			   sed '/\//!d;s|^|$(distdir)/|;s,/[^/]*$$,,' | \
			   sort -u` ;; \
	esac; \
	for file in $$dist_files; do \
	  if test -f $$file || test -d $$file; then d=.; else d=$(srcdir); fi; \
	  if test -d $$d/$$file; then \
	    dir=`echo "/$$file" | sed -e 's,/[^/]*$$,,'`; \
	    if test -d "$(distdir)/$$file"; then \
	      find "$(distdir)/$$file" -type d ! -perm -700 -exec chmod u+rwx {} \;; \
	    fi; \
	    if test -d $(srcdir)/$$file && test $$d != $(srcdir); then \
	      cp -fpR $(srcdir)/$$file "$(distdir)$$dir" || exit 1; \
	      find "$(distdir)/$$file" -type d ! -perm -700 -exec chmod u+rwx {} \;; \
	    fi; \
	    cp -fpR $$d/$$file "$(distdir)$$dir" || exit 1; \
	  else \
	    test -f "$(distdir)/$$file" \
	    || cp -p $$d/$$file "$(distdir)/$$file" \
	    || exit 1; \
	  fi; \
	done
	@list='$(DIST_SUBDIRS)'; for subdir in $$list; do \
	  if test "$$subdir" = .; then :; else \
	    $(am__make_dryrun) \
	      || test -d "$(distdir)/$$subdir" \
	      || $(MKDIR_P) "$(distdir)/$$subdir" \
	      || exit 1; \
	    dir1=$$subdir; dir2="$(distdir)/$$subdir"; \
	    $(am__relativize); \
	    new_distdir=$$reldir; \
	    dir1=$$subdir; dir2="$(top_distdir)"; \
	    $(am__relativize); \
	    new_top_distdir=$$reldir; \
	    echo " (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) top_distdir="$$new_top_distdir" distdir="$$new_distdir" \\"; \
	    echo "     am__remove_distdir=: am__skip_length_check=: am__skip_mode_fix=: distdir)"; \
	    ($(am__cd) $$subdir && \
	      $(MAKE) $(AM_MAKEFLAGS) \
	        top_distdir="$$new_top_distdir" \
	        distdir="$$new_distdir" \
		am__remove_distdir=: \
		am__skip_length_check=: \
		am__skip_mode_fix=: \
	        distdir) \
	      || exit 1; \
	  fi; \
	done
	-test -n "$(am__skip_mode_fix)" \
	|| find "$(distdir)" -type d ! -perm -755 \
		-exec chmod u+rwx,go+rx {} \; -o \
	  ! -type d ! -perm -444 -links 1 -exec chmod a+r {} \; -o \
	  ! -type d ! -perm -400 -exec chmod a+r {} \; -o \
	  ! -type d ! -perm -444 -exec $(install_sh) -c -m a+r {} {} \; \
	|| chmod -R a+r "$(distdir)"
dist-gzip: distdir
	tardir=$(distdir) && $(am__tar) | eval GZIP= gzip $(GZIP_ENV) -c >$(distdir).tar.gz
	$(am__post_remove_distdir)

dist-bzip2: distdir
	tardir=$(distdir) && $(am__tar) | BZIP2=$${BZIP2--9} bzip2 -c >$(distdir).tar.bz2
	$(am__post_remove_distdir)

dist-lzip: distdir
	tardir=$(distdir) && $(am__tar) | lzip -c $${LZIP_OPT--9} >$(distdir).tar.lz
	$(am__post_remove_distdir)

dist-xz: distdir
	tardir=$(distdir) && $(am__tar) | XZ_OPT=$${XZ_OPT--e} xz -c >$(distdir).tar.xz
	$(am__post_remove_distdir)

dist-tarZ: distdir
	@echo WARNING: "Support for distribution archives compressed with" \
		       "legacy program 'compress' is deprecated." >&2
	@echo WARNING: "It will be removed altogether in Automake 2.0" >&2
	tardir=$(distdir) && $(am__tar) | compress -c >$(distdir).tar.Z
	$(am__post_remove_distdir)

dist-shar: distdir
	@echo WARNING: "Support for shar distribution archives is" \
	               "deprecated." >&2
	@echo WARNING: "It will be removed altogether in Automake 2.0" >&2
	shar $(distdir) | eval GZIP= gzip $(GZIP_ENV) -c >$(distdir).shar.gz
	$(am__post_remove_distdir)

dist-zip: distdir
	-rm -f $(distdir).zip
	zip -rq $(distdir).zip $(distdir)
	$(am__post_remove_distdir)

dist dist-all:
	$(MAKE) $(AM_MAKEFLAGS) $(DIST_TARGETS) am__post_remove_distdir='@:'
	$(am__post_remove_distdir)

# This target untars the dist file and tries a VPATH configuration.  Then
# it guarantees that the distribution is self-contained by making another
# tarfile.
distcheck: dist
	case '$(DIST_ARCHIVES)' in \
	*.tar.gz*) \
	  eval GZIP= gzip $(GZIP_ENV) -dc $(distdir).tar.gz | $(am__untar) ;;\
	*.tar.bz2*) \
	  bzip2 -dc $(distdir).tar.bz2 | $(am__untar) ;;\
	*.tar.lz*) \
	  lzip -dc $(distdir).tar.lz | $(am__untar) ;;\
	*.tar.xz*) \
	  xz -dc $(distdir).tar.xz | $(am__untar) ;;\
	*.tar.Z*) \
	  uncompress -c $(distdir).tar.Z | $(am__untar) ;;\
	*.shar.gz*) \
	  eval GZIP= gzip $(GZIP_ENV) -dc $(distdir).shar.gz | unshar ;;\
	*.zip*) \
	  unzip $(distdir).zip ;;\
	esac
	chmod -R a-w $(distdir)
	chmod u+w $(distdir)
	mkdir $(distdir)/_build $(distdir)/_build/sub $(distdir)/_inst
	chmod a-w $(distdir)
	test -d $(distdir)/_build || exit 0; \
	dc_install_base=`$(am__cd) $(distdir)/_inst && pwd | sed -e 's,^[^:\\/]:[\\/],/,'` \
	  && dc_destdir="$${TMPDIR-/tmp}/am-dc-$$$$/" \
	  && am__cwd=`pwd` \
	  && $(am__cd) $(distdir)/_build/sub \
	  && ../../configure \
	    $(AM_DISTCHECK_CONFIGURE_FLAGS) \
	    $(DISTCHECK_CONFIGURE_FLAGS) \
	    --srcdir=../.. --prefix="$$dc_install_base" \
	  && $(MAKE) $(AM_MAKEFLAGS) \
	  && $(MAKE) $(AM_MAKEFLAGS) dvi \
	  && $(MAKE) $(AM_MAKEFLAGS) check \
	  && $(MAKE) $(AM_MAKEFLAGS) install \
	  && $(MAKE) $(AM_MAKEFLAGS) installcheck \
	  && $(MAKE) $(AM_MAKEFLAGS) uninstall \
	  && $(MAKE) $(AM_MAKEFLAGS) distuninstallcheck_dir="$$dc_install_base" \
	        distuninstallcheck \
	  && chmod -R a-w "$$dc_install_base" \
	  && ({ \
	       (cd ../.. && umask 077 && mkdir "$$dc_destdir") \
	       && $(MAKE) $(AM_MAKEFLAGS) DESTDIR="$$dc_destdir" install \
	       && $(MAKE) $(AM_MAKEFLAGS) DESTDIR="$$dc_destdir" uninstall \
	       && $(MAKE) $(AM_MAKEFLAGS) DESTDIR="$$dc_destdir" \
	            distuninstallcheck_dir="$$dc_destdir" distuninstallcheck; \
	      } || { rm -rf "$$dc_destdir"; exit 1; }) \
	  && rm -rf "$$dc_destdir" \
	  && $(MAKE) $(AM_MAKEFLAGS) dist \
	  && rm -rf $(DIST_ARCHIVES) \
	  && $(MAKE) $(AM_MAKEFLAGS) distcleancheck \
	  && cd "$$am__cwd" \
	  || exit 1
	$(am__post_remove_distdir)
	@(echo "$(distdir) archives ready for distribution: "; \
	  list='$(DIST_ARCHIVES)'; for i in $$list; do echo $$i; done) | \
	  sed -e 1h -e 1s/./=/g -e 1p -e 1x -e '$$p' -e '$$x'
distuninstallcheck:
	@test -n '$(distuninstallcheck_dir)' || { \
	  echo 'ERROR: trying to run $@ with an empty' \
	       '$$(distuninstallcheck_dir)' >&2; \
	  exit 1; \
	}; \
	$(am__cd) '$(distuninstallcheck_dir)' || { \
	  echo 'ERROR: cannot chdir into $(distuninstallcheck_dir)' >&2; \
	  exit 1; \
	}; \
	test `$(am__distuninstallcheck_listfiles) | wc -l` -eq 0 \
	   || { echo "ERROR: files left after uninstall:" ; \
	        if test -n "$(DESTDIR)"; then \
	          echo "  (check DESTDIR support)"; \
	        fi ; \
	        $(distuninstallcheck_listfiles) ; \
	        exit 1; } >&2
distcleancheck: distclean
	@if test '$(srcdir)' = . ; then \
	  echo "ERROR: distcleancheck can only run from a VPATH build" ; \
	  exit 1 ; \
	fi
	@test `$(distcleancheck_listfiles) | wc -l` -eq 0 \
	  || { echo "ERROR: files left in build directory after distclean:" ; \
	       $(distcleancheck_listfiles) ; \
	       exit 1; } >&2
check-am: all-am
check: check-recursive
all-am: Makefile
installdirs: installdirs-recursive
installdirs-am:
install: install-recursive
install-exec: install-exec-recursive
install-data: install-data-recursive
uninstall: uninstall-recursive

install-am: all-am
	@$(MAKE) $(AM_MAKEFLAGS) install-exec-am install-data-am

installcheck: installcheck-recursive
install-strip:
	if test -z '$(STRIP)'; then \
	  $(MAKE) $(AM_MAKEFLAGS) INSTALL_PROGRAM="$(INSTALL_STRIP_PROGRAM)" \
	    install_sh_PROGRAM="$(INSTALL_STRIP_PROGRAM)" INSTALL_STRIP_FLAG=-s \
	      install; \
	else \
	  $(MAKE) $(AM_MAKEFLAGS) INSTALL_PROGRAM="$(INSTALL_STRIP_PROGRAM)" \
	    install_sh_PROGRAM="$(INSTALL_STRIP_PROGRAM)" INSTALL_STRIP_FLAG=-s \
	    "INSTALL_PROGRAM_ENV=STRIPPROG='$(STRIP)'" install; \
	fi
mostlyclean-generic:

clean-generic:

distclean-generic:
	-test -z "$(CONFIG_CLEAN_FILES)" || rm -f $(CONFIG_CLEAN_FILES)
	-test . = "$(srcdir)" || test -z "$(CONFIG_CLEAN_VPATH_FILES)" || rm -f $(CONFIG_CLEAN_VPATH_FILES)

maintainer-clean-generic:
	@echo "This command is intended for maintainers to use"
	@echo "it deletes files that may require special tools to rebuild."
clean: clean-recursive

clean-am: clean-generic mostlyclean-am

distclean: distclean-recursive
	-rm -f $(am__CONFIG_DISTCLEAN_FILES)
	-rm -f Makefile
distclean-am: clean-am distclean-generic distclean-tags

dvi: dvi-recursive

dvi-am:

html: html-recursive

html-am:

info: info-recursive

info-am:

install-data-am:

install-dvi: install-dvi-recursive

install-dvi-am:

install-exec-am:

install-html: install-html-recursive

install-html-am:

install-info: install-info-recursive

install-info-am:

install-man:

install-pdf: install-pdf-recursive

install-pdf-am:

install-ps: install-ps-recursive

install-ps-am:

installcheck-am:

maintainer-clean: maintainer-clean-recursive
	-rm -f $(am__CONFIG_DISTCLEAN_FILES)
	-rm -rf $(top_srcdir)/autom4te.cache
	-rm -f Makefile
maintainer-clean-am: distclean-am maintainer-clean-generic

mostlyclean: mostlyclean-recursive

mostlyclean-am: mostlyclean-generic

pdf: pdf-recursive

pdf-am:

ps: ps-recursive

ps-am:

uninstall-am:

.MAKE: $(am__recursive_targets) install-am install-strip

.PHONY: $(am__recursive_targets) CTAGS GTAGS TAGS all all-am \
	am--refresh check check-am clean clean-cscope clean-generic \
	cscope cscopelist-am ctags ctags-am dist dist-all dist-bzip2 \
	dist-gzip dist-lzip dist-shar dist-tarZ dist-xz dist-zip \
	distcheck distclean distclean-generic distclean-tags \
	distcleancheck distdir distuninstallcheck dvi dvi-am html \
	html-am info info-am install install-am install-data \
	install-data-am install-dvi install-dvi-am install-exec \
	install-exec-am install-html install-html-am install-info \
	install-info-am install-man install-pdf install-pdf-am \
	install-ps install-ps-am install-strip installcheck \
	installcheck-am installdirs installdirs-am maintainer-clean \
	maintainer-clean-generic mostlyclean mostlyclean-generic pdf \
	pdf-am ps ps-am tags tags-am uninstall uninstall-am

.PRECIOUS: Makefile


# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
