ACLOCAL_AMFLAGS = -I m4
AC_CONFIG_FILES = lib/Makefile
SUBDIRS = lib
EXTRA_DIST = m4/gnulib-cache.m4
AM_CONDITIONAL = GL_GENERATE_ERRNO_H GL_GENERATE_STDALIGN_H
AM_CPPFLAGS = -I$(top_builddir)/lib -I$(top_srcdir)/lib
LDADD = lib/libgnu.a

