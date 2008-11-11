#  $Id$
AUTOMAKE_OPTIONS=foreign

include $(top_srcdir)/rtems-pre.am

EXEEXT=$(OBJEXEEXT)

libamdEth_a_SOURCES     = amdeth.c wrap/wrap.h

include_HEADERS         = amdeth.h

lib_LIBRARIES           = libamdEth.a

bin_PROGRAMS            = amdeth
amdeth_SOURCES          =
amdeth_LDADD            = $(libamdEth_a_OBJECTS)
amdeth_DEPENDENCIES     = $(libamdEth_a_OBJECTS)
amdeth_LINK             = $(OBJLINK)

include $(top_srcdir)/rtems.am
