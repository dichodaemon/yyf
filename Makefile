##############################################################################
#
#    file                 : Makefile
#    created              : Thu Dec 19 15:29:07 CET 2013
#    copyright            : (C) 2002 YU Yufeng
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = yyf
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp driver.cpp opponent.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml kc-p4.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-yyf_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-yyf_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
