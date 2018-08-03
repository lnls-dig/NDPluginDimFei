TOP = ..
include $(TOP)/configure/CONFIG

module_name := $(shell basename $(realpath ${TOP}))

MODULE_NAME ?= ${module_name}
INSTALL_PREFIX ?= /opt/epics
export MODULE_NAME
export INSTALL_PREFIX

install:
	$(TOP)/install/install.sh ${INSTALL_PREFIX}/${MODULE_NAME} $(TOP)

uninstall:
	$(TOP)/install/uninstall.sh ${INSTALL_PREFIX}/${MODULE_NAME} $(TOP)

realuninstall: uninstall

clean:

realclean: clean

distclean: realclean realuninstall

mrproper: clean

