ifndef BOARD_TYPE
$(error "You have to define Board Type to use this Makefile")
#export BOARD_TYPE=ap81fus
#export BOARD_TYPE=ap93fus
endif

#
# board related targets
#
include Makefile.$(BOARD_TYPE)

export WEBDIR=$(TOPDIR)/webs/web_pages/

######## build fakeroot host tool ########
fakeroot_build:
	cd $(TOPDIR)/util/fakeroot && make -f make.fakeroot fakeroot
	
#
#  CONFIG setting
#

# modules used by all boards
export BOARD_KERNEL_MODULES = netfilter netsched ts_kmp


# applications used by all board types 
BOARD_APPLICATIONS = busybox pppoe pptp l2tp bpa iptables tc_build lltd arp

# applications specified by special board
ifeq ($(BOARD_TYPE),ap83fus)
BOARD_APPLICATIONS += nas
endif 

#
# common build
#

#fs_prep:image_prep libs_copy dev_create
fs_prep:install_root_clean check_tftp libs_copy etc_prep empty_dir_prep

install_root_clean:
	-rm -rf $(INSTALL_ROOT)/*

libs_copy:
	$(TOPDIR)/util/cplibs
	
dev_create:
	rm -rf $(INSTALL_ROOT)/dev/*
	$(TOPDIR)/util/mkdev

etc_prep:
	rm -rf $(INSTALL_ROOT)/etc
	cp -r $(TOPDIR)/filesystem/ap93fus/etc $(INSTALL_ROOT)/

empty_dir_prep:
	if test -d $(INSTALL_ROOT)/mnt; then echo $(INSTALL_ROOT)/mnt exists; else mkdir $(INSTALL_ROOT)/mnt; fi;
	if test -d $(INSTALL_ROOT)/proc; then echo $(INSTALL_ROOT)/proc exists; else mkdir $(INSTALL_ROOT)/proc; fi;
	if test -d $(INSTALL_ROOT)/root; then echo $(INSTALL_ROOT)/root exists; else mkdir $(INSTALL_ROOT)/root; fi;
	if test -d $(INSTALL_ROOT)/tmp; then echo $(INSTALL_ROOT)/mnt exists; else mkdir $(INSTALL_ROOT)/tmp; fi;
	if test -d $(INSTALL_ROOT)/var; then echo $(INSTALL_ROOT)/var exists; else mkdir -p $(INSTALL_ROOT)/var/run; fi;
	
#
# kernel modules 
#

KERNEL_MODULE_PATH=$(TOPDIR)/images/$(BOARD_TYPE)/kernel_modules/

kernel_modules: kernel_modules_prep $(BOARD_KERNEL_MODULES) 

kernel_modules_install:
	mkdir -p $(INSTALL_ROOT_FOR_BOARD)/lib/modules/2.6.15/kernel/
	cp -Rf $(KERNEL_MODULE_PATH)/*.ko $(INSTALL_ROOT_FOR_BOARD)/lib/modules/2.6.15/kernel/

kernel_modules_prep:
	test -d $(KERNEL_MODULE_PATH) || mkdir -p $(KERNEL_MODULE_PATH)
	
netfilter:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=net/ipv4/netfilter modules	
	cp $(KERNELPATH)/net/ipv4/netfilter/*.ko $(KERNEL_MODULE_PATH)

netsched:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=net/sched modules	
	cp $(KERNELPATH)/net/sched/*.ko $(KERNEL_MODULE_PATH)

ts_kmp:
	$(MAKEARCH) -C $(KERNELPATH) SUBDIRS=lib modules	
	cp $(KERNELPATH)/lib/ts_kmp.ko $(KERNEL_MODULE_PATH)
	



apps_build: $(BOARD_APPLICATIONS)

busybox: # easy build for busybox, no clean
	cd $(TOPDIR)/apps/$(BUSYBOX); $(MAKE) PREFIX=$(INSTALL_ROOT) uninstall; \
	find . -name "*.o" | xargs rm -rf; \
	$(MAKE); \
	$(MAKE) PREFIX=$(INSTALL_ROOT) install
	
pppoe:
	cd $(TOPDIR)/apps/ppp-2.4.3/pppd && make clean && make CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar && \
	cp pppd $(INSTALL_ROOT)/usr/sbin

pptp:
	cd $(TOPDIR)/apps/pptp-1.7.1 && make clean && make CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar && \
	cp pptp $(INSTALL_ROOT)/usr/sbin

l2tp:
	cd $(TOPDIR)/apps/xl2tpd-1.1.12 && make clean &&  make CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar && \
	cp xl2tpd $(INSTALL_ROOT)/usr/sbin

bpa:
	cd $(TOPDIR)/apps/bpalogin-2.0.2 && make clean &&  make CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld AR=$(TOOLPREFIX)ar && \
	cp bpalogin $(INSTALL_ROOT)/usr/sbin
	
iptables:
	cd $(TOPDIR)/apps/iptables-1.3.7/ && \
	$(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)ld DO_IPV6=0 KERNEL_DIR=$(KERNELPATH) LIBDIR=/lib && \
	cp iptables $(INSTALL_ROOT)/sbin/ && \
	cd extensions && \
	test -d $(INSTALL_ROOT)/lib/iptables/ || mkdir -p $(INSTALL_ROOT)/lib/iptables/ && \
	mv -f libipt_DNAT.so libipt_MASQUERADE.so libipt_SNAT.so libipt_TCPMSS.so libipt_TRIGGER.so \
	libipt_comment.so libipt_icmp.so libipt_iprange.so libipt_mac.so \
	libipt_multiport.so libipt_standard.so libipt_state.so libipt_string.so \
	libipt_tcp.so libipt_time.so libipt_udp.so libipt_MARK.so libipt_multiurl.so $(INSTALL_ROOT)/lib/iptables/
	

tc_build:
	cd $(TOPDIR)/apps/iproute2-2.6.15 && make CC=$(TOOLPREFIX)gcc && \
	cp $(TOPDIR)/apps/iproute2-2.6.15/tc/tc $(INSTALL_ROOT)/sbin
	
lltd:
	@echo ********** making WinRally ************
	cd $(TOPDIR)/apps/lltd/wr841n-linux && \
	make clean && $(MAKE) CC=$(TOOLPREFIX)gcc LD=$(TOOLPREFIX)gcc && \
	cp lld2d $(INSTALL_ROOT)/usr/bin/

arp:
	cd $(TOPDIR)/apps/arp && \
	$(TOOLPREFIX)gcc -o arp arp.c && cp arp $(INSTALL_ROOT)/usr/

#
# utils
#

#
# fs and image
#
strip_bin:
	$(STRIP) $(INSTALL_ROOT_FOR_BOARD)/bin/*
	-$(STRIP) $(INSTALL_ROOT_FOR_BOARD)/sbin/*
	$(STRIP) $(INSTALL_ROOT_FOR_BOARD)/usr/bin/*
	$(STRIP) $(INSTALL_ROOT_FOR_BOARD)/usr/sbin/*
	
ifndef lzma
encode = -nolzma
endif

mk_squashfs_build:
	echo "" >> $(TOPDIR)/webs/models/model_tmp.conf
	cat $(TOPDIR)/webs/models/version.conf >> $(TOPDIR)/webs/models/model_tmp.conf
	echo "" >> $(TOPDIR)/webs/models/model_tmp.conf
	cat $(TOPDIR)/web_server/sys/buildtime.h >> $(TOPDIR)/webs/models/model_tmp.conf
	$(TOPDIR)/util/md5_des $(TOPDIR)/webs/models/model_tmp.conf $(INSTALL_ROOT_FOR_BOARD)/web/oem/model.conf enc
	find $(INSTALL_ROOT_FOR_BOARD) -name ".svn" | xargs rm -rf
	$(TOPDIR)/util/fakeroot/fakeroot/fakeroot $(TOPDIR)/util/buildFS_LZ
	ls -l $(IMAGEPATH)/$(BOARD_TYPE)-squashfs	
	cp $(IMAGEPATH)/$(BOARD_TYPE)-squashfs $(TFTPPATH)
	ls -l $(TFTPPATH)/$(BOARD_TYPE)-squashfs
	
build_all:fs_prep uboot kernel_build kernel_modules wireless apps_build 

