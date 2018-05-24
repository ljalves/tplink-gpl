#define CONFIG_CMDLINE "console=ttyS0,115200 root=/dev/mtdblock2 rootfstype=squashfs init=/sbin/init mtdparts=ar7100-nor0:128k(u-boot),1024k(kernel),2816k(rootfs),64k(config),64k(art) mem=32M"
