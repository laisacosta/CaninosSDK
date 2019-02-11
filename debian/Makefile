
.PHONY: all rootfs initramfs clean

Q=

TOP_DIR=$(shell pwd)
ROOTFS_DIR=$(TOP_DIR)/rootfs_tmp
PATCH_DIR=$(TOP_DIR)/rootfs_patch
MODULES_DIR=$(TOP_DIR)/modules
SYS_DIR=$(TOP_DIR)/system



rootfs:
	$(Q)rm -f $(TOP_DIR)/system.img
	
	$(Q)tar jxvf $(SYS_DIR)/debian-buster.tar.bz2 -C $(TOP_DIR)/
	$(Q)mkdir -p $(ROOTFS_DIR)
	$(Q)sudo mount -o loop $(TOP_DIR)/system.img $(ROOTFS_DIR)
	
	$(Q)sudo rm -rf $(ROOTFS_DIR)/lib/modules/*
	$(Q)sudo cp -rf $(MODULES_DIR)/lib/* $(ROOTFS_DIR)/lib
	$(Q)sudo rm -rf $(ROOTFS_DIR)/lib/modules/3.10.37/build
	$(Q)sudo rm -rf $(ROOTFS_DIR)/lib/modules/3.10.37/source
	
	$(Q)sudo rm -rf $(PATCH_DIR)
	$(Q)sudo mkdir $(PATCH_DIR)
	$(Q)sudo cp -r $(SYS_DIR)/boards/common/rootfs/* $(PATCH_DIR)
	
	$(Q)sudo chown -R root:root $(PATCH_DIR)
	$(Q)sudo cp -rf $(PATCH_DIR)/etc/* $(ROOTFS_DIR)/etc
	$(Q)sudo cp -rf $(PATCH_DIR)/lib/* $(ROOTFS_DIR)/lib
	#$(Q)sudo cp -rf $(PATCH_DIR)/usr/* $(ROOTFS_DIR)/usr
	
	$(Q)sudo rm -rf $(PATCH_DIR)
	
	$(Q)sudo sync
	$(Q)sudo umount -lf $(ROOTFS_DIR)
	$(Q)sudo chmod 777 $(TOP_DIR)/system.img
	$(Q)rm -rf $(ROOTFS_DIR)

initramfs:
	$(Q)$(MAKE) -C $(SYS_DIR)/initramfs
	$(Q)mv $(SYS_DIR)/initramfs/ramdisk.img $(TOP_DIR)/

all: rootfs initramfs

