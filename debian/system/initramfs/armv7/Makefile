IMG_CPIO=../initrd_v7.cpio
all:
	find . ! -path "./Makefile" \
	| cpio -H newc  -ov > ${IMG_CPIO}

