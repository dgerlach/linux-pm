# Generic MAILBOX API

obj-$(CONFIG_MAILBOX)		+= mailbox.o

obj-$(CONFIG_PL320_MBOX)	+= pl320.o

obj-$(CONFIG_OMAP1_MBOX)	+= mailbox_omap1.o
mailbox_omap1-objs		:= omap-mailbox.o mailbox-omap1.o
obj-$(CONFIG_OMAP2PLUS_MBOX)	+= mailbox_omap2.o
mailbox_omap2-objs		:= omap-mailbox.o mailbox-omap2.o
