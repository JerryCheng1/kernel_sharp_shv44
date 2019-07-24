#
# Makefile for DTV tuner dev.(Fseg FCI)
#

EXTRA_CFLAGS += -Idrivers/media/shtunerdrv_fseg/inc
TunerDev_fseg-objs := bbm.o fc8350.o fc8350_bb.o fc8350_i2c.o fc8350_isr.o fc8350_spi.o fc8350_tun.o fc8350_tun_table.o fci_hal.o fci_hpi.o fci_oal.o fci_ringbuffer.o fci_tun.o
obj-$(CONFIG_SHARP_DTV_TUNERDEV_FSEG) += TunerDev_fseg.o
