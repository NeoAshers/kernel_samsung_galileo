#
# Makefile for the SWAP modules
#

obj-y += master/ \
         buffer/ \
         ksyms/ \
         driver/ \
         writer/ \
         kprobe/ \
         uprobe/ \
         us_manager/ \
         ks_features/ \
         loader/ \
         sampler/ \
         energy/ \
         parser/ \
         retprobe/ \
         preload/ \
         fbiprobe/ \
         wsp/ \
         nsp/ \
         task_ctx/ \
         got_patcher/

ifneq ($(CONFIG_SWAP_KERNEL_IMMUTABLE), y)
endif # CONFIG_SWAP_KERNEL_IMMUTABLE
