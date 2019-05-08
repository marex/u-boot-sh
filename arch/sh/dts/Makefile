# SPDX-License-Identifier: GPL-2.0+

dtb-y += \
	r8a7791-porter-u-boot.dtb

targets += $(dtb-y)

# Add any required device tree compiler flags here
DTC_FLAGS +=

PHONY += dtbs
dtbs: $(addprefix $(obj)/, $(dtb-y))
	@:

clean-files := *.dtb *_HS
