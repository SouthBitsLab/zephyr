# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(icg20660l.c icg20660l_i2c.c icg20660l_spi.c)
zephyr_library_sources_ifdef(CONFIG_ICG20660L_TRIGGER icg20660l_trigger.c)
