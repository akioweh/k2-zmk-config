add_subdirectory(hardware/drivers/iqs5xx)

if(CONFIG_SHIELD_TOUCAN_LEFT AND CONFIG_ZMK_DISPLAY_STATUS_SCREEN_CUSTOM)
  target_sources(app PRIVATE hardware/display/status_screen.c)
endif()
