hardware_modules := gralloc hwcomposer audio nfc nfc-nci local_time power usbaudio audio_remote_submix
ifneq ($(TARGET_USE_CUSTOM_WRAPPER),true)
hardware_modules += camera
endif
include $(call all-named-subdir-makefiles,$(hardware_modules))
