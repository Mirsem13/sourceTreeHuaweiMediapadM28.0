# Boot animation
TARGET_SCREEN_HEIGHT := 1920
TARGET_SCREEN_WIDTH := 1080

# Inherit device configuration
$(call inherit-product, device/lge/hammerhead/full_mozart.mk)

# Screen density
PRODUCT_AAPT_CONFIG := normal
PRODUCT_AAPT_PREF_CONFIG := xhdpi

PRODUCT_DEVICE := mozart
PRODUCT_NAME := lineage_mozart
PRODUCT_BRAND := huawei
PRODUCT_MODEL := Mediapda M2 8.0
PRODUCT_MANUFACTURER := HUAWEI

PRODUCT_BUILD_PROP_OVERRIDES += \
    PRODUCT_NAME=mozart \
    


