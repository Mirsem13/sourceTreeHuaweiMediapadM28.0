$(call inherit-product, $(SRC_TARGET_DIR)/product/generic.mk)
PRODUCT_BRAND :=Huawei
PRODUCT_DEVICE :=MediaPadM2
PRODUCT_NAME :=MediaPadM2
PRODUCT_AAPT_CONFIG :=ldpi mdpi hdpi xhdpi
PRODUCT_AAPT_PREF_CONFIG := - (ldpi mdpi hdpi xhdpi)
PRODUCT_CHARACTERISTICS :=tablet
PRODUCT_COPY_FILES += \
	vendor/huawei/ffmpeg/lib/libbt-vendor.so:/system/lib/libbt-vendor.so \
	vendor/huawei/ffmpeg/lib/libOpenCL.so:/system/lib/libOpenCL.so \
	vendor/huawei/ffmpeg/lib/libOpenCL.so.1.1:/system/lib/libOpenCL.so.1.1 \
	vendor/huawei/ffmpeg/lib/libwvdrm_L3.so:/system/lib/libwvdrm_L3.so \
	vendor/huawei/ffmpeg/lib/libwvm.so:/system/lib/libwvm.so \
	vendor/huawei/ffmpeg/lib/mediadrm/libdrmclearkeyplugin.so:/system/lib/mediadrm/libdrmclearkeyplugin.so \
	vendor/huawei/ffmpeg/lib/mediadrm/libwvdrmengine.so:/system/lib/mediadrm/libwvdrmengine.so \
	vendor/huawei/ffmpeg/lib/egl/libGLES_mali.so:/system/lib/egl/libGLES_mali.so \
	vendor/huawei/ffmpeg/lib/drm/libdrmwvmplugin.so:/system/lib/drm/libdrmwvmplugin.so \
	vendor/huawei/ffmpeg/bin/foo:/system/bin/foo \