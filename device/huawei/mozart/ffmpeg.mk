$(call inherit-product, device/hisi/ffmpeg/ffmpeg.mk)
PRODUCT_BRAND :=Huawei
PRODUCT_DEVICE :=MediaPadM2
PRODUCT_NAME :=MediaPadM2
PRODUCT_AAPT_CONFIG := normal xhdpi
PRODUCT_AAPT_PREF_CONFIG := xhdpi 
PRODUCT_CHARACTERISTICS :=tablet
PRODUCT_COPY_FILES += \
	vendor/hisi/ffmpeg/lib/libbt-vendor.so:/system/lib/libbt-vendor.so \
	vendor/hisi/ffmpeg/lib/libOpenCL.so:/system/lib/libOpenCL.so \
	vendor/hisi/ffmpeg/lib/libOpenCL.so.1.1:/system/lib/libOpenCL.so.1.1 \
	vendor/hisi/ffmpeg/lib/libwvdrm_L3.so:/system/lib/libwvdrm_L3.so \
	vendor/hisi/ffmpeg/lib/libwvm.so:/system/lib/libwvm.so \
	vendor/hisi/ffmpeg/lib/mediadrm/libdrmclearkeyplugin.so:/system/lib/mediadrm/libdrmclearkeyplugin.so \
	vendor/hisi/ffmpeg/lib/mediadrm/libwvdrmengine.so:/system/lib/mediadrm/libwvdrmengine.so \
	vendor/hisi/ffmpeg/lib/egl/libGLES_mali.so:/system/lib/egl/libGLES_mali.so \
	vendor/hisi/ffmpeg/lib/drm/libdrmwvmplugin.so:/system/lib/drm/libdrmwvmplugin.so \
	vendor/hisi/ffmpeg/bin/foo:/system/bin/foo \