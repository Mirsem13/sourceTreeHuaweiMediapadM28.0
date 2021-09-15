#
# Copyright (C) 2015 The CyanogenMod Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
$(call inherit-product, $(SRC_TARGET_DIR)/product/aosp_base.mk)

PRODUCT_NAME := full_mozart
PRODUCT_DEVICE := mozart
PRODUCT_BRAND := Android
PRODUCT_MODEL := AOSP on Mozart
PRODUCT_MANUFACTURER := huawei
PRODUCT_RESTRICT_VENDOR_FILES := false

$(call inherit-product, device/huawei/mozart/device.mk)
$(call inherit-product-if-exists, vendor/huawei/mozart/mozart-vendor.mk)



