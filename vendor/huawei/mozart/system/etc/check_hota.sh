#!/system/bin/sh
hota_flag=/cache/recovery/last_recovery_log_upload
[ -f "${hota_flag}" ] && echo "dsm_recovery,20900,recovery_fail" > /dev/dsm
