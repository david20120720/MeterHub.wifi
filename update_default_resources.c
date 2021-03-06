
//----------------------------------------------------------------------------
//   The confidential and proprietary information contained in this file may
//   only be used by a person authorised under and to the extent permitted
//   by a subsisting licensing agreement from ARM Limited or its affiliates.
//
//          (C) COPYRIGHT 2013-2017 ARM Limited or its affiliates.
//              ALL RIGHTS RESERVED
//
//   This entire notice must be reproduced on all copies of this file
//   and copies of this file may only be made by a person if such person is
//   permitted to do so under the terms of a subsisting license agreement
//   from ARM Limited or its affiliates.
//----------------------------------------------------------------------------

#ifdef MBED_CLOUD_CLIENT_USER_CONFIG_FILE
#include MBED_CLOUD_CLIENT_USER_CONFIG_FILE
#endif

#include <stdint.h>

#ifdef MBED_CLOUD_DEV_UPDATE_ID
const uint8_t arm_uc_vendor_id[] = {
    0x47, 0x1a, 0xe, 0x7b, 0xcb, 0xf9, 0x5e, 0x94, 0xaa, 0x4, 0x82, 0xd, 0x64, 0x62, 0x3a, 0xda
};
const uint16_t arm_uc_vendor_id_size = sizeof(arm_uc_vendor_id);

const uint8_t arm_uc_class_id[]  = {
    0xdc, 0xd2, 0x4c, 0x7f, 0x98, 0x66, 0x5b, 0x53, 0x9c, 0xe0, 0xc, 0x2a, 0x6f, 0xb0, 0x4a, 0x34
};
const uint16_t arm_uc_class_id_size = sizeof(arm_uc_class_id);
#endif

#ifdef MBED_CLOUD_DEV_UPDATE_CERT
const uint8_t arm_uc_default_fingerprint[] =  {
    0x2d, 0xc2, 0x8b, 0x7c, 0x52, 0x11, 0x19, 0x2e, 0x6d, 0x5f, 0xd0, 0x85, 0x7, 0x32, 0xf4, 0x3c,
    0x7c, 0xf6, 0x39, 0x97, 0x7c, 0x64, 0xa8, 0x59, 0xfd, 0x7b, 0xb1, 0x9, 0xfd, 0xb, 0x6a, 0xf0
};
const uint16_t arm_uc_default_fingerprint_size =
    sizeof(arm_uc_default_fingerprint);

const uint8_t arm_uc_default_subject_key_identifier[] =  {
    0x3a, 0x3a, 0x31, 0x2a, 0x63, 0xd3, 0xd0, 0x34, 0x42, 0x6, 0x26, 0x8c, 0xa8, 0x26, 0x4c, 0x0,
    0xad, 0x8f, 0xd5, 0xe5
};
const uint16_t arm_uc_default_subject_key_identifier_size =
    sizeof(arm_uc_default_subject_key_identifier);

const uint8_t arm_uc_default_certificate[] = {
    0x30, 0x82, 0x1, 0x80, 0x30, 0x82, 0x1, 0x25, 0xa0, 0x3, 0x2, 0x1, 0x2, 0x2, 0x14, 0x59,
    0xa3, 0x7b, 0xc1, 0x45, 0x4e, 0x2b, 0x7f, 0x91, 0xf8, 0x19, 0x9b, 0x2f, 0xd5, 0x27, 0xba, 0x66,
    0x34, 0x40, 0x9a, 0x30, 0xa, 0x6, 0x8, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x4, 0x3, 0x2, 0x30,
    0x12, 0x31, 0x10, 0x30, 0xe, 0x6, 0x3, 0x55, 0x4, 0x3, 0xc, 0x7, 0x4d, 0x6f, 0x74, 0x2e,
    0x63, 0x6f, 0x6d, 0x30, 0x1e, 0x17, 0xd, 0x31, 0x39, 0x31, 0x32, 0x31, 0x35, 0x30, 0x36, 0x35,
    0x35, 0x32, 0x30, 0x5a, 0x17, 0xd, 0x32, 0x30, 0x30, 0x33, 0x31, 0x34, 0x30, 0x36, 0x35, 0x35,
    0x32, 0x30, 0x5a, 0x30, 0x12, 0x31, 0x10, 0x30, 0xe, 0x6, 0x3, 0x55, 0x4, 0x3, 0xc, 0x7,
    0x4d, 0x6f, 0x74, 0x2e, 0x63, 0x6f, 0x6d, 0x30, 0x59, 0x30, 0x13, 0x6, 0x7, 0x2a, 0x86, 0x48,
    0xce, 0x3d, 0x2, 0x1, 0x6, 0x8, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x3, 0x1, 0x7, 0x3, 0x42,
    0x0, 0x4, 0x67, 0x36, 0x3a, 0xab, 0x22, 0xc5, 0x93, 0x1b, 0x4f, 0xfd, 0x98, 0x3c, 0x9e, 0x96,
    0xdb, 0x75, 0x75, 0x43, 0x52, 0x18, 0xef, 0xf6, 0xf, 0x3f, 0xa9, 0x62, 0xe3, 0xf5, 0xa, 0x6f,
    0xea, 0x15, 0xf6, 0xe6, 0x8, 0xe0, 0x82, 0x59, 0x7f, 0xbc, 0x66, 0x2a, 0xf8, 0xcd, 0xee, 0xa4,
    0x65, 0xe5, 0xa4, 0x59, 0xa5, 0x81, 0xa5, 0x10, 0xca, 0xcb, 0x71, 0xdd, 0x95, 0xc6, 0x26, 0x1a,
    0x42, 0x66, 0xa3, 0x59, 0x30, 0x57, 0x30, 0xb, 0x6, 0x3, 0x55, 0x1d, 0xf, 0x4, 0x4, 0x3,
    0x2, 0x7, 0x80, 0x30, 0x14, 0x6, 0x3, 0x55, 0x1d, 0x11, 0x4, 0xd, 0x30, 0xb, 0x82, 0x9,
    0x6c, 0x6f, 0x63, 0x61, 0x6c, 0x68, 0x6f, 0x73, 0x74, 0x30, 0x13, 0x6, 0x3, 0x55, 0x1d, 0x25,
    0x4, 0xc, 0x30, 0xa, 0x6, 0x8, 0x2b, 0x6, 0x1, 0x5, 0x5, 0x7, 0x3, 0x3, 0x30, 0x1d,
    0x6, 0x3, 0x55, 0x1d, 0xe, 0x4, 0x16, 0x4, 0x14, 0x3a, 0x3a, 0x31, 0x2a, 0x63, 0xd3, 0xd0,
    0x34, 0x42, 0x6, 0x26, 0x8c, 0xa8, 0x26, 0x4c, 0x0, 0xad, 0x8f, 0xd5, 0xe5, 0x30, 0xa, 0x6,
    0x8, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x4, 0x3, 0x2, 0x3, 0x49, 0x0, 0x30, 0x46, 0x2, 0x21,
    0x0, 0xa2, 0x50, 0xff, 0x90, 0x26, 0x62, 0x74, 0x80, 0x9a, 0x3b, 0x1, 0x54, 0x2e, 0xf, 0xfc,
    0x53, 0x9e, 0x58, 0x2b, 0xf7, 0x26, 0x80, 0x79, 0xad, 0xa8, 0x36, 0x2f, 0xd, 0xcb, 0x9e, 0x96,
    0x40, 0x2, 0x21, 0x0, 0x88, 0xa, 0x86, 0xc5, 0xef, 0x89, 0x4b, 0x1e, 0xf9, 0xe, 0x76, 0x52,
    0xe3, 0x99, 0x2e, 0x49, 0xdd, 0x11, 0x3e, 0x1, 0x2d, 0xe8, 0x9f, 0xff, 0x36, 0x2d, 0x60, 0xa0,
    0x82, 0xfd, 0x22, 0xd4
};
const uint16_t arm_uc_default_certificate_size = sizeof(arm_uc_default_certificate);
#endif


#ifdef MBED_CLOUD_DEV_UPDATE_PSK
const uint8_t arm_uc_default_psk[] = {
    
};
const uint8_t arm_uc_default_psk_size = sizeof(arm_uc_default_psk);
const uint16_t arm_uc_default_psk_bits = sizeof(arm_uc_default_psk)*8;

const uint8_t arm_uc_default_psk_id[] = {
    
};
const uint8_t arm_uc_default_psk_id_size = sizeof(arm_uc_default_psk_id);
#endif
