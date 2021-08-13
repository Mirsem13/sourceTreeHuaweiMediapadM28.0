

#ifndef __ADSNDINTERFACE_H__
#define __ADSNDINTERFACE_H__

/*****************************************************************************
  1 ͷ�ļ�����
*****************************************************************************/
#include "vos.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#pragma pack(4)


/*****************************************************************************
  2 �궨��
*****************************************************************************/
/* !!!!!!!!!!!��ʱ���壬���յ�ֵ�ɱ���ȷ�� */
#define ADS_NDCLIENT_MSG_HDR            (0x00)
#define NDCLIENT_ADS_MSG_HDR            (0x00)


/*****************************************************************************
  3 ö�ٶ���
*****************************************************************************/

enum ADS_NDCLIENT_MSG_ID_ENUM
{
    ID_ADS_NDCLIENT_DATA_IND                  =   ADS_NDCLIENT_MSG_HDR + 0x00,  /* ADS->NDCLIENT DATA IND */
    ID_ADS_NDCLIENT_ERR_IND                   =   ADS_NDCLIENT_MSG_HDR + 0x01,  /* ADS->NDCLIENT ERR IND */
    ID_ADS_NDCLIENT_MSG_ID_ENUM_BUTT
};
typedef VOS_UINT32  ADS_NDCLIENT_MSG_ID_ENUM_UINT32;


enum ADS_NDCLIENT_ERR_TYPE_ENUM
{
    ADS_NDCLIENT_ERR_TYPE_MTU_OVERLONG      = 0x00,                             /* ���ݰ����ȴ���MTU */
    ADS_NDCLIENT_ERR_TYPE_BUTT
};
typedef VOS_UINT8 ADS_NDCLIENT_ERR_TYPE_ENUM_UINT8;


/*****************************************************************************
  4 ȫ�ֱ�������
*****************************************************************************/


/*****************************************************************************
  5 ��Ϣͷ����
*****************************************************************************/


/*****************************************************************************
  6 ��Ϣ����
*****************************************************************************/


/*****************************************************************************
  7 STRUCT����
*****************************************************************************/

typedef struct
{
    VOS_MSG_HEADER                                                              /*_H2ASN_Skip*/
    ADS_NDCLIENT_MSG_ID_ENUM_UINT32     enMsgId;                                /*_H2ASN_Skip*/
    MODEM_ID_ENUM_UINT16                enModemId;
    VOS_UINT8                           ucRabId;                                /* RAB��ʶ��ȡֵ��Χ:[5,15] */
    ADS_NDCLIENT_ERR_TYPE_ENUM_UINT8    enErrType;                              /*��������*/
    VOS_UINT16                          usLen;                                  /*���ݰ�����*/
    VOS_UINT8                           aucData[2];                             /*���ݰ�����*/ 
}ADS_NDCLIENT_ERR_IND_STRU;


typedef struct
{
    VOS_MSG_HEADER                                                              /*_H2ASN_Skip*/
    ADS_NDCLIENT_MSG_ID_ENUM_UINT32     enMsgId;                                /*_H2ASN_Skip*/
    MODEM_ID_ENUM_UINT16                enModemId;
    VOS_UINT8                           ucRabId;                                /* RAB��ʶ��ȡֵ��Χ:[5,15] */
    VOS_UINT8                           aucRsv[1];                              /*����λ*/
    VOS_UINT16                          usLen;                                  /*���ݰ�����*/
    VOS_UINT8                           aucData[2];                             /*���ݰ�����*/ 
}ADS_NDCLIENT_DATA_IND_STRU;


/*****************************************************************************
  8 UNION����
*****************************************************************************/


/*****************************************************************************
  9 OTHERS����
*****************************************************************************/

/*****************************************************************************
  10 ��������
*****************************************************************************/






#if (VOS_OS_VER == VOS_WIN32)
#pragma pack()
#else
#pragma pack(0)
#endif


#ifdef __cplusplus
    #if __cplusplus
        }
    #endif
#endif

#endif
