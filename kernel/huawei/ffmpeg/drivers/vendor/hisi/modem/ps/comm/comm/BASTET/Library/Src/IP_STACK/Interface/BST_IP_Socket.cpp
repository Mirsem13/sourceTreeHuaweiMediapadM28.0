
/*****************************************************************************
  1 头文件包含
*****************************************************************************/
#include "BST_IP_Socket.h"
#include "BST_IP_PreProc.h"
#include "lwip/sockets.h"
#include "lwip/memp.h"
/*lint -e767*/
#define    THIS_FILE_ID        PS_FILE_ID_BST_IP_SOCKET_CPP
/*lint +e767*/
/*****************************************************************************
  2 函数声明
*****************************************************************************/
extern  "C" struct tcp_pcb *GetPcbBySock(int s);
/******************************************************************************
   3 私有定义
******************************************************************************/

/******************************************************************************
   4 全局变量定义
******************************************************************************/

/******************************************************************************
   5 函数实现
******************************************************************************/

#if( BST_VER_TYPE == BST_DBG_VER )
BST_IP_CSocket::BST_IP_CSocket(
    BST_PROCID_T    usProcId,
    BST_TASKID_T    usTaskId ):m_usProcId ( usProcId ), m_usTaskId ( usTaskId )
{
    m_Arg                               = BST_NULL_PTR;
    m_pCallBacks                        = BST_NULL_PTR;
    m_pFunGroup                         = BST_NULL_PTR;
    m_fd.lFd                            = BST_INVALID_FD;
    m_enSocketType                      = BST_IP_INVALID_SOCKET_TYPE;
    m_TrafficPool.ulRxByte              = 0;
    m_TrafficPool.ulTxByte              = 0;
    BST_IP_InitIpAddress( &m_stIpAddress );
}
#else
BST_IP_CSocket::BST_IP_CSocket()
{
    m_Arg                               = BST_NULL_PTR;
    m_pCallBacks                        = BST_NULL_PTR;
    m_pFunGroup                         = BST_NULL_PTR;
    m_fd.lFd                            = BST_INVALID_FD;
    m_enSocketType                      = BST_IP_INVALID_SOCKET_TYPE;
    m_TrafficPool.ulRxByte              = 0;
    m_TrafficPool.ulTxByte              = 0;
    BST_IP_InitIpAddress( &m_stIpAddress );
}
#endif


BST_IP_CSocket::~BST_IP_CSocket()
{
    BST_IP_PROTOCOL_TYPE                usProtocol;
    BST_IP_CRcverMng                   *RawRcvMng;

    RawRcvMng                           = BST_NULL_PTR;
    usProtocol                          = BST_IP_PROTOCOL_INVALID;

#if ( BST_OS_VER != BST_QCOM_ROS )
    try
    {
#endif
        BST_DBG_LOG( "BST_IP_CSocket::~BST_IP_CSocket" );
        if ( BST_INVALID_PORT != m_stIpAddress.usLocalPort )
        {
            usProtocol                  = GetProtoType();
            BST_IP_ApiRmvPort( usProtocol, m_stIpAddress.usLocalPort, BST_NULL_PTR );
        }
        RawRcvMng                       = BST_IP_CRcverMng::GetInstance();
        if( BST_NULL_PTR != RawRcvMng )
        {
            RawRcvMng->Detach( &m_fd );
        }
#if ( BST_OS_VER != BST_QCOM_ROS )
    }
    catch (...)
    {
        BST_RLS_LOG("BST_IP_CRcverMng: BST_IP_CRcverMng::GetInstance Error");
    }
#endif

    m_Arg                               = BST_NULL_PTR;
    m_pCallBacks                        = BST_NULL_PTR;
    m_pFunGroup                         = BST_NULL_PTR;
    m_fd.lFd                            = BST_INVALID_FD;
    m_enSocketType                      = BST_IP_INVALID_SOCKET_TYPE;
}

BST_IP_ERR_T BST_IP_CSocket::Socket ( BST_VOID )
{
    if ( !BST_IP_IsValidSocketType( m_enSocketType ) )
    {
        BST_RLS_LOG1( "CSocket m_enSocketType=%d is invalid.", m_enSocketType );
        return BST_IP_ERR_ARG;
    }
    if ( BST_NULL_PTR == m_pFunGroup )
    {
        BST_RLS_LOG("CSocket error m_pFunGroup = NULL.");
        return BST_IP_ERR_ARG;
    }
    if (BST_NULL_PTR == m_pFunGroup->pfSocket)
    {
        BST_RLS_LOG("CSocket error m_pFunGroup->pfSocket = NULL.");
        return BST_IP_ERR_ARG;
    }
    if( BST_IP_ERR_OK != BST_IP_CheckSocketSupport( m_enSocketType ) )
    {
        BST_RLS_LOG1("SocketType=%d is not Supported.", m_enSocketType );
        return BST_IP_ERR_ARG;
    }

    if( BST_SCKT_TYPE_RAW_UDP == m_enSocketType )
    {
        m_pFunGroup->pfSocket( &m_fd, &m_Arg, BST_IP_PROTOCOL_UDP );
    }
    else
    {
        m_pFunGroup->pfSocket( &m_fd, &m_Arg, BST_IP_PROTOCOL_TCP );
    }

    if( BST_IP_IsBsdSocketType( m_enSocketType ) )
    {
        if( !BST_IP_IsBsdFdValid( m_fd ) )
        {
            BST_RLS_LOG1( "BSD m_fd=%d is not valid.", m_fd.lFd );
            return BST_IP_ERR_CONN;
        }
    }
    else
    {
        if( !BST_IP_IsRawFdValid( m_fd ) )
        {
            BST_RLS_LOG1( "Raw m_fd=0x%x is not valid.", m_fd.pFd );
            return BST_IP_ERR_CONN;
        }
    }
    return BST_IP_ERR_OK;
}

BST_IP_ERR_T BST_IP_CSocket::Connect( BST_VOID )
{
    if( BST_IP_ERR_OK != CheckConfig() )
    {
        return BST_IP_ERR_ARG;
    }
    if ( !BST_IP_IsValidFunGrp( m_pFunGroup ) )
    {
        return BST_IP_ERR_ARG;
    }
    if( !BST_IP_IsValidFun( m_pFunGroup->pfConnect ) )
    {
        return BST_IP_ERR_CONN;
    }
    BST_IP_ApiAddPort( GetProtoType(), m_stIpAddress.usLocalPort );
    BST_IP_ApiSetPreprocWorking( GetProtoType(), m_stIpAddress.usLocalPort );
    return m_pFunGroup->pfConnect( m_fd, m_Arg, &m_stIpAddress );
}


BST_IP_ERR_T BST_IP_CSocket::Clone( BST_VOID )
{
    struct tcp_pcb                     *pstPcb;
    BST_IP_ERR_T                        lRtnVal;

    if( BST_IP_ERR_OK != CheckConfig() )
    {
        return BST_IP_ERR_ARG;
    }
    if ( !BST_IP_IsValidFunGrp( m_pFunGroup ) )
    {
        return BST_IP_ERR_ARG;
    }
    if( !BST_IP_IsValidFun( m_pFunGroup->pfClone ) )
    {
        return BST_IP_ERR_CONN;
    }

    if ( BST_IP_IsBsdSocketType ( m_enSocketType ) )
    {
        if (!BST_IP_IsBsdFdValid( m_fd ))
        {
            return BST_IP_ERR_ARG;
        }
        pstPcb                      = GetPcbBySock( m_fd.lFd );
    }
    else
    {
        if (!BST_IP_IsRawFdValid(m_fd))
        {
            return BST_IP_ERR_ARG;
        }
        pstPcb                      = (struct tcp_pcb*)m_fd.pFd;
    }

    if ( BST_NULL_PTR == pstPcb )
    {
        return BST_IP_ERR_ARG;
    }
    pstPcb->local_ip.addr           = IPADDR_ANY;
    lRtnVal                         = m_pFunGroup->pfClone( m_fd, &m_stIpAddress );
    if ( BST_IP_ERR_OK !=  lRtnVal )
    {
        BST_RLS_LOG1( "BST_IP_CSocket::Clone failed, Error:%u", lRtnVal );
    }
    return lRtnVal;
}


BST_IP_ERR_T BST_IP_CSocket::FreePcb( BST_VOID )
{
    struct tcp_pcb                     *pstPcb;
    pstPcb                              = BST_NULL_PTR;

    if ( BST_IP_IsBsdSocketType ( m_enSocketType ) )
    {
        if ( !BST_IP_IsBsdFdValid( m_fd ) )
        {
            return BST_IP_ERR_ARG;
        }
        pstPcb                          = GetPcbBySock( m_fd.lFd );
        if ( BST_NULL_PTR == pstPcb )
        {
            return BST_IP_ERR_ARG;
        }
        BST_IP_ApiDropTcpPcb( pstPcb, BST_TRUE );
        m_fd.lFd                        = BST_INVALID_FD;
    }
    else
    {
        if ( !BST_IP_IsRawFdValid( m_fd ) )
        {
            return BST_IP_ERR_ARG;
        }
        pstPcb                          = ( struct tcp_pcb * )m_fd.pFd;
        BST_IP_ApiDropTcpPcb( pstPcb, BST_TRUE );
        m_fd.pFd                        = BST_NULL_PTR;
    }

    return BST_IP_ERR_OK;
}


BST_IP_ERR_T BST_IP_CSocket::Bind( BST_VOID )
{
    if( BST_IP_ERR_OK != CheckConfig() )   //这里需要校验的是本地的端口号
    {
        return BST_IP_ERR_ARG;
    }

    if ( !BST_IP_IsValidFunGrp( m_pFunGroup ) )
    {
        return BST_IP_ERR_ARG;
    }
    if ( !BST_IP_IsValidFun( m_pFunGroup->pfBind ) )
    {
        return BST_IP_ERR_ARG;
    }
    BST_DBG_LOG1( "BST_IP_CSocket::Bind usLocalPort=%u",
                  m_stIpAddress.usLocalPort );
    return (m_pFunGroup->pfBind( m_fd, m_stIpAddress.usLocalPort ));
}


BST_IP_ERR_T BST_IP_CSocket::Close( BST_VOID )
{
    BST_IP_ERR_T                        usErrVal; 

    if ( BST_IP_ERR_OK != CheckConfig() )
    {
        return BST_IP_ERR_ARG;
    }

    if ( !BST_IP_IsValidFunGrp( m_pFunGroup ) )
    {
        return BST_IP_ERR_ARG;
    }
    if ( !BST_IP_IsValidFun( m_pFunGroup->pfClose ) )
    {
        return BST_IP_ERR_ARG;
    }

    usErrVal                            = m_pFunGroup->pfClose(m_fd, m_Arg);

    if ( BST_IP_IsBsdSocketType ( m_enSocketType ) )
    {
        m_fd.lFd                        = BST_INVALID_FD;
    }
    else
    {
        /* LwIP has freed this pcb, cut pcb connection. */
        if ( BST_IP_ERR_CLR == usErrVal )
        {
            m_fd.pFd                    = BST_NULL_PTR;
        }
    }

    return usErrVal;
}
BST_IP_ERR_T BST_IP_CSocket::Write( const BST_UINT8 *pucSdu, BST_UINT16 usLength )
{
    if(( BST_NULL_PTR == pucSdu ) || (0 == usLength))
    {
       return BST_IP_ERR_ARG;
    }

    if( BST_IP_ERR_OK != CheckConfig() )
    {
        return BST_IP_ERR_ARG;
    }

    if ( !BST_IP_IsValidFunGrp( m_pFunGroup ) )
    {
        return BST_IP_ERR_ARG;
    }
    if ( !BST_IP_IsValidFun( m_pFunGroup->pfSend ) )
    {
        return BST_IP_ERR_ARG;
    }

    return ( m_pFunGroup->pfSend( m_fd, const_cast<BST_UINT8 *>(pucSdu), usLength ) );
}


BST_IP_ERR_T BST_IP_CSocket::Read( BST_UINT8 *pucSdu, BST_UINT16 usLength )
{
    if ( BST_NULL_PTR == pucSdu )
    {
        return BST_IP_ERR_ARG;
    }
    if ( 0 == usLength )
    {
        return BST_IP_ERR_ARG;
    }
    if ( BST_IP_ERR_OK != CheckConfig() )
    {
        return BST_IP_ERR_ARG;
    }
    if ( !BST_IP_IsValidFunGrp( m_pFunGroup ) )
    {
        return BST_IP_ERR_ARG;
    }
    if ( !BST_IP_IsValidFun( m_pFunGroup->pfReceive ) )
    {
        return BST_IP_ERR_ARG;
    }

    return (m_pFunGroup->pfReceive(m_fd, pucSdu, usLength));
}


BST_IP_ERR_T BST_IP_CSocket::IoCtrl( BST_SCKT_CMD_ENUM_UINT32 enCmd, BST_VOID *pvPara )
{
    BST_IP_SOCKET_ADD_T               **ppSocketAddr;
    BST_SCKT_TYPE_ENUM_UINT32           ulSocketType;
    BST_IP_ERR_T                        enIpErr;

    if ( BST_NULL_PTR == pvPara )
    {
        return BST_IP_ERR_ARG;
    }

    enIpErr                             = BST_IP_ERR_OK;
    switch (enCmd)
    {
    case BST_SCKT_CMD_GET_ADDRPTR:
        ppSocketAddr                    = ( BST_IP_SOCKET_ADD_T **)pvPara;
       *ppSocketAddr                    = &m_stIpAddress;
        break;
    case BST_SCKT_CMD_SET_ADDRESS:
        enIpErr                         = SetAddr( ( BST_IP_SOCKET_ADD_T *)pvPara );
        break;
    case BST_SCKT_CMD_SET_PROPERTY:
        enIpErr                         = SetPrpty( (BST_IP_SKT_PROPTY_STRU *)pvPara );
        BST_IP_ApiSetPreprocWorking( BST_IP_PROTOCOL_TCP, m_stIpAddress.usLocalPort );
        break;

    case BST_SCKT_CMD_GET_PROPERTY:
        enIpErr                         = GetPrpty( (BST_IP_SKT_PROPTY_STRU *)pvPara );
        break;

    case BST_SCKT_CMD_RPT_PROPERTY:
        BST_IP_ApiSetPreprocSuspend( BST_IP_PROTOCOL_TCP, m_stIpAddress.usLocalPort );
        enIpErr                         = GetPrpty( (BST_IP_SKT_PROPTY_STRU *)pvPara );
        break;

    case BST_SCKT_CMD_GET_TRAFFIC_FLOW:
        enIpErr                         = GetTrafficFlow( (BST_IP_TRAFFIC_FLOW_STRU *)pvPara, BST_TRUE );
        break;

    case BST_SCKT_CMD_SET_TRAFFIC_FLOW:
        enIpErr                         = SetTrafficFlow( (BST_IP_TRAFFIC_FLOW_STRU *)pvPara );
        break;

    case BST_SCKT_CMD_REG_CALLBACK:
        enIpErr                         = RegCallBack( (BST_IP_CNetRcver *)pvPara );
        break;

    case BST_SCKT_CMD_INQ_STATE:
        break;

    case BST_SCKT_CMD_INQ_FLOW:
        break;

    case BST_SCKT_CMD_CHK_FD:
        enIpErr                         = CheckFd( (BST_FD_T *)pvPara );
        break;

    case BST_SCKT_CMD_DEF_TYPE:
        BST_OS_MEMCPY( &ulSocketType, pvPara, BST_OS_SIZEOF(BST_SCKT_TYPE_ENUM_UINT32) );
        enIpErr                         = SetSocketType( ulSocketType );
        break;

    case BST_SCKT_CMD_GET_FD:
        enIpErr                         = GetSocketFd( (BST_FD_T *)pvPara );
        break;

    default:
        enIpErr                         = BST_IP_ERR_ARG;
        break;
    }
    return enIpErr;
}


BST_IP_ERR_T BST_IP_CSocket::SetAddr( BST_IP_SOCKET_ADD_T *pstAddress )
{
    BST_IP_SOCKET_ADD_T                 stSockAddrTmp;
    BST_IP_TRAFFIC_FLOW_STRU            stPortTrfcFlow;
    BST_IP_PROTOCOL_TYPE                usProtocol;
    usProtocol                          = BST_IP_PROTOCOL_INVALID;

    BST_ASSERT_NULL_RTN( pstAddress, BST_IP_ERR_ARG );

    BST_DBG_LOG4( "BST_IP_CSocket::SetAddr IP Address %d.%d.%d.%d",\
                  pstAddress->aucRemoteIp[3],\
                  pstAddress->aucRemoteIp[2],\
                  pstAddress->aucRemoteIp[1],\
                  pstAddress->aucRemoteIp[0] );

    BST_DBG_LOG2( "BST_IP_CSocket::SetAddr Dst Port %d, Src Port %d",\
                  pstAddress->usRemotePort,\
                  pstAddress->usLocalPort );

    BST_OS_MEMCPY( &stSockAddrTmp, pstAddress, sizeof( BST_IP_SOCKET_ADD_T ) );

    /* 若两次相等，说明是重复下发*/
    if( 0 == BST_OS_Memcmp( &m_stIpAddress, &stSockAddrTmp, 
                            BST_OS_SIZEOF( BST_IP_SOCKET_ADD_T ) ) )
    {
        return BST_IP_ERR_OK;
    }

    usProtocol                          = GetProtoType();
    if ( BST_INVALID_PORT != m_stIpAddress.usLocalPort )
    {
        stPortTrfcFlow.ulRxByte         = 0;
        stPortTrfcFlow.ulTxByte         = 0;
        BST_IP_ApiRmvPort( usProtocol, m_stIpAddress.usLocalPort, &stPortTrfcFlow );
        m_TrafficPool.ulRxByte         += stPortTrfcFlow.ulRxByte;
        m_TrafficPool.ulTxByte         += stPortTrfcFlow.ulTxByte;
    }
    if ( BST_INVALID_PORT != stSockAddrTmp.usLocalPort )
    {
        BST_IP_ApiAddPort( usProtocol, stSockAddrTmp.usLocalPort );
    }

    BST_OS_MEMCPY( &m_stIpAddress, &stSockAddrTmp, BST_OS_SIZEOF( BST_IP_SOCKET_ADD_T ) );
    return BST_IP_ERR_OK;
}
BST_IP_ERR_T BST_IP_CSocket::SetPrpty( BST_IP_SKT_PROPTY_STRU *pstProperty )
{
    struct tcp_pcb                     *pstPcb;
    BST_IP_SKT_PROPTY_STRU              stSockProptyTmp = { 0u };

    if ( BST_NULL_PTR == pstProperty )
    {
        return BST_IP_ERR_ARG;
    }

    if( 0 == BST_OS_Memcmp( &stSockProptyTmp, pstProperty,
                            BST_OS_SIZEOF( BST_IP_SKT_PROPTY_STRU )))
    {
        return BST_IP_ERR_OK;
    }
    pstPcb                              = BST_NULL_PTR;

    BST_OS_MEMCPY( &stSockProptyTmp, pstProperty, BST_OS_SIZEOF( BST_IP_SKT_PROPTY_STRU ) );

    switch( m_enSocketType )
    {
        case BST_SCKT_TYPE_RAW_TCP:
            if (!BST_IP_IsRawFdValid(m_fd))
            {
                return BST_IP_ERR_ARG;
            }
            pstPcb                      = (struct tcp_pcb*)m_fd.pFd;
            break;

        case BST_SCKT_TYPE_BSD:
            if (!BST_IP_IsBsdFdValid( m_fd ))
            {
                return BST_IP_ERR_ARG;
            }
            pstPcb                      = GetPcbBySock( m_fd.lFd );
            break;

        default:
            return BST_IP_ERR_ARG;
    }

    if ( BST_NULL_PTR == pstPcb )
    {
        return BST_IP_ERR_ARG;
    }

    if( BST_NO_ERROR_MSG != BST_IP_ApiSetTcpPcbProperty( pstPcb, &stSockProptyTmp ) )
    {
       BST_RLS_LOG( "BST_IP_CSocket::SetPrpty failed" );
       return BST_IP_ERR_ARG;
    }
    return BST_IP_ERR_OK;
}

BST_IP_ERR_T BST_IP_CSocket::GetPrpty( BST_IP_SKT_PROPTY_STRU *pstProperty )
{
    struct tcp_pcb                     *pstPcb;
    
    if ( ( BST_NULL_PTR == pstProperty ) )
    {
        return BST_IP_ERR_ARG;
    }

    switch( m_enSocketType )
    {
        case BST_SCKT_TYPE_RAW_TCP:
            if ( !BST_IP_IsRawFdValid(m_fd) )
            {
                return BST_IP_ERR_ARG;
            }
            pstPcb                      = ( struct tcp_pcb* )m_fd.pFd;
            break;

        case BST_SCKT_TYPE_BSD:
            if ( !BST_IP_IsBsdFdValid(m_fd) )
            {
                return BST_IP_ERR_ARG;
            }
            pstPcb                      = GetPcbBySock( m_fd.lFd );
            break;

        default:
            return BST_IP_ERR_ARG;
    }
    if( BST_NO_ERROR_MSG != BST_IP_ApiGetTcpPcbProperty( pstPcb, pstProperty ) )  //该函数已经区分当前是透传数据包的seq还是其他场景的seq
    {
        BST_RLS_LOG( "BST_IP_CSocket::GetPrpty failed" );
        return BST_IP_ERR_VAL;
    }

    return BST_IP_ERR_OK;
}


BST_IP_ERR_T BST_IP_CSocket::GetTrafficFlow( BST_IP_TRAFFIC_FLOW_STRU *pstProperty, BST_UINT16  usIsReset)
{
    BST_ERR_ENUM_UINT8                  enRtnVal;
    BST_IP_TRAFFIC_FLOW_STRU            stTmpTrfcFlow;

    if ( ( BST_NULL_PTR == pstProperty ) )
    {
        return BST_IP_ERR_ARG;
    }

   /*
    * Get the traffic flow value from the buffer pool firstly
    */
    pstProperty->ulRxByte               = m_TrafficPool.ulRxByte;
    pstProperty->ulTxByte               = m_TrafficPool.ulTxByte;
    
    if ( ( 0 != m_TrafficPool.ulRxByte )
       ||( 0 != m_TrafficPool.ulTxByte ) )
    {
        BST_IP_ApiUpdateTrafficFlow ( BST_IP_TRAFFIC_DEC,
                                    ( m_TrafficPool.ulRxByte +
                                      m_TrafficPool.ulTxByte ) );
        m_TrafficPool.ulRxByte          = 0;
        m_TrafficPool.ulTxByte          = 0;
    }

   /*
    * Try to get the traffic flow value from the current port property
    */
    stTmpTrfcFlow.ulRxByte              = 0;
    stTmpTrfcFlow.ulTxByte              = 0;
    enRtnVal                            = BST_IP_ApiGetTrafficFlow( usIsReset,
                                                                    m_stIpAddress.usLocalPort,
                                                                   &stTmpTrfcFlow );
    if( BST_NO_ERROR_MSG == enRtnVal ) 
    {
        pstProperty->ulRxByte          += stTmpTrfcFlow.ulRxByte;
        pstProperty->ulTxByte          += stTmpTrfcFlow.ulTxByte;
        return BST_IP_ERR_OK;
    }
    else if( BST_ERR_ITEM_NOT_EXISTED == enRtnVal )
    {
        return BST_IP_ERR_OK;
    }
    else
    {
        BST_RLS_LOG1( "BST_IP_CSocket::GetTrafficFlow Error=u%", enRtnVal );
        return BST_IP_ERR_VAL;
    }
}

BST_IP_ERR_T BST_IP_CSocket::SetTrafficFlow( BST_IP_TRAFFIC_FLOW_STRU *pstProperty )
{
    if ( ( BST_NULL_PTR == pstProperty ) )
    {
        return BST_IP_ERR_ARG;
    }

    m_TrafficPool.ulRxByte              += pstProperty->ulRxByte;
    m_TrafficPool.ulTxByte              += pstProperty->ulTxByte;
    return BST_IP_ERR_OK;
}


BST_IP_ERR_T BST_IP_CSocket::InqFlow( BST_UINT32 *pTx, BST_UINT32 *pRx )
{
    return BST_IP_ERR_OK;
}


BST_IP_ERR_T BST_IP_CSocket::InqState( BST_VOID )    //查询的状态指的是什么？？？
{
    return BST_IP_ERR_OK;
}


BST_IP_ERR_T BST_IP_CSocket::RegCallBack( BST_IP_CNetRcver *pC_CallBack )
{
    BST_IP_CRcverMng                   *RawRcvMng;
    BST_VOID                           *pvRtnVal;

    if ( BST_NULL_PTR == pC_CallBack )
    {
        return BST_IP_ERR_ARG;
    }

    RawRcvMng                           = BST_IP_CRcverMng::GetInstance();
    BST_ASSERT_NULL_RTN ( RawRcvMng, BST_IP_ERR_ARG );
    pvRtnVal                            = RawRcvMng->Attach( &m_fd, pC_CallBack );

    if( BST_NULL_PTR == pvRtnVal )
    {
        BST_RLS_LOG( "BST_IP_CSocket::RegCallBack Attach pvRtnVal=NULL" );
        return BST_IP_ERR_VAL;
    }
    return BST_IP_ERR_OK;
}

BST_IP_ERR_T BST_IP_CSocket::SetSocketType( BST_SCKT_TYPE_ENUM_UINT32 enSocketType )
{
    BST_DBG_LOG1("BST_IP_CSocket::SetSocketType enSocketType = %d.", enSocketType);
    if( !BST_IP_IsValidSocketType( enSocketType ) )
    {
        BST_RLS_LOG("BST_IP_CSocket::SetSocketType enSocketType is invalid.");
        return BST_IP_ERR_ARG;
    }
    if( BST_IP_ERR_OK != BST_IP_CheckSocketSupport( enSocketType ) )
    {
        BST_RLS_LOG("BST_IP_CSocket::SetSocketType enSocketType is not support.");
        return BST_IP_ERR_ARG;
    }
    m_enSocketType                  = enSocketType;
    m_pFunGroup                     = BST_IP_GetSocketFunGrp( m_enSocketType );

    return BST_IP_ERR_OK;
}


BST_IP_ERR_T BST_IP_CSocket::GetSocketFd( BST_FD_T *pSocketFd )
{
    if ( BST_NULL_PTR == pSocketFd )
    {
        return BST_IP_ERR_ARG;
    }

    pSocketFd->pFd = m_fd.pFd;
    pSocketFd->lFd = m_fd.lFd;

    return BST_IP_ERR_OK;
}


BST_IP_ERR_T BST_IP_CSocket::CheckConfig( BST_VOID ) const
{
    if( !BST_IP_IsValidFunGrp( m_pFunGroup ) )
    {
        return BST_IP_ERR_ARG;
    }
    if( !BST_IP_IsValidIpAddress( &m_stIpAddress ) )
    {
        return BST_IP_ERR_ARG;
    }
    if( !BST_IP_IsValidSocketType( m_enSocketType ) )
    {
        return BST_IP_ERR_ARG;
    }
    if( BST_IP_IsBsdSocketType( m_enSocketType ) )
    {
        if( !BST_IP_IsBsdFdValid( m_fd ) )
        {
            return BST_IP_ERR_CONN;
        }
    }
    else
    {
        if( !BST_IP_IsRawFdValid( m_fd ) )
        {
            return BST_IP_ERR_CONN;
        }
    }
    return BST_IP_ERR_OK;
}


BST_IP_ERR_T BST_IP_CSocket::CheckFd ( const BST_FD_T *pfd ) const
{
    if ( BST_NULL_PTR == pfd )
    {
        return BST_IP_ERR_VAL;
    }
    if( !BST_IP_IsValidIpAddress( &m_stIpAddress ) )
    {
        return BST_IP_ERR_VAL;
    }

    if ( !BST_IP_IsValidSocketType( m_enSocketType ) )
    {
        return BST_IP_ERR_VAL;
    }
    if ( BST_IP_IsBsdSocketType( m_enSocketType ) )
    {
        if ( !BST_IP_IsBsdFdValid( m_fd ) )
        {
            return BST_IP_ERR_VAL;
        }
        if ( pfd->lFd != m_fd.lFd )
        {
            return BST_IP_ERR_VAL;
        }
    }
    else
    {
        if ( !BST_IP_IsRawFdValid( m_fd ) )
        {
            return BST_IP_ERR_VAL;
        }
        if ( pfd->pFd != m_fd.pFd )
        {
            return BST_IP_ERR_VAL;
        }
    }
    return BST_IP_ERR_OK;
}


BST_IP_PROTOCOL_TYPE BST_IP_CSocket::GetProtoType( BST_VOID )
{
    switch ( m_enSocketType )
    {
        case BST_SCKT_TYPE_BSD:
        case BST_SCKT_TYPE_SSL:
        case BST_SCKT_TYPE_RAW_TCP:
            return BST_IP_PROTOCOL_TCP;

        case BST_SCKT_TYPE_RAW_UDP:
            return BST_IP_PROTOCOL_UDP;

        default:
            return BST_IP_PROTOCOL_INVALID;
    }
}

