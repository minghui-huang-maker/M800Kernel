/******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/
#define _RTL8188FS_RECV_C_

#include <rtl8188f_hal.h>

#ifdef CONFIG_SDIO_RX_COPY
s32 rtl8188fs_recv_hdl(_adapter *padapter)
{
	PHAL_DATA_TYPE		pHalData;
	struct recv_priv		*precvpriv;
	struct recv_buf *precvbuf;
	union recv_frame		*precvframe;
	struct recv_frame_hdr	*phdr;
	struct rx_pkt_attrib	*pattrib;
	u8	*ptr;
	u32 pkt_len, pkt_offset;
	u8	rx_report_sz = 0;
	s32 ret = _SUCCESS;

	pHalData = GET_HAL_DATA(padapter);
	precvpriv = &padapter->recvpriv;

	rx_bh_tk_set_stage(precvpriv, RX_BH_STG_HDL_ENTER);

	do {
		precvbuf = rtw_dequeue_recvbuf(&precvpriv->recv_buf_pending_queue);
		if (precvbuf == NULL) {
			rx_bh_tk_set_buf(precvpriv, NULL, NULL, 0);
			break;
		}

		rx_bh_tk_set_stage(precvpriv, RX_BH_STG_NEW_BUF);
		rx_bh_tk_set_buf(precvpriv, precvbuf, precvbuf->pdata, precvbuf->ptail - precvbuf->pdata);

		ptr = precvbuf->pdata;

		while (ptr < precvbuf->ptail) {
			rx_bh_tk_set_buf_pos(precvpriv, ptr);

			precvframe = rtw_alloc_recvframe(&precvpriv->free_recv_queue);
			if (precvframe == NULL) {
				RTW_INFO("%s: no enough recv frame!\n", __FUNCTION__);
				rtw_enqueue_recvbuf_to_head(precvbuf, &precvpriv->recv_buf_pending_queue);
				ret = RTW_RFRAME_UNAVAIL;
				goto exit;
			}

			rx_bh_tk_set_stage(precvpriv, RX_BH_STG_NEW_FRAME);
			rx_bh_tk_set_frame(precvpriv, precvframe);

			/* rx desc parsing */
			rtl8188f_query_rx_desc_status(precvframe, ptr);

			pattrib = &precvframe->u.hdr.attrib;

			/* fix Hardware RX data error, drop whole recv_buffer */
			if (!rtw_hal_rcr_check(padapter, RCR_ACRC32) && pattrib->crc_err) {
#if !(MP_DRIVER == 1)
				RTW_INFO("%s()-%d: RX Warning! rx CRC ERROR !!\n", __FUNCTION__, __LINE__);
#endif
				rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
				rx_bh_tk_set_frame(precvpriv, NULL);
				break;
			}

			rx_report_sz = RXDESC_SIZE + pattrib->drvinfo_sz;
			pkt_offset = rx_report_sz + pattrib->shift_sz + pattrib->pkt_len;

			if ((ptr + pkt_offset) > precvbuf->ptail) {
				RTW_INFO("%s()-%d: : next pkt len(%p,%d) exceed ptail(%p)!\n", __FUNCTION__, __LINE__, ptr, pkt_offset, precvbuf->ptail);
				rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
				rx_bh_tk_set_frame(precvpriv, NULL);
				break;
			}

			if ((pattrib->crc_err) || (pattrib->icv_err)) {
#ifdef CONFIG_MP_INCLUDED
				if (padapter->registrypriv.mp_mode == 1) {
					if ((check_fwstate(&padapter->mlmepriv, WIFI_MP_STATE) == _TRUE)) { /* &&(padapter->mppriv.check_mp_pkt == 0)) */
						if (pattrib->crc_err == 1)
							padapter->mppriv.rx_crcerrpktcount++;
					}
				} else
#endif
				{
					RTW_INFO("%s: crc_err=%d icv_err=%d, skip!\n", __FUNCTION__, pattrib->crc_err, pattrib->icv_err);
				}
				rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
				rx_bh_tk_set_frame(precvpriv, NULL);
			} else {
#ifdef CONFIG_RX_PACKET_APPEND_FCS
				if (check_fwstate(&padapter->mlmepriv, WIFI_MONITOR_STATE) == _FALSE)
					if ((pattrib->pkt_rpt_type == NORMAL_RX) && rtw_hal_rcr_check(padapter, RCR_APPFCS))
						pattrib->pkt_len -= IEEE80211_FCS_LEN;
#endif

				if (rtw_os_alloc_recvframe(padapter, precvframe,
					(ptr + rx_report_sz + pattrib->shift_sz), precvbuf->pskb) == _FAIL) {
					rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
					rx_bh_tk_set_frame(precvpriv, NULL);
					break;
				}
				recvframe_put(precvframe, pattrib->pkt_len);
				/*recvframe_pull(precvframe, drvinfo_sz + RXDESC_SIZE); */

				/* move to drv info position */
				ptr += RXDESC_SIZE;

				/* update drv info */
				if (rtw_hal_rcr_check(padapter, RCR_APP_BA_SSN)) {
					/* rtl8188s_update_bassn(padapter, pdrvinfo); */
					ptr += 4;
				}

				if (pattrib->pkt_rpt_type == NORMAL_RX) {
					rx_bh_tk_set_stage(precvpriv, RX_BH_STG_NORMAL_RX);

					/* skip the rx packet with abnormal length */
					if (pattrib->pkt_len < 14 || pattrib->pkt_len > 8192) {
						RTW_INFO("skip abnormal rx packet(%d)\n", pattrib->pkt_len);
						rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
						rx_bh_tk_set_stage(precvpriv, RX_BH_STG_NORMAL_RX_END);
						rx_bh_tk_set_frame(precvpriv, NULL);
						break;
					}

					pre_recv_entry(precvframe, pattrib->physt ? ptr : NULL);

					rx_bh_tk_set_stage(precvpriv, RX_BH_STG_NORMAL_RX_END);
					rx_bh_tk_set_frame(precvpriv, NULL);

				} else {
#ifdef CONFIG_FW_C2H_PKT
					if (pattrib->pkt_rpt_type == C2H_PACKET) {
						rx_bh_tk_set_stage(precvpriv, RX_BH_STG_C2H);
						rtw_hal_c2h_pkt_pre_hdl(padapter, precvframe->u.hdr.rx_data, pattrib->pkt_len);
						rx_bh_tk_set_stage(precvpriv, RX_BH_STG_C2H_END);
					} else {
						RTW_INFO("%s: [WARNNING] RX type(%d) not be handled!\n",
							__FUNCTION__, pattrib->pkt_rpt_type);
					}
#endif
					rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
					rx_bh_tk_set_frame(precvpriv, NULL);
				}
			}

			pkt_offset = _RND8(pkt_offset);
			precvbuf->pdata += pkt_offset;
			ptr = precvbuf->pdata;
			precvframe = NULL;
		}

		rtw_enqueue_recvbuf(precvbuf, &precvpriv->free_recv_buf_queue);
	} while (1);

exit:
	rx_bh_tk_set_stage(precvpriv, RX_BH_STG_HDL_EXIT);
	return ret;
}

static void rtl8188fs_recv_tasklet(void *priv)
{
	_adapter *adapter = (_adapter *)priv;
	s32 ret;

	ret = rtl8188fs_recv_hdl(adapter);
	if (ret == RTW_RFRAME_UNAVAIL
		|| ret == RTW_RFRAME_PKT_UNAVAIL
	) {
		/* schedule again and hope recvframe/packet is available next time. */
		#ifdef PLATFORM_LINUX
		tasklet_schedule(&adapter->recvpriv.recv_tasklet);
		#endif
	}
}
#else
static void rtl8188fs_recv_tasklet(void *priv)
{
	PADAPTER				padapter;
	PHAL_DATA_TYPE			pHalData;
	struct recv_priv		*precvpriv;
	struct recv_buf		*precvbuf;
	union recv_frame		*precvframe;
	struct recv_frame_hdr	*phdr;
	struct rx_pkt_attrib	*pattrib;
	u8		*ptr;
	_pkt		*ppkt;
	u32		pkt_offset;


	padapter = (PADAPTER)priv;
	pHalData = GET_HAL_DATA(padapter);
	precvpriv = &padapter->recvpriv;

	do {
		precvbuf = rtw_dequeue_recvbuf(&precvpriv->recv_buf_pending_queue);
		if (NULL == precvbuf)
			break;

		ptr = precvbuf->pdata;

		while (ptr < precvbuf->ptail) {
			precvframe = rtw_alloc_recvframe(&precvpriv->free_recv_queue);
			if (precvframe == NULL) {
				rtw_enqueue_recvbuf_to_head(precvbuf, &precvpriv->recv_buf_pending_queue);

				/* The case of can't allocte recvframe should be temporary, */
				/* schedule again and hope recvframe is available next time. */
#ifdef PLATFORM_LINUX
				tasklet_schedule(&precvpriv->recv_tasklet);
#endif
				return;
			}

			phdr = &precvframe->u.hdr;
			pattrib = &phdr->attrib;

			rtl8188f_query_rx_desc_status(precvframe, ptr);

#if 0
			{
				int i, len = 64;
				u8 *pptr = ptr;

				if ((*(pptr + RXDESC_SIZE + pattrib->drvinfo_sz) != 0x80) && (*(pptr + RXDESC_SIZE + pattrib->drvinfo_sz) != 0x40)) {
					RTW_INFO("##############RxDESC###############\n");
					for (i = 0; i < 32; i = i + 16)
						RTW_INFO("%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:\n", *(pptr + i),
							*(pptr + i + 1), *(pptr + i + 2) , *(pptr + i + 3) , *(pptr + i + 4), *(pptr + i + 5), *(pptr + i + 6), *(pptr + i + 7), *(pptr + i + 8),
							*(pptr + i + 9), *(pptr + i + 10),
							*(pptr + i + 11), *(pptr + i + 12), *(pptr + i + 13), *(pptr + i + 14), *(pptr + i + 15));

					if (pattrib->pkt_len < 100)
						len = pattrib->pkt_len;
					pptr = ptr + RXDESC_SIZE + pattrib->drvinfo_sz;
					RTW_INFO("##############Len=%d###############\n", pattrib->pkt_len);
					for (i = 0; i < len; i = i + 16)
						RTW_INFO("%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:\n", *(pptr + i),
							*(pptr + i + 1), *(pptr + i + 2) , *(pptr + i + 3) , *(pptr + i + 4), *(pptr + i + 5), *(pptr + i + 6), *(pptr + i + 7), *(pptr + i + 8),
							*(pptr + i + 9), *(pptr + i + 10),
							*(pptr + i + 11), *(pptr + i + 12), *(pptr + i + 13), *(pptr + i + 14), *(pptr + i + 15));
					RTW_INFO("#############################\n");
				}
			}
#endif

			/* fix Hardware RX data error, drop whole recv_buffer */
			if (!rtw_hal_rcr_check(padapter, RCR_ACRC32) && pattrib->crc_err) {
				RTW_INFO("%s()-%d: RX Warning! rx CRC ERROR !!\n", __FUNCTION__, __LINE__);
				rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
				break;
			}

			pkt_offset = RXDESC_SIZE + pattrib->drvinfo_sz + pattrib->pkt_len;
#if 0 /* reduce check to speed up */
			if ((ptr + pkt_offset) > precvbuf->ptail) {
				rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
				break;
			}
#endif

			if ((pattrib->crc_err) || (pattrib->icv_err)) {
#ifdef CONFIG_MP_INCLUDED
				if (padapter->registrypriv.mp_mode == 1) {
					if ((check_fwstate(&padapter->mlmepriv, WIFI_MP_STATE) == _TRUE)) { /* &&(padapter->mppriv.check_mp_pkt == 0)) */
						if (pattrib->crc_err == 1)
							padapter->mppriv.rx_crcerrpktcount++;
					}
				} else
#endif
				{
					RTW_INFO("%s: crc_err=%d icv_err=%d, skip!\n", __FUNCTION__, pattrib->crc_err, pattrib->icv_err);
				}
				rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
			} else {
				ppkt = rtw_skb_clone(precvbuf->pskb);
				if (ppkt == NULL) {
					RTW_INFO("%s: no enough memory to allocate SKB!\n", __FUNCTION__);
					rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
					rtw_enqueue_recvbuf_to_head(precvbuf, &precvpriv->recv_buf_pending_queue);

					/* The case of can't allocte skb is serious and may never be recovered, */
					/* once bDriverStopped is enable, this task should be stopped. */
					if (!rtw_is_drv_stopped(padapter)) {
#ifdef PLATFORM_LINUX
						tasklet_schedule(&precvpriv->recv_tasklet);
#endif
					}

					return;
				}

				phdr->pkt = ppkt;
				phdr->len = 0;
				phdr->rx_head = precvbuf->phead;
				phdr->rx_data = phdr->rx_tail = precvbuf->pdata;
				phdr->rx_end = precvbuf->pend;
				recvframe_put(precvframe, pkt_offset);
				recvframe_pull(precvframe, RXDESC_SIZE + pattrib->drvinfo_sz);
				skb_pull(ppkt, RXDESC_SIZE + pattrib->drvinfo_sz);

#ifdef CONFIG_RX_PACKET_APPEND_FCS
				if (check_fwstate(&padapter->mlmepriv, WIFI_MONITOR_STATE) == _FALSE) {
					if ((pattrib->pkt_rpt_type == NORMAL_RX) && rtw_hal_rcr_check(padapter, RCR_APPFCS)) {
						recvframe_pull_tail(precvframe, IEEE80211_FCS_LEN);
						pattrib->pkt_len -= IEEE80211_FCS_LEN;
						ppkt->len = pattrib->pkt_len;
					}
				}
#endif

				/* move to drv info position */
				ptr += RXDESC_SIZE;

				/* update drv info */
				if (rtw_hal_rcr_check(padapter, RCR_APP_BA_SSN)) {
					/* rtl8188s_update_bassn(padapter, pdrvinfo); */
					ptr += 4;
				}

				if (pattrib->pkt_rpt_type == NORMAL_RX)
					pre_recv_entry(precvframe, pattrib->physt ? ptr : NULL);
				else {
#ifdef CONFIG_FW_C2H_PKT
					if (pattrib->pkt_rpt_type == C2H_PACKET)
						rtw_hal_c2h_pkt_pre_hdl(padapter, precvframe->u.hdr.rx_data, pattrib->pkt_len);
					else {
						RTW_INFO("%s: [WARNNING] RX type(%d) not be handled!\n",
							__FUNCTION__, pattrib->pkt_rpt_type);
					}
#endif
					rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
				}
			}

			pkt_offset = _RND8(pkt_offset);
			precvbuf->pdata += pkt_offset;
			ptr = precvbuf->pdata;
		}

		rtw_skb_free(precvbuf->pskb);
		precvbuf->pskb = NULL;
		rtw_enqueue_recvbuf(precvbuf, &precvpriv->free_recv_buf_queue);
	} while (1);
}
#endif

#ifndef CONFIG_SDIO_RECVBUF_PWAIT_CONF_ARGS
#define CONFIG_SDIO_RECVBUF_PWAIT_CONF_ARGS RTW_PWAIT_TYPE_MSLEEP, 10, 10
#endif

/*
 * Initialize recv private variable for hardware dependent
 * 1. recv buf
 * 2. recv tasklet
 *
 */
s32 rtl8188fs_init_recv_priv(PADAPTER padapter)
{
	struct registry_priv *regsty = adapter_to_regsty(padapter);
	s32			res;
	u32			i, n;
	struct recv_priv	*precvpriv;
	struct recv_buf		*precvbuf;


	res = _SUCCESS;
	precvpriv = &padapter->recvpriv;

	/* 3 1. init recv buffer */
	_rtw_init_queue(&precvpriv->free_recv_buf_queue);
	_rtw_init_queue(&precvpriv->recv_buf_pending_queue);

	if (!is_primary_adapter(padapter))
		goto exit;

	n = regsty->recvbuf_nr * sizeof(struct recv_buf) + 4;
	precvpriv->pallocated_recv_buf = rtw_zmalloc(n);
	if (precvpriv->pallocated_recv_buf == NULL) {
		res = _FAIL;
		RTW_ERR("alloc recv_buf fail!\n");
		goto exit;
	}

	precvpriv->precv_buf = (u8 *)N_BYTE_ALIGMENT((SIZE_PTR)(precvpriv->pallocated_recv_buf), 4);

	/* init each recv buffer */
	precvbuf = (struct recv_buf *)precvpriv->precv_buf;
	for (i = 0; i < regsty->recvbuf_nr; i++) {
		res = sdio_initrecvbuf(precvbuf, padapter);
		if (res == _FAIL)
			break;

		res = rtw_os_recvbuf_resource_alloc(padapter, precvbuf, MAX_RECVBUF_SZ);
		if (res == _FAIL) {
			sdio_freerecvbuf(precvbuf);
			break;
		}

		rtw_list_insert_tail(&precvbuf->list, &precvpriv->free_recv_buf_queue.queue);

		precvbuf++;
	}
	precvpriv->free_recv_buf_queue_cnt = i;

	if (res == _FAIL)
		goto initbuferror;

#ifdef CONFIG_SDIO_RECVBUF_AGGREGATION
	precvpriv->recvbuf_agg = CONFIG_SDIO_RECVBUF_AGGREGATION_EN;
#endif

	rtw_pwctx_config(&precvpriv->recvbuf_pwait, CONFIG_SDIO_RECVBUF_PWAIT_CONF_ARGS);

	/* 3 2. init tasklet */
#ifdef PLATFORM_LINUX
	tasklet_init(&precvpriv->recv_tasklet,
		     (void(*)(unsigned long))rtl8188fs_recv_tasklet,
		     (unsigned long)padapter);
#endif

	goto exit;

initbuferror:
	precvbuf = (struct recv_buf *)precvpriv->precv_buf;
	if (precvbuf) {
		n = precvpriv->free_recv_buf_queue_cnt;
		precvpriv->free_recv_buf_queue_cnt = 0;
		for (i = 0; i < n ; i++) {
			rtw_list_delete(&precvbuf->list);
			rtw_os_recvbuf_resource_free(padapter, precvbuf);
			sdio_freerecvbuf(precvbuf);
			precvbuf++;
		}
		precvpriv->precv_buf = NULL;
	}

	if (precvpriv->pallocated_recv_buf) {
		n = regsty->recvbuf_nr * sizeof(struct recv_buf) + 4;
		rtw_mfree(precvpriv->pallocated_recv_buf, n);
		precvpriv->pallocated_recv_buf = NULL;
	}

exit:
	return res;
}

/*
 * Free recv private variable of hardware dependent
 * 1. recv buf
 * 2. recv tasklet
 *
 */
void rtl8188fs_free_recv_priv(PADAPTER padapter)
{
	struct registry_priv *regsty;
	u32			i, n;
	struct recv_priv	*precvpriv;
	struct recv_buf		*precvbuf;

	if (!is_primary_adapter(padapter))
		return;

	regsty = adapter_to_regsty(padapter);
	precvpriv = &padapter->recvpriv;

	/* 3 1. kill tasklet */
#ifdef PLATFORM_LINUX
	tasklet_kill(&precvpriv->recv_tasklet);
#endif

#ifdef CONFIG_SDIO_RECVBUF_PWAIT_RUNTIME_ADJUST
	do {
		precvbuf = rtw_dequeue_recvbuf(&precvpriv->free_recv_buf_queue);
		if (!precvbuf)
			break;

		if (precvbuf->type == RBUF_TYPE_PWAIT_ADJ) {
			sdio_freerecvbuf(precvbuf);
			rtw_mfree(precvbuf, sizeof(*precvbuf) + sizeof(struct rtw_pwait_conf));
		}
	} while (1);
#endif

	/* 3 2. free all recv buffers */
	precvbuf = (struct recv_buf *)precvpriv->precv_buf;
	if (precvbuf) {
		n = regsty->recvbuf_nr;
		precvpriv->free_recv_buf_queue_cnt = 0;
		for (i = 0; i < n ; i++) {
			rtw_list_delete(&precvbuf->list);
			rtw_os_recvbuf_resource_free(padapter, precvbuf);
			sdio_freerecvbuf(precvbuf);
			precvbuf++;
		}
		precvpriv->precv_buf = NULL;
	}

	if (precvpriv->pallocated_recv_buf) {
		n = regsty->recvbuf_nr * sizeof(struct recv_buf) + 4;
		rtw_mfree(precvpriv->pallocated_recv_buf, n);
		precvpriv->pallocated_recv_buf = NULL;
	}
}
