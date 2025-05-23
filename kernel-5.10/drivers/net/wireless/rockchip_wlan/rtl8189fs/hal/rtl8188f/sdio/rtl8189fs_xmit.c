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
#define _RTL8188FS_XMIT_C_

#include <rtl8188f_hal.h>

static u8 rtw_sdio_wait_enough_TxOQT_space(PADAPTER padapter, u8 agg_num)
{
	u32 n = 0;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

	while (pHalData->SdioTxOQTFreeSpace < agg_num) {
		if (RTW_CANNOT_RUN(padapter)) {
			RTW_INFO("%s: bSurpriseRemoved or bDriverStopped (wait TxOQT)\n", __func__);
			return _FALSE;
		}

		HalQueryTxOQTBufferStatus8188FSdio(padapter);

		if ((++n % 60) == 0) {
			if ((n % 300) == 0) {
				RTW_INFO("%s(%d): QOT free space(%d), agg_num: %d\n",
					__func__, n, pHalData->SdioTxOQTFreeSpace, agg_num);
			}
			rtw_msleep_os(1);
			/* yield(); */
		}
	}

	pHalData->SdioTxOQTFreeSpace -= agg_num;

	/* if (n > 1) */
	/*	++priv->pshare->nr_out_of_txoqt_space; */

	return _TRUE;
}

s32 _dequeue_writeport(PADAPTER padapter)
{
#if defined(DBG_TX_FREE_PAGE) || defined(CONFIG_SDIO_TX_ENABLE_AVAL_INT)
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(padapter);
#endif
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;
	struct xmit_priv *pxmitpriv = &padapter->xmitpriv;
	struct dvobj_priv	*pdvobjpriv = adapter_to_dvobj(padapter);
	struct xmit_buf *pxmitbuf;
	u8	PageIdx = 0;
	u32	deviceId;
#ifdef CONFIG_SDIO_TX_ENABLE_AVAL_INT
	u8	bUpdatePageNum = _FALSE;
#else
	u32	polling_num = 0;
#endif

	pxmitbuf = select_and_dequeue_pending_xmitbuf(padapter);

	if (pxmitbuf == NULL)
		return _TRUE;

	deviceId = ffaddr2deviceId(pdvobjpriv, pxmitbuf->ff_hwaddr);

	/* translate fifo addr to queue index */
	switch (deviceId) {
	case WLAN_TX_HIQ_DEVICE_ID:
		PageIdx = HI_QUEUE_IDX;
		break;

	case WLAN_TX_MIQ_DEVICE_ID:
		PageIdx = MID_QUEUE_IDX;
		break;

	case WLAN_TX_LOQ_DEVICE_ID:
		PageIdx = LOW_QUEUE_IDX;
		break;
	}

query_free_page:
	/* check if hardware tx fifo page is enough */
	if (_FALSE == rtw_hal_sdio_query_tx_freepage(padapter, PageIdx, pxmitbuf->pg_num)) {
		if (RTW_CANNOT_RUN(padapter)) {
			RTW_INFO("%s: bDriverStopped(%d) bSurpriseRemoved(%d)!\n"
				, __func__
				, rtw_is_drv_stopped(padapter)
				, rtw_is_surprise_removed(padapter));
			goto free_xmitbuf;
		}
#ifdef CONFIG_SDIO_TX_ENABLE_AVAL_INT
		if (!bUpdatePageNum) {
			rtw_hal_sdio_avail_page_threshold_en(padapter, PageIdx, pxmitbuf->pg_num);

			/* Total number of page is NOT available, so update current FIFO status */
			HalQueryTxBufferStatus8188FSdio(padapter);
			bUpdatePageNum = _TRUE;
			goto query_free_page;
		} else {
			bUpdatePageNum = _FALSE;
			enqueue_pending_xmitbuf_to_head(pxmitpriv, pxmitbuf);
			#ifdef DBG_TX_FREE_PAGE
			RTW_INFO("DQWP page not enough, len:%d, agg:%d, req %s:%u, cur H:%u, M:%u, L:%u, P:%u\n"
				, pxmitbuf->len, pxmitbuf->agg_num
				, sdio_tx_queue_str(PageIdx), pxmitbuf->pg_num
				, hal_data->SdioTxFIFOFreePage[HI_QUEUE_IDX]
				, hal_data->SdioTxFIFOFreePage[MID_QUEUE_IDX]
				, hal_data->SdioTxFIFOFreePage[LOW_QUEUE_IDX]
				, hal_data->SdioTxFIFOFreePage[PUBLIC_QUEUE_IDX]
			);
			#endif
			return _TRUE;
		}
#else /* CONFIG_SDIO_TX_ENABLE_AVAL_INT */
		polling_num++;
		if ((polling_num % 10) == 0) {
			enqueue_pending_xmitbuf_to_head(pxmitpriv, pxmitbuf);
			#ifdef DBG_TX_FREE_PAGE
			RTW_INFO("DQWP FIFO starvation!(%d), len:%u, agg:%u, req %s:%u, cur H:%u, M:%u, L:%u, P:%u\n"
				, polling_num, pxmitbuf->len, pxmitbuf->agg_num
				, sdio_tx_queue_str(PageIdx), pxmitbuf->pg_num
				, hal_data->SdioTxFIFOFreePage[HI_QUEUE_IDX]
				, hal_data->SdioTxFIFOFreePage[MID_QUEUE_IDX]
				, hal_data->SdioTxFIFOFreePage[LOW_QUEUE_IDX]
				, hal_data->SdioTxFIFOFreePage[PUBLIC_QUEUE_IDX]
			);
			#endif
			rtw_usleep_os(50);
			return _FALSE;
		}

		/* Total number of page is NOT available, so update current FIFO status */
		HalQueryTxBufferStatus8188FSdio(padapter);
		goto query_free_page;
#endif /* CONFIG_SDIO_TX_ENABLE_AVAL_INT */
	}

	if (rtw_sdio_wait_enough_TxOQT_space(padapter, pxmitbuf->agg_num) == _FALSE)
		goto free_xmitbuf;

#ifdef CONFIG_CHECK_LEAVE_LPS
	#ifdef CONFIG_LPS_CHK_BY_TP
	if (!adapter_to_pwrctl(padapter)->lps_chk_by_tp)
	#endif
		traffic_check_for_leave_lps(padapter, _TRUE, pxmitbuf->agg_num);
#endif

	#ifdef DBG_TX_FREE_PAGE
	RTW_INFO("DQWP write, %s:%u\n", sdio_tx_queue_str(PageIdx), pxmitbuf->pg_num);
	#endif

	rtw_write_port(padapter, deviceId, pxmitbuf->len, (u8 *)pxmitbuf);

	rtw_hal_sdio_update_tx_freepage(padapter, PageIdx, pxmitbuf->pg_num);
	#ifdef DBG_TX_FREE_PAGE
	RTW_INFO("DQWP write done, cur H:%u, M:%u, L:%u, P:%u\n"
		, hal_data->SdioTxFIFOFreePage[HI_QUEUE_IDX]
		, hal_data->SdioTxFIFOFreePage[MID_QUEUE_IDX]
		, hal_data->SdioTxFIFOFreePage[LOW_QUEUE_IDX]
		, hal_data->SdioTxFIFOFreePage[PUBLIC_QUEUE_IDX]
	);
	#endif

free_xmitbuf:
	/* rtw_free_xmitframe(pxmitpriv, pframe); */
	/* pxmitbuf->priv_data = NULL; */
	rtw_free_xmitbuf(pxmitpriv, pxmitbuf);
	#ifdef SDIO_FREE_XMIT_BUF_SEMA
	if (pxmitbuf->buf_tag == XMITBUF_DATA)
		rtw_sdio_free_xmitbuf_sema_up(pxmitpriv);
	#endif

#if 0 /* improve TX/RX throughput balance */
	{
		PSDIO_DATA psdio;
		struct sdio_func *func;
		static u8 i = 0;
		u32 sdio_hisr;
		u8 j;

		psdio = &adapter_to_dvobj(padapter)->intf_data;
		func = psdio->func;

		if (i == 2) {
			j = 0;
			while (j < 10) {
				sdio_hisr = SdioLocalCmd52Read1Byte(padapter, SDIO_REG_HISR);
				sdio_hisr &= GET_HAL_DATA(padapter)->sdio_himr;
				if (sdio_hisr & SDIO_HISR_RX_REQUEST) {
					sdio_claim_host(func);
					sd_int_hdl(GET_PRIMARY_ADAPTER(padapter));
					sdio_release_host(func);
				} else
					break;
				j++;
			}
			i = 0;
		} else
			i++;
	}
#endif

#ifdef CONFIG_SDIO_TX_TASKLET
	tasklet_hi_schedule(&pxmitpriv->xmit_tasklet);
#endif

	return _FALSE;
}

/*
 * Description
 *	Transmit xmitbuf to hardware tx fifo
 *
 * Return
 *	_SUCCESS	ok
 *	_FAIL		something error
 */
s32 rtl8188fs_xmit_buf_handler(PADAPTER padapter)
{
	struct xmit_priv *pxmitpriv;
	u8	queue_empty;
	s32	ret;

	pxmitpriv = &padapter->xmitpriv;

	ret = _rtw_down_sema(&pxmitpriv->xmit_sema);
	if (_FAIL == ret) {
		RTW_ERR("%s: down SdioXmitBufSema fail!\n", __FUNCTION__);
		return _FAIL;
	}

	if (RTW_CANNOT_RUN(padapter)) {
		RTW_DBG(FUNC_ADPT_FMT "- bDriverStopped(%s) bSurpriseRemoved(%s)\n",
			FUNC_ADPT_ARG(padapter),
			rtw_is_drv_stopped(padapter) ? "True" : "False",
			rtw_is_surprise_removed(padapter) ? "True" : "False");
		return _FAIL;
	}

	if (rtw_mi_check_pending_xmitbuf(padapter) == 0)
		return _SUCCESS;

#ifdef CONFIG_LPS_LCLK
	ret = rtw_register_tx_alive(padapter);
	if (ret != _SUCCESS)
		return _SUCCESS;
#endif

	do {
		queue_empty = rtw_mi_dequeue_writeport(padapter);
	} while (!queue_empty);

#ifdef CONFIG_LPS_LCLK
	rtw_unregister_tx_alive(padapter);
#endif

	return _SUCCESS;
}

/*
 * Description:
 *	Aggregation packets and send to hardware
 *
 * Return:
 *	0	Success
 *	-1	Hardware resource(TX FIFO) not ready
 *	-2	Software resource(xmitbuf) not ready
 */
static s32 xmit_xmitframes(PADAPTER padapter, struct xmit_priv *pxmitpriv)
{
	s32 err, ret;
	u32 k = 0;
	struct hw_xmit *hwxmits, *phwxmit;
	u8 no_res, idx, hwentry;
	_irqL irql;
	struct tx_servq *ptxservq;
	_list *sta_plist, *sta_phead, *frame_plist, *frame_phead;
	struct xmit_frame *pxmitframe;
	_queue *pframe_queue;
	struct xmit_buf *pxmitbuf;
	u32 txlen, max_xmit_len, page_size;
	u8 txdesc_size = TXDESC_SIZE;
	int inx[4];
	u8 pre_qsel = 0xFF, next_qsel = 0xFF;
	u8 single_sta_in_queue = _FALSE;
#ifdef SDIO_FREE_XMIT_BUF_SEMA
	u32 consume;
#endif

	err = 0;
	no_res = _FALSE;
	hwxmits = pxmitpriv->hwxmits;
	hwentry = pxmitpriv->hwxmit_entry;
	ptxservq = NULL;
	pxmitframe = NULL;
	pframe_queue = NULL;
	pxmitbuf = NULL;

	rtw_hal_get_def_var(padapter, HAL_DEF_TX_PAGE_SIZE, &page_size);

	if (padapter->registrypriv.wifi_spec == 1) {
		for (idx = 0; idx < 4; idx++)
			inx[idx] = pxmitpriv->wmm_para_seq[idx];
	} else {
		inx[0] = 0;
		inx[1] = 1;
		inx[2] = 2;
		inx[3] = 3;
	}

	/* 0(VO), 1(VI), 2(BE), 3(BK) */
	for (idx = 0; idx < hwentry; idx++) {
		phwxmit = hwxmits + inx[idx];
		#ifdef SDIO_FREE_XMIT_BUF_SEMA
		consume = 0;
		#endif

		if ((check_pending_xmitbuf(pxmitpriv) == _TRUE) && (padapter->mlmepriv.LinkDetectInfo.bHigherBusyTxTraffic == _TRUE)) {
			if ((phwxmit->accnt > 0) && (phwxmit->accnt < 5)) {
				err = RTW_TX_WAIT_MORE_FRAME;
				break;
			}
		}

		max_xmit_len = rtw_hal_get_sdio_tx_max_length(padapter, inx[idx]);

		_enter_critical_bh(&pxmitpriv->lock, &irql);

		sta_phead = get_list_head(phwxmit->sta_queue);
		sta_plist = get_next(sta_phead);
		/* because stop_sta_xmit may delete sta_plist at any time */
		/* so we should add lock here, or while loop can not exit */

		single_sta_in_queue = rtw_end_of_queue_search(sta_phead, get_next(sta_plist));

		while (rtw_end_of_queue_search(sta_phead, sta_plist) == _FALSE) {
			ptxservq = LIST_CONTAINOR(sta_plist, struct tx_servq, tx_pending);
			sta_plist = get_next(sta_plist);

#ifdef DBG_XMIT_BUF
			RTW_INFO("%s idx:%d hwxmit_pkt_num:%d ptxservq_pkt_num:%d\n", __func__, idx, phwxmit->accnt, ptxservq->qcnt);
			RTW_INFO("%s free_xmit_extbuf_cnt=%d free_xmitbuf_cnt=%d free_xmitframe_cnt=%d\n",
				__func__, pxmitpriv->free_xmit_extbuf_cnt, pxmitpriv->free_xmitbuf_cnt,
				 pxmitpriv->free_xmitframe_cnt);
#endif
			pframe_queue = &ptxservq->sta_pending;

			frame_phead = get_list_head(pframe_queue);

			while (rtw_is_list_empty(frame_phead) == _FALSE) {
				frame_plist = get_next(frame_phead);
				pxmitframe = LIST_CONTAINOR(frame_plist, struct xmit_frame, list);

				/* check xmit_buf size enough or not */
				txlen = txdesc_size + rtw_wlan_pkt_size(pxmitframe);
				next_qsel = pxmitframe->attrib.qsel;
				if ((NULL == pxmitbuf)
					|| (pxmitbuf->pg_num + PageNum(txlen, page_size) > PageNum(max_xmit_len, page_size))
					|| (k >= (rtw_hal_sdio_max_txoqt_free_space(padapter) - 1))
					|| ((k != 0) && (_FAIL == rtw_hal_busagg_qsel_check(padapter, pre_qsel, next_qsel)))
				) {
					if (pxmitbuf) {
						/* pxmitbuf->priv_data will be NULL, and will crash here */
						if (pxmitbuf->len > 0 && pxmitbuf->priv_data) {
							struct xmit_frame *pframe;
							pframe = (struct xmit_frame *)pxmitbuf->priv_data;
							pframe->agg_num = k;
							pxmitbuf->agg_num = k;
							rtl8188f_update_txdesc(pframe, pframe->buf_addr);
							rtw_free_xmitframe(pxmitpriv, pframe);
							pxmitbuf->priv_data = NULL;
							enqueue_pending_xmitbuf(pxmitpriv, pxmitbuf);

							/* can not yield under lock */
							/* rtw_yield_os(); */
							if (single_sta_in_queue == _FALSE) {
								/* break the loop in case there is more than one sta in this ac queue */
								pxmitbuf = NULL;
								err = RTW_TX_BALANCE;
								break;
							}

						} else {
							rtw_free_xmitbuf(pxmitpriv, pxmitbuf);
							#ifdef SDIO_FREE_XMIT_BUF_SEMA
							consume--;
							#endif
						}
					}

					pxmitbuf = rtw_alloc_xmitbuf(pxmitpriv);
					if (pxmitbuf == NULL) {
#ifdef DBG_XMIT_BUF
						RTW_ERR("%s: xmit_buf is not enough!\n", __FUNCTION__);
#endif
						err = RTW_XBUF_UNAVAIL;
						break;
					}
					#ifdef SDIO_FREE_XMIT_BUF_SEMA
					consume++;
					#endif
					k = 0;
				}

				/* ok to send, remove frame from queue */
#ifdef CONFIG_AP_MODE
				if (MLME_IS_AP(padapter) || MLME_IS_MESH(padapter)) {
					if ((pxmitframe->attrib.psta->state & WIFI_SLEEP_STATE) &&
					    (pxmitframe->attrib.triggered == 0)) {
						RTW_INFO("%s: one not triggered pkt in queue when this STA sleep,"
							" break and goto next sta\n", __func__);
						break;
					}
				}
#endif
				rtw_list_delete(&pxmitframe->list);
				ptxservq->qcnt--;
				phwxmit->accnt--;

				if (k == 0) {
					pxmitbuf->ff_hwaddr = rtw_get_ff_hwaddr(pxmitframe);
					pxmitbuf->priv_data = (u8 *)pxmitframe;
				}

				/* coalesce the xmitframe to xmitbuf */
				pxmitframe->pxmitbuf = pxmitbuf;
				pxmitframe->buf_addr = pxmitbuf->ptail;

				ret = rtw_xmitframe_coalesce(padapter, pxmitframe->pkt, pxmitframe);
				if (ret == _FAIL) {
					RTW_ERR("%s: coalesce FAIL!", __FUNCTION__);
					/* Todo: error handler */
				} else {
					k++;
					if (k != 1)
						rtl8188f_update_txdesc(pxmitframe, pxmitframe->buf_addr);
					rtw_count_tx_stats(padapter, pxmitframe, pxmitframe->attrib.last_txcmdsz);
					pre_qsel = pxmitframe->attrib.qsel;
					txlen = txdesc_size + pxmitframe->attrib.last_txcmdsz;
					pxmitframe->pg_num = (txlen + 127) / 128;
					pxmitbuf->pg_num += (txlen + 127) / 128;
					/* if (k != 1) */
					/*	((struct xmit_frame*)pxmitbuf->priv_data)->pg_num += pxmitframe->pg_num; */
					pxmitbuf->ptail += _RND(txlen, 8); /* round to 8 bytes alignment */
					pxmitbuf->len = _RND(pxmitbuf->len, 8) + txlen;
				}

				if (k != 1)
					rtw_free_xmitframe(pxmitpriv, pxmitframe);
				pxmitframe = NULL;
			}

			if (_rtw_queue_empty(pframe_queue) == _TRUE)
				rtw_list_delete(&ptxservq->tx_pending);
			else if (err == RTW_TX_BALANCE) {
				/* Re-arrange the order of stations in this ac queue to balance the service for these stations */
				rtw_list_delete(&ptxservq->tx_pending);
				rtw_list_insert_tail(&ptxservq->tx_pending, get_list_head(phwxmit->sta_queue));
			}

			if (err)
				break;
		}
		_exit_critical_bh(&pxmitpriv->lock, &irql);

		#ifdef SDIO_FREE_XMIT_BUF_SEMA
		#ifdef DBG_SDIO_FREE_XMIT_BUF_SEMA
		if (consume)
			RTW_INFO(FUNC_ADPT_FMT" acq[%u], consume:%u\n", FUNC_ADPT_ARG(padapter), inx[idx], consume);
		#endif
		while (consume--)
			rtw_sdio_free_xmitbuf_sema_down(pxmitpriv);
		#endif

		/* dump xmit_buf to hw tx fifo */
		if (pxmitbuf) {

			if (pxmitbuf->len > 0) {
				struct xmit_frame *pframe;
				pframe = (struct xmit_frame *)pxmitbuf->priv_data;
				pframe->agg_num = k;
				pxmitbuf->agg_num = k;
				rtl8188f_update_txdesc(pframe, pframe->buf_addr);
				rtw_free_xmitframe(pxmitpriv, pframe);
				pxmitbuf->priv_data = NULL;
				enqueue_pending_xmitbuf(pxmitpriv, pxmitbuf);
				rtw_yield_os();
			} else {
				rtw_free_xmitbuf(pxmitpriv, pxmitbuf);
				#ifdef SDIO_FREE_XMIT_BUF_SEMA
				rtw_sdio_free_xmitbuf_sema_up(pxmitpriv);
				#endif
			}
			pxmitbuf = NULL;
		}

		if (err == RTW_XBUF_UNAVAIL)
			break;
	}

	return err;
}

/*
 * Description
 *	Transmit xmitframe from queue
 *
 * Return
 *	_SUCCESS	ok
 *	_FAIL		something error
 */
s32 rtl8188fs_xmit_handler(PADAPTER padapter)
{
	struct xmit_priv *pxmitpriv;
	s32 ret;
	_irqL irql;


	pxmitpriv = &padapter->xmitpriv;

	ret = _rtw_down_sema(&pxmitpriv->SdioXmitSema);
	if (_FAIL == ret) {
		RTW_ERR("%s: down sema fail!\n", __FUNCTION__);
		return _FAIL;
	}

next:
	if (RTW_CANNOT_RUN(padapter)) {
		RTW_DBG(FUNC_ADPT_FMT "- bDriverStopped(%s) bSurpriseRemoved(%s)\n",
			FUNC_ADPT_ARG(padapter),
			rtw_is_drv_stopped(padapter) ? "True" : "False",
			rtw_is_surprise_removed(padapter) ? "True" : "False");
		return _FAIL;
	}

	_enter_critical_bh(&pxmitpriv->lock, &irql);
	ret = rtw_txframes_pending(padapter);
	_exit_critical_bh(&pxmitpriv->lock, &irql);
	if (ret == 0)
		return _SUCCESS;

	/* dequeue frame and write to hardware */

	ret = xmit_xmitframes(padapter, pxmitpriv);
	if (ret == RTW_XBUF_UNAVAIL
		|| ret == RTW_TX_WAIT_MORE_FRAME
	) {
		#ifdef SDIO_FREE_XMIT_BUF_SEMA
		if (ret == RTW_XBUF_UNAVAIL) {
			rtw_sdio_free_xmitbuf_sema_down(pxmitpriv);
			rtw_sdio_free_xmitbuf_sema_up(pxmitpriv);
			goto next;
		}
		#endif

		if (padapter->registrypriv.wifi_spec)
			rtw_msleep_os(1);
		else {
			#ifdef CONFIG_REDUCE_TX_CPU_LOADING
			rtw_msleep_os(1);
			#else
			rtw_yield_os();
			#endif
		}
		goto next;
	}

	_enter_critical_bh(&pxmitpriv->lock, &irql);
	ret = rtw_txframes_pending(padapter);
	_exit_critical_bh(&pxmitpriv->lock, &irql);
	if (ret == 1) {
#ifdef CONFIG_REDUCE_TX_CPU_LOADING
		rtw_msleep_os(1);
#endif
		goto next;
	}

	return _SUCCESS;
}

thread_return rtl8188fs_xmit_thread(thread_context context)
{
	s32 ret;
	PADAPTER padapter;
	struct xmit_priv *pxmitpriv;
	u8 thread_name[20] = {0};


	ret = _SUCCESS;
	padapter = (PADAPTER)context;
	pxmitpriv = &padapter->xmitpriv;

	rtw_sprintf(thread_name, 20, "RTWHALXT-"ADPT_FMT, ADPT_ARG(padapter));
	thread_enter(thread_name);

	RTW_INFO("start "FUNC_ADPT_FMT"\n", FUNC_ADPT_ARG(padapter));

	do {
		ret = rtl8188fs_xmit_handler(padapter);
		flush_signals_thread();
	} while (_SUCCESS == ret);

	RTW_INFO(FUNC_ADPT_FMT " Exit\n", FUNC_ADPT_ARG(padapter));

	rtw_thread_wait_stop();
	return 0;

}

s32 rtl8188fs_mgnt_xmit(PADAPTER padapter, struct xmit_frame *pmgntframe)
{
	s32 ret = _SUCCESS;
	struct pkt_attrib *pattrib;
	struct xmit_buf *pxmitbuf;
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	struct dvobj_priv	*pdvobjpriv = adapter_to_dvobj(padapter);
	u8 *pframe = (u8 *)(pmgntframe->buf_addr) + TXDESC_OFFSET;
	u8 txdesc_size = TXDESC_SIZE;


	pattrib = &pmgntframe->attrib;
	pxmitbuf = pmgntframe->pxmitbuf;

	rtl8188f_update_txdesc(pmgntframe, pmgntframe->buf_addr);

	pxmitbuf->len = txdesc_size + pattrib->last_txcmdsz;
	/* pmgntframe->pg_num = (pxmitbuf->len + 127)/128; // 128 is tx page size */
	pxmitbuf->pg_num = (pxmitbuf->len + 127) / 128; /* 128 is tx page size */
	pxmitbuf->ptail = pmgntframe->buf_addr + pxmitbuf->len;
	pxmitbuf->ff_hwaddr = rtw_get_ff_hwaddr(pmgntframe);

	rtw_count_tx_stats(padapter, pmgntframe, pattrib->last_txcmdsz);

	rtw_free_xmitframe(pxmitpriv, pmgntframe);

	pxmitbuf->priv_data = NULL;

	if (get_frame_sub_type(pframe) == WIFI_BEACON) { /* dump beacon directly */
		ret = rtw_write_port(padapter, pdvobjpriv->Queue2Pipe[pxmitbuf->ff_hwaddr], pxmitbuf->len, (u8 *)pxmitbuf);
		if (ret != _SUCCESS)
			rtw_sctx_done_err(&pxmitbuf->sctx, RTW_SCTX_DONE_WRITE_PORT_ERR);

		rtw_free_xmitbuf(pxmitpriv, pxmitbuf);
	} else
		enqueue_pending_xmitbuf(pxmitpriv, pxmitbuf);

	return ret;
}

/*
 * Description:
 *	Handle xmitframe(packet) come from rtw_xmit()
 *
 * Return:
 *	_TRUE	dump packet directly ok
 *	_FALSE	enqueue, temporary can't transmit packets to hardware
 */
s32 rtl8188fs_hal_xmit(PADAPTER padapter, struct xmit_frame *pxmitframe)
{
	struct xmit_priv *pxmitpriv;
	_irqL irql;
	s32 err;


	pxmitframe->attrib.qsel = pxmitframe->attrib.priority;
	pxmitpriv = &padapter->xmitpriv;

#ifdef CONFIG_80211N_HT
	if ((pxmitframe->frame_tag == DATA_FRAMETAG) &&
	    (pxmitframe->attrib.ether_type != 0x0806) &&
	    (pxmitframe->attrib.ether_type != 0x888e) &&
	    (pxmitframe->attrib.dhcp_pkt != 1)) {
		rtw_issue_addbareq_cmd(padapter, pxmitframe, _TRUE);
	}
#endif

	_enter_critical_bh(&pxmitpriv->lock, &irql);
	err = rtw_xmitframe_enqueue(padapter, pxmitframe);
	_exit_critical_bh(&pxmitpriv->lock, &irql);
	if (err != _SUCCESS) {
		rtw_free_xmitframe(pxmitpriv, pxmitframe);

		pxmitpriv->tx_drop++;
		return _TRUE;
	}

	_rtw_up_sema(&pxmitpriv->SdioXmitSema);

	return _FALSE;
}

s32	rtl8188fs_hal_xmitframe_enqueue(_adapter *padapter, struct xmit_frame *pxmitframe)
{
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	s32 err;

	err = rtw_xmitframe_enqueue(padapter, pxmitframe);
	if (err != _SUCCESS) {
		rtw_free_xmitframe(pxmitpriv, pxmitframe);

		pxmitpriv->tx_drop++;
	} else {
#ifdef CONFIG_SDIO_TX_TASKLET
		tasklet_hi_schedule(&pxmitpriv->xmit_tasklet);
#else
		_rtw_up_sema(&pxmitpriv->SdioXmitSema);
#endif
	}

	return err;

}

/*
 * Return
 *	_SUCCESS	start thread ok
 *	_FAIL		start thread fail
 *
 */
s32 rtl8188fs_init_xmit_priv(PADAPTER padapter)
{
	struct xmit_priv *xmitpriv = &padapter->xmitpriv;
	PHAL_DATA_TYPE phal;


	phal = GET_HAL_DATA(padapter);

	_rtw_spinlock_init(&phal->SdioTxFIFOFreePageLock);
	_rtw_init_sema(&xmitpriv->SdioXmitSema, 0);
	#ifdef SDIO_FREE_XMIT_BUF_SEMA
	_rtw_init_sema(&xmitpriv->sdio_free_xmitbuf_sema, xmitpriv->free_xmitbuf_cnt);
	#endif

	return _SUCCESS;
}

void rtl8188fs_free_xmit_priv(PADAPTER padapter)
{
	PHAL_DATA_TYPE phal;
	struct xmit_priv *pxmitpriv;
	struct xmit_buf *pxmitbuf;
	_queue *pqueue;
	_list *plist, *phead;
	_list tmplist;
	_irqL irql;


	phal = GET_HAL_DATA(padapter);
	pxmitpriv = &padapter->xmitpriv;
	pqueue = &pxmitpriv->pending_xmitbuf_queue;
	phead = get_list_head(pqueue);
	_rtw_init_listhead(&tmplist);

	_enter_critical_bh(&pqueue->lock, &irql);
	if (_rtw_queue_empty(pqueue) == _FALSE) {
		/* Insert tmplist to end of queue, and delete phead */
		/* then tmplist become head of queue. */
		rtw_list_insert_tail(&tmplist, phead);
		rtw_list_delete(phead);
	}
	_exit_critical_bh(&pqueue->lock, &irql);

	phead = &tmplist;
	while (rtw_is_list_empty(phead) == _FALSE) {
		plist = get_next(phead);
		rtw_list_delete(plist);

		pxmitbuf = LIST_CONTAINOR(plist, struct xmit_buf, list);
		rtw_free_xmitframe(pxmitpriv, (struct xmit_frame *)pxmitbuf->priv_data);
		pxmitbuf->priv_data = NULL;
		rtw_free_xmitbuf(pxmitpriv, pxmitbuf);
		#ifdef SDIO_FREE_XMIT_BUF_SEMA
		if (pxmitbuf->buf_tag == XMITBUF_DATA)
			rtw_sdio_free_xmitbuf_sema_up(pxmitpriv);
		#endif
	}

	_rtw_spinlock_free(&phal->SdioTxFIFOFreePageLock);
}
