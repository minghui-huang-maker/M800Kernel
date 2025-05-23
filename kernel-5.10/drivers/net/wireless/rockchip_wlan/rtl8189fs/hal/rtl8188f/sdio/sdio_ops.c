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
#define _SDIO_OPS_C_

#include <rtl8188f_hal.h>

/* #define SDIO_DEBUG_IO 1 */
#define CONFIG_RTW_SDIO_REG_FORCE_CMD52 0

#define WLAN_IOREG_CMD_ADDR(addr) ((WLAN_IOREG_DEVICE_ID << 13) | ((addr) & WLAN_IOREG_MSK))
#define WLAN_TXHIQ_CMD_ADDR(r4_cnt) ((WLAN_TX_HIQ_DEVICE_ID << 13) | ((r4_cnt) & WLAN_FIFO_MSK))
#define WLAN_TXMIQ_CMD_ADDR(r4_cnt) ((WLAN_TX_MIQ_DEVICE_ID << 13) | ((r4_cnt) & WLAN_FIFO_MSK))
#define WLAN_TXLOQ_CMD_ADDR(r4_cnt) ((WLAN_TX_LOQ_DEVICE_ID << 13) | ((r4_cnt) & WLAN_FIFO_MSK))
#define WLAN_TXEXQ_CMD_ADDR(r4_cnt) ((WLAN_TX_EXQ_DEVICE_ID << 13) | ((r4_cnt) & WLAN_FIFO_MSK))
#define WLAN_RX0FF_CMD_ADDR(seq) ((WLAN_RX0FF_DEVICE_ID << 13) | ((seq) & WLAN_RX0FF_MSK))

/*
 * Description:
 *	The following mapping is for SDIO host local register space.
 *
 * Creadted by Roger, 2011.01.31.
 *   */
static void HalSdioGetCmdAddr8188FSdio(
		PADAPTER			padapter,
		u8				DeviceID,
		u32				Addr,
		u32				*pCmdAddr
)
{
	switch (DeviceID) {
	case SDIO_LOCAL_DEVICE_ID:
		*pCmdAddr = ((SDIO_LOCAL_DEVICE_ID << 13) | (Addr & SDIO_LOCAL_MSK));
		break;

	case WLAN_IOREG_DEVICE_ID:
		*pCmdAddr = ((WLAN_IOREG_DEVICE_ID << 13) | (Addr & WLAN_IOREG_MSK));
		break;

	case WLAN_TX_HIQ_DEVICE_ID:
		*pCmdAddr = ((WLAN_TX_HIQ_DEVICE_ID << 13) | (Addr & WLAN_FIFO_MSK));
		break;

	case WLAN_TX_MIQ_DEVICE_ID:
		*pCmdAddr = ((WLAN_TX_MIQ_DEVICE_ID << 13) | (Addr & WLAN_FIFO_MSK));
		break;

	case WLAN_TX_LOQ_DEVICE_ID:
		*pCmdAddr = ((WLAN_TX_LOQ_DEVICE_ID << 13) | (Addr & WLAN_FIFO_MSK));
		break;

	case WLAN_RX0FF_DEVICE_ID:
		*pCmdAddr = ((WLAN_RX0FF_DEVICE_ID << 13) | (Addr & WLAN_RX0FF_MSK));
		break;

	default:
		break;
	}
}

static u8 get_deviceid(u32 addr)
{
	u8 devideId;
	u16 pseudoId;


	pseudoId = (u16)(addr >> 16);
	switch (pseudoId) {
	case 0x1025:
		devideId = SDIO_LOCAL_DEVICE_ID;
		break;

	case 0x1026:
		devideId = WLAN_IOREG_DEVICE_ID;
		break;

	/*		case 0x1027:
	 *			devideId = SDIO_FIRMWARE_FIFO;
	 *			break; */

	case 0x1031:
		devideId = WLAN_TX_HIQ_DEVICE_ID;
		break;

	case 0x1032:
		devideId = WLAN_TX_MIQ_DEVICE_ID;
		break;

	case 0x1033:
		devideId = WLAN_TX_LOQ_DEVICE_ID;
		break;

	case 0x1034:
		devideId = WLAN_RX0FF_DEVICE_ID;
		break;

	default:
		/*			devideId = (u8)((addr >> 13) & 0xF); */
		devideId = WLAN_IOREG_DEVICE_ID;
		break;
	}

	return devideId;
}

/*
 * Ref:
 *	HalSdioGetCmdAddr8188FSdio()
 */
static u32 _cvrt2ftaddr(const u32 addr, u8 *pdeviceId, u16 *poffset)
{
	u8 deviceId;
	u16 offset;
	u32 ftaddr;


	deviceId = get_deviceid(addr);
	offset = 0;

	switch (deviceId) {
	case SDIO_LOCAL_DEVICE_ID:
		offset = addr & SDIO_LOCAL_MSK;
		break;

	case WLAN_TX_HIQ_DEVICE_ID:
	case WLAN_TX_MIQ_DEVICE_ID:
	case WLAN_TX_LOQ_DEVICE_ID:
		offset = addr & WLAN_FIFO_MSK;
		break;

	case WLAN_RX0FF_DEVICE_ID:
		offset = addr & WLAN_RX0FF_MSK;
		break;

	case WLAN_IOREG_DEVICE_ID:
	default:
		deviceId = WLAN_IOREG_DEVICE_ID;
		offset = addr & WLAN_IOREG_MSK;
		break;
	}
	ftaddr = (deviceId << 13) | offset;

	if (pdeviceId)
		*pdeviceId = deviceId;
	if (poffset)
		*poffset = offset;

	return ftaddr;
}


u8 sdio_read8(struct intf_hdl *pintfhdl, u32 addr)
{
	u32 ftaddr;
	u8 device_id;
	u16 offset;
	u8 val;
	u8 sus_leave = _FALSE;

	ftaddr = _cvrt2ftaddr(addr, &device_id, &offset);

	if (device_id == WLAN_IOREG_DEVICE_ID && offset < 0x100)
		sus_leave = sdio_chk_hci_resume(pintfhdl);

	val = sd_read8(pintfhdl, ftaddr, NULL);

	if (sus_leave == _TRUE)
		sdio_chk_hci_suspend(pintfhdl);

	return val;
}

u16 sdio_read16(struct intf_hdl *pintfhdl, u32 addr)
{
	u32 ftaddr;
	u8 device_id;
	u16 offset;
	u16 val;
	u8 sus_leave = _FALSE;

	ftaddr = _cvrt2ftaddr(addr, &device_id, &offset);

	if (device_id == WLAN_IOREG_DEVICE_ID && offset < 0x100)
		sus_leave = sdio_chk_hci_resume(pintfhdl);

	val = 0;
	sd_cmd52_read(pintfhdl, ftaddr, 2, (u8 *)&val);
	val = le16_to_cpu(val);

	if (sus_leave == _TRUE)
		sdio_chk_hci_suspend(pintfhdl);

	return val;
}

u32 sdio_read32(struct intf_hdl *pintfhdl, u32 addr)
{
	PADAPTER padapter;
	u8 bMacPwrCtrlOn;
	u8 deviceId;
	u16 offset;
	u32 ftaddr;
	u8 shift;
	u32 val;
	s32 err;


	padapter = pintfhdl->padapter;
	ftaddr = _cvrt2ftaddr(addr, &deviceId, &offset);

	bMacPwrCtrlOn = _FALSE;
	rtw_hal_get_hwreg(padapter, HW_VAR_APFM_ON_MAC, &bMacPwrCtrlOn);

	if (((deviceId == WLAN_IOREG_DEVICE_ID) && (offset < 0x100))
		|| (bMacPwrCtrlOn == _FALSE)
		#ifdef CONFIG_LPS_LCLK
		|| (adapter_to_pwrctl(padapter)->bFwCurrentInPSMode == _TRUE)
		#endif
		|| CONFIG_RTW_SDIO_REG_FORCE_CMD52
	) {
		u8 sus_leave = _FALSE;

		if (deviceId == WLAN_IOREG_DEVICE_ID && offset < 0x100)
			sus_leave = sdio_chk_hci_resume(pintfhdl);

		val = 0;
		err = sd_cmd52_read(pintfhdl, ftaddr, 4, (u8 *)&val);

		if (sus_leave == _TRUE)
			sdio_chk_hci_suspend(pintfhdl);

#ifdef SDIO_DEBUG_IO
		if (!err) {
#endif
			val = le32_to_cpu(val);
			return val;
#ifdef SDIO_DEBUG_IO
		}

		RTW_ERR("%s: Mac Power off, Read FAIL(%d)! addr=0x%x\n", __func__, err, addr);
		return SDIO_ERR_VAL32;
#endif
	}

	/* 4 bytes alignment */
	shift = ftaddr & 0x3;
	if (shift == 0)
		val = sd_read32(pintfhdl, ftaddr, NULL);
	else {
		u8 *ptmpbuf;

		ptmpbuf = (u8 *)rtw_malloc(8);
		if (NULL == ptmpbuf) {
			RTW_ERR("%s: Allocate memory FAIL!(size=8) addr=0x%x\n", __func__, addr);
			return SDIO_ERR_VAL32;
		}

		ftaddr &= ~(u16)0x3;
		err = sd_read(pintfhdl, ftaddr, 8, ptmpbuf);
		if (err)
			return SDIO_ERR_VAL32;
		_rtw_memcpy(&val, ptmpbuf + shift, 4);
		val = le32_to_cpu(val);

		rtw_mfree(ptmpbuf, 8);
	}


	return val;
}

s32 sdio_readN(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *pbuf)
{
	PADAPTER padapter;
	u8 bMacPwrCtrlOn;
	u8 deviceId;
	u16 offset;
	u32 ftaddr;
	u8 shift;
	s32 err;


	padapter = pintfhdl->padapter;
	err = 0;

	ftaddr = _cvrt2ftaddr(addr, &deviceId, &offset);

	bMacPwrCtrlOn = _FALSE;
	rtw_hal_get_hwreg(padapter, HW_VAR_APFM_ON_MAC, &bMacPwrCtrlOn);
	if (((deviceId == WLAN_IOREG_DEVICE_ID) && (offset < 0x100))
		|| (bMacPwrCtrlOn == _FALSE)
		#ifdef CONFIG_LPS_LCLK
		|| (adapter_to_pwrctl(padapter)->bFwCurrentInPSMode == _TRUE)
		#endif
		|| CONFIG_RTW_SDIO_REG_FORCE_CMD52
	) {
		u8 sus_leave = _FALSE;

		if (deviceId == WLAN_IOREG_DEVICE_ID && offset < 0x100)
			sus_leave = sdio_chk_hci_resume(pintfhdl);

		err = sd_cmd52_read(pintfhdl, ftaddr, cnt, pbuf);

		if (sus_leave == _TRUE)
			sdio_chk_hci_suspend(pintfhdl);

		return err;
	}

	/* 4 bytes alignment */
	shift = ftaddr & 0x3;
	if (shift == 0)
		err = sd_read(pintfhdl, ftaddr, cnt, pbuf);
	else {
		u8 *ptmpbuf;
		u32 n;

		ftaddr &= ~(u16)0x3;
		n = cnt + shift;
		ptmpbuf = rtw_malloc(n);
		if (NULL == ptmpbuf)
			return -1;
		err = sd_read(pintfhdl, ftaddr, n, ptmpbuf);
		if (!err)
			_rtw_memcpy(pbuf, ptmpbuf + shift, cnt);
		rtw_mfree(ptmpbuf, n);
	}


	return err;
}

s32 sdio_write8(struct intf_hdl *pintfhdl, u32 addr, u8 val)
{
	u32 ftaddr;
	u8 device_id;
	u16 offset;
	s32 err;
	u8 sus_leave = _FALSE;

	ftaddr = _cvrt2ftaddr(addr, &device_id, &offset);

	if (device_id == WLAN_IOREG_DEVICE_ID && offset < 0x100)
		sus_leave = sdio_chk_hci_resume(pintfhdl);

	err = 0;
	sd_write8(pintfhdl, ftaddr, val, &err);

	if (sus_leave == _TRUE)
		sdio_chk_hci_suspend(pintfhdl);

	return err;
}

s32 sdio_write16(struct intf_hdl *pintfhdl, u32 addr, u16 val)
{
	u32 ftaddr;
	u8 device_id;
	u16 offset;
	s32 err;
	u8 sus_leave = _FALSE;

	ftaddr = _cvrt2ftaddr(addr, &device_id, &offset);

	if (device_id == WLAN_IOREG_DEVICE_ID && offset < 0x100)
		sus_leave = sdio_chk_hci_resume(pintfhdl);

	val = cpu_to_le16(val);
	err = sd_cmd52_write(pintfhdl, ftaddr, 2, (u8 *)&val);

	if (sus_leave == _TRUE)
		sdio_chk_hci_suspend(pintfhdl);

	return err;
}

s32 sdio_write32(struct intf_hdl *pintfhdl, u32 addr, u32 val)
{
	PADAPTER padapter;
	u8 bMacPwrCtrlOn;
	u8 deviceId;
	u16 offset;
	u32 ftaddr;
	u8 shift;
	s32 err;


	padapter = pintfhdl->padapter;
	err = 0;

	ftaddr = _cvrt2ftaddr(addr, &deviceId, &offset);

	bMacPwrCtrlOn = _FALSE;
	rtw_hal_get_hwreg(padapter, HW_VAR_APFM_ON_MAC, &bMacPwrCtrlOn);

	if (((deviceId == WLAN_IOREG_DEVICE_ID) && (offset < 0x100))
		|| (bMacPwrCtrlOn == _FALSE)
		#ifdef CONFIG_LPS_LCLK
		|| (adapter_to_pwrctl(padapter)->bFwCurrentInPSMode == _TRUE)
		#endif
		|| CONFIG_RTW_SDIO_REG_FORCE_CMD52
	) {
		u8 sus_leave = _FALSE;

		if (deviceId == WLAN_IOREG_DEVICE_ID && offset < 0x100)
			sus_leave = sdio_chk_hci_resume(pintfhdl);

		val = cpu_to_le32(val);
		err = sd_cmd52_write(pintfhdl, ftaddr, 4, (u8 *)&val);

		if (sus_leave == _TRUE)
			sdio_chk_hci_suspend(pintfhdl);

		return err;
	}

	/* 4 bytes alignment */
	shift = ftaddr & 0x3;
#if 1
	if (shift == 0)
		sd_write32(pintfhdl, ftaddr, val, &err);
	else {
		val = cpu_to_le32(val);
		err = sd_cmd52_write(pintfhdl, ftaddr, 4, (u8 *)&val);
	}
#else
	if (shift == 0)
		sd_write32(pintfhdl, ftaddr, val, &err);
	else {
		u8 *ptmpbuf;

		ptmpbuf = (u8 *)rtw_malloc(8);
		if (NULL == ptmpbuf)
			return -1;

		ftaddr &= ~(u16)0x3;
		err = sd_read(pintfhdl, ftaddr, 8, ptmpbuf);
		if (err) {
			rtw_mfree(ptmpbuf, 8);
			return err;
		}
		val = cpu_to_le32(val);
		_rtw_memcpy(ptmpbuf + shift, &val, 4);
		err = sd_write(pintfhdl, ftaddr, 8, ptmpbuf);

		rtw_mfree(ptmpbuf, 8);
	}
#endif


	return err;
}

s32 sdio_writeN(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *pbuf)
{
	PADAPTER padapter;
	u8 bMacPwrCtrlOn;
	u8 deviceId;
	u16 offset;
	u32 ftaddr;
	u8 shift;
	s32 err;


	padapter = pintfhdl->padapter;
	err = 0;

	ftaddr = _cvrt2ftaddr(addr, &deviceId, &offset);

	bMacPwrCtrlOn = _FALSE;
	rtw_hal_get_hwreg(padapter, HW_VAR_APFM_ON_MAC, &bMacPwrCtrlOn);
	if (((deviceId == WLAN_IOREG_DEVICE_ID) && (offset < 0x100))
		|| (bMacPwrCtrlOn == _FALSE)
		#ifdef CONFIG_LPS_LCLK
		|| (adapter_to_pwrctl(padapter)->bFwCurrentInPSMode == _TRUE)
		#endif
		|| CONFIG_RTW_SDIO_REG_FORCE_CMD52
	) {
		u8 sus_leave = _FALSE;

		if (deviceId == WLAN_IOREG_DEVICE_ID && offset < 0x100)
			sus_leave = sdio_chk_hci_resume(pintfhdl);

		err = sd_cmd52_write(pintfhdl, ftaddr, cnt, pbuf);

		if (sus_leave == _TRUE)
			sdio_chk_hci_suspend(pintfhdl);

		return err;
	}

	shift = ftaddr & 0x3;
	if (shift == 0)
		err = sd_write(pintfhdl, ftaddr, cnt, pbuf);
	else {
		u8 *ptmpbuf;
		u32 n;

		ftaddr &= ~(u16)0x3;
		n = cnt + shift;
		ptmpbuf = rtw_malloc(n);
		if (NULL == ptmpbuf)
			return -1;
		err = sd_read(pintfhdl, ftaddr, 4, ptmpbuf);
		if (err) {
			rtw_mfree(ptmpbuf, n);
			return err;
		}
		_rtw_memcpy(ptmpbuf + shift, pbuf, cnt);
		err = sd_write(pintfhdl, ftaddr, n, ptmpbuf);
		rtw_mfree(ptmpbuf, n);
	}


	return err;
}

u8 sdio_f0_read8(struct intf_hdl *pintfhdl, u32 addr)
{
	u32 ftaddr;
	u8 val;

	val = sd_f0_read8(pintfhdl, addr, NULL);


	return val;
}

void sdio_read_mem(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *rmem)
{
	s32 err;


	err = sdio_readN(pintfhdl, addr, cnt, rmem);

}

void sdio_write_mem(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *wmem)
{

	sdio_writeN(pintfhdl, addr, cnt, wmem);

}

static u32 sdio_read_port_no_align(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *mem)
{
	PADAPTER padapter;
	PHAL_DATA_TYPE phal;
	s32 err;

	padapter = pintfhdl->padapter;
	phal = GET_HAL_DATA(padapter);

	HalSdioGetCmdAddr8188FSdio(padapter, addr, phal->SdioRxFIFOCnt++, &addr);

	err = _sd_read(pintfhdl, addr, cnt, mem);
	if (err)
		return _FAIL;
	return _SUCCESS;
}

/*
 * Description:
 *	Read from RX FIFO
 *	Round read size to block size,
 *	and make sure data transfer will be done in one command.
 *
 * Parameters:
 *	pintfhdl	a pointer of intf_hdl
 *	addr		port ID
 *	cnt			size to read
 *	rmem		address to put data
 *
 * Return:
 *	_SUCCESS(1)		Success
 *	_FAIL(0)		Fail
 */
static u32 sdio_read_port(
	struct intf_hdl *pintfhdl,
	u32 addr,
	u32 cnt,
	u8 *mem)
{
	PADAPTER padapter;
	u32 oldcnt;
#ifdef SDIO_DYNAMIC_ALLOC_MEM
	u8 *oldmem;
#endif
	u32 ret;

	padapter = pintfhdl->padapter;

	oldcnt = cnt;
	cnt = rtw_sdio_cmd53_align_size(adapter_to_dvobj(padapter), cnt);

	if (oldcnt != cnt) {
#ifdef SDIO_DYNAMIC_ALLOC_MEM
		oldmem = mem;
		mem = rtw_malloc(cnt);
		if (mem == NULL) {
			RTW_WARN("%s: allocate memory %d bytes fail!\n", __func__, cnt);
			mem = oldmem;
			oldmem == NULL;
		}
#else
		/* in this case, caller should gurante the buffer is big enough */
		/* to receive data after alignment */
#endif
	}

	ret = sdio_read_port_no_align(pintfhdl, addr, cnt, mem);

#ifdef SDIO_DYNAMIC_ALLOC_MEM
	if ((oldcnt != cnt) && (oldmem)) {
		_rtw_memcpy(oldmem, mem, oldcnt);
		rtw_mfree(mem, cnt);
	}
#endif

	return ret;
}

/*
 * Description:
 *	Write to TX FIFO
 *	Align write size block size,
 *	and make sure data could be written in one command.
 *
 * Parameters:
 *	pintfhdl	a pointer of intf_hdl
 *	addr		port ID
 *	cnt			size to write
 *	wmem		data pointer to write
 *
 * Return:
 *	_SUCCESS(1)		Success
 *	_FAIL(0)		Fail
 */
static u32 sdio_write_port(
	struct intf_hdl *pintfhdl,
	u32 addr,
	u32 cnt,
	u8 *mem)
{
	PADAPTER padapter;
	s32 err;
	struct xmit_buf *xmitbuf = (struct xmit_buf *)mem;

	padapter = pintfhdl->padapter;

	if (!rtw_is_hw_init_completed(padapter)) {
		RTW_INFO("%s [addr=0x%x cnt=%d] padapter->hw_init_completed == _FALSE\n", __func__, addr, cnt);
		return _FAIL;
	}

	cnt = _RND4(cnt);
	HalSdioGetCmdAddr8188FSdio(padapter, addr, cnt >> 2, &addr);

	cnt = rtw_sdio_cmd53_align_size(adapter_to_dvobj(padapter), cnt);

	err = sd_write(pintfhdl, addr, cnt, xmitbuf->pdata);

	rtw_sctx_done_err(&xmitbuf->sctx,
		  err ? RTW_SCTX_DONE_WRITE_PORT_ERR : RTW_SCTX_DONE_SUCCESS);

	if (err)
		return _FAIL;
	return _SUCCESS;
}

/*
 * Todo: align address to 4 bytes.
 */
s32 _sdio_local_read(
	PADAPTER	padapter,
	u32			addr,
	u32			cnt,
	u8			*pbuf)
{
	struct intf_hdl *pintfhdl;
	u8 bMacPwrCtrlOn;
	s32 err;
	u8 *ptmpbuf;
	u32 n;


	pintfhdl = &padapter->iopriv.intf;

	HalSdioGetCmdAddr8188FSdio(padapter, SDIO_LOCAL_DEVICE_ID, addr, &addr);

	bMacPwrCtrlOn = _FALSE;
	rtw_hal_get_hwreg(padapter, HW_VAR_APFM_ON_MAC, &bMacPwrCtrlOn);
	if (bMacPwrCtrlOn == _FALSE
		|| CONFIG_RTW_SDIO_REG_FORCE_CMD52
	) {
		err = _sd_cmd52_read(pintfhdl, addr, cnt, pbuf);
		return err;
	}

	n = RND4(cnt);
	ptmpbuf = (u8 *)rtw_malloc(n);
	if (!ptmpbuf)
		return -1;

	err = _sd_read(pintfhdl, addr, n, ptmpbuf);
	if (!err)
		_rtw_memcpy(pbuf, ptmpbuf, cnt);

	if (ptmpbuf)
		rtw_mfree(ptmpbuf, n);

	return err;
}

/*
 * Todo: align address to 4 bytes.
 */
s32 sdio_local_read(
	PADAPTER	padapter,
	u32			addr,
	u32			cnt,
	u8			*pbuf)
{
	struct intf_hdl *pintfhdl;
	u8 bMacPwrCtrlOn;
	s32 err;
	u8 *ptmpbuf;
	u32 n;

	pintfhdl = &padapter->iopriv.intf;

	HalSdioGetCmdAddr8188FSdio(padapter, SDIO_LOCAL_DEVICE_ID, addr, &addr);

	bMacPwrCtrlOn = _FALSE;
	rtw_hal_get_hwreg(padapter, HW_VAR_APFM_ON_MAC, &bMacPwrCtrlOn);
	if ((_FALSE == bMacPwrCtrlOn)
		#ifdef CONFIG_LPS_LCLK
		|| (adapter_to_pwrctl(padapter)->bFwCurrentInPSMode == _TRUE)
		#endif
		|| CONFIG_RTW_SDIO_REG_FORCE_CMD52
	) {
		err = sd_cmd52_read(pintfhdl, addr, cnt, pbuf);
		return err;
	}

	n = RND4(cnt);
	ptmpbuf = (u8 *)rtw_malloc(n);
	if (!ptmpbuf)
		return -1;

	err = sd_read(pintfhdl, addr, n, ptmpbuf);
	if (!err)
		_rtw_memcpy(pbuf, ptmpbuf, cnt);

	if (ptmpbuf)
		rtw_mfree(ptmpbuf, n);

	return err;
}

/*
 * Todo: align address to 4 bytes.
 */
s32 _sdio_local_write(
	PADAPTER	padapter,
	u32			addr,
	u32			cnt,
	u8			*pbuf)
{
	struct intf_hdl *pintfhdl;
	u8 bMacPwrCtrlOn;
	s32 err;
	u8 *ptmpbuf;

	if (addr & 0x3)
		RTW_INFO("%s, address must be 4 bytes alignment\n", __FUNCTION__);

	if (cnt  & 0x3)
		RTW_INFO("%s, size must be the multiple of 4\n", __FUNCTION__);

	pintfhdl = &padapter->iopriv.intf;

	HalSdioGetCmdAddr8188FSdio(padapter, SDIO_LOCAL_DEVICE_ID, addr, &addr);

	bMacPwrCtrlOn = _FALSE;
	rtw_hal_get_hwreg(padapter, HW_VAR_APFM_ON_MAC, &bMacPwrCtrlOn);
	if ((_FALSE == bMacPwrCtrlOn)
		#ifdef CONFIG_LPS_LCLK
		|| (adapter_to_pwrctl(padapter)->bFwCurrentInPSMode == _TRUE)
		#endif
		|| CONFIG_RTW_SDIO_REG_FORCE_CMD52
	) {
		err = _sd_cmd52_write(pintfhdl, addr, cnt, pbuf);
		return err;
	}

	ptmpbuf = (u8 *)rtw_malloc(cnt);
	if (!ptmpbuf)
		return -1;

	_rtw_memcpy(ptmpbuf, pbuf, cnt);

	err = _sd_write(pintfhdl, addr, cnt, ptmpbuf);

	if (ptmpbuf)
		rtw_mfree(ptmpbuf, cnt);

	return err;
}

/*
 * Todo: align address to 4 bytes.
 */
s32 sdio_local_write(
	PADAPTER	padapter,
	u32		addr,
	u32		cnt,
	u8		*pbuf)
{
	struct intf_hdl *pintfhdl;
	u8 bMacPwrCtrlOn;
	s32 err;
	u8 *ptmpbuf;

	if (addr & 0x3)
		RTW_INFO("%s, address must be 4 bytes alignment\n", __FUNCTION__);

	if (cnt  & 0x3)
		RTW_INFO("%s, size must be the multiple of 4\n", __FUNCTION__);

	pintfhdl = &padapter->iopriv.intf;

	HalSdioGetCmdAddr8188FSdio(padapter, SDIO_LOCAL_DEVICE_ID, addr, &addr);

	bMacPwrCtrlOn = _FALSE;
	rtw_hal_get_hwreg(padapter, HW_VAR_APFM_ON_MAC, &bMacPwrCtrlOn);
	if ((_FALSE == bMacPwrCtrlOn)
		#ifdef CONFIG_LPS_LCLK
		|| (adapter_to_pwrctl(padapter)->bFwCurrentInPSMode == _TRUE)
		#endif
		|| CONFIG_RTW_SDIO_REG_FORCE_CMD52
	) {
		err = sd_cmd52_write(pintfhdl, addr, cnt, pbuf);
		return err;
	}

	ptmpbuf = (u8 *)rtw_malloc(cnt);
	if (!ptmpbuf)
		return -1;

	_rtw_memcpy(ptmpbuf, pbuf, cnt);

	err = sd_write(pintfhdl, addr, cnt, ptmpbuf);

	if (ptmpbuf)
		rtw_mfree(ptmpbuf, cnt);

	return err;
}

u8 SdioLocalCmd52Read1Byte(PADAPTER padapter, u32 addr)
{
	u8 val = 0;
	struct intf_hdl *pintfhdl = &padapter->iopriv.intf;

	HalSdioGetCmdAddr8188FSdio(padapter, SDIO_LOCAL_DEVICE_ID, addr, &addr);
	sd_cmd52_read(pintfhdl, addr, 1, &val);

	return val;
}

u16 SdioLocalCmd52Read2Byte(PADAPTER padapter, u32 addr)
{
	u16 val = 0;
	struct intf_hdl *pintfhdl = &padapter->iopriv.intf;

	HalSdioGetCmdAddr8188FSdio(padapter, SDIO_LOCAL_DEVICE_ID, addr, &addr);
	sd_cmd52_read(pintfhdl, addr, 2, (u8 *)&val);

	val = le16_to_cpu(val);

	return val;
}

u32 SdioLocalCmd52Read4Byte(PADAPTER padapter, u32 addr)
{
	u32 val = 0;
	struct intf_hdl *pintfhdl = &padapter->iopriv.intf;

	HalSdioGetCmdAddr8188FSdio(padapter, SDIO_LOCAL_DEVICE_ID, addr, &addr);
	sd_cmd52_read(pintfhdl, addr, 4, (u8 *)&val);

	val = le32_to_cpu(val);

	return val;
}

u32 SdioLocalCmd53Read4Byte(PADAPTER padapter, u32 addr)
{

	u8 bMacPwrCtrlOn;
	u32 val = 0;
	struct intf_hdl *pintfhdl = &padapter->iopriv.intf;

	HalSdioGetCmdAddr8188FSdio(padapter, SDIO_LOCAL_DEVICE_ID, addr, &addr);
	bMacPwrCtrlOn = _FALSE;
	rtw_hal_get_hwreg(padapter, HW_VAR_APFM_ON_MAC, &bMacPwrCtrlOn);
	if ((_FALSE == bMacPwrCtrlOn)
		#ifdef CONFIG_LPS_LCLK
		|| (adapter_to_pwrctl(padapter)->bFwCurrentInPSMode == _TRUE)
		#endif
		|| CONFIG_RTW_SDIO_REG_FORCE_CMD52
	) {
		sd_cmd52_read(pintfhdl, addr, 4, (u8 *)&val);
		val = le32_to_cpu(val);
	} else
		val = sd_read32(pintfhdl, addr, NULL);

	return val;
}

void SdioLocalCmd52Write1Byte(PADAPTER padapter, u32 addr, u8 v)
{
	struct intf_hdl *pintfhdl = &padapter->iopriv.intf;

	HalSdioGetCmdAddr8188FSdio(padapter, SDIO_LOCAL_DEVICE_ID, addr, &addr);
	sd_cmd52_write(pintfhdl, addr, 1, &v);
}

void SdioLocalCmd52Write2Byte(PADAPTER padapter, u32 addr, u16 v)
{
	struct intf_hdl *pintfhdl = &padapter->iopriv.intf;

	HalSdioGetCmdAddr8188FSdio(padapter, SDIO_LOCAL_DEVICE_ID, addr, &addr);
	v = cpu_to_le16(v);
	sd_cmd52_write(pintfhdl, addr, 2, (u8 *)&v);
}

void SdioLocalCmd52Write4Byte(PADAPTER padapter, u32 addr, u32 v)
{
	struct intf_hdl *pintfhdl = &padapter->iopriv.intf;
	HalSdioGetCmdAddr8188FSdio(padapter, SDIO_LOCAL_DEVICE_ID, addr, &addr);
	v = cpu_to_le32(v);
	sd_cmd52_write(pintfhdl, addr, 4, (u8 *)&v);
}

void sdio_set_intf_ops(_adapter *padapter, struct _io_ops *pops)
{

	pops->_read8 = &sdio_read8;
	pops->_read16 = &sdio_read16;
	pops->_read32 = &sdio_read32;
	pops->_read_mem = &sdio_read_mem;
	pops->_read_port = &sdio_read_port;

	pops->_write8 = &sdio_write8;
	pops->_write16 = &sdio_write16;
	pops->_write32 = &sdio_write32;
	pops->_writeN = &sdio_writeN;
	pops->_write_mem = &sdio_write_mem;
	pops->_write_port = &sdio_write_port;

	pops->_sd_f0_read8 = sdio_f0_read8;

#ifdef CONFIG_SDIO_INDIRECT_ACCESS
	pops->_sd_iread8 = sdio_iread8;
	pops->_sd_iread16 = sdio_iread16;
	pops->_sd_iread32 = sdio_iread32;
	pops->_sd_iwrite8 = sdio_iwrite8;
	pops->_sd_iwrite16 = sdio_iwrite16;
	pops->_sd_iwrite32 = sdio_iwrite32;
#endif

}

/*
 *	Description:
 *		Initialize SDIO Host Interrupt Mask configuration variables for future use.
 *
 *	Assumption:
 *		Using SDIO Local register ONLY for configuration.
 *
 *	Created by Roger, 2011.02.11.
 *   */
void InitInterrupt8188FSdio(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData;

	pHalData = GET_HAL_DATA(padapter);
	pHalData->sdio_himr = 0
		| SDIO_HIMR_RX_REQUEST_MSK
#ifdef CONFIG_SDIO_TX_ENABLE_AVAL_INT
		| SDIO_HIMR_AVAL_MSK
#endif
#if 0
		| SDIO_HIMR_TXERR_MSK
		| SDIO_HIMR_RXERR_MSK
		| SDIO_HIMR_TXFOVW_MSK
		| SDIO_HIMR_RXFOVW_MSK
		| SDIO_HIMR_TXBCNOK_MSK
		| SDIO_HIMR_TXBCNERR_MSK
		| SDIO_HIMR_BCNERLY_INT_MSK
		| SDIO_HIMR_C2HCMD_MSK
#endif
#if defined(CONFIG_LPS_LCLK) && !defined(CONFIG_DETECT_CPWM_BY_POLLING)
		| SDIO_HIMR_CPWM1_MSK
#endif
#ifdef CONFIG_WOWLAN
		| SDIO_HIMR_CPWM2_MSK
#endif
#if 0
		| SDIO_HIMR_HSISR_IND_MSK
		| SDIO_HIMR_GTINT3_IND_MSK
		| SDIO_HIMR_GTINT4_IND_MSK
		| SDIO_HIMR_PSTIMEOUT_MSK
		| SDIO_HIMR_OCPINT_MSK
		| SDIO_HIMR_ATIMEND_MSK
		| SDIO_HIMR_ATIMEND_E_MSK
		| SDIO_HIMR_CTWEND_MSK
#endif
		;
}

/*
 *	Description:
 *		Initialize System Host Interrupt Mask configuration variables for future use.
 *
 *	Created by Roger, 2011.08.03.
 *   */
void InitSysInterrupt8188FSdio(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData;


	pHalData = GET_HAL_DATA(padapter);

	pHalData->SysIntrMask = (\
				 /*							HSIMR_GPIO12_0_INT_EN			|
				  *							HSIMR_SPS_OCP_INT_EN			|
				  *							HSIMR_RON_INT_EN				|
				  *							HSIMR_PDNINT_EN				|
				  *							HSIMR_GPIO9_INT_EN				| */
				 0);
}

#if defined(CONFIG_WOWLAN) || defined(CONFIG_AP_WOWLAN)
/*
 *	Description:
 *		Clear corresponding SDIO Host ISR interrupt service.
 *
 *	Assumption:
 *		Using SDIO Local register ONLY for configuration.
 *
 *	Created by Roger, 2011.02.11.
 *   */
void ClearInterrupt8188FSdio(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData;
	u8 *clear;


	if (rtw_is_surprise_removed(padapter))
		return;

	pHalData = GET_HAL_DATA(padapter);
	clear = rtw_zmalloc(4);

	/* Clear corresponding HISR Content if needed */
	*(u32 *)clear = cpu_to_le32(pHalData->sdio_hisr & MASK_SDIO_HISR_CLEAR);
	if (*(u32 *)clear) {
		/* Perform write one clear operation */
		sdio_local_write(padapter, SDIO_REG_HISR, 4, clear);
	}

	rtw_mfree(clear, 4);
}
#endif

/*
 *	Description:
 *		Clear corresponding system Host ISR interrupt service.
 *
 *
 *	Created by Roger, 2011.02.11.
 *   */
void ClearSysInterrupt8188FSdio(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData;
	u32 clear;


	if (rtw_is_surprise_removed(padapter))
		return;

	pHalData = GET_HAL_DATA(padapter);

	/* Clear corresponding HISR Content if needed */
	clear = pHalData->SysIntrStatus & MASK_HSISR_CLEAR;
	if (clear) {
		/* Perform write one clear operation */
		rtw_write32(padapter, REG_HSISR, clear);
	}
}

/*
 *	Description:
 *		Enalbe SDIO Host Interrupt Mask configuration on SDIO local domain.
 *
 *	Assumption:
 *		1. Using SDIO Local register ONLY for configuration.
 *		2. PASSIVE LEVEL
 *
 *	Created by Roger, 2011.02.11.
 *   */
void EnableInterrupt8188FSdio(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData;
	u32 himr;

	pHalData = GET_HAL_DATA(padapter);

	himr = cpu_to_le32(pHalData->sdio_himr);
	sdio_local_write(padapter, SDIO_REG_HIMR, 4, (u8 *)&himr);


	/* Update current system IMR settings */
	himr = rtw_read32(padapter, REG_HSIMR);
	rtw_write32(padapter, REG_HSIMR, himr | pHalData->SysIntrMask);


	/*  */
	/* <Roger_Notes> There are some C2H CMDs have been sent before system interrupt is enabled, e.g., C2H, CPWM. */
	/* So we need to clear all C2H events that FW has notified, otherwise FW won't schedule any commands anymore. */
	/* 2011.10.19. */
	/*  */
	rtw_write8(padapter, REG_C2HEVT_CLEAR, C2H_EVT_HOST_CLOSE);
}

/*
 *	Description:
 *		Disable SDIO Host IMR configuration to mask unnecessary interrupt service.
 *
 *	Assumption:
 *		Using SDIO Local register ONLY for configuration.
 *
 *	Created by Roger, 2011.02.11.
 *   */
void DisableInterrupt8188FSdio(PADAPTER padapter)
{
	u32 himr;

	himr = cpu_to_le32(SDIO_HIMR_DISABLED);
	sdio_local_write(padapter, SDIO_REG_HIMR, 4, (u8 *)&himr);

}

/*
 *	Description:
 *		Using 0x100 to check the power status of FW.
 *
 *	Assumption:
 *		Using SDIO Local register ONLY for configuration.
 *
 *	Created by Isaac, 2013.09.10.
 *   */
u8 CheckIPSStatus(PADAPTER padapter)
{
	RTW_INFO("%s(): Read 0x100=0x%02x 0x86=0x%02x\n", __func__,
		 rtw_read8(padapter, 0x100), rtw_read8(padapter, 0x86));

	if (rtw_read8(padapter, 0x100) == 0xEA)
		return _TRUE;
	else
		return _FALSE;
}

#ifdef CONFIG_WOWLAN
void DisableInterruptButCpwm28188FSdio(PADAPTER padapter)
{
	u32 himr, tmp;

	sdio_local_read(padapter, SDIO_REG_HIMR, 4, (u8 *)&tmp);
	RTW_INFO("DisableInterruptButCpwm28188FSdio(): Read SDIO_REG_HIMR: 0x%08x\n", tmp);

	himr = cpu_to_le32(SDIO_HIMR_DISABLED) | SDIO_HIMR_CPWM2_MSK;
	sdio_local_write(padapter, SDIO_REG_HIMR, 4, (u8 *)&himr);

	sdio_local_read(padapter, SDIO_REG_HIMR, 4, (u8 *)&tmp);
	RTW_INFO("DisableInterruptButCpwm28188FSdio(): Read again SDIO_REG_HIMR: 0x%08x\n", tmp);
}
#endif /* CONFIG_WOWLAN
 *
 *	Description:
 *		Update SDIO Host Interrupt Mask configuration on SDIO local domain.
 *
 *	Assumption:
 *		1. Using SDIO Local register ONLY for configuration.
 *		2. PASSIVE LEVEL
 *
 *	Created by Roger, 2011.02.11.
 *   */
void UpdateInterruptMask8188FSdio(PADAPTER padapter, u32 AddMSR, u32 RemoveMSR)
{
	HAL_DATA_TYPE *pHalData;

	pHalData = GET_HAL_DATA(padapter);

	if (AddMSR)
		pHalData->sdio_himr |= AddMSR;

	if (RemoveMSR)
		pHalData->sdio_himr &= (~RemoveMSR);

	DisableInterrupt8188FSdio(padapter);
	EnableInterrupt8188FSdio(padapter);
}

#ifdef CONFIG_MAC_LOOPBACK_DRIVER
static void sd_recv_loopback(PADAPTER padapter, u32 size)
{
	PLOOPBACKDATA ploopback;
	u32 readsize, allocsize;
	u8 *preadbuf;


	readsize = size;
	RTW_INFO("%s: read size=%d\n", __func__, readsize);
	allocsize = _RND(readsize, rtw_sdio_get_block_size(adapter_to_dvobj(padapter)));

	ploopback = padapter->ploopback;
	if (ploopback) {
		ploopback->rxsize = readsize;
		preadbuf = ploopback->rxbuf;
	} else {
		preadbuf = rtw_malloc(allocsize);
		if (preadbuf == NULL) {
			RTW_INFO("%s: malloc fail size=%d\n", __func__, allocsize);
			return;
		}
	}

	/*	rtw_read_port(padapter, WLAN_RX0FF_DEVICE_ID, readsize, preadbuf); */
	sdio_read_port(&padapter->iopriv.intf, WLAN_RX0FF_DEVICE_ID, readsize, preadbuf);

	if (ploopback)
		_rtw_up_sema(&ploopback->sema);
	else {
		u32 i;

		RTW_INFO("%s: drop pkt\n", __func__);
		for (i = 0; i < readsize; i += 4) {
			RTW_INFO("%08X", *(u32 *)(preadbuf + i));
			if ((i + 4) & 0x1F)
				printk(" ");
			else
				printk("\n");
		}
		printk("\n");
		rtw_mfree(preadbuf, allocsize);
	}
}
#endif /* CONFIG_MAC_LOOPBACK_DRIVER */

#ifdef CONFIG_SDIO_RX_COPY
static u32 sd_recv_alloc_buf(_adapter *adapter, u32 size, struct recv_buf **recvbuf_ret)
{
#ifndef CONFIG_TEST_RBUF_UNAVAIL
#define CONFIG_TEST_RBUF_UNAVAIL 0
#endif

#if CONFIG_TEST_RBUF_UNAVAIL
#define TEST_RBUF_UNAVAIL_CYCLE_MS (10 * 1000)
#define TEST_RBUF_UNAVAIL_TIME_MS (50)
	static systime test_start = 0;
#endif
	struct recv_priv *precvpriv;
	struct recv_buf	*precvbuf;
	u32 ret;

	*recvbuf_ret = NULL;

#if CONFIG_TEST_RBUF_UNAVAIL
	if (test_start == 0)
		test_start = rtw_get_current_time();

	if (rtw_get_passing_time_ms(test_start) >= TEST_RBUF_UNAVAIL_CYCLE_MS) {
		if (rtw_get_passing_time_ms(test_start) >= TEST_RBUF_UNAVAIL_CYCLE_MS + TEST_RBUF_UNAVAIL_TIME_MS)
			test_start = rtw_get_current_time();
		ret = RTW_RBUF_UNAVAIL;
		goto exit;
	}
#endif

	precvpriv = &adapter->recvpriv;
#ifdef CONFIG_SDIO_RECVBUF_PWAIT_RUNTIME_ADJUST
alloc_recvbuf:
#endif
	/* alloc recvbuf */
	precvbuf = rtw_dequeue_recvbuf(&precvpriv->free_recv_buf_queue);
	if (precvbuf == NULL) {
		if (0)
			RTW_INFO("%s: recvbuf unavailable\n", __func__);
		ret = RTW_RBUF_UNAVAIL;
		goto exit;
	}

#ifdef CONFIG_SDIO_RECVBUF_PWAIT_RUNTIME_ADJUST
	if (precvbuf->type == RBUF_TYPE_PWAIT_ADJ) {
		recvbuf_pwait_config_hdl(precvpriv, precvbuf);
		goto alloc_recvbuf;
	}
#endif

	if (precvbuf->pskb == NULL) {
		/*  alloc skb */
		SIZE_PTR tmpaddr = 0;
		SIZE_PTR alignment = 0;

		precvbuf->pskb = rtw_skb_alloc(MAX_RECVBUF_SZ + RECVBUFF_ALIGN_SZ);
		if (precvbuf->pskb == NULL) {
			RTW_INFO("%s: alloc_skb fail! size=%d\n", __func__, MAX_RECVBUF_SZ + RECVBUFF_ALIGN_SZ);
			rtw_enqueue_recvbuf(precvbuf, &precvpriv->free_recv_buf_queue);
			ret = RTW_RBUF_PKT_UNAVAIL;
			goto exit;
		}

		precvbuf->pskb->dev = adapter->pnetdev;

		tmpaddr = (SIZE_PTR)precvbuf->pskb->data;
		alignment = tmpaddr & (RECVBUFF_ALIGN_SZ - 1);
		skb_reserve(precvbuf->pskb, (RECVBUFF_ALIGN_SZ - alignment));

		/* init recvbuf */
		precvbuf->phead = precvbuf->pskb->head;
		precvbuf->pdata = precvbuf->pskb->data;
		precvbuf->ptail = skb_tail_pointer(precvbuf->pskb);
		precvbuf->pend = skb_end_pointer(precvbuf->pskb);
		precvbuf->len = 0;

	} else {
		/* reset recvbuf to len 0*/
		precvbuf->len = 0;
		precvbuf->pdata = precvbuf->pskb->data;
		skb_set_tail_pointer(precvbuf->pskb, 0);
		precvbuf->ptail = skb_tail_pointer(precvbuf->pskb);
	}

	*recvbuf_ret = precvbuf;
	ret = _SUCCESS;

exit:
	return ret;
}

static u32 sd_recv_rxfifo(_adapter *adapter, struct recv_buf *rbuf, u32 read_sz, u32 ocp_sz)
{
	struct recv_priv *precvpriv = &adapter->recvpriv;
	u32 ret;

	/* read data from rxfifo */
	/*	rtw_read_port(adapter, WLAN_RX0FF_DEVICE_ID, read_sz, rbuf->data); */
	ret = sdio_read_port_no_align(&adapter->iopriv.intf, WLAN_RX0FF_DEVICE_ID, read_sz, rbuf->ptail);
	if (ret == _FAIL) {
		rtw_enqueue_recvbuf(rbuf, &precvpriv->free_recv_buf_queue);
		goto exit;
	}

	/* increase recvbuf len */
	skb_put(rbuf->pskb, ocp_sz);
	rbuf->ptail = skb_tail_pointer(rbuf->pskb);
	rbuf->len += ocp_sz;

exit:
	return ret;
}
#else /* !CONFIG_SDIO_RX_COPY */
static struct recv_buf *sd_recv_rxfifo(PADAPTER padapter, u32 size)
{
	u32 readsize, allocsize, ret;
	u8 *preadbuf;
	_pkt *ppkt;
	struct recv_priv *precvpriv;
	struct recv_buf	*precvbuf;

	/* 3 1. alloc skb */
	/* align to block size */
	allocsize = rtw_sdio_cmd53_align_size(adapter_to_dvobj(padapter), readsize);

	ppkt = rtw_skb_alloc(allocsize);

	if (ppkt == NULL) {
		return NULL;
	}

	/* 3 2. read data from rxfifo */
	preadbuf = skb_put(ppkt, size);
	/*	rtw_read_port(padapter, WLAN_RX0FF_DEVICE_ID, readsize, preadbuf); */
	ret = sdio_read_port(&padapter->iopriv.intf, WLAN_RX0FF_DEVICE_ID, readsize, preadbuf);
	if (ret == _FAIL) {
		rtw_skb_free(ppkt);
		return NULL;
	}

	/* 3 3. alloc recvbuf */
	precvpriv = &padapter->recvpriv;
	precvbuf = rtw_dequeue_recvbuf(&precvpriv->free_recv_buf_queue);
	if (precvbuf == NULL) {
		rtw_skb_free(ppkt);
		RTW_ERR("%s: alloc recvbuf FAIL!\n", __FUNCTION__);
		return NULL;
	}

	/* 3 4. init recvbuf */
	precvbuf->pskb = ppkt;

	precvbuf->len = ppkt->len;

	precvbuf->phead = ppkt->head;
	precvbuf->pdata = ppkt->data;
	precvbuf->ptail = skb_tail_pointer(precvbuf->pskb);
	precvbuf->pend = skb_end_pointer(precvbuf->pskb);

	return precvbuf;
}
#endif /* !CONFIG_SDIO_RX_COPY */

static void sd_rxhandler(PADAPTER padapter, struct recv_buf *precvbuf)
{
	struct recv_priv *precvpriv;
	_queue *ppending_queue;


	precvpriv = &padapter->recvpriv;
	ppending_queue = &precvpriv->recv_buf_pending_queue;

	/* 3 1. enqueue recvbuf */
	rtw_enqueue_recvbuf(precvbuf, ppending_queue);

	/* 3 2. trigger recv hdl */
#ifdef CONFIG_RECV_THREAD_MODE
	_rtw_up_sema(&precvpriv->recv_sema);
#else
	#ifdef PLATFORM_LINUX
	tasklet_schedule(&precvpriv->recv_tasklet);
	#endif
#endif
}

#ifndef SD_INT_HDL_DIS_HIMR_RX_REQ
#define SD_INT_HDL_DIS_HIMR_RX_REQ 0
#endif

#ifndef CMD52_ACCESS_HISR_RX_REQ_LEN
#define CMD52_ACCESS_HISR_RX_REQ_LEN 0
#endif

#if SD_INT_HDL_DIS_HIMR_RX_REQ
#define DIS_HIMR_RX_REQ_WITH_CMD52 1
static void disable_himr_rx_req_8188f_sdio(_adapter *adapter)
{
	HAL_DATA_TYPE *hal = GET_HAL_DATA(adapter);
	u32 himr = cpu_to_le32(hal->sdio_himr & ~SDIO_HISR_RX_REQUEST);

#if DIS_HIMR_RX_REQ_WITH_CMD52
	SdioLocalCmd52Write1Byte(adapter, SDIO_REG_HIMR, *((u8 *)&himr));
#else
	sdio_local_write(adapter, SDIO_REG_HIMR, 4, (u8 *)&himr);
#endif
}

static void restore_himr_8188f_sdio(_adapter *adapter)
{
	HAL_DATA_TYPE *hal = GET_HAL_DATA(adapter);
	u32 himr = cpu_to_le32(hal->sdio_himr);

#if DIS_HIMR_RX_REQ_WITH_CMD52
	SdioLocalCmd52Write1Byte(adapter, SDIO_REG_HIMR, *((u8 *)&himr));
#else
	sdio_local_write(adapter, SDIO_REG_HIMR, 4, (u8 *)&himr);
#endif
}
#endif /* SD_INT_HDL_DIS_HIMR_RX_REQ */

#if CMD52_ACCESS_HISR_RX_REQ_LEN
static s32 ReadInterrupt8188FSdio(PADAPTER padapter, u32 *phisr)
{
	u32 hisr, himr;
	u8 val8, hisr_len;
	int i;

	if (phisr == NULL)
		return _FALSE;

	himr = GET_HAL_DATA(padapter)->sdio_himr;

	/* decide how many bytes need to be read */
	hisr_len = 0;
	while (himr) {
		hisr_len++;
		himr >>= 8;
	}

	hisr = 0;
	for (i = 0; i < hisr_len; i++) {
		val8 = SdioLocalCmd52Read1Byte(padapter, SDIO_REG_HISR + i);
		hisr |= (val8 << (8 * i));
	}

	*phisr = hisr;

	return _TRUE;
}
#endif /* CMD52_ACCESS_HISR_RX_REQ_LEN */

void sd_recv(_adapter *adapter)
{
	PHAL_DATA_TYPE phal = GET_HAL_DATA(adapter);
	struct recv_priv *recvpriv = &adapter->recvpriv;
	struct recv_buf *rbuf = NULL;
	u32 read_sz, ocp_sz, buf_sz;
	u32 recv_fail_cnt = 0;
	u32 rx_cnt = 0;
	u32 ret;

	do {
		if (phal->SdioRxFIFOSize == 0) {
			#if CMD52_ACCESS_HISR_RX_REQ_LEN
			u16 rx_req_len;

			rx_req_len = SdioLocalCmd52Read2Byte(adapter, SDIO_REG_RX0_REQ_LEN);
			if (rx_req_len) {
				if (rx_req_len % 256 == 0)
					rx_req_len += SdioLocalCmd52Read1Byte(adapter, SDIO_REG_RX0_REQ_LEN);
				phal->SdioRxFIFOSize = rx_req_len;
			}
			#else
			u8 data[4];

			_sdio_local_read(adapter, SDIO_REG_RX0_REQ_LEN, 4, data);
			phal->SdioRxFIFOSize = le16_to_cpu(*(u16 *)data);
			#endif

			if (phal->SdioRxFIFOSize == 0)
				break;
		}

		#ifdef CONFIG_MAC_LOOPBACK_DRIVER
		sd_recv_loopback(adapter, phal->SdioRxFIFOSize);
		#else
		read_sz = rtw_sdio_cmd53_align_size(adapter_to_dvobj(adapter), phal->SdioRxFIFOSize);
		read_sz = RND4(read_sz); /* Patch for some SDIO Host 4 bytes issue, ex. RK3188 */
		ocp_sz = _RND8(phal->SdioRxFIFOSize);
		buf_sz = rtw_max(read_sz, ocp_sz);
		if (buf_sz > MAX_RECVBUF_SZ) {
			RTW_INFO(FUNC_ADPT_FMT" %u > MAX_RECVBUF_SZ(%u)\n", FUNC_ADPT_ARG(adapter), buf_sz, MAX_RECVBUF_SZ);
			rtw_warn_on(1);
		}

		if (rbuf
			&& (!recv_buf_agg(recvpriv)
				|| _rtw_queue_empty(&recvpriv->recv_buf_pending_queue)
				|| rbuf->ptail + buf_sz > rbuf->pend
				)
		) {
			sd_rxhandler(adapter, rbuf);
			rbuf = NULL;
		}

		if (!rbuf) {
			struct rtw_pwait_ctx *pwctx = &recvpriv->recvbuf_pwait;

			rtw_pwctx_reset(pwctx);
			while (sd_recv_alloc_buf(adapter, buf_sz, &rbuf) != _SUCCESS) {
				if (rtw_pwctx_exceed(&recvpriv->recvbuf_pwait)) {
					if (rx_cnt)
						break;
				}
				rtw_pwctx_wait(&recvpriv->recvbuf_pwait);
			}
			if (!rbuf)
				break;
		}

		ret = sd_recv_rxfifo(adapter, rbuf, read_sz, ocp_sz);
		if (ret == _SUCCESS) {
			if (!recv_buf_agg(recvpriv)
				|| _rtw_queue_empty(&recvpriv->recv_buf_pending_queue)
				|| rbuf->ptail >= rbuf->pend
			) {
				sd_rxhandler(adapter, rbuf);
				rbuf = NULL;
			}
			rx_cnt++;
			phal->SdioRxFIFOSize = 0;
		} else {
			recv_fail_cnt++;
			RTW_WARN("%s: recv fail! cnt:%u\n", __func__, recv_fail_cnt);
			phal->SdioRxFIFOSize = 0;
			if (recv_fail_cnt >= 10) {
				RTW_WARN("%s: exit because recv_fail_cnt=%u! (rx_cnt=%u)\n"
					, __func__, recv_fail_cnt, rx_cnt);
				break;
			}
		}
		#endif
	} while (1);

	if (rbuf)
		sd_rxhandler(adapter, rbuf);
}

void sd_int_dpc(PADAPTER padapter)
{
	PHAL_DATA_TYPE phal;
	struct dvobj_priv *dvobj;
	struct intf_hdl *pintfhdl = &padapter->iopriv.intf;
	struct pwrctrl_priv *pwrctl;


	phal = GET_HAL_DATA(padapter);
	dvobj = adapter_to_dvobj(padapter);
	pwrctl = dvobj_to_pwrctl(dvobj);

#ifdef CONFIG_SDIO_TX_ENABLE_AVAL_INT
	if (phal->sdio_hisr & SDIO_HISR_AVAL) {
		u8 freepage[8];

		_sdio_local_read(padapter, SDIO_REG_FREE_TXPG, 8, freepage);
		#ifdef DBG_TX_FREE_PAGE
		RTW_INFO("SDIO_HISR_AVAL, Tx Free Page = H:%u, M:%u, L:%u, P:%u\n",
			freepage[0], freepage[2], freepage[4], freepage[6]);
		#endif

		_rtw_up_sema(&(padapter->xmitpriv.xmit_sema));
	}
#endif

	if (phal->sdio_hisr & SDIO_HISR_CPWM1) {
		struct reportpwrstate_parm report;

#ifdef CONFIG_LPS_RPWM_TIMER
		_cancel_timer_ex(&(pwrctl->pwr_rpwm_timer));
#endif /* CONFIG_LPS_RPWM_TIMER */

		report.state = SdioLocalCmd52Read1Byte(padapter, SDIO_REG_HCPWM1_8188F);

#ifdef CONFIG_LPS_LCLK
		/* cpwm_int_hdl(padapter, &report); */
		_set_workitem(&(pwrctl->cpwm_event));
#endif
	}

	if (phal->sdio_hisr & SDIO_HISR_TXERR) {
		u8 *status;
		u32 addr;

		status = rtw_malloc(4);
		if (status) {
			addr = REG_TXDMA_STATUS;
			HalSdioGetCmdAddr8188FSdio(padapter, WLAN_IOREG_DEVICE_ID, addr, &addr);
			_sd_read(pintfhdl, addr, 4, status);
			_sd_write(pintfhdl, addr, 4, status);
			RTW_INFO("%s: SDIO_HISR_TXERR (0x%08x)\n", __func__, le32_to_cpu(*(u32 *)status));
			rtw_mfree(status, 4);
		} else
			RTW_INFO("%s: SDIO_HISR_TXERR, but can't allocate memory to read status!\n", __func__);
	}

	if (phal->sdio_hisr & SDIO_HISR_TXBCNOK)
		RTW_INFO("%s: SDIO_HISR_TXBCNOK\n", __func__);

	if (phal->sdio_hisr & SDIO_HISR_TXBCNERR)
		RTW_INFO("%s: SDIO_HISR_TXBCNERR\n", __func__);

#ifdef CONFIG_FW_C2H_REG
	if (phal->sdio_hisr & SDIO_HISR_C2HCMD) {
		RTW_INFO("%s: C2H Command\n", __func__);
		sd_c2h_hisr_hdl(padapter);
	}
#endif

	if (phal->sdio_hisr & SDIO_HISR_RXFOVW)
		RTW_INFO("%s: Rx Overflow\n", __func__);
	if (phal->sdio_hisr & SDIO_HISR_RXERR)
		RTW_INFO("%s: Rx Error\n", __func__);

	if (phal->sdio_hisr & SDIO_HISR_RX_REQUEST) {
		phal->sdio_hisr ^= SDIO_HISR_RX_REQUEST;
		sd_recv(padapter);
	}
}

#ifndef DBG_SD_INT_HISR_HIMR
#define DBG_SD_INT_HISR_HIMR 0
#endif

void sd_int_hdl(PADAPTER padapter)
{
	PHAL_DATA_TYPE phal;
	#if !CMD52_ACCESS_HISR_RX_REQ_LEN
	u8 data[6];
	#endif

	if (RTW_CANNOT_RUN(padapter))
		return;

	phal = GET_HAL_DATA(padapter);

	#if SD_INT_HDL_DIS_HIMR_RX_REQ
	disable_himr_rx_req_8188f_sdio(padapter);
	#endif

	#if CMD52_ACCESS_HISR_RX_REQ_LEN
	phal->sdio_hisr = 0;
	ReadInterrupt8188FSdio(padapter, &phal->sdio_hisr);
	#else
	_sdio_local_read(padapter, SDIO_REG_HISR, 6, data);
	phal->sdio_hisr = le32_to_cpu(*(u32 *)data);
	phal->SdioRxFIFOSize = le16_to_cpu(*(u16 *)&data[4]);
	#endif

	if (phal->sdio_hisr & phal->sdio_himr) {
		u32 v32;
		#if DBG_SD_INT_HISR_HIMR
		static u32 match_cnt = 0;

		if ((match_cnt++) % 1000 == 0)
			RTW_INFO("%s: HISR(0x%08x) and HIMR(0x%08x) match!\n"
				, __func__, phal->sdio_hisr, phal->sdio_himr);
		#endif

		phal->sdio_hisr &= phal->sdio_himr;

		/* clear HISR */
		v32 = phal->sdio_hisr & MASK_SDIO_HISR_CLEAR;
		if (v32) {
			#if CMD52_ACCESS_HISR_RX_REQ_LEN
			SdioLocalCmd52Write4Byte(padapter, SDIO_REG_HISR, v32);
			#else
			v32 = cpu_to_le32(v32);
			_sdio_local_write(padapter, SDIO_REG_HISR, 4, (u8 *)&v32);
			#endif
		}

		sd_int_dpc(padapter);
	}
	#if DBG_SD_INT_HISR_HIMR
	else
		RTW_INFO("%s: HISR(0x%08x) and HIMR(0x%08x) not match!\n"
			, __func__, phal->sdio_hisr, phal->sdio_himr);
	#endif

	#if SD_INT_HDL_DIS_HIMR_RX_REQ
	restore_himr_8188f_sdio(padapter);
	#endif
}

/*
 *	Description:
 *		Query SDIO Local register to query current the number of Free TxPacketBuffer page.
 *
 *	Assumption:
 *		1. Running at PASSIVE_LEVEL
 *		2. RT_TX_SPINLOCK is NOT acquired.
 *
 *	Created by Roger, 2011.01.28.
 *   */
u8 HalQueryTxBufferStatus8188FSdio(PADAPTER padapter)
{
	/* TODO: EXQ */
	PHAL_DATA_TYPE phal;
	u8 NumOfFreePage[8];
	/* _irqL irql; */

	phal = GET_HAL_DATA(padapter);

	sdio_local_read(padapter, SDIO_REG_FREE_TXPG, 8, NumOfFreePage);

	/* _enter_critical_bh(&phal->SdioTxFIFOFreePageLock, &irql); */
	phal->SdioTxFIFOFreePage[HI_QUEUE_IDX] = NumOfFreePage[0];
	phal->SdioTxFIFOFreePage[MID_QUEUE_IDX]  = NumOfFreePage[2];
	phal->SdioTxFIFOFreePage[LOW_QUEUE_IDX] = NumOfFreePage[4];
	phal->SdioTxFIFOFreePage[PUBLIC_QUEUE_IDX] = NumOfFreePage[6];
	/* _exit_critical_bh(&phal->SdioTxFIFOFreePageLock, &irql); */

	return _TRUE;
}

/*
 *	Description:
 *		Query SDIO Local register to get the current number of TX OQT Free Space.
 *   */
u8 HalQueryTxOQTBufferStatus8188FSdio(PADAPTER padapter)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	pHalData->SdioTxOQTFreeSpace = SdioLocalCmd52Read1Byte(padapter, SDIO_REG_AC_OQT_FREEPG_8188F);
	return _TRUE;
}

#if defined(CONFIG_WOWLAN) || defined(CONFIG_AP_WOWLAN)
u8 RecvOnePkt(PADAPTER padapter)
{
	struct recv_buf *precvbuf;
	struct dvobj_priv *psddev;
	PSDIO_DATA psdio_data;
	struct sdio_func *func;
	u32 tmp = 0;
	u16 len = 0;
	u32 read_sz, ocp_sz, buf_sz;
	u8 res = _FALSE;

	if (padapter == NULL) {
		RTW_ERR("%s: padapter is NULL!\n", __func__);
		return _FALSE;
	}

	psddev = adapter_to_dvobj(padapter);
	psdio_data = &psddev->intf_data;
	func = psdio_data->func;

	/* If RX_DMA is not idle, receive one pkt from DMA */
	res = sdio_local_read(padapter,
			  SDIO_REG_RX0_REQ_LEN, 4, (u8 *)&tmp);
	len = le16_to_cpu(tmp);
	RTW_INFO("+%s: size: %d+\n", __func__, len);

	if (len) {
		read_sz = rtw_sdio_cmd53_align_size(adapter_to_dvobj(padapter), len);
		read_sz = RND4(read_sz); /* Patch for some SDIO Host 4 bytes issue, ex. RK3188 */
		ocp_sz = _RND8(len);
		buf_sz = rtw_max(read_sz, ocp_sz);
		if (buf_sz > MAX_RECVBUF_SZ) {
			RTW_INFO(FUNC_ADPT_FMT" %u > MAX_RECVBUF_SZ(%u)\n", FUNC_ADPT_ARG(padapter), buf_sz, MAX_RECVBUF_SZ);
			rtw_warn_on(1);
		}
		if (sd_recv_alloc_buf(padapter, buf_sz, &precvbuf) != _SUCCESS) {
			res = _FALSE;
			goto exit;
		}
		sdio_claim_host(func);
		if (sd_recv_rxfifo(padapter, precvbuf, read_sz, ocp_sz) == _SUCCESS) {
			/* printk("Completed Recv One Pkt.\n"); */
			sd_rxhandler(padapter, precvbuf);
			res = _TRUE;
		} else
			res = _FALSE;
		sdio_release_host(func);
	}

exit:
	RTW_INFO("-%s-\n", __func__);
	return res;
}
#endif /* CONFIG_WOWLAN */
