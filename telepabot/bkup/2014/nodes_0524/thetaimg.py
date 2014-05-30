#-*- coding: utf-8 -*-
import const

def getXAxisFromAzimuth(azimuth):
	xaxis = (azimuth + const.IMG_HOR_HALF_VIEW_AGL)*const.PIXEL_PER_AZIMUTH
	if xaxis > const.WIN_WID -1:
		return const.WIN_WID -1
	elif xaxis < 0:
		return 0
	else:
		return xaxis 

