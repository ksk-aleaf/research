#-*- coding: utf-8 -*-
import const

def getXAxisFromAzimuth(azimuth):
	xaxis = (azimuth + const.IMG_HOR_HALF_VIEW_AGL)*const.PIXEL_PER_AZIMUTH
	if xaxis > const.CAM_WHOLE_IMG_WID + const.CAM_IMG_OFS_X -1:
		return const.CAM_WHOLE_IMG_WID + const.CAM_IMG_OFS_X -1
	elif xaxis < const.CAM_IMG_OFS_X:
		return const.CAM_IMG_OFS_X
	else:
		return xaxis 

def getXAxisOnImage(xaxis):
	return xaxis - const.CAM_IMG_OFS_X

def getThetaFromXAxis(xaxis):
	return - const.IMG_HOR_HALF_VIEW_AGL + ( const.IMG_HOR_VIEW_AGL *  getXAxisOnImage(xaxis) ) / const.CAM_WHOLE_IMG_WID 
