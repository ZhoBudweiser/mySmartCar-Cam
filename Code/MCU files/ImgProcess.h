/*
 * ImgProcess.h
 *
 *  Created on: 2022年2月11日
 *      Author: Zho
 */

#ifndef CODE_IMGPROCESS_H_
#define CODE_IMGPROCESS_H_

#include "common.h"
#include "headfile.h"
#include "stdio.h"
#include "SEEKFREE_MT9V03X.h"
#define IMG_H MT9V03X_H        // 图像高
#define IMG_W MT9V03X_W        // 图像宽

void 	init_param				(Img_Param *prm); // 图像参数结构体初始化
void	display_param			(Img_Param *prm); // 显示参数
void	display_grey			(uint8 *p, Img_Param *prm, uint16 x,uint16 y, uint16 width, uint16 height);
void 	display_color(uint8 *p, Img_Param *prm, uint16 x, uint16 y, uint16 width, uint16 height);
void 	Get_Use_Image			(void);
void 	eliminate_noise		(Img_Param *prm, uint8 imageIn[IMG_H][IMG_W], uint8 imageOut[IMG_H][IMG_W]);
void 	get_bin_image 		(Img_Param *prm, uint8 mode);
uint8 conv_bin_special	(Img_Param *prm, const uint8 img[IMG_H][IMG_W], uint8 out[IMG_H][IMG_W]);
void 	Seek_Beacon				(Img_Param *prm, Domain_queue *queue_ptr);
uint8 get_connected_domain(Img_Param *prm, const uint8 img[IMG_H][IMG_W],uint8 tag[IMG_H][IMG_W]);
void	extract_domain_info(Img_Param *prm, const uint8 tag[IMG_H][IMG_W], Domain_queue *q);

// private
void 	lq_sobelAutoThreshold 	(uint8 imageIn[IMG_H][IMG_W], uint8 imageOut[IMG_H][IMG_W]);
void	lq_sobel				(uint8 imageIn[IMG_H][IMG_W], uint8 imageOut[IMG_H][IMG_W], uint8 Threshold);
short GetOSTU 				(uint8 tmImage[IMG_H][IMG_W]);
void	fill_Image			(uint8 imageIn[IMG_H][IMG_W], uint8 imageOut[IMG_H][IMG_W]);

extern Domain_queue * Get_Domain_queue(void);

#endif

