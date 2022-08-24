/*
 *  ImgProcess.c     图像处理函数
 *
 *  Created on     2022年2月11日   加入调试界面
 */

#include "ImgProcess.h"
#include "seqQueue.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "bpnn_fit.h"

uint8 Bin_Image[IMG_H][IMG_W];
uint8 temp_Image[IMG_H][IMG_W];
extern struct bpnn nn;

Domain_queue domain_queue;
double in[11];
double maxx[11] = {
	188, 120, 188, 120, 188, 120, 1000, 500, 255, 255, 255
};

uint8 lqv = 35;
unsigned short Threshold = 0;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化参数函数
//  @param      prm     图像参数结构体
//  @return     void
//  @creation date       2022年4月30日
//  Sample usage:               init_param(&prm);
//-------------------------------------------------------------------------------------------------------------------
void init_param(Img_Param *prm)
{
		prm->target_pos.x = 0;
		prm->target_pos.y = 0;
		prm->target_lu.x = 0;
		prm->target_lu.y = 0;
		prm->target_rd.x = 0;
		prm->target_rd.y = 0;
		prm->range_lu.x = 0;
		prm->range_lu.y = 0;
		prm->range_rd.x = IMG_W-1;
		prm->range_rd.y = IMG_H-1;
//		prm->range_lu.x = 50;
//		prm->range_lu.y = 50;
//		prm->range_rd.x = 75;
//		prm->range_rd.y = 75;
		prm->focus = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      消除噪点
//  @param      prm     图像参数结构体
//  @return     void
//  @creation date       2022年4月30日
//  Sample usage:               init_param(&prm);
//-------------------------------------------------------------------------------------------------------------------
void eliminate_noise(Img_Param *prm, uint8 imageIn[IMG_H][IMG_W], uint8 imageOut[IMG_H][IMG_W])
{
	unsigned short i = 0, j = 0;
	for (i = prm->range_lu.y; i <= prm->range_rd.y; i++)
	{
		for (j = prm->range_lu.x; j <= prm->range_rd.x; j++)
		{
			uint8 cnt = 0;
			if(imageIn[i][j]>=100)
			{
					cnt+=fabs(imageIn[i][j]-imageIn[i-1][j-1])>10;
					cnt+=fabs(imageIn[i][j]-imageIn[i-1][j+1])>10;
					cnt+=fabs(imageIn[i][j]-imageIn[i+1][j-1])>10;
					cnt+=fabs(imageIn[i][j]-imageIn[i+1][j+1])>10;
			}
			imageOut[i][j] = ( cnt>=4 ? (
					(imageIn[i-1][j-1]+imageIn[i-1][j]+imageIn[i-1][j+1]
					+imageIn[i][j-1]+imageIn[i][j]+imageIn[i][j+1]
					+imageIn[i+1][j-1]+imageIn[i+1][j]+imageIn[i+1][j+1])/9
					):imageIn[i][j]
			);
		}
	}
}


void get_bin_image (Img_Param *prm, uint8 mode)
{
  unsigned short i = 0, j = 0;
  unsigned short Threshold = 80;
  unsigned long tv = 0;
	
	// 去噪
	eliminate_noise(prm, mt9v03x_image, temp_Image);
	
	// 直接填充方法无效，改用新增数组
//	fill_Image(Bin_Image, mt9v03x_image);
//	memcpy(mt9v03x_image, Bin_Image, IMG_H*IMG_W*sizeof(uint8));
  
	switch (mode)
	{
		// 大津法阈值		
		case 0 : 
			Threshold = GetOSTU(mt9v03x_image);
			break;
		
		// 平均阈值
		case 1:
			//累加
			for (i = 0; i < IMG_H; i++)
				for (j = 0; j < IMG_W; j++)
					tv += mt9v03x_image[i][j];   //累加
			Threshold = (unsigned short)(tv / IMG_H / IMG_W);   //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
			Threshold = Threshold + 20;      //此处阈值设置，根据环境的光线来设定
			break;
			
		// 联合边缘检测的二值化方法
		case 2:
//    lq_sobel(mt9v03x_image, Bin_Image, (uint8) Threshold);
			conv_bin_special(prm, temp_Image, Bin_Image);
			break;
		
		// 动态调节阈值
		case 3:
			lq_sobelAutoThreshold(mt9v03x_image, Bin_Image);
			return;
	}
	
	// 二值化
		for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
			for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
			Bin_Image[i][j] = (Bin_Image[i][j] > Threshold) ?  255 : 0;
	
}

uint8 conv_bin_special(Img_Param *prm, const uint8 img[IMG_H][IMG_W], uint8 out[IMG_H][IMG_W])
{
		// 初始化输出图像
		for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
			for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
           out[i][j] = 0XC0;
    for(uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++) out[i][0] &= 0XBF, out[i][IMG_W-1] &= 0XBF;
    for(uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++) out[0][j] &= 0XBF, out[IMG_H-1][j] &= 0XBF;

		// 边沿检测
		for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
			for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
			{
					if(img[i][j] <= 15)
							out[i][j] &= 0X7F;
					// x轴边缘检测
					int32 cx = img[i-1][j-1]     -  img[i-1][j+1]
									 +(img[i  ][j-1]<<1) - (img[i  ][j+1]<<1)
									 + img[i+1][j-1]     -  img[i+1][j+1];
					// y轴边缘检测
					int32 cy =  img[i-1][j-1] 	 + (img[i-1][j]<<1)
									 +  img[i-1][j+1] 	 -  img[i+1][j-1]
									 - (img[i+1][j]<<1)	 -	img[i+1][j+1];
					// 主对角线边缘检测
					int32 cz = (img[i-1][j-1]<<1)+	img[i-1][j]
									 +  img[i  ][j-1]		 - 	img[i  ][j+1]
									 -	img[i+1][j]			 - (img[i+1][j+1]<<1);
					// 副对角线边缘检测
					int32 cf =  img[i-1][j]			 + (img[i-1][j+1]<<1)
									 - 	img[i  ][j-1]		 +  img[i  ][j+1]
									 - (img[i+1][j-1]<<1)-	img[i+1][j];
					
					if(fabs(cx)>=50)out[i][j+(cx>0?1:-1)]&=0XBF;
					if(fabs(cy)>=60)out[i+(cy>0?1:-1)][j]&=0XBF;
					if(fabs(cz)>=35)out[i+(cz>0?1:-1)][j+(cz>0?1:-1)]&=0XBF;
					if(fabs(cf)>=35)out[i+(cf>0?1:-1)][j+(cf>0?-1:1)]&=0XBF;
					if((cx*cx+cy*cy)>(65*65+75*75))out[i][j]|=0X20;
			}
    return TRUE;
}


// 初始化各个结点为下标
void scns_bcj_init(uint8 node[],uint8 n)
{
    for(uint8 i = 0; i < n; node[i] = i, ++i);
}

// 找到根结点
uint8 scns_bcj_find(uint8 node[],uint8 x)
{
    if(x == node[x]) 
        return x; 
    return node[x] = scns_bcj_find(node, node[x]);
}

// 合并两个连通域
void scns_bcj_unite(uint8 node[],uint8 x,uint8 y)
{
		// 找到连通域
    x = scns_bcj_find(node, x);
    y = scns_bcj_find(node, y);
    if (x == y) return; 
    if (x > y)
        node[x] = y;
    else
        node[y] = x;
}


uint8 get_connected_domain(Img_Param *prm, const uint8 img[IMG_H][IMG_W],uint8 tag[IMG_H][IMG_W])
{
    uint8 bcj[CONNECTED_DOMAIN_MAXN], labs = 1;
    uint8 gc_cnt = 0;
	
    scns_bcj_init(bcj, CONNECTED_DOMAIN_MAXN);
	
		for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
			for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
				tag[i][j] = 255;
		for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
			for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
					// 找到有效的像素点
					if (img[i][j] >= 0XC0)
					{
							uint8 minn = 255;
							minn = min(minn, tag[i-1][j-1]);	//	左上
							minn = min(minn, tag[i-1][j  ]);	//	正上
							minn = min(minn, tag[i-1][j+1]);	//	右上
							minn = min(minn, tag[i  ][j-1]);	//	左侧
							// 在其他的连通域中
							if (minn != 255)
							{
									tag[i][j] = minn;
									// 合并其他域
									if (tag[i-1][j-1] != 255) scns_bcj_unite(bcj, tag[i-1][j-1], minn);
									if (tag[i-1][j  ] != 255) scns_bcj_unite(bcj, tag[i-1][j  ], minn);
									if (tag[i-1][j+1] != 255) scns_bcj_unite(bcj, tag[i-1][j+1], minn);
									if (tag[i  ][j-1] != 255) scns_bcj_unite(bcj, tag[i  ][j-1], minn);
							}
							// 全部都是不连通的
							else
							{
									// 新建连通域
									tag[i][j] = labs;
									++labs;
									if (labs >= CONNECTED_DOMAIN_MAXN)
									{
											++gc_cnt;
											// 超过128个连通域（共255+128）
											if (gc_cnt > 128)
												return 255;
											
											uint8 mapp[CONNECTED_DOMAIN_MAXN], mapp_cnt=0;
											
											for (uint8 i = 0; i < CONNECTED_DOMAIN_MAXN; ++i) mapp[i] = 255;
											// 垃圾回收
											for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
												for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
															// 找到一个连通域
															if (tag[i][j] != 255)
															{
																	// 找根结点
																	uint8 a = scns_bcj_find(bcj, tag[i][j]);
																	if (mapp[a] == 255)
																	{
																			// 重新从最低序号开始赋值
																			mapp[a] = mapp_cnt;
																			++mapp_cnt;
																	}
																	tag[i][j] = mapp[a];
															}
											labs = mapp_cnt;
											if (labs >= CONNECTED_DOMAIN_MAXN)
												return 255;
											// 重新初始化，重新开始编号
											scns_bcj_init(bcj, CONNECTED_DOMAIN_MAXN);
									}
							}
					}
		for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
			for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
            if (tag[i][j] != 255)
								// 更新为根结点
                tag[i][j] = scns_bcj_find(bcj, tag[i][j]);
    return labs;
}

 void extract_domain_info(Img_Param *prm, const uint8 tag[IMG_H][IMG_W], Domain_queue *q)
 {
		uint8 labmap[CONNECTED_DOMAIN_MAXN];
		for (uint8 i = 0; i < CONNECTED_DOMAIN_MAXN; i++)
			labmap[i] = CONNECTED_DOMAIN_MAXN;
	 
		initialQueue(q);
	  rt_mutex_take(&q->mutex, RT_WAITING_FOREVER);
		for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
			for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
				if (CONNECTED_DOMAIN_MAXN != tag[i][j]) {
					if (CONNECTED_DOMAIN_MAXN == labmap[tag[i][j]]) {
						labmap[tag[i][j]] = updateQueue(q, j, i, labmap[tag[i][j]], TRUE);
					} else {
						updateQueue(q, j, i, labmap[tag[i][j]], FALSE);
					}
				}
		for (uint8 i = q->front+1; i != (q->rear+1) % CONNECTED_DOMAIN_MAXN; i = (i+1) % CONNECTED_DOMAIN_MAXN) {
			q->domains[i].avg_lumi = q->domains[i].avg_lumi / q->domains[i].area;
			q->domains[i].core.x = q->domains[i].core.x / q->domains[i].area;
			q->domains[i].core.y = q->domains[i].core.y / q->domains[i].area;
		}
		rt_mutex_release(&q->mutex);
 }

 void adjust_range(Img_Param *prm)
{
	int diff = IMG_H - prm->target_pos.y + 8;
//	int diff = 10;
	int zoom1 = 8;
	int zoom2 = 8;
	
	if (prm->target_lu.x > (prm->target_rd.x - prm->target_pos.x) * (diff / zoom1))
		prm->range_lu.x = prm->target_lu.x - (prm->target_rd.x - prm->target_pos.x) * (diff / zoom1);
	else
		prm->range_lu.x = 0;
	
	if (prm->target_lu.y > (prm->target_rd.y - prm->target_pos.y) * (diff / zoom2))
		prm->range_lu.y = prm->target_lu.y - (prm->target_rd.y - prm->target_pos.y) * (diff / zoom2);
	else
		prm->range_lu.y = 0;
	
	if (prm->target_rd.x - (prm->target_lu.x - prm->target_pos.x) * (diff / zoom1) < IMG_W)
		prm->range_rd.x = prm->target_rd.x - (prm->target_lu.x - prm->target_pos.x) * (diff / zoom1);
	else
		prm->range_rd.x = IMG_W-1;
	
	if (prm->target_rd.y - (prm->target_lu.y - prm->target_pos.y) * (diff / zoom2) < IMG_H)
		prm->range_rd.y = prm->target_rd.y - (prm->target_lu.y - prm->target_pos.y) * (diff / zoom2);
	else
		prm->range_rd.y = IMG_H-1;
}

void reset_range(Img_Param *prm)
{
	prm->range_lu.x = 0;
	prm->range_lu.y = 0;
	prm->range_rd.x = IMG_W-1;
	prm->range_rd.y = IMG_H-1;
}
 
/*************************************************************************
*  函数名称：void Seek_Beacon (void)
*  功能说明：识别信标灯
*  参数说明prm     图像参数结构体指针
*  函数返回：无
*************************************************************************/
void Seek_Beacon(Img_Param *prm, Domain_queue *queue_ptr)
{
	T bpnn = &nn;
	double maxRes = 0.0;
	uint8 maxPos = 0;
	
	rt_mutex_take(&queue_ptr->mutex, RT_WAITING_FOREVER);
	
	for (uint8 i = queue_ptr->front+1; i != (queue_ptr->rear+1) % CONNECTED_DOMAIN_MAXN; i = (i+1) % CONNECTED_DOMAIN_MAXN) {
		in[0] = queue_ptr->domains[i].lu.x *1.0 / maxx[0];
		in[1] = queue_ptr->domains[i].lu.y *1.0 / maxx[1];
		in[2] = queue_ptr->domains[i].rd.x *1.0 / maxx[2];
		in[3] = queue_ptr->domains[i].rd.y *1.0 / maxx[3];
		in[4] = queue_ptr->domains[i].core.x *1.0 / maxx[4];
		in[5] = queue_ptr->domains[i].core.y *1.0 / maxx[5];
		in[6] = queue_ptr->domains[i].area *1.0 / maxx[6];
		in[7] = queue_ptr->domains[i].circ *1.0 / maxx[7];
		in[8] = queue_ptr->domains[i].max_lumi *1.0 / maxx[8];
		in[9] = queue_ptr->domains[i].min_lumi *1.0 / maxx[9];
		in[10] = queue_ptr->domains[i].avg_lumi *1.0 / maxx[10];
		double res = bpnn_fit(bpnn, in);
		if (maxRes < res) {
			maxRes = res;
			maxPos = i;
		}
		if (res < 0.50)
			queue_ptr->domains[i].valid = FALSE;
		else {
			queue_ptr->domains[i].valid = TRUE;
		}
	}
	
	if (maxRes>0.50) {
    prm->target_pos.x = queue_ptr->domains[maxPos].core.x;      // 灯的左右中心点
    prm->target_pos.y = queue_ptr->domains[maxPos].core.y;     // 灯的远近中心点，暂时没用到
		prm->target_lu.x = queue_ptr->domains[maxPos].lu.x;
		prm->target_lu.y = queue_ptr->domains[maxPos].lu.y;
		prm->target_rd.x = queue_ptr->domains[maxPos].rd.x;
		prm->target_rd.y = queue_ptr->domains[maxPos].rd.y;
		
		adjust_range(prm);
		
		prm->focus = TRUE;
		queue_ptr->targetPos = maxPos;
	} else {
		prm->focus = FALSE;
		
		if(maxRes > 0.3) {
			prm->target_pos.x = queue_ptr->domains[maxPos].core.x;      // 灯的左右中心点
			prm->target_pos.y = queue_ptr->domains[maxPos].core.y;     // 灯的远近中心点，暂时没用到
			prm->target_lu.x = queue_ptr->domains[maxPos].lu.x;
			prm->target_lu.y = queue_ptr->domains[maxPos].lu.y;
			prm->target_rd.x = queue_ptr->domains[maxPos].rd.x;
			prm->target_rd.y = queue_ptr->domains[maxPos].rd.y;
			adjust_range(prm);
		} else {
			reset_range(prm);
		}
		queue_ptr->targetPos = 255;
	}
	
	rt_mutex_release(&queue_ptr->mutex);

  return;
}

/**
  * @brief          获取Domain_queue指针，以便各文件中使用
  * @param[in]      void
  * @retval         Domain_queue *
  */
Domain_queue * Get_Domain_queue(void)
{
	
		return &domain_queue;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      参数显示函数
//  @param      prm     图像参数结构体
//  @return     void
//  @creation date       2021年5月13日
//  Sample usage:               display_param(&prm);
//-------------------------------------------------------------------------------------------------------------------
void display_param(Img_Param *prm)
{
		char str[20];
		sprintf(str, "Target: %3d, %3d", prm->target_pos.x, prm->target_pos.y);
		ips200_showstr(0, 10, str);
		if (TRUE == prm->focus)
			ips200_showstr(0, 20, "FOCUS");
		else
			ips200_showstr(0, 20, "     ");
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      在屏幕上显示图像
//  @param      *Img     图像入口地址
//  @return     void
//  @creation date       2021年6月6日
//  Sample usage:               display_image(Img[0]);
//-------------------------------------------------------------------------------------------------------------------
void display_grey(uint8 *p, Img_Param *prm, uint16 x, uint16 y, uint16 width, uint16 height)
{
		uint16 i,j;
								
		uint16 color = 0;
		uint16 temp = 0;

		uint16 coord_x = 0;
		uint16 coord_y = 0;

		coord_x = width>IPS200_X_MAX?IPS200_X_MAX:width;
		coord_y = height>IPS200_Y_MAX?IPS200_Y_MAX:height;
		ips200_address_set(x,y,x+coord_x-1,y+coord_y-1);

		for(j=0;j<coord_y;j++)
		{
				for(i=0;i<coord_x;i++)
				{
            if (j == prm->target_pos.y || i == prm->target_pos.x)
                ips200_wr_data16(RED);
						else
						{
							temp = *(p+j*width+i*width/coord_x);//读取像素点
							color=(0x001f&((temp)>>3))<<11;
							color=color|(((0x003f)&((temp)>>2))<<5);
							color=color|(0x001f&((temp)>>3));
							ips200_wr_data16(color); 							
						}

				}
		}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      在屏幕上显示图像
//  @param      *Img     图像入口地址
//  @return     void
//  @creation date       2021年6月6日
//  Sample usage:               display_image(Img[0]);
//-------------------------------------------------------------------------------------------------------------------
void display_color(uint8 *p, Img_Param *prm, uint16 x, uint16 y, uint16 width, uint16 height)
{
		uint16 i,j;
	
		const uint8 size = 6;								
		uint16 color[size] = {BLUE, YELLOW, GREEN, BROWN, PURPLE, PINK};

		uint16 temp = 0;

		uint16 coord_x = 0;
		uint16 coord_y = 0;

		coord_x = width>IPS200_X_MAX?IPS200_X_MAX:width;
		coord_y = height>IPS200_Y_MAX?IPS200_Y_MAX:height;
		ips200_address_set(x,y,x+coord_x-1,y+coord_y-1);

//		uint8 pos = domain_queue.targetPos;
		
		for(j=0;j<coord_y;j++)
		{
				for(i=0;i<coord_x;i++)
				{
						if (TRUE == prm->focus) {
							if (j == prm->target_pos.y 
								|| i == prm->target_pos.x)
									ips200_wr_data16(RED);
							// 目标外框
							else if (j == prm->target_lu.y && i >= prm->target_lu.x
								&& i <= prm->target_rd.x)
								ips200_wr_data16(RED);
							else if (j == prm->target_rd.y && i >= prm->target_lu.x
								&& i <= prm->target_rd.x)
								ips200_wr_data16(RED);
							else if (i == prm->target_lu.x && j >= prm->target_lu.y
								&& j <= prm->target_rd.y)
								ips200_wr_data16(RED);
							else if (i == prm->target_rd.x && j >= prm->target_lu.y
								&& j <= prm->target_rd.y)
								ips200_wr_data16(RED);
							// 识别范围
							else if (j == prm->range_lu.y && i >= prm->range_lu.x
								&& i <= prm->range_rd.x)
								ips200_wr_data16(PINK);
							else if (j == prm->range_rd.y && i >= prm->range_lu.x
								&& i <= prm->range_rd.x)
								ips200_wr_data16(PINK);
							else if (i == prm->range_lu.x && j >= prm->range_lu.y
								&& j <= prm->range_rd.y)
								ips200_wr_data16(PINK);
							else if (i == prm->range_rd.x && j >= prm->range_lu.y
								&& j <= prm->range_rd.y)
								ips200_wr_data16(PINK);
							else
							{
								temp = *(p+j*width+i*width/coord_x);	//读取像素点
								if (255 == temp)
								{
									ips200_wr_data16(GRAY);
								}
								else
								{
									ips200_wr_data16(color[temp%size]);
								}
							}							
						}
						else
						{
							temp = *(p+j*width+i*width/coord_x);	//读取像素点
							if (255 == temp)
							{
								ips200_wr_data16(GRAY);
							}
							else
							{
								ips200_wr_data16(color[temp%size]);
							}
						}

				}
		}
}

void fill_Image(uint8 imageIn[IMG_H][IMG_W], uint8 imageOut[IMG_H][IMG_W])
{
	unsigned short i = 0, j = 0;
	for (i = 0; i < IMG_H; i++)
	{
		for (j = 0; j < IMG_W; j++)
		{
			imageOut[i][j] = imageIn[i][j];
		}
	}
}

short GetOSTU (uint8 tmImage[IMG_H][IMG_W])
{
  signed short i, j;
  unsigned long Amount = 0;
  unsigned long PixelBack = 0;
  unsigned long PixelshortegralBack = 0;
  unsigned long Pixelshortegral = 0;
  signed long PixelshortegralFore = 0;
  signed long PixelFore = 0;
  float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
  signed short MinValue, MaxValue;
  signed short Threshold = 0;
  uint8 HistoGram[256];              //
  
  for (j = 0; j < 256; j++)
    HistoGram[j] = 0; //初始化灰度直方图
  
  for (j = 0; j < IMG_H; j++)
  {
    for (i = 0; i < IMG_W; i++)
    {
      HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
    }
  }
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值
  
  if (MaxValue == MinValue)
    return MaxValue;         // 图像中只有一个颜色
  if (MinValue + 1 == MaxValue)
    return MinValue;        // 图像中只有二个颜色
  
  for (j = MinValue; j <= MaxValue; j++)
    Amount += HistoGram[j];        //  像素总数
  
  Pixelshortegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    Pixelshortegral += HistoGram[j] * j;        //灰度值总数
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];     //前景像素点数
    PixelFore = Amount - PixelBack;           //背景像素点数
    OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
    OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
    PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
    PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
    MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
    MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
    if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //返回最佳阈值;
}

void lq_sobel (uint8 imageIn[IMG_H][IMG_W], uint8 imageOut[IMG_H][IMG_W], uint8 Threshold)
{
  //* 卷积核大小 //
  short KERNEL_SIZE = 3;
  short xStart = KERNEL_SIZE / 2;
  short xEnd = IMG_W - KERNEL_SIZE / 2;
  short yStart = KERNEL_SIZE / 2;
  short yEnd = IMG_H - KERNEL_SIZE / 2;
  short i, j, k;
  short temp[4];
  for (i = yStart; i < yEnd; i++)
  {
    for (j = xStart; j < xEnd; j++)
    {
      // 计算不同方向梯度幅值  //
      temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
        - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]        // {-1, 0, 1},
          - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};
      
      temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
        - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};
      
      temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
        - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0
      
      temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
        - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
          - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1
      
      temp[0] = fabs(temp[0]);
      temp[1] = fabs(temp[1]);
      temp[2] = fabs(temp[2]);
      temp[3] = fabs(temp[3]);
      
      // 找出梯度幅值最大值  //
      for (k = 1; k < 4; k++)
      {
        if (temp[0] < temp[k])
        {
          temp[0] = temp[k];
        }
      }
      
      if (temp[0] > Threshold)
      {
        imageOut[i][j] = 255;
      }
      else
      {
        imageOut[i][j] = 0;
      }
    }
  }
}

void lq_sobelAutoThreshold (uint8 imageIn[IMG_H][IMG_W], uint8 imageOut[IMG_H][IMG_W])
{
  //* 卷积核大小 //
  short KERNEL_SIZE = 3;
  short xStart = KERNEL_SIZE / 2;
  short xEnd   = IMG_W - KERNEL_SIZE / 2;
  short yStart = KERNEL_SIZE / 2;
  short yEnd   = IMG_H - KERNEL_SIZE / 2;
  short i, j, k;
  short temp[4];
  for (i = yStart; i < yEnd; i++)
  {
    for (j = xStart; j < xEnd; j++)
    {
      // 计算不同方向梯度幅值  //
      temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
        - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]       // {-1, 0, 1},
          - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};
      
      temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
        - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};
      
      temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
        - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
          - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0
      
      temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
        - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
          - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1
      
      temp[0] = fabs(temp[0]);
      temp[1] = fabs(temp[1]);
      temp[2] = fabs(temp[2]);
      temp[3] = fabs(temp[3]);
      
      // 找出梯度幅值最大值  //
      for (k = 1; k < 4; k++)
      {
        if (temp[0] < temp[k])
        {
          temp[0] = temp[k];
        }
      }
      
      // 使用像素点邻域内像素点之和的一定比例    作为阈值  //
      temp[3] = (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
        + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) imageIn[i][j + 1]
          + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];
      
      if (temp[0] > temp[3] / 12.0f)
      {
        imageOut[i][j] = 255;
      }
      else
      {
        imageOut[i][j] = 0;
      }
      
    }
  }
}

