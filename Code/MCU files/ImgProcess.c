/*
 *  ImgProcess.c     ͼ������
 *
 *  Created on     2022��2��11��   ������Խ���
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
//  @brief      ��ʼ����������
//  @param      prm     ͼ������ṹ��
//  @return     void
//  @creation date       2022��4��30��
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
//  @brief      �������
//  @param      prm     ͼ������ṹ��
//  @return     void
//  @creation date       2022��4��30��
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
	
	// ȥ��
	eliminate_noise(prm, mt9v03x_image, temp_Image);
	
	// ֱ����䷽����Ч��������������
//	fill_Image(Bin_Image, mt9v03x_image);
//	memcpy(mt9v03x_image, Bin_Image, IMG_H*IMG_W*sizeof(uint8));
  
	switch (mode)
	{
		// �����ֵ		
		case 0 : 
			Threshold = GetOSTU(mt9v03x_image);
			break;
		
		// ƽ����ֵ
		case 1:
			//�ۼ�
			for (i = 0; i < IMG_H; i++)
				for (j = 0; j < IMG_W; j++)
					tv += mt9v03x_image[i][j];   //�ۼ�
			Threshold = (unsigned short)(tv / IMG_H / IMG_W);   //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100
			Threshold = Threshold + 20;      //�˴���ֵ���ã����ݻ����Ĺ������趨
			break;
			
		// ���ϱ�Ե���Ķ�ֵ������
		case 2:
//    lq_sobel(mt9v03x_image, Bin_Image, (uint8) Threshold);
			conv_bin_special(prm, temp_Image, Bin_Image);
			break;
		
		// ��̬������ֵ
		case 3:
			lq_sobelAutoThreshold(mt9v03x_image, Bin_Image);
			return;
	}
	
	// ��ֵ��
		for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
			for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
			Bin_Image[i][j] = (Bin_Image[i][j] > Threshold) ?  255 : 0;
	
}

uint8 conv_bin_special(Img_Param *prm, const uint8 img[IMG_H][IMG_W], uint8 out[IMG_H][IMG_W])
{
		// ��ʼ�����ͼ��
		for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
			for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
           out[i][j] = 0XC0;
    for(uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++) out[i][0] &= 0XBF, out[i][IMG_W-1] &= 0XBF;
    for(uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++) out[0][j] &= 0XBF, out[IMG_H-1][j] &= 0XBF;

		// ���ؼ��
		for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
			for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
			{
					if(img[i][j] <= 15)
							out[i][j] &= 0X7F;
					// x���Ե���
					int32 cx = img[i-1][j-1]     -  img[i-1][j+1]
									 +(img[i  ][j-1]<<1) - (img[i  ][j+1]<<1)
									 + img[i+1][j-1]     -  img[i+1][j+1];
					// y���Ե���
					int32 cy =  img[i-1][j-1] 	 + (img[i-1][j]<<1)
									 +  img[i-1][j+1] 	 -  img[i+1][j-1]
									 - (img[i+1][j]<<1)	 -	img[i+1][j+1];
					// ���Խ��߱�Ե���
					int32 cz = (img[i-1][j-1]<<1)+	img[i-1][j]
									 +  img[i  ][j-1]		 - 	img[i  ][j+1]
									 -	img[i+1][j]			 - (img[i+1][j+1]<<1);
					// ���Խ��߱�Ե���
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


// ��ʼ���������Ϊ�±�
void scns_bcj_init(uint8 node[],uint8 n)
{
    for(uint8 i = 0; i < n; node[i] = i, ++i);
}

// �ҵ������
uint8 scns_bcj_find(uint8 node[],uint8 x)
{
    if(x == node[x]) 
        return x; 
    return node[x] = scns_bcj_find(node, node[x]);
}

// �ϲ�������ͨ��
void scns_bcj_unite(uint8 node[],uint8 x,uint8 y)
{
		// �ҵ���ͨ��
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
					// �ҵ���Ч�����ص�
					if (img[i][j] >= 0XC0)
					{
							uint8 minn = 255;
							minn = min(minn, tag[i-1][j-1]);	//	����
							minn = min(minn, tag[i-1][j  ]);	//	����
							minn = min(minn, tag[i-1][j+1]);	//	����
							minn = min(minn, tag[i  ][j-1]);	//	���
							// ����������ͨ����
							if (minn != 255)
							{
									tag[i][j] = minn;
									// �ϲ�������
									if (tag[i-1][j-1] != 255) scns_bcj_unite(bcj, tag[i-1][j-1], minn);
									if (tag[i-1][j  ] != 255) scns_bcj_unite(bcj, tag[i-1][j  ], minn);
									if (tag[i-1][j+1] != 255) scns_bcj_unite(bcj, tag[i-1][j+1], minn);
									if (tag[i  ][j-1] != 255) scns_bcj_unite(bcj, tag[i  ][j-1], minn);
							}
							// ȫ�����ǲ���ͨ��
							else
							{
									// �½���ͨ��
									tag[i][j] = labs;
									++labs;
									if (labs >= CONNECTED_DOMAIN_MAXN)
									{
											++gc_cnt;
											// ����128����ͨ�򣨹�255+128��
											if (gc_cnt > 128)
												return 255;
											
											uint8 mapp[CONNECTED_DOMAIN_MAXN], mapp_cnt=0;
											
											for (uint8 i = 0; i < CONNECTED_DOMAIN_MAXN; ++i) mapp[i] = 255;
											// ��������
											for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
												for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
															// �ҵ�һ����ͨ��
															if (tag[i][j] != 255)
															{
																	// �Ҹ����
																	uint8 a = scns_bcj_find(bcj, tag[i][j]);
																	if (mapp[a] == 255)
																	{
																			// ���´������ſ�ʼ��ֵ
																			mapp[a] = mapp_cnt;
																			++mapp_cnt;
																	}
																	tag[i][j] = mapp[a];
															}
											labs = mapp_cnt;
											if (labs >= CONNECTED_DOMAIN_MAXN)
												return 255;
											// ���³�ʼ�������¿�ʼ���
											scns_bcj_init(bcj, CONNECTED_DOMAIN_MAXN);
									}
							}
					}
		for (uint8 i = prm->range_lu.y; i <= prm->range_rd.y; i++)
			for (uint8 j = prm->range_lu.x; j <= prm->range_rd.x; j++)
            if (tag[i][j] != 255)
								// ����Ϊ�����
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
*  �������ƣ�void Seek_Beacon (void)
*  ����˵����ʶ���ű��
*  ����˵���pprm     ͼ������ṹ��ָ��
*  �������أ���
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
    prm->target_pos.x = queue_ptr->domains[maxPos].core.x;      // �Ƶ��������ĵ�
    prm->target_pos.y = queue_ptr->domains[maxPos].core.y;     // �Ƶ�Զ�����ĵ㣬��ʱû�õ�
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
			prm->target_pos.x = queue_ptr->domains[maxPos].core.x;      // �Ƶ��������ĵ�
			prm->target_pos.y = queue_ptr->domains[maxPos].core.y;     // �Ƶ�Զ�����ĵ㣬��ʱû�õ�
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
  * @brief          ��ȡDomain_queueָ�룬�Ա���ļ���ʹ��
  * @param[in]      void
  * @retval         Domain_queue *
  */
Domain_queue * Get_Domain_queue(void)
{
	
		return &domain_queue;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������ʾ����
//  @param      prm     ͼ������ṹ��
//  @return     void
//  @creation date       2021��5��13��
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
//  @brief      ����Ļ����ʾͼ��
//  @param      *Img     ͼ����ڵ�ַ
//  @return     void
//  @creation date       2021��6��6��
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
							temp = *(p+j*width+i*width/coord_x);//��ȡ���ص�
							color=(0x001f&((temp)>>3))<<11;
							color=color|(((0x003f)&((temp)>>2))<<5);
							color=color|(0x001f&((temp)>>3));
							ips200_wr_data16(color); 							
						}

				}
		}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����Ļ����ʾͼ��
//  @param      *Img     ͼ����ڵ�ַ
//  @return     void
//  @creation date       2021��6��6��
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
							// Ŀ�����
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
							// ʶ��Χ
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
								temp = *(p+j*width+i*width/coord_x);	//��ȡ���ص�
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
							temp = *(p+j*width+i*width/coord_x);	//��ȡ���ص�
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
  float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
  signed short MinValue, MaxValue;
  signed short Threshold = 0;
  uint8 HistoGram[256];              //
  
  for (j = 0; j < 256; j++)
    HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ
  
  for (j = 0; j < IMG_H; j++)
  {
    for (i = 0; i < IMG_W; i++)
    {
      HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    }
  }
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //��ȡ��С�Ҷȵ�ֵ
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //��ȡ���Ҷȵ�ֵ
  
  if (MaxValue == MinValue)
    return MaxValue;         // ͼ����ֻ��һ����ɫ
  if (MinValue + 1 == MaxValue)
    return MinValue;        // ͼ����ֻ�ж�����ɫ
  
  for (j = MinValue; j <= MaxValue; j++)
    Amount += HistoGram[j];        //  ��������
  
  Pixelshortegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    Pixelshortegral += HistoGram[j] * j;        //�Ҷ�ֵ����
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];     //ǰ�����ص���
    PixelFore = Amount - PixelBack;           //�������ص���
    OmegaBack = (float) PixelBack / Amount;   //ǰ�����ذٷֱ�
    OmegaFore = (float) PixelFore / Amount;   //�������ذٷֱ�
    PixelshortegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
    PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //�����Ҷ�ֵ
    MicroBack = (float) PixelshortegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
    MicroFore = (float) PixelshortegralFore / PixelFore;   //�����ҶȰٷֱ�
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //������䷽��
    if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
    {
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return Threshold;                        //���������ֵ;
}

void lq_sobel (uint8 imageIn[IMG_H][IMG_W], uint8 imageOut[IMG_H][IMG_W], uint8 Threshold)
{
  //* ����˴�С //
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
      // ���㲻ͬ�����ݶȷ�ֵ  //
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
      
      // �ҳ��ݶȷ�ֵ���ֵ  //
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
  //* ����˴�С //
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
      // ���㲻ͬ�����ݶȷ�ֵ  //
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
      
      // �ҳ��ݶȷ�ֵ���ֵ  //
      for (k = 1; k < 4; k++)
      {
        if (temp[0] < temp[k])
        {
          temp[0] = temp[k];
        }
      }
      
      // ʹ�����ص����������ص�֮�͵�һ������    ��Ϊ��ֵ  //
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

