#include "headfile.h"

extern uint8 Bin_Image[IMG_H][IMG_W];

//1. 初始化顺序队列
void initialQueue(Domain_queue* Q)
{
	Q->front = 0;
	Q->rear = 0;
}

//2. 判队空
uint8 queueEmpty(Domain_queue* Q)
{
	return Q->front == Q->rear;
}

//3. 判队满
uint8 queueFull(Domain_queue* Q)
{
	return ((Q->rear+1) % CONNECTED_DOMAIN_MAXN) == Q->front;
}

//4. 取队头元素
uint8 getFront(Domain_queue* Q, Domain_info *x)
{
	if(queueEmpty(Q))
		return FALSE;
	else
	{
		*x = Q->domains[(Q->front+1) % CONNECTED_DOMAIN_MAXN];
		return TRUE;
	}
}

//5. 入队
uint8 enQueue(Domain_queue* Q, Domain_info* x)
{
	if(queueFull(Q))
		return FALSE; 
	else
	{
		Q->rear = ((Q->rear)+1) % CONNECTED_DOMAIN_MAXN;
		Q->domains[Q->rear] = *x;
		return  TRUE;
	}
}

uint8 updateQueue(Domain_queue* Q, uint8 x, uint8 y, uint8 pos, uint8 create)
{
	if (TRUE == create)
	{
		if (queueFull(Q))
			return 255;
		else
		{
			Q->rear = ((Q->rear)+1) % CONNECTED_DOMAIN_MAXN;
			Q->domains[Q->rear].valid = TRUE;
			Q->domains[Q->rear].lu.x = Q->domains[Q->rear].rd.x = Q->domains[Q->rear].core.x = x;
			Q->domains[Q->rear].lu.y = Q->domains[Q->rear].rd.y = Q->domains[Q->rear].core.y = y;
			
			Q->domains[Q->rear].area = Q->domains[Q->rear].circ = 1;
			Q->domains[Q->rear].max_lumi = Q->domains[Q->rear].min_lumi 
																	 = Q->domains[Q->rear].avg_lumi = mt9v03x_image[y][x];
			return Q->rear;
		}		
	}
	else
	{
		// 更新左上角和右下角点的位置
		if (x < Q->domains[pos].lu.x)	Q->domains[pos].lu.x = x;
		else if (x > Q->domains[pos].rd.x)	Q->domains[pos].rd.x = x;
		if (y < Q->domains[pos].lu.y)	Q->domains[pos].lu.y = y;
		else if (y > Q->domains[pos].rd.y)	Q->domains[pos].rd.y = y;
		
		// 更新重心
		Q->domains[pos].core.x += x;
		Q->domains[pos].core.y += y;
		
		// 计算面积
		Q->domains[pos].area += 1;
		
		// 计算周长
		if (0 < x && x < IMG_W-1 && 0 < y && y < IMG_H-1)
		{
			if (Bin_Image[y+1][x] == BLACK || Bin_Image[y-1][x] == BLACK 
			|| Bin_Image[y][x+1] == BLACK || Bin_Image[y][x-1] == BLACK)
			{
				Q->domains[pos].circ += 1;
			}
		}
		
		// 计算亮度
		if (mt9v03x_image[y][x] < Q->domains[pos].min_lumi)	Q->domains[pos].min_lumi = mt9v03x_image[y][x];
		else if (mt9v03x_image[y][x] > Q->domains[pos].max_lumi)	Q->domains[pos].max_lumi = mt9v03x_image[y][x];
		Q->domains[pos].avg_lumi += mt9v03x_image[y][x];
		
		return pos;
	}
	
}

//6. 出队
uint8 outQueue(Domain_queue* Q, Domain_info* x)
{
	if(queueEmpty(Q))
		return FALSE;
	else
	{
		Q->front = (Q->front+1) % CONNECTED_DOMAIN_MAXN;
		*x = Q->domains[Q->front];
	}
}
