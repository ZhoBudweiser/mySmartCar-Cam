#include "headfile.h"
#include "encoder.h"
#include "display.h"
#include "AHRS.h"
#include "ImgProcess.h"
#include "timer_pit.h"

car_control_t *car_display;
Img_Param *img_param_ptr;

extern Domain_queue domain_queue;
Domain_queue* queue_ptr = &domain_queue;

/**
  * @brief          调试时显示函数
  * @param[in]      void *
  * @retval         void
  */
void display_entry(void *parameter)
{
    while(1)
    {
			  #if TEST_IPS114
					ips114_showstr(0,0,"pitch:");
					ips114_showfloat(50,0,car_display->euler_data.pitch,3,5);
					ips114_showstr(0,1,"speed_l:");
					ips114_showint16(65,1,car_display->speed_l);
					ips114_showstr(120,1,"speed_r:");
					ips114_showint16(180,1,car_display->speed_r);
					ips114_showstr(0,2,"speed_ave:");
					ips114_showfloat(80,2,car_display->speed_ave,2,5);			
//					ips114_showstr(0,1,"x:");
//					ips114_showint16(30,1,img_param_ptr->target_pos.x - 94);				
//					ips114_showstr(120,1,"y:");
//					ips114_showint16(150,1,img_param_ptr->target_pos.y);			
//					ips114_showstr(0,2,"focus:");
//					ips114_showint16(65,2,img_param_ptr->focus);						
//					ips114_showstr(0,3,"ramp_d");
//					ips114_showfloat(65,3,car_display->ramp_angle_d,3,5);
			
//					ips114_showstr(0,3,"gyro_z:");
//					ips114_showfloat(65,3,car_display->icm_data.icm_gyro_z,2,5);		
//					ips114_showstr(0,4,"gyro_y:");
//					ips114_showfloat(65,4,car_display->icm_data.icm_gyro_y,2,5);	
//					ips114_showstr(0,5,"gyro_x:");
//					ips114_showfloat(65,5,car_display->icm_data.icm_gyro_x,2,5);				
//					ips114_showstr(0,6,"acc_x:");
//					ips114_showfloat(65,6,car_display->icm_data.icm_acc_x,2,5);	
					ips114_showstr(0,4,"distance:");
					ips114_showfloat(100,4,car_display->distance,4,4);						
					ips114_showstr(0,5,"yaw:");
					ips114_showfloat(50,5,car_display->euler_data.yaw,3,5);	
					ips114_showstr(0,6,"roll:");
					ips114_showfloat(50,6,car_display->euler_data.roll,3,5);			
					car_mode_display(0);
					ips114_showstr(50,7,"gyinte:");
					ips114_showfloat(120,7,car_display->gyro_integral,4,5);	
			  #endif
				#if TEST_IPS200
			
					char str[25];
					sprintf(str, "Target: %3d, %3d", img_param_ptr->target_pos.x, img_param_ptr->target_pos.y);
					ips200_showstr(0, 18, str);
					if (TRUE == img_param_ptr->focus)
						ips200_showstr(0, 19, "FOCUS");
					else
						ips200_showstr(0, 19, "NULL  ");
//					car_mode_display(1);
					rt_mutex_take(&queue_ptr->mutex, RT_WAITING_FOREVER);
					
//					sprintf(str, "lu: %3d, %3d", img_param_ptr->range_lu.x, img_param_ptr->range_lu.y);
//					ips200_showstr(0, 10, str);
//					sprintf(str, "rd: %3d, %3d", img_param_ptr->range_rd.x, img_param_ptr->range_rd.y);
//					ips200_showstr(0, 11, str);
					sprintf(str, "size:%3d", (queue_ptr->rear-queue_ptr->front+CONNECTED_DOMAIN_MAXN)%CONNECTED_DOMAIN_MAXN);
					ips200_showstr(0, 16, str);
					
					uint8 pos = 0;
					// 显示目标参数
					if (255 != queue_ptr->targetPos)
						pos = queue_ptr->targetPos;
					else {
						rt_mutex_release(&queue_ptr->mutex);
						continue;
					}
						
					
					// 显示面积最大的参数
//					uint16 max_area = 0;
//					for (uint8 i = queue_ptr->front+1; i != (queue_ptr->rear+1) % CONNECTED_DOMAIN_MAXN; i = (i+1) % CONNECTED_DOMAIN_MAXN) {
//						if (max_area < queue_ptr->domains[i].area) {
//							max_area = queue_ptr->domains[i].area;
//							pos = i;
//						}
//					}
//					// 显示目标参数
//					if (255 != queue_ptr->targetPos)
//						pos = queue_ptr->targetPos;
//					else
//						continue;
//					if(car_display->data_back_flag)
//						wireless_data_send(car_display,queue_ptr);						
					sprintf(str, "lu: %3d, %3d", queue_ptr->domains[pos].lu.x, queue_ptr->domains[pos].lu.y);
					ips200_showstr(0, 10, str);
					sprintf(str, "rd: %3d, %3d", queue_ptr->domains[pos].rd.x, queue_ptr->domains[pos].rd.y);
					ips200_showstr(0, 11, str);
					sprintf(str, "core: %3ld, %3ld", queue_ptr->domains[pos].core.x, queue_ptr->domains[pos].core.y);
					ips200_showstr(0, 12, str);
					sprintf(str, "area:%6d circ:%6d", queue_ptr->domains[pos].area, queue_ptr->domains[pos].circ);
					ips200_showstr(0, 13, str);
					sprintf(str, "max:%3d min:%3d", queue_ptr->domains[pos].max_lumi, queue_ptr->domains[pos].min_lumi);
					ips200_showstr(0, 14, str);

					rt_mutex_release(&queue_ptr->mutex);
					
					sprintf(str, "avg:%3ld", queue_ptr->domains[pos].avg_lumi);
					ips200_showstr(0, 15, str);
//					ips200_showfloat(0,16,car_display->euler_data.pitch,3,5);
//					rt_thread_delay(5);

					
				#endif
    }
    
}

//0为ips114显示，1为ips200显示
void car_mode_display(uint8 ips_mode)
{
		if(!ips_mode)
		{
			if(car_display->car_mode == STOP)
				ips114_showstr(0,7,"stop  ");
			else if(car_display->car_mode == FIND_READY)
				ips114_showstr(0,7,"ready ");
			else if (car_display->car_mode == FIND_READY_CONTINUE)
				ips114_showstr(0,7,"cready");
			else if(car_display->car_mode == FINDING)
				ips114_showstr(0,7,"find  ");
			else if (car_display->car_mode == LAMP_UP)
				ips114_showstr(0,7,"lamp  ");
			else if (car_display->car_mode == FIND_DONE)
				ips114_showstr(0,7,"done  ");
			else if (car_display->car_mode == Forward_TOFIND)
				ips114_showstr(0,7,"forw  ");
		}
		else
		{
			if(car_display->car_mode == STOP)
				ips200_showstr(140,10,"stop  ");
			else if(car_display->car_mode == FIND_READY)
				ips200_showstr(140,10,"ready ");
			else if (car_display->car_mode == FIND_READY_CONTINUE)
				ips200_showstr(140,10,"cready");
			else if(car_display->car_mode == FINDING)
				ips200_showstr(140,10,"find  ");
			else if (car_display->car_mode == LAMP_UP)
				ips200_showstr(140,10,"lamp  ");
			else if (car_display->car_mode == FIND_DONE)
				ips200_showstr(140,10,"done  ");
			else if (car_display->car_mode == Forward_TOFIND)
				ips200_showstr(140,10,"forw  ");
		}
}


void wireless_data_send(car_control_t * car_move, Domain_queue *queue_ptr)
{
		uint8 f = queue_ptr->front, r = queue_ptr->rear;
		int8 str[180];
		int8 str1[10];
		fp32 in[11];
		for (uint8 i = f+1; i != (r+1) % CONNECTED_DOMAIN_MAXN; i = (i+1) % CONNECTED_DOMAIN_MAXN) 
		{
			in[0] = queue_ptr->domains[i].lu.x;
			in[1] = queue_ptr->domains[i].lu.y;
			in[2] = queue_ptr->domains[i].rd.x;
			in[3] = queue_ptr->domains[i].rd.y;
			in[4] = queue_ptr->domains[i].core.x;
			in[5] = queue_ptr->domains[i].core.y;
			in[6] = queue_ptr->domains[i].area;
			in[7] = queue_ptr->domains[i].circ;
			in[8] = queue_ptr->domains[i].max_lumi;
			in[9] = queue_ptr->domains[i].min_lumi;
			in[10] = queue_ptr->domains[i].avg_lumi;
			sprintf(str,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n\r",in[0],in[1],in[2],in[3],in[4],in[5],in[6],in[7],in[8],in[9],in[10]);
			seekfree_wireless_send_buff((uint8*)str,strlen(str));			
		}			
		sprintf(str1,"\n\r");
		seekfree_wireless_send_buff((uint8*)str1,strlen(str1));	
		car_move->data_back_flag = 0;
}


void display_init(void)
{
    rt_thread_t tid;
    
    //初始化屏幕
		#if TEST_IPS114
    ips114_init();
		#endif
		#if TEST_IPS200
    ips200_init();
		#endif
    car_display = Get_car_move();
		img_param_ptr = Get_Img_move();
    //创建显示线程 优先级设置为31 
    tid = rt_thread_create("display", display_entry, RT_NULL, 1024, 31, 30);
    
    //启动显示线程
    if(RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
}