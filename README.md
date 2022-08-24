# 一颗摄像头的成长之路：第十七届智能车大赛



### 大赛信息

组别与任务：

<img src="C:\Users\hw\AppData\Roaming\Typora\typora-user-images\image-20211211193844296.png" alt="image-20211211193844296" style="zoom: 67%;" />

芯片、车模、传感器：

- 芯片：MindMotion 微控制器

  - MM32SPIN27PS 96MHz M0
  - MM32F3277G9P120MHz M3

- 车模：D/E 车模，外形尺寸无限制

  - D车模：RS-380电机

    <img src="C:\Users\hw\AppData\Roaming\Typora\typora-user-images\image-20211211200043229.png" alt="image-20211211200043229" style="zoom:67%;" />

  - E车模：RS-380电机

    <img src="C:\Users\hw\AppData\Roaming\Typora\typora-user-images\image-20211211195944180.png" alt="image-20211211195944180" style="zoom: 67%;" />

- 传感器选择：

  - 摄像头 80*60、
  - CCD
  - 光电管等



### 摄像头方案初步

1. 无法利用电磁信号的导航的方案
2. 红外光和红光必须选择其一：发光的红外光进行引导，并在摄像头前加装**红外滤光片**
3. 双摄像头同时各45帧的接收帧率
4. 若为信标灯，则通过得到信标灯的上下左右点求出其**中点**坐标
5. 当信标灯显示不全时，需要将图像边缘的信标灯进行**反推**，按照椭圆特征进行图像外**补全**，随后再进行坐标分析
6. 主模式用于正常灭灯，有**三级**速度控制，包括<u>启动，加速及减速环节</u>，启动环节避免车模撞击蓝色灯罩，减速环节避免车模冲过信标灯而增加路程，从而有“点灯”效果。寻灯模式用于寻灯，根据预测结果决定本次转向。停车模式和主模式的减速环节类似，距离信标灯一定距离时<u>开始减速直至看不到灯为止</u>。电量充足时，系统主要在主模式和寻灯模式之间切换，看到灯则切入主模式，看不到灯切入寻灯模式。
7. 摄像头拍到的灰度图像通过 DMA传输。信标灯产生的光是闪烁的，因此需要对拍到的图像进行稳定，不然车在朝着信标灯运动时会剧烈抖动，我们采用丢弃帧的方法，对一定帧数之内的丢灯情况进行忽略，继而得到较为稳定的图像。再对稳定之后的图像进行二值化处理。寻找图像中白色区域的最小外接矩形并进行判断，提取灯的中心。通过计算灯的位置，计算偏差和距离，驱动小车朝着灯的中心直线冲过去。
8. 可能存在的问题：
   1. 离灯太近时摄像头看到的灯图像占比过大甚至看得太清楚看到信标灯板上的灯珠影像；导致离灯近时出现判灯出错，导致异常转向之类的问题，这里需要计算灯的中心位置，然后再锁定灯中心并完成转向。
   2. 灭灯后寻找下一个灯的逻辑控制；涉及到灭灯后如何快速寻找下一个灯，这一个阶段通常会耗去大量时间，简单的可以在视野中丢失灯时进行顺时针或者逆时针转向搜索，但一定要记住按照一定间隔退出绕圈向其他位置行走然后再进行绕圈搜灯，否则容易出现距离灯太远无法准确识别导致一直原地转向。
   3. 冲的太快踩灯起飞落地失控。关乎是否会被判定违反规则，当速度提升到踩灯落地后出现姿态震荡或者出现转向震荡是就需要处理了，可以适当检测距离灯的距离，在固定距离减速上灯，适当减弱姿态和转向的控制强度，允许偏离目标姿态一定范围，通过PID过渡调整恢复，防止出现姿态控制震荡。


<img src="C:\Users\hw\AppData\Roaming\Typora\typora-user-images\image-20220203103120723.png" alt="image-20220203103120723" style="zoom: 50%;" />



<img src="C:\Users\hw\AppData\Roaming\Typora\typora-user-images\image-20220202111908627.png" alt="image-20220202111908627" style="zoom:50%;" />



<img src="C:\Users\hw\AppData\Roaming\Typora\typora-user-images\image-20220202112508485.png" alt="image-20220202112508485" style="zoom: 50%;" />



### 图像预处理

#### 图像采集

首先开启**场中断**，接着启动 **DMA 中断**；当场中断触发时，DMA 中断用于**传输数据**；DMA中断的触发信号为**PCLK上升沿**，最后将数据传到设定的数组，每次传输**一个像素**。<u>当 DMA 传输停止，场中断关闭时，说明图像采集完毕</u>，当要采集下一张图像时重复上述步骤即可。

MT9V034的寄存器宽度都是**16bit**，如果程序或者硬件只支持8bit时序，需要采用特殊的读写方法。它的特殊寄存器就是**0xF0**，用于存放使用8bit时序读写芯片时，数据的低8位。也就是写16位数据时，<u>高八位写入目标地址寄存器，低八位写入0xF0</u>；当16位数据被写完毕后，同时更新到目标地址，<u>仅仅写8位数据则不更新</u>。

总钻风摄像头采集回来的图像是大小是**188*120**，以**左上角**为坐标原点向**右下方**为图像坐标**正方向**。

以总钻风130度红外摄像头为主，以130度红光摄像头为辅采集图像，我们将所采集的图像置于一个120*188的数组当中。

170°的不带红外截止加装红外滤光的镜头。

在实际使用中摄像头我们采用 188×120 的分辨率，以每秒 50 帧(fps)的形 式输出。

双摄像头：为了让摄像头读图稳定，不出现倾斜、割裂、乱点等奇怪现象。两个摄像头**不同时**读取，严格地读完一个的数据再读下一个。且提高摄像头帧率，测定应**大于50fps**以避免影响程序稳定性。通过合理的分配各中断与DMA通道，并合理的利用缓冲区，团队最终实现了双摄像头同时**各45帧**的接收帧率。

```c
// 兰州交通：图像接收与处理
  void Seek_Road (void)
  {
      sint16 nr; //行
      sint16 nc; //列
      sint16 temp = 0; //临时数值
      //for(nr=1; nr<MAX_ROW-1; nr++)
      temp = 0;
      for (nr = 8; nr < 24; nr++)
      {
          for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
          {
              if (Bin_Image[nr][nc])
              {
                  ++temp;
              }
          }
          for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
          {
              if (Bin_Image[nr][nc])
              {
                  --temp;
              }
          }
      }
      OFFSET0 = temp;
      temp = 0;
      for (nr = 24; nr < 40; nr++)
      {
          for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
          {
              if (Bin_Image[nr][nc])
              {
                  ++temp;
              }
          }
          for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
          {
              if (Bin_Image[nr][nc])
              {
                  --temp;
              }
          }
      }
      OFFSET1 = temp;
      temp = 0;
      for (nr = 40; nr < 56; nr++)
      {
          for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
          {
              if (Bin_Image[nr][nc])
              {
                  ++temp;
              }
          }
          for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
          {
              if (Bin_Image[nr][nc])
              {
                  --temp;
              }
          }
      }
      OFFSET2 = temp;
      return;
  }

  /***********通过边沿提取找到灯的重心点********/
    void Seek_Beacon(void)
    {
      uint8 nr=0; //行
      uint8 nc=0; //列

      dotcnt=0;
      y_sum = 0;
      x_sum = 0;
      for (nr = x_zuobiao; nr < MT9V03X_H - 1; nr++)//7.22 nr=1
      {
        for (nc = 1; nc <Image_W - 1; nc++)
        {
          if ((Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] > 255))//若该点上下左右有大于一个白点，记录坐标信息
          {
              y_sum += nc;
              x_sum += nr;
              dotcnt++;
          }
        }
      }
      dotcnt_new = dotcnt;
      x_sum_new = x_sum;
      y_sum_new = y_sum;
    }

    void Seek_Beacon_peng (void)
      {
        uint8 nr=0; //行
        uint8 nc=0; //列

        dotcnt=0;
        y_sum = 0;
        x_sum = 0;
        for (nr = 33; nr < MT9V03X_H - 1; nr++)//7.24-30
        {
          for (nc = 0; nc < Image_W - 1; nc++)
          {
           // if ((dilation_Image[nr - 1][nc] + dilation_Image[nr + 1][nc] + dilation_Image[nr][nc + 1] + dilation_Image[nr][nc - 1] > (1*255)))//若该点左右下有大于一个白点，记录坐标信息
            if(dilation_Image[nr][nc])
              {
                y_sum += nc;
                x_sum += nr;
                dotcnt++;
            }
          }
        }
        dotcnt_new = dotcnt;
        x_sum_new = x_sum;
        y_sum_new = y_sum;
        return;
      }


//膨胀运算
void dilation(unsigned char *data, uint8 width, uint8 height)
{
    uint8 i, j, flag;

    for(i = 1;i < height - 1;i++)
    {
        for(j = 1;j < width - 1;j++)
        {
            flag = 1;
            for(int m = i - 1;m < i + 2;m++)
            {
                for(int n = j - 1; n < j + 2;n++)
                {
                    //自身及领域中若有一个为255
                    //则将该点设为255
                    if(data[i * width + j] == 255 || data[m * width + n] == 255)
                    {
                        flag = 0;
                        break;
                    }
                }
                if(flag == 0)
                {
                    break;
                }
            }
            if(flag == 0)
            {
                dilation_Image[i][j] = 255;
            }
            else
            {
                dilation_Image[i][j]  = 0;
            }
        }
    }
}

void select_pengzhang(void)
{
    if(white_1 >= 0 && white_1 < 35)
    {
        dilation(&Bin_Image[0][0], MT9V03X_W, MT9V03X_H);
        Bin_Image_Filter_peng();
        Seek_Beacon_peng ();
    }
    if(white_1 >= 35)
    {
//        Bin_Image_Filter();
        Bin_Image_Filter();
        Seek_Beacon();
    }

}

void getspeed(void)
 {
     Pulses_l = gpt12_get(GPT12_T2);//左轮速度
     Pulses_r = -gpt12_get(GPT12_T6);//右轮速度
     gpt12_clear(GPT12_T2);
     gpt12_clear(GPT12_T6);
     speed_ave_now = (Pulses_l + Pulses_r) / 2;   // 总速度   占空比与速度对应关系3000--44,10000--145
 }


/********位置式PID********/
float pid_pos(float *err, float *PID_pos, float now, float target)
{

    float pe, ie, de;
    float out;

    err[1] = err[0];

    err[0] = target - now;
    if(err[0] < 10)
    err[2] += err[0] * PID_pos[1]; //积分误差
    err[2] = (err[2] > PID_pos[3]) ? PID_pos[3] : err[2]; //限幅保护
    err[2] = (err[2] < -PID_pos[3]) ? -PID_pos[3] : err[2];


    pe = err[0];
    ie = err[2];
    de = err[0] - err[1];

    out = pe * PID_pos[0] + de * PID_pos[2] + ie;

    return out;
}

/******增量式PID******/
float pid_increase(float *err, float *PID_inc, float now, float target)
{
    float pe, ie, de;
    float out;

    err[2] = err[1];
    err[1] = err[0];
    err[0] = target - now;

    pe = err[0] - err[1];
    ie = err[0];
    de = err[0] - 2 * err[1] + err[2];

    out = pe * PID_inc[0] + ie * PID_inc[1] + de * PID_inc[2];
    return out;
}
```

#### 图像畸变的矫正

对图像进行**标定**（每个镜头的畸变程度各不相同，通过相机标定可以校正这种镜头畸变。其实可以认为用这种标定的方式来求解相机内参和畸变参数，相当于一种相机校准），我们使用**张氏标定法**。利用摄像头在阳光下从不同角度拍摄打印出来的标准**棋盘格标定纸**，使用 Matlab 的 Camera Calibrator 工具对图像进行标定，它可以很方便地导出相机的**内参**。
对图像进行**去畸变**。使用标定后导出的**内参**，建立<u>畸变前图像坐标和畸变后图像坐标的关系式</u>，即去畸变图像的某一坐标点是原畸变图像上的某一坐标点，但是很多时候计算出来的坐标点不为整数，所以还需要进行**插值处理**，我们使用**邻插值法**，即将计算出来的坐标四舍五入为整数。为加快图像处理速度，在程序初始化阶段建立**坐标映射表**，<u>每一帧图像数据到来时，可以由映射表快速得到去畸变图像的数据。</u>

#### 图像的去噪

- 将周围为黑色的白色点判断为噪点，将之变为黑色区域，以避免干扰。如果有 3 个连续白点，则认为是可能的信标灯。
- 对于连续的四个像素点，用不是最大和最小的那个像素点的值来做当前像素点的灰度值。
- 当信标灯显示不全时，需要将图像边缘的信标灯进行反推，按照椭圆特征 进行图像外补全，随后再进行坐标分析。
- 信标灯图像的大小会随着距离增大而减小，因此高处面积大的区域必然不是信标灯。同理，近处面积小的区域也必然不是信标灯。在图像当中，信标灯的整体形状大多为椭圆状，面积随距离增大而减小，单边的斜率变化最多一次。
- 噪点的平均亮度往往非常高，而且在图像中很突兀，因此我们仅仅对于亮度大于 100 且与四周相差过大的像素进行滤波，取得了较好的效果。
- 设定符合场地的阈值，对于小于阈值的点进行滤波，对于大于的点进行处理作为**有效点**。从**最下一行**开始逐行进行有效点的判断，一行内有94个像素点，**从左往右**判断，当判断到第一个有效点时，开始记录该点的位置，它的横坐标和纵坐标，再从这个点作为起始点向右判断是否为有效点，当遇到<u>第一个不为有效点时</u>该点则为信标灯的**边界**，再记录这个点的横纵坐标。此时得到了两个有效点，在同一行的不同位置，进行简单的平均，就得到了灯的大致位置，将它的**纵坐标**记录便于后续判断车离灯的距离，将其**横坐标**作为误差。由上可知横坐标有0到93共有94个则其中点为47，将横坐标与47进行减法运算，得到的误差就为偏移中心点的误差，在进行后续控制计算。

```C
// 山东威海：去噪点
static void fuck_zaodian(const point lu,const point rd,const uint8 img[SCNS_CAM_HEIGHT][SCNS_CAM_WIDTH],uint8 out[SCNS_CAM_HEIGHT][SCNS_CAM_WIDTH])
{
    for(uint8 i=((uint8)lu.y);i<=((uint8)rd.y);++i)
        for(uint8 j=((uint8)lu.x);j<=((uint8)rd.x);++j)
        {
            uint8 cnt=0;
            if(img[i][j]>=100)
            {
                cnt+=abs(img[i][j]-img[i-1][j-1])>10;
                cnt+=abs(img[i][j]-img[i-1][j+1])>10;
                cnt+=abs(img[i][j]-img[i+1][j-1])>10;
                cnt+=abs(img[i][j]-img[i+1][j+1])>10;
            }
            out[i][j]=(cnt>=4?((img[i-1][j-1]+img[i-1][j]+img[i-1][j+1]+img[i][j-1]+img[i][j]+img[i][j+1]+img[i+1][j-1]+img[i+1][j]+img[i+1][j+1])/9):img[i][j]);
        }
}
```

#### 图像的二值化

##### 固定阈值法

##### OTSU大津法

将采集到的图像数据用大津阈值法进行图像分割处理，按图像的灰度特性，将图像分成**背景**和**前景**两部分。因方差是灰度分布均匀性的一种度量,<u>背景和前景之间的类间方差越大,说明构成图像的两部分的差别越大</u>,当部分前景错分为背景或部分背景错分为前景都会导致两部分差别变小，因此,<u>使类间方差最大的分割意味着错分概率最小</u>,利用此算法实现了图像背景过滤，只留下了红色信标灯影像。

<img src="C:\Users\hw\AppData\Roaming\Typora\typora-user-images\image-20220203105510794.png" alt="image-20220203105510794" style="zoom:50%;" />

```c
// 兰州交通：二值化
void Get_Bin_Image (unsigned char mode)
  {
      unsigned short i = 0, j = 0;
      unsigned long  tv = 0;
      //char txt[16];

      if (mode == 0)
      {
          Threshold = GetOSTU(mt9v03x_image);  //大津法阈值
      }
      else if (mode == 1)
      {
          //累加
          for (i = 33; i < MT9V03X_H; i++)
          {
              for (j = 0 ; j < Image_W; j++)
              {
                  tv += mt9v03x_image[i][j];   //累加
              }
          }
          Threshold =(unsigned short)(tv / 87 / 120);   //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
          Threshold = Threshold + lqv;      //此处阈值设置，根据环境的光线来设定
      }
      else if (mode == 2)
      {
 //         Threshold = 110;
          //累加
                    for (i = 0; i < MT9V03X_H; i++)
                    {
                        for (j = 0; j < MT9V03X_W; j++)
                        {
                            tv += mt9v03x_image[i][j];   //累加
                        }
                    }
                    Threshold =(unsigned short)(tv / MT9V03X_H / MT9V03X_W);   //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
                    Threshold = Threshold + lqv;      //此处阈值设置，根据环境的光线来设定//手动调节阈值
          lq_sobel(mt9v03x_image, Bin_Image, (unsigned char) Threshold);

          return;

      }
      else if (mode == 3)
      {
          lq_sobelAutoThreshold(mt9v03x_image, Bin_Image);  //动态调节阈值
          return;
      }
      white_1 = 0;
      /* 二值化 */
      for (i = 33; i < MT9V03X_H; i++)
      {
          for (j = 0; j < Image_W; j++)
          {
              if (mt9v03x_image[i][j] > Threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
              {
                  Bin_Image[i][j] = 255;
                  white_1++; //数出第一次二值化完后的白点数 white_1
              }
              else
                  Bin_Image[i][j] = 0;
          }
      }
  }

  short GetOSTU (unsigned char tmImage[MT9V03X_H][MT9V03X_W])
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
      unsigned char HistoGram[256];              //

      for (j = 0; j < 256; j++)
          HistoGram[j] = 0; //初始化灰度直方图

      for (j = 0; j < MT9V03X_H; j++)
      {
          for (i = 0; i < MT9V03X_W; i++)
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
```

##### 联合边缘的二值化方法

通过联合边缘的二值化方法，包含即**二值化部分**和**边缘检测部分**，其中的二值化部分使用传统的固定阈值二值化，而边缘检测部分使用改进型 的 sobel 算子进行检测。

sobel 算子有两个，一个是检测**水平边缘**的，另一个是检测**垂直边缘**的，增加了对于**主副对角线**的判断。一般的处理方式是将这两个算子的结果**求平方和**作为最终的结果判定该点是否为边缘。

```c
// 山东威海：联合边缘检测的图像二值化
static uint8 conv_bin_special(const point lu,const point rd,const uint8 img[SCNS_CAM_HEIGHT][SCNS_CAM_WIDTH],uint8 out[SCNS_CAM_HEIGHT][SCNS_CAM_WIDTH])
{
    for(uint8 i=((uint8)lu.y);i<=((uint8)rd.y);++i)
        for(uint8 j=((uint8)lu.x);j<=((uint8)rd.x);++j)
            out[i][j]=0XC0;
    for(uint8 i=((uint8)lu.y);i<=((uint8)rd.y);++i)out[i][lu.x]&=0XBF,out[i][rd.x]&=0XBF;
    for(uint8 j=((uint8)lu.x);j<=((uint8)rd.x);++j)out[lu.y][j]&=0XBF,out[rd.y][j]&=0XBF;
    for(uint8 i=((uint8)lu.y);i<=((uint8)rd.y);++i)
        for(uint8 j=((uint8)lu.x);j<=((uint8)rd.x);++j)
        {
            if(img[i][j]<=15)
                out[i][j]&=0X7F;
            //x轴边缘检测
            int32 cx= img[i-1][j-1]    - img[i-1][j+1]
                    +(img[i  ][j-1]<<1)-(img[i  ][j+1]<<1)
                    + img[i+1][j-1]    - img[i+1][j+1];
            //y轴边缘检测
            int32 cy=img[i-1][j-1]+(img[i-1][j]<<1)+img[i-1][j+1]
                    -img[i+1][j-1]-(img[i+1][j]<<1)-img[i+1][j+1];
            //主对角线边缘检测
            int32 cz=(img[i-1][j-1]<<1)+img[i-1][j]
                    + img[i  ][j-1]                - img[i  ][j+1]
                                       -img[i+1][j]-(img[i+1][j+1]<<1);
            //副对角线边缘检测
            int32 cf=                   img[i-1][j]+(img[i-1][j+1]<<1)
                    - img[i  ][j-1]                + img[i  ][j+1]
                    -(img[i+1][j-1]<<1)-img[i+1][j];
            if(abs(cx)>=50)out[i][j+(cx>0?1:-1)]&=0XBF;
            if(abs(cy)>=60)out[i+(cy>0?1:-1)][j]&=0XBF;
            if(abs(cz)>=35)out[i+(cz>0?1:-1)][j+(cz>0?1:-1)]&=0XBF;
            if(abs(cf)>=35)out[i+(cf>0?1:-1)][j+(cf>0?-1:1)]&=0XBF;
            if((cx*cx+cy*cy)>(65*65+75*75))out[i][j]|=0X20;
    	}
    return 1;
}
```

```c
// 兰州交通：边沿检测
/*!
   * @brief    基于soble边沿检测算子的一种边沿检测
   *
   * @param    imageIn    输入数组
   *           imageOut   输出数组      保存的二值化后的边沿信息
   *           Threshold  阈值
   */
  void lq_sobel (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold)
  {
      /** 卷积核大小 */
      short KERNEL_SIZE = 3;
      short xStart = KERNEL_SIZE / 2;
      short xEnd = MT9V03X_W - KERNEL_SIZE / 2;
      short yStart = KERNEL_SIZE / 2;
      short yEnd = MT9V03X_H - KERNEL_SIZE / 2;
      short i, j, k;
      short temp[4];
      for (i = yStart; i < yEnd; i++)
      {
          for (j = xStart; j < xEnd; j++)
          {
              /* 计算不同方向梯度幅值  */
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

              temp[0] = abs(temp[0]);
              temp[1] = abs(temp[1]);
              temp[2] = abs(temp[2]);
              temp[3] = abs(temp[3]);

              /* 找出梯度幅值最大值  */
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

  /*!
   * @brief    基于soble边沿检测算子的一种自动阈值边沿检测
   *
   * @param    imageIn    输入数组
   *           imageOut   输出数组      保存的二值化后的边沿信息
   */
  void lq_sobelAutoThreshold (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W])
  {
      /** 卷积核大小 */
      short KERNEL_SIZE = 3;
      short xStart = KERNEL_SIZE / 2;
      short xEnd = MT9V03X_W - KERNEL_SIZE / 2;
      short yStart = KERNEL_SIZE / 2;
      short yEnd = MT9V03X_H - KERNEL_SIZE / 2;
      short i, j, k;
      short temp[4];
      for (i = yStart; i < yEnd; i++)
      {
          for (j = xStart; j < xEnd; j++)
          {
              /* 计算不同方向梯度幅值  */
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

              temp[0] = abs(temp[0]);
              temp[1] = abs(temp[1]);
              temp[2] = abs(temp[2]);
              temp[3] = abs(temp[3]);

              /* 找出梯度幅值最大值  */
              for (k = 1; k < 4; k++)
              {
                  if (temp[0] < temp[k])
                  {
                      temp[0] = temp[k];
                  }
              }

              /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
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

  void Bin_Image_Filter (void)
  {
      sint16 nr; //行
      sint16 nc; //列

      for (nr = 33; nr < MT9V03X_H - 1; nr++)
      {
          for (nc = 1; nc < Image_W - 1; nc = nc + 1)
          {
              if ((Bin_Image[nr][nc] == 0)//黑点
                      && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] > (2*255)))//若黑点上下右三方向有大于两个白点
              {
                  Bin_Image[nr][nc] = 255;//将该黑点置为白点
              }
              else if ((Bin_Image[nr][nc] == 255)//白点
                      && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] < (2*255)))//若白点上下右三方向有小于两个白点
              {
                  Bin_Image[nr][nc] = 0;//将该白点置为黑点
              }
          }
      }
  }

  void Bin_Image_Filter_peng(void)
            {
                sint16 nr; //行
                sint16 nc; //列

                for (nr = 33; nr < MT9V03X_H - 1; nr++)
                {
                    for (nc = 31; nc < Image_W - 1; nc = nc + 1)
                    {
                        if ((dilation_Image[nr][nc] == 0)//黑点
                                && (dilation_Image[nr - 1][nc] + dilation_Image[nr + 1][nc] + dilation_Image[nr][nc + 1] + dilation_Image[nr][nc - 1] > (2*255)))//若黑点上下右三方向有大于两个白点
                        {
                            dilation_Image[nr][nc] = 255;//将该黑点置为白点
                        }
                        else if ((dilation_Image[nr][nc] == 255)//白点
                                && (dilation_Image[nr - 1][nc] + dilation_Image[nr + 1][nc] + dilation_Image[nr][nc + 1] + dilation_Image[nr][nc - 1] < (2*255)))//若白点上下右三方向有小于两个白点
                        {
                            dilation_Image[nr][nc] = 0;//将该白点置为黑点
                        }
                    }
                }
            }
```


### 信标灯提取

#### 形状匹配

借鉴霍夫变换的思想，由于信标灯是椭圆形，场地灯管长条形，地面反光在上一步已滤除，故找出图像中每个白色连通域的水平距离和竖直距离， 分别作为椭圆长轴短轴，通过椭圆面积公式计算出测量面积 S1，通过遍历该连通域像素点个数得到实际面积 S2，两者越接近则认为是椭圆形信标灯的概率越高。如果二值化后出现单像素点连通域则形成误判；需要浮点运算，对 于无 FPU 模块的单片机运算较慢

#### 闪烁识别算法

异或前后两组图像，找出信标灯，简单高效，适用于红色引导信号，内存要求高。

#### 差值法

1. 首先搜索左边场地,左边场地搜完之后再搜索右边场地
2. 由图像中心向左一定范围后开始逐点往左边搜索,具体的搜索方式是相邻点灰度值**作差**，当差值大于某一个阈值时,表明灰度发生了跳变,即是搜索到了信标灯,将该行存入对应的**存储矩阵**,跳入下一行,重复步骤(2)
3. 如果连续几行都搜索到跳变后,将通过**四周预测**的方法确定下一行跳变所在位置的范围,从而减少搜索的计算量。具体的方法即计算出前几行跳变延所在点连成**直线**的斜率,从而延长直线与下一行相交,得到预测到的跳变位置。该过程将一直重复,直到出现某行搜索不到跳变沿或者搜索到图像尽头
4. 当某行搜索不到跳变沿时,将启动**跳跃搜索**的算法,是通过隔3行扫描一个跳变沿。当连续几行搜索跳变沿后,再次进入连续**四周**预测的搜索算法
5. 搜线完毕后,再进行滤波处理,就得到了处理好的能够进行分析的完整的矩阵。得到众多场地二级图像后，根据霍夫变换得到一个与圆最相似的光斑，并获取坐标

```c
void Image_seek(uint8*date,uint8 width,uint8 i_min,uint8 i_max)
{
    uint8 i,j,th=180;k=0;
    for(i=40;i<i_max;i++)
    {  
      for(j=5;j<(width-5);j++)
       {
        if((date[i*width+j-1]>th) && (date[i*width+j]>th)
        && (date[i*width+j+1]>th) && (date[(i+1)*width+j]>th)
        && (date[(i-1)*width+j]>th))
        image_cnn[i][j]=0xff;
        else  image_cnn[i][j]=0x00;
       }
    }
    for(i=i_min;i<60;i++)
      {  
        for(j=5;j<(width-5);j++)
        {
          if(date[i*width+j-1]>th-30)
                image_cnn[i][j]=0xff;
          else  image_cnn[i][j]=0x00;
        }
    }
}
```


#### 连通域计算与提取

将图像中具有**相同像素值且位置相邻的像素点**组成的图像区域添加一个**标记**，让每个单独的连通区域形成一个被标识的块，进一步的我们就可以获取这些块的<u>轮廓、面积、坐标</u>等参数。如果能选出信标灯对应的连通度，并计算该连通度的中心进行输出，则可以获得很高的定位精度。

一般而言，求解连通域可以用BFS或者DFS进行搜索。但是BFS所依赖的队列会产生巨大的内存开销，而TC264在DFS地过程中一旦递归超过64层后会直接卡入硬件中断，因而这两种算法在当前情况下并不适用。

常见的连通区域标记算法有 **Two-Pass 法**和 **Seed-Filling 种子填充法**。

##### Two-Pass 法

Two-Pass 法正如其名，指的就是通过扫描两遍图像，就可以将图像中存在的所有连通区域找出并标记。第一遍扫描时赋予每个像素位置一个标签，扫描过程中同一个连通区域内的像素集合中可能会被赋予一个或多个不同标签，因此需要将这些属于**同一个连通区域但不同的标签**合并，也就是记录它们之间的相等关系；第二遍扫描就是将具有相等关系的标签所标记的像素<u>归为一个连通区域并赋予一个相同的标签</u>，具体过程如下：

<img src="C:\Users\hw\AppData\Roaming\Typora\typora-user-images\image-20220202120010927.png" alt="image-20220202120010927" style="zoom: 50%;" />

使用双遍历法求解连通域，并使用带压缩的并查集进行优化，取得了较好的效果。

```C
void scns_bcj_init(uint8 node[],uint8 n)
{
    for(uint8 i=0;i<n;node[i]=i,++i);
}
uint8 scns_bcj_find(uint8 node[],uint8 x)
{
    if(x==node[x]) 
        return x; 
    return node[x]=scns_bcj_find(node,node[x]);
} 
void scns_bcj_unite(uint8 node[],uint8 x,uint8 y)
{ 
    x=scns_bcj_find(node,x);
    y=scns_bcj_find(node,y);
    if(x==y)return; 
    if(x>y)
        node[x]=y; 
    else
        node[y]=x;  
}
```

面对一些较为复杂的图片时仍然容易出现不够用的问题。为此，团队在双遍历法的中间创新新的加入了**垃圾回收策略**。即当并查集装满时，对于已扫描的区域进行**合并**，并将松散排列在集合中的连通域统一移到集合的最开头部分，进而节省空间以便继续加入新的连通域。

```c
//求连通域
static uint8 get_connected_domain(const point lu,const point rd,const uint8 img[SCNS_CAM_HEIGHT][SCNS_CAM_WIDTH],uint8 tag[SCNS_CAM_HEIGHT][SCNS_CAM_WIDTH])
{
    uint8 bcj[CONNECTED_DOMAIN_MAXN],labs=0;
    uint8 gc_cnt=0;
    scns_bcj_init(bcj,CONNECTED_DOMAIN_MAXN);
    for(uint8 i=lu.y-1;i<=rd.y+1;++i)for(uint8 j=lu.x-1;j<=rd.x+1;++j)tag[i][j]=255;
    for(uint8 i=((uint8)lu.y);i<=((uint8)rd.y);++i)
        for(uint8 j=((uint8)lu.x);j<=((uint8)rd.x);++j)
            if(img[i][j]>=0XC0)
            {
                uint8 minn=255;
                minn=min(minn,tag[i-1][j-1]);//左上
                minn=min(minn,tag[i-1][j  ]);//正上
                minn=min(minn,tag[i-1][j+1]);//右上
                minn=min(minn,tag[i  ][j-1]);//右侧
                if(minn!=255)
                {
                    tag[i][j]=minn;
                    if(tag[i-1][j-1]!=255)scns_bcj_unite(bcj,tag[i-1][j-1],minn);
                    if(tag[i-1][j  ]!=255)scns_bcj_unite(bcj,tag[i-1][j  ],minn);
                    if(tag[i-1][j+1]!=255)scns_bcj_unite(bcj,tag[i-1][j+1],minn);
                    if(tag[i  ][j-1]!=255)scns_bcj_unite(bcj,tag[i  ][j-1],minn);
                }
                else
                {
                    tag[i][j]=labs;
                    ++labs;
                    if(labs>=CONNECTED_DOMAIN_MAXN)
                    {
                        ++gc_cnt;
                        if(gc_cnt>128)return 255;
                        uint8 mapp[CONNECTED_DOMAIN_MAXN],mapp_cnt=0;
                        for(uint8 i=0;i<CONNECTED_DOMAIN_MAXN;++i)mapp[i]=255;
                        //垃圾回收
                        for(uint8 i=((uint8)lu.y);i<=((uint8)rd.y);++i)
                            for(uint8 j=((uint8)lu.x);j<=((uint8)rd.x);++j)
                                if(tag[i][j]!=255)
                                {
                                    uint8 a=scns_bcj_find(bcj,tag[i][j]);
                                    if(mapp[a]==255)
                                    {
                                        mapp[a]=mapp_cnt;
                                        ++mapp_cnt;
                                    }
                                    tag[i][j]=mapp[a];
                                }
                        labs=mapp_cnt;
                        if(labs>=CONNECTED_DOMAIN_MAXN)return 255;
                        scns_bcj_init(bcj,CONNECTED_DOMAIN_MAXN);
                    }
                }
            }
    for(uint8 i=((uint8)lu.y);i<=((uint8)rd.y);++i)
        for(uint8 j=((uint8)lu.x);j<=((uint8)rd.x);++j)
            if(tag[i][j]!=255)
                tag[i][j]=scns_bcj_find(bcj,tag[i][j]);
    return labs;
}
```

参考算法：

> Tow-Pass算法
>
> 基于对图像的两次遍历，第一次遍历获取**标识矩阵**，并通过标记矩阵中各标识之间的联通关系存储在一个**树A**中，更新并维护树A，然后根据树A第二次遍历图像**合并连通域**，从而得到所有的图像中所有的连通域位置。其具体实现步骤为：
>
> ​    第一次pass，从左到右，从上向下扫描，会将各个有效像素置一个 label 值，判断规则如下 ( 以 4 邻域为例 ) ：
>
> 1. 当该像素的**左邻像素和上邻像素为无效值**时，给该像素置一个新的label值，label ++;
>
> 2. 当该像素的**左邻像素或者上邻像素有一个为有效值**时，将有效值像素的label赋给该像素的label值；
>
> 3. 当该像素的**左邻像素和上邻像素都为有效值**时，选取其中较小的label值赋给该像素的label值。
>
> 同时维护一个**各标识之间联通关系的关系表**，记录哪几个label值属于**同一个联通区域**，表的索引为其自身label值，其值为其根的label值。
>
> 第二次pass:根据关系表更新标识矩阵，将关系表中所有的根label与自己的label不同的标识更换成自己**根label**的标识。

> 区域生长算法
>
> 有效白点的判断：点的上下左右都是白点，则为有效白点
>
> 有阳光时则采用区域生长算法：遍历图像矩阵，找到第一个白点，将其设置为种子点，同时开辟一个新的相同大小的空白矩阵，记录当前种子点；按自定规则将当前种子点相邻的上、下、左、右四个方向的白点扩展， 直到相连的每个白点都被记录，这样一个区域的白点就被记录完毕，设为区域 1；遍历图像矩阵，此时不能选取之前空白矩阵中坐标对应值为 1 的白点；遍历完整个图像矩阵，即可得到一幅图像中全部的连通区域信息。

遍历整张图像，判断**当前像素所属连通域**，并更新连通域的数据。

```c
uint32 toti[CONNECTED_DOMAIN_MAXN];
uint32 totj[CONNECTED_DOMAIN_MAXN];
uint32 totb[CONNECTED_DOMAIN_MAXN];
uint16 cnt [CONNECTED_DOMAIN_MAXN];
uint16 cnte[CONNECTED_DOMAIN_MAXN];
uint8  maxi[CONNECTED_DOMAIN_MAXN];
uint8  maxj[CONNECTED_DOMAIN_MAXN];
uint8  mini[CONNECTED_DOMAIN_MAXN];
uint8  minj[CONNECTED_DOMAIN_MAXN];
uint8  maxb[CONNECTED_DOMAIN_MAXN];
for(uint8 k=0;k<CONNECTED_DOMAIN_MAXN;++k)toti[k]=totj[k]=totb[k]=cnt[k]=cnte[k]=maxi[k]=maxj[k]=maxb[k]=0,mini[k]=minj[k]=255;
for(uint8 i=((uint8)lu.y);i<=((uint8)rd.y);++i)
    for(uint8 j=((uint8)lu.x);j<=((uint8)rd.x);++j)
        if(tag[i][j]!=255)
        {
            ++cnt[tag[i][j]];
            if(bin[i][j]&0X20)++cnte[tag[i][j]];
            toti[tag[i][j]]+=i;
            totj[tag[i][j]]+=j;
            totb[tag[i][j]]+=src[i][j];
            if(i>maxi[tag[i][j]])maxi[tag[i][j]]=i;
            if(i<mini[tag[i][j]])mini[tag[i][j]]=i;
            if(j>maxj[tag[i][j]])maxj[tag[i][j]]=j;
            if(j<minj[tag[i][j]])minj[tag[i][j]]=j;
            if(src[i][j]>maxb[tag[i][j]])maxb[tag[i][j]]=src[i][j];
        }
uint8 totk=0;
for(uint8 k=0;k<tagn;++k)
    if(cnt[k])
        ++totk;
```

从图像中获得信标灯位置的任务就转化为了调出**最可能的连通域**的任务。

##### 基于连通域的形状进行判断

大致处理过程为：对于连通域，首先将其进行缩放，统一尺寸后利用ahsash算法与模板进行匹配。该算法在距离灯较近的地方可以获得非常理想的效果，这主要是因为在距灯较近时，采集到的信标灯具有明显的形状特征，但是灯较远时，采集到的信标灯只有几个像素，而且往往歪七扭八，并不具有明显的形状特征，因而有较高的误判率。

```C
uint32 toti[CONNECTED_DOMAIN_MAXN];
uint32 totj[CONNECTED_DOMAIN_MAXN];
uint32 totb[CONNECTED_DOMAIN_MAXN];
uint16 cnt [CONNECTED_DOMAIN_MAXN];
uint16 cnte[CONNECTED_DOMAIN_MAXN];
uint8  maxi[CONNECTED_DOMAIN_MAXN];
uint8  maxj[CONNECTED_DOMAIN_MAXN];
uint8  mini[CONNECTED_DOMAIN_MAXN];
uint8  minj[CONNECTED_DOMAIN_MAXN];
uint8  maxb[CONNECTED_DOMAIN_MAXN];
for(uint8 k=0;k<CONNECTED_DOMAIN_MAXN;++k)toti[k]=totj[k]=totb[k]=cnt[k]=cnte[k]=maxi[k]=maxj[k]=maxb[k]=0,mini[k]=minj[k]=255;
for(uint8 i=((uint8)lu.y);i<=((uint8)rd.y);++i)
    for(uint8 j=((uint8)lu.x);j<=((uint8)rd.x);++j)
        if(tag[i][j]!=255)
        {
            ++cnt[tag[i][j]];
            if(bin[i][j]&0X20)++cnte[tag[i][j]];
            toti[tag[i][j]]+=i;
            totj[tag[i][j]]+=j;
            totb[tag[i][j]]+=src[i][j];
            if(i>maxi[tag[i][j]])maxi[tag[i][j]]=i;
            if(i<mini[tag[i][j]])mini[tag[i][j]]=i;
            if(j>maxj[tag[i][j]])maxj[tag[i][j]]=j;
            if(j<minj[tag[i][j]])minj[tag[i][j]]=j;
            if(src[i][j]>maxb[tag[i][j]])maxb[tag[i][j]]=src[i][j];
        }
uint8 totk=0;
for(uint8 k=0;k<tagn;++k)
    if(cnt[k])
        ++totk;
```



##### 信标灯的亮度和坐标信息存在一定的对应关系

用C语言把拟合的结果表达出来较难，而手动选取熟悉的函数模型拟合不仅非常麻烦而且效果也不是很好。

##### 基于BPNN的连通域选取

模型参考：https://github.com/ThreeClassMrWang/c-bpnn

采用**人工求导**的方法来解决训练过程中梯度下降时C语言无法自动对表达式进行求导的问题。

```C
static double train_once(bpnn *net,double x[BPNN_MAX],double y[BPNN_MAX])
{
    double a[BPNN_MAX];
    double b[BPNN_MAX];
    double e[BPNN_MAX];
    double g[BPNN_MAX];
    for(uint8 h=0;h<net->hn;++h)
    {
        double alpha_h=0;
        for (uint8 i=0;i<net->in;++i)
            alpha_h+=net->v[i][h]*x[i];
        b[h]=ACTIVATION_FUNC(alpha_h-net->r[h]);
    }
    for(uint8 j=0;j<net->on;++j)
    {
        double beta_j=0;
        for(uint8 h=0;h<net->hn;++h)
            beta_j+=net->w[h][j]*b[h];
        a[j]=ACTIVATION_FUNC(beta_j-net->o[j]);
    }
    double Ek=0;
    for(uint8 j=0;j<net->on;++j)
    {
        g[j]=a[j]*(1-a[j])*(y[j]-a[j]);
        Ek+=(a[j]-y[j])*(a[j]-y[j]);
    }
    Ek=0.5*Ek;
    for(uint8 h=0;h<net->hn;++h)
    {
        double temp=0;
        for(uint8 j=0;j<net->on;++j)
            temp+=net->w[h][j]*g[j];
        e[h]=b[h]*(1-b[h])*temp;
    }
    for(uint8 i=0;i<net->in;++i)
        for(uint8 h=0;h<net->hn;++h)
            net->v[i][h]+=LEARN_RATE*e[h]*x[i];
    for(uint8 h=0;h<net->hn;++h)
        for(uint8 j=0;j<net->on;++j)
            net->w[h][j]+=LEARN_RATE*g[j]*b[h];
    for(uint8 h=0;h<net->hn;++h)
        net->r[h]+=((-LEARN_RATE)*e[h]);
    for(uint8 j=0;j<net->on;++j)
        net->o[j]+=((-LEARN_RATE)*g[j]);
    return Ek;
}
```

**输出层神经元**只有一个，**输入层神经元**包括<u>连通域左上角坐标，右下角坐标，重心坐标，最大亮度，平均亮度，边界个数和面积，共10个</u>。所以真正决定效果的是隐层的数量，数量过多，网络过大，运算缓慢；数量过少，网络过小，难以达到较高的正确率，而且虽然采用随机数据对网络初始化，仍然不可避免在梯度下降的过程中网络陷入局部极小值，会在某些情况下产生极其离谱的结果。团队经过实践，发现网络的隐层神经元数量介于**3至12个**之间时比较合适，因此团队通过编写代码，使其对于3-12个之间的每一种情况自动训练5个网络，并选择其中正确率最高的一个网络作为最终结果。

```c
/*BPNN train at 2021-08-20-01-32-42 with  9 HIDDEN by  1000 loops  99.035525%  ce.x ce.y lu.x lu.y rd.x rd.y maxb aveb  cnt cnte*/
static double get_diff_pbnn_front(const scns_cam_result ans)
{
    const static double v[10][9]={{0.1009224173,2.2340151742,5.3660175417,0.2187064938,-0.0125957716,3.6955148771,0.6228394087,-0.0678285300,-1.6066114801,},{1.4832228324,-0.2682225114,6.7393594564,1.6940154663,-1.7799300053,-5.8342177148,2.5811163027,1.3120270789,10.1097956161,},{0.1468017881,2.6593786910,5.1857812074,-0.2419668619,0.7483611267,4.1717273916,0.3407934234,0.5461294029,-2.3855211573,},{0.9586959889,-1.5378442239,6.7368385525,1.0812663059,-1.8582986066,-5.1507685650,1.4076587230,1.0814145775,9.9661563505,},{0.4075427660,2.7630034086,4.6627801776,-0.5457218663,0.5819033252,3.7115247174,0.3145174858,0.1169668535,-2.1509586771,},{2.0509768071,-0.1028532733,6.3887720005,1.9256786217,-0.8045271982,-6.6714538087,3.6245325558,1.9911162491,9.5077371408,},{1.3760075324,-4.6459418456,0.0851776670,1.7362285746,-3.1360970356,5.2188289915,-7.9085654412,1.4775278323,2.2215591823,},{3.4355414928,-0.8326054561,-0.5297887048,3.7000775299,1.0270550443,-0.5373580805,-1.2916235244,3.5189093932,-2.1707440568,},{0.9027612095,0.0246621802,0.4831892598,0.3637992306,0.7619367815,0.5501739252,0.8481481290,0.6276521978,0.1818755495,},{0.8835499865,0.1766008698,0.3482257774,0.3807061497,0.2499271253,0.6796860666,0.5508763942,0.3739367133,0.2226186436,},},
    w[9][1]={{4.6120697849,},{6.9906604574,},{-11.8732122807,},{4.7801471675,},{7.3965579933,},{-11.1612190303,},{10.7888927302,},{4.8136700027,},{-15.8025674166,},},r[9]={2.3612767334,0.3397001916,4.1119747835,2.4061564339,-3.3654738496,5.2898042103,0.0602073281,2.3800751921,-1.0177080090,},o[1]={-5.3943026021,},
    input_maxx[10]={160.0000000000,100.0000000000,160.0000000000,100.0000000000,160.0000000000,100.0000000000,255.0000000000,255.0000000000,16000.0000000000,16000.0000000000,};
    double x[10],y[1],b[9];
    x[0]=ans.ce.x;
    x[1]=ans.ce.y;
    x[2]=ans.lu.x;
    x[3]=ans.lu.y;
    x[4]=ans.rd.x;
    x[5]=ans.rd.y;
    x[6]=ans.maxb;
    x[7]=ans.aveb;
    x[8]=ans.cnt;
    x[9]=ans.cnte;
    for(uint8 j=0;j<10;++j)x[j]=(x[j])/(input_maxx[j]);
    for(uint8 h=0;h<9;++h)
    {
        double alpha_h=0;
        for(uint8 i=0;i<10;++i)
            alpha_h+=v[i][h]*x[i];
        b[h]=(1.0/(1+exp(-((alpha_h-r[h])))));
    }
    for(uint8 j=0;j<1;++j)
    {
        double beta_j=0;
        for(uint8 h=0;h<9;++h)
            beta_j+=w[h][j]*b[h];
        y[j]=(1.0/(1+exp(-((beta_j-o[j])))));
    }
    return y[0];
}
/*BPNN train at 2021-08-19-18-24-56 with  3 HIDDEN by  2000 loops  99.465600%  ce.x ce.y lu.x lu.y rd.x rd.y maxb aveb  cnt cnte*/
static double get_diff_pbnn_back (const scns_cam_result ans)
{
    const static double v[10][3]={{0.1454843220,-0.1547541710,0.0511131942,},{0.5308478963,9.5343809702,-3.4796464819,},{-0.1524603825,-0.1517862579,-0.0467092360,},{0.3944211199,10.5611720138,-2.6122127937,},{0.0773863649,0.2930922703,0.0080681712,},{1.4271605054,10.0415394517,-3.7126576615,},{-0.5492350554,2.0213265990,8.2577426999,},{-0.2192009126,-4.8172315843,5.0036228469,},{0.0138030542,0.9219688791,0.1486456658,},{0.5978536917,0.3270177840,0.2700980528,},},w[3][1]={{8.3270533342,},{-13.6910388331,},{-11.4485450546,},},r[3]={-1.5631197848,8.0950557741,-0.9918931032,},o[1]={-10.2009469332,},input_maxx[10]={160.0000000000,100.0000000000,160.0000000000,100.0000000000,160.0000000000,100.0000000000,255.0000000000,255.0000000000,16000.0000000000,16000.0000000000,};
    double x[10],y[1],b[3];
    x[0]=ans.ce.x;
    x[1]=ans.ce.y;
    x[2]=ans.lu.x;
    x[3]=ans.lu.y;
    x[4]=ans.rd.x;
    x[5]=ans.rd.y;
    x[6]=ans.maxb;
    x[7]=ans.aveb;
    x[8]=ans.cnt;
    x[9]=ans.cnte;
    for(uint8 j=0;j<10;++j)
        x[j]=(x[j])/(input_maxx[j]);
    for(uint8 h=0;h<3;++h)
    {
        double alpha_h=0;
        for(uint8 i=0;i<10;++i)
            alpha_h+=v[i][h]*x[i];
        b[h]=(1.0/(1+exp(-((alpha_h-r[h])))));
    }
    for(uint8 j=0;j<1;++j)
    {
        double beta_j=0;
        for(uint8 h=0;h<3;++h)
            beta_j+=w[h][j]*b[h];
        y[j]=(1.0/(1+exp(-((beta_j-o[j])))));
    }
    return y[0];
}
```

得到答案连通域后，通过**桶形畸变修复**和**逆透视变换**得到灯在车模坐标系下的位置，方便后期控制。

信标灯在**相邻的两张图像中**不会突然移动非常大的距离，所以在当前图片已经计算得到答案时，可以猜出下一张图片中信标灯的大致位置，并**减小扫描范围**，在整体的算法设计中，通过在每一步的入口处加入`const point lu`、`const point rd`两个参数，对于扫描窗口的**左上角**和**右下角**进行约束，并在每一次扫描前**更新扫描窗口**，经过测试，该方法可以将计算帧率提高4倍以上。

```C
//重置扫描窗口
static void reset_window(const scns_cam_enum camn)
{
    crd[camn].scan.lu=((point){1,1}),crd[camn].scan.rd=((point){SCNS_CAM_WIDTH-2,SCNS_CAM_HEIGHT-2});
}
//更新扫描窗口
static void update_window(const scns_cam_enum camn)
{
    if(scns_cam_data[camn].useable)
    {
        int16 dx=scns_cam_data[camn].rd.x-scns_cam_data[camn].lu.x;
        int16 dy=scns_cam_data[camn].rd.y-scns_cam_data[camn].lu.y;
        dx=max(dx,60);
        dy=max(dy,24);
        crd[camn].scan.lu.x=scns_cam_data[camn].lu.x-dx;
        crd[camn].scan.lu.y=scns_cam_data[camn].lu.y-dy;
        crd[camn].scan.rd.x=scns_cam_data[camn].rd.x+dx;
        crd[camn].scan.rd.y=scns_cam_data[camn].rd.y+dy;
        if(crd[camn].scan.lu.x<=1)crd[camn].scan.lu.x=1;
        if(crd[camn].scan.lu.y<=1)crd[camn].scan.lu.y=1;
        if(crd[camn].scan.rd.x>=SCNS_CAM_WIDTH -2)crd[camn].scan.rd.x=SCNS_CAM_WIDTH -2;
        if(crd[camn].scan.rd.y>=SCNS_CAM_HEIGHT-2)crd[camn].scan.rd.y=SCNS_CAM_HEIGHT-2;
    }
    else
        reset_window(camn);
}
```

由于上述算法**内存开销**过于巨大，为了更加合理的分配内存，团队还设计了一套简单的**动态内存管理系统**，可以在不需要显示或发送图片时节约内存用于计算。

#### 获取信标灯位置

- 提取中点：一幅图像中，设定一个阈值后，超过阈值为1否则为0；当信标灯亮时，在图像数据中便出现高于阈值的数据，记录高于阈值的数据所在的第一行行数a、列数b及高于阈值的数据的最后一行的行数A、列数B;灯中点行数X,列数Y则可运算出来。X=a+[(A-a)/2]、Y=b+[(B-b)/2]。从而获得信标灯的位置。
- 一场图像中，当信标灯点亮时，在图像数据中便出现高于阈值的数据，记录高于阈值的数据所在的行数、列数及高于阈值的数据的总的行数R（最大行与最小行的差）、总的列数L（最大列与最小列的差），并将高于阈值（固定值）的数据所在的行数、列数分别相加获得总行数R0和总列数L0，用总行数R0除以总的行数R得到行数作为信标所在行，用总列数L0除以总的列数L得到列数作为信标所在列，从而获得信标的位置。

### 进度

#### 2021-12-11

1. 阅读完毕官方规则，并提炼相关信息
2. 下载和整理群内的相关资料
3. 初步阅读山威的技术报告

#### 2022-01-24

1. 阅读完山威的技术报告摄像头部分

#### 2022-01-26

1. 阅读其他部分高校的第十五届声音信标和直立节能的技术报告，发现与此次比赛还有很大的区别
2. 比较历年信标组的规则，发现此次比赛主要是限制了车模，取消了节能
2. 获取了十六届的节能信标技术报告
2. 看了几篇节能信标的技术报告，感觉比较拉跨，但大致能够确定需要红外滤波、计算连通域

#### 2022-01-28

1. 继续阅读技术报告，看到了巢湖学院
1. 学习并加入了一些信标灯提取的算法

#### 2022-02-02

1. 河南理工大学：操作系统、开源代码
1. 继续阅读技术报告，看到了湖北工业大学
1. 完善了信标灯提取算法，加入了图像去畸变

#### 2022-02-03

1. 继续阅读技术报告，看到了厦门理工大学
2. 摘录了技术报告中一些源码

#### 2022-02-04

1. 继续阅读技术报告，看到了上海海事大学
2. 优化了文档的结构

#### 2022-02-06

1. 继续阅读技术报告，看到了五邑大学

#### 2022-02-07

1. 看完技术报告

### 相关资料

1. [龙邱方案](https://mp.weixin.qq.com/s?__biz=MzAxMTM4NDg2OA==&mid=2649283927&idx=1&sn=81a2aeb980931dda54894c84412b28d8&chksm=835d9c1cb42a150ace3af7ddd403557456d443204ac7d4596b6df7da449689fc08c77906546b&mpshare=1&scene=23&srcid=1209fuh6vWUE9pMM8LLNXRZD&sharer_sharetime=1639060459954&sharer_shareid=7ae6acfe8330d2f32958df3679d9dbe5#rd)


### 参考代码

#### 十六-节能信标-南京师范

```c
// 南京师范
void OTSU(void)//大津法计算阈值消耗时间：0.3ms
{
    # define GrayScale 256
    int pixelCount[GrayScale]={0};
    float pixelPro[GrayScale]={0};
    int i, j, pixelSum = MT9V03X_W * MT9V03X_H ; //180*90/9 简化：相邻像素点的灰度值近似相等，间隔取点
    float usum = 0,w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0, 
    delta[5]={0};//一定记得置零!
    //第一步：统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < MT9V03X_H; i+=1)
    {
        for (j = 0; j < MT9V03X_W; j+=1)
        {
            pixelCount[(int)data[i * MT9V03X_W + j]]++; //将像素值作为计数数组的下标
        }
    }
    //第二步：计算每个像素在整幅图像中的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
        usum += pixelPro[i] * i;
    }
    //第三步：遍历灰度级[0,255]
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0.0f;
    for (j = 0; j < GrayScale; j++) //j为阈值
    {
        i = 0;
        //背景部分
        w0 += pixelPro[j]; //每个像素点的灰度值所占比例之和——背景部分比例
        u0tmp += j * pixelPro[j]; //每个像素点的灰度值所占比例*灰度值——背景部分加权灰度值
        //前景部分
        w1 = 1 - w0;
        deltaTmp = pow((w0 * usum - u0tmp) , 2) / (w0 * w1); //简化：直接代入公式
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (uint8)j;
        }
        //简化：一副图像的最佳阈值附近的灰度值所对应的类间方差呈二次变化，且在某一灰度值取得最大值，当有减小趋势时，不再遍历，退出循环
        delta[0] = delta[1];
        delta[1] = delta[2];
        delta[2] = delta[3];
        delta[3] = delta[4];
        delta[4] = deltaTmp;

        if (delta[4] < delta[0])
        {
            break;
        }
    }
}
void Binaryzation(void)//二值化消耗时间：1.0ms
{
    uint16 i,j;
    for (i = 0; i < MT9V03X_H; i+=1)
    {
        for (j = 0; j < MT9V03X_W; j+=1)
        {
            if (data[i * MT9V03X_W + j] > threshold)
                temp_image[i][j] = 255; //黑点：背景
            else
                temp_image[i][j] = 0; //白点：可疑信标
        }
    }
}
void sobel(uint8 imageIn[MT9V03X_H][MT9V03X_W], uint8
           imageOut[MT9V03X_H][MT9V03X_W])
{
    /** 卷积核大小 */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = MT9V03X_W - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = MT9V03X_H - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* 计算不同方向梯度幅值 */
            temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j 
                                                                              + 1] //{{-1, 0, 1},
                - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1] // {-1, 0, 1},
                - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1]; // {-1, 0, 1}};
            temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j- 1] //{{-1, -1, -1},
                - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j] // { 0, 0, 0},
                - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1]; // { 1, 1, 1}};
            temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1] // 0, -1, -1
                - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j] // 1, 0, -1
                - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1]; // 1, 1, 0
            temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]  // -1, -1, 0
                - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j] // -1, 0, 1
                - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1]; //0, 1, 1
            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);
            /* 找出梯度幅值最大值 */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                    temp[0] = temp[k];
            }
            /* 使用像素点邻域内像素点之和的一定比例 作为阈值 */
            temp[3] = (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] 
                + (short) imageIn[i - 1][j + 1]
                + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) 
                imageIn[i][j + 1]
                + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] 
                + (short) imageIn[i + 1][j + 1];
            if (temp[0] > threshold ) //手动阈值
                imageOut[i][j] = 255;
            else
                imageOut[i][j] = 0;
        }
    }
}

void Filter(void)
{
    uint16 row; //行
    uint16 col; //列
    for (row = 1; row < MT9V03X_H-1; row++)
    {
        for (col = 1; col < MT9V03X_W-1; col++)
        {
            if (temp_image[row][col] == 0 && temp_image[row-1][col] + 
                temp_image[row+1][col] + temp_image[row][col+1] + temp_image[row][col-1] >= 510)
            {
                temp_image[row][col] = 255;
            }
            // else if (temp_image[row][col] == 255 && temp_image[row-1][col] + 
            temp_image[row+1][col] + temp_image[row][col+1] + temp_image[row][col-1] <= 
                510)
                // {
                // temp_image[row][col] = 0;
                // }
        }
    }
}
void Seek(void)
{
    uint8 i=0,row=0,col=0,startcol=10,real_startrow=0;
    dotcnt = 0;
    for (col = startcol; col < MT9V03X_W-startcol; col+=2)
    {
        if (col < cut_col || col > 180-cut_col) // 阳光算法：梯形屏蔽
            real_startrow = (uint8)startrow + add_row;
        else
            real_startrow = (uint8)startrow;
        for (row = real_startrow; row < MT9V03X_H; row+=2)
        {
            //120*188分辨率下，有3个连续白点，则认为是可能的信标灯
            if (dotcnt < 100 && temp_image[row][col] == 255 && temp_image[row -
                                                                          1][col] + temp_image[row + 1][col] + temp_image[row][col + 1] + 
                temp_image[row][col - 1] >= 510)
            {
                dotc[dotcnt] = col; // 记录所有的白点所在列，左右
                dotr[dotcnt++] = row; // 记录所有的白点所在行，远近
            }
            if (dotcnt >= 100)
                break;
        }
    }
    if (dotcnt)//发现有白点
    {
        beacon_r_last=beacon_r;
        beacon_c = 0, beacon_r = 0; // 清除上次的结果
        for(i = 0 ;i < dotcnt; i++)
        {
            beacon_c += dotc[i]; // 所有白点左右偏差值求和
            beacon_r += dotr[i]; // 所有白点上下偏差值求和
        }
        beacon_c = beacon_c / dotcnt; // 灯的左右中心点
        beacon_r = beacon_r / dotcnt; // 灯的远近中心点
        if(beacon_r<35 && beacon_r_last>70) charge_flag=1;
    }
    return;
}
void Camera_Control(void)
{
     uint8 Key = 1;
     if (mt9v03x_finish_flag)
     {
         sobel(&mt9v03x_image[0][0],&temp_image[0][0]);
         Filter();
         Seek();
         Key = readKey();
         lcd_showstr(10,7,"c"); lcd_showuint16(30,7,beacon_c);
         lcd_showstr(90,7,"r"); lcd_showuint16(110,7,beacon_r);
         mt9v03x_finish_flag = 0; //在图像使用完毕后 务必清除标志位，否则不会开始采集下一幅图像
     }
}
```

#### 十六-节能信标-青岛农业

```C
void image_transform(uint8 tmImage[MT9V03X_H2][MT9V03X_W2]) //  188*120-----94*60
{
    int i=0,j=0;
    uint8 figure;
    for(i=0;i<MT9V03X_H2;i++)
    {
        for(j=0;j<MT9V03X_W2;j++)
        {
            tmImage[i][j]=mt9v03x_image[i*2][j*2];
        }
    }
}
short GetOSTU (uint8 tmImage[MT9V03X_H2][MT9V03X_W2])
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
    signed short Threshold2 = 0;
    unsigned char HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0;

    for (j = 0; j < MT9V03X_H2; j++)
    {
        for (i = 0; i < MT9V03X_W2; i++)
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
            Threshold2 = j;
        }
    }
    return Threshold2;                        //返回最佳阈值;
}

void sobel(uint8 imageIn[MT9V03X_H2][MT9V03X_W2], uint8 imageOut[MT9V03X_H2][MT9V03X_W2])
{
    int temp=0;
    int tempx=0,tempy=0,i=0,j=0;
    for(i=1;i <MT9V03X_H2-1; i++)
    {
        for(j=1;j<MT9V03X_W2-1;j++)
        {
            tempx=  imageIn[i-1][j-1]  //(*(p+(i-1)*COL+(j-1)))
                +2*imageIn[i-1][j+1]
                +imageIn[i-1][j+1]
                -imageIn[i+1][j-1]
                -2*imageIn[i+1][j]
                -imageIn[i+1][j+1];
            if(tempx<0)
                tempx=-tempx;
            tempy=  imageIn[i-1][j-1]  //(*(p+(i-1)*COL+(j-1)))
                +2*imageIn[i][j-1]
                +imageIn[i+1][j-1]
                -imageIn[i-1][j+1]
                -2*imageIn[i][j+1]
                -imageIn[i+1][j+1];
            if(tempy<0)
                tempy=-tempy;
            if(tempx>tempy)
                temp=tempx;
            else temp=tempy;
            //                    temp=tempx+tempy;
            if(temp>255)
                temp=255;
            imageOut[i][j]=temp;

        }
    }
}

void binaryzaton(unsigned char imageIn[MT9V03X_H2][MT9V03X_W2], unsigned char imageOut[MT9V03X_H2][MT9V03X_W2],uint16 Threshold)
{
    int i,j;
    for(i=0;i<MT9V03X_H2;i++)
    {
        for(j=0;j<MT9V03X_W2;j++)
        {
            if(imageIn[i][j]>Threshold)
                imageOut[i][j] = 255;
            else
                imageOut[i][j] = 0;
        }
    }
}
void expand(unsigned char imageIn[MT9V03X_H2][MT9V03X_W2], unsigned char imageOut[MT9V03X_H2][MT9V03X_W2])
{
    int i,j;
    for(i=0;i<MT9V03X_H2;i++)
    {
        for(j=0;j<MT9V03X_W2;j++)
        {
            if(imageIn[i][j]<128)
            {
                if((imageIn[i-1][j-1]>128)||(imageIn[i-1][j]>128)||(imageIn[i-1][j+1]>128)||
                   (imageIn[i][j-1]>128)||(imageIn[i][j+1]>128)||
                   (imageIn[i+1][j-1]>128)||(imageIn[i+1][j]>128)||(imageIn[i+1][j+1]>128))
                    imageOut[i][j] = 255;
                else
                    imageOut[i][j] = 0;
            }
            else
                imageOut[i][j] = 255;
        }
    }
}
void erosion(unsigned char imageIn[MT9V03X_H2][MT9V03X_W2], unsigned char imageOut[MT9V03X_H2][MT9V03X_W2])
{
    int i,j;
    for(i=1;i<MT9V03X_H2-1;i++)
    {
        for(j=1;j<MT9V03X_W2-1;j++)
        {
            if(imageIn[i][j]>128)
            {
                if((imageIn[i-1][j-1]>128)&&(imageIn[i-1][j]>128)&&(imageIn[i-1][j+1]>128)&&
                   (imageIn[i][j-1]>128)&&(imageIn[i][j+1]>128)&&
                   (imageIn[i+1][j-1]>128)&&(imageIn[i+1][j]>128)&&(imageIn[i+1][j+1]>128))
                    imageOut[i][j] = 255;
                else
                    imageOut[i][j] = 0;
            }
            else
                imageOut[i][j] = 0;
        }
    }
}
void find_light(uint8 imageIn[MT9V03X_H2][MT9V03X_W2])
{
    int i,j;
    float num=0;
    uint8 first_label=0;
    float label_x_sum=0,label_y_sum=0;
    uint8 label_x=0,label_y=0;
    for(i=MT9V03X_H2-2;i>=1;i--)        {
        for(j=1;j<=MT9V03X_W2-2;j++)            {
            if((imageIn[i][j]>0)&&(first_label==0))              {
                first_label=imageIn[i][j];
            }
            if((imageIn[i][j]==first_label)&&(first_label>0))
            {
                num++;
                label_x_sum+=j;
                label_y_sum+=i;
            }
        }
    }
    light_label_num=num;
    light_label=first_label;
    if((label_area_num<5)&&(label_area_num>0))     {
        label_x=(uint8)((int)(label_x_sum/num));       label_y=(uint8)((int)(label_y_sum/num));
        if((label_y>=4)&&(label_y<45)&&(label_x>=0)&&(label_x<=94))       {
            if((find_light_flag==0)&&(label_y<20))
            {
                light_point_x=label_x;
                light_point_y=label_y;
                find_light_flag=1;
            }
            else
            {
                light_point_x=label_x;
                light_point_y=label_y;
            }
            light_point_y_last=light_point_y;
            lose_num=0;
        }
    }
    else
    {
        lose_num++;
        if((light_point_y_last>30)||(lose_num>5))
        {
            light_point_x=0;
            light_point_y=0;
            find_light_flag=0;
        }

    }

}
void show_light_xy(void)
{
    int i;
    lcd_set_region(light_point_x,0,light_point_x,MT9V03X_H2-1);
    for(i=0;i<MT9V03X_H2;i++)
    {
        lcd_writedata_16bit(RED);
    }

    lcd_set_region(0,light_point_y,MT9V03X_W2-1,light_point_y);
    for(i=0;i<MT9V03X_W2;i++)
    {
        lcd_writedata_16bit(RED);
    }
}
```

#### 十六-节能信标-厦门理工学院

```c
void find_mid5()     //图像处理
{
    rside=0;
    for(y=59;y>lvceng;y--)         //遍历高******
    {
        for(rside=10;rside<84;rside++)      //寻找边界 从左边开始
        {
            if(Image_Use[y][rside]>=yuzhi)//找到第一个有效点
            {
                get_wide[1]=rside;
                for(;rside<84;rside++)
                {
                    if(Image_Use[y][rside]<yuzhi)//找到有效行后一连串有效点的边界
                    {
                        get_wide[2]=(get_wide[1]+rside-1)/2;//得到误差
                        get_high[2]=y;//得到灯的高度(用于大致判断位置)
                        get_high_last=get_high[2];//得到最后一次灯的位置用于车自行判断是灭了灯还是环境影响
                        break;
                    }
                    if(rside==83)
                    {
                        get_wide[2]=get_wide[1];
                        get_high[2]=y;
                        get_high_last=get_high[2];
                    }
                }
            }
            if(get_high[2]!=0)//找到点后便可以退出循环
            {
                break;
            }
        }
        if(get_high[2]!=0)
        {
            break;
        }
    }
    if (get_high[2]==0)//当未找到灯时
    {
        black++;//用于加快或减慢车的反应速度
        if(PIN_Read(DSW1)==1)
        {
            find_led();//找灯程序
        }
        get_high_last=0;
        for(ql=0;ql<3;ql++)
        {
            get_high[ql]=0;
            get_wide[ql]=0;
        }
    }
    else
    {
        black=0;
        error=get_wide[2]-47+offset;//误差处理 offset用于认为给定误差
        PWM();//用于控制

        if(PIN_Read(DSW1)==0)//用于图像处理时更直观的画出中心点的坐标与灰度值
        {
            TFTSPI_Draw_Line(0, get_high[2], 93, get_high[2], u16RED);
            TFTSPI_Draw_Line(get_wide[2], 0, get_wide[2], 59, u16RED);
            sprintf(txt2, "H: %d", get_high[2]);
            TFTSPI_P8X16Str(12, 0, txt2,u16ORANGE, u16BLACK);
            sprintf(txt1, "W: %d", get_wide[2]);
            TFTSPI_P8X16Str(12,1,txt1,u16ORANGE, u16BLACK);
            sprintf(txt2, "z: %d", Image_Use[get_high[2]][get_wide[2]]);
            TFTSPI_P8X16Str(12, 2, txt2,u16ORANGE, u16BLACK);
            if(Image_Use[get_high[2]][get_wide[2]]<100)
            {
                sprintf(txt2, " ");
                TFTSPI_P8X16Str(17,2,txt2,u16BLACK, u16BLACK);
            }

        }
        for(ql=0;ql<3;ql++)
        {
            get_high[ql]=0;
            get_wide[ql]=0;
        }
    }
}

void find_led()
{

    if(black==black_begin)//反应速度调整
    {
        get_black=0;//不满足中断的任何一个判断，不进行速度处理

        if(get_high_last>=30&&get_high_now!=59)//充电的条件
        {
            if(stop_now==1)
            {

                begin1();//充电分压处理
                if(huan==0)//充完电后各标志位清零速度回调
                {
                    stop_now=0;
                    stop_now_begin=0;
                    rotation1=u32rBuff2[2];
                }
            }

        }
        if(stop_now_begin==1)//下一盏灯进行充电的标志位 进入单独充电程序
        {
            stop_now=1;

        }
        rotation=rotation1;//设定转速
        MotorCtrl4w(rotation-500,-rotation,0,0);//根据车模自身给定差速自转

        while(get_black==0)
        {
            if (Camera_Flag == 2)
            {
                Get_Use_Image();
                Camera_Flag = 0;
                for(i=lvceng;i<60;i++)//简易阈值二值化
                {
                    for(j=30;j<64;j++)
                    {
                        if(Image_Use[i][j]>=yuzhi)
                        {
                            Bin_Image[i][j]=1;

                        }
                        else
                        {
                            Bin_Image[i][j]=0;
                        }
                    }
                }
                find_mid2();//图像处理简易版找到亮点即退出
            }

        }

        get_black=0;

        get_high_now=get_high[2];

        get_high_stop=get_high[2];

        if((get_high_last>=50)&&(get_high_now!=59))//判断灭灯 后续加自转速度保证找灯效率
        {
            rotation1=rotation1+diejia;
        }
        if(get_high[2]<8)//远灯特殊处理降速点
        {

            stopp=stop_begin-5;
            get_stop=u32rBuff4[3]-4;
            if(huan==1)
            {
                get_stop=u32rBuff4[3]+1;
            }

        }
        else
        {
            stopp=stop_begin;
            get_stop=u32rBuff4[3];
            if(huan==1)
            {
                get_stop=u32rBuff4[3]+1;
            }
        }
        black=0;
        if(get_high[2]==59)//上灯但是磁标没触发特殊处理近灯切灯
        {
            MotorCtrl4w(2500,3000,0,0);
            delayms(200);

        }
        else
        {
            if(huizhen==1||wending==1)//回震保证了车模旋转找灯后的稳定性
            {
                MotorCtrl4w(-rotation+500,rotation,0,0);
                delayms(35);
                huizhen=0;
            }
        }
        shijian=0;//退出找灯程序后的初始化
        speed_L=5000;
        speed_R=5000;
        output_L_pwm=0;
        output_R_pwm=0;
    }
}
```

