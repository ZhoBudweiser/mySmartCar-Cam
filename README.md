# 一颗摄像头的成长之路



### 项目信息

**任务：**项目实现车模从发车区出发，在信标场地内检测点亮的信标，逐一通过熄灭信标灯，尽可能在短时间内熄灭足够多的信标灯。

**芯片：**MindMotion 微控制器 MM32F3277G9P120MHz M3

**车模：**E 车模，RS-380电机

**摄像头：**MT9V034（总钻风） + 红外滤光片

**显示屏：**ST7789V 2.0寸液晶屏



### 方案特征

- 基于场景特点对输入图像进行降噪，通过 sobel 算子提取场景信息，实现图像的二值化；
- 二值化后的图像通过特征提取，得到坐标、重心、周长、亮度等 11 个特征；
- 输入含 1 层隐藏层（5 个神经元）的神经网络进行预测分析，完成信标灯识别；
- 通过限制扫描范围优化图像处理的速度；
- 支持多张图像和多种参数的显示，画面中的信标灯将会被标记。

##### 

### 文件特征

```
├─Bpnn code
│  │  test_bpnn.c：神经网络测试
│  │  test_bpnn_train.c：神经网络训练
│  │  
│  ├─bpnn
│  │      bpnn.c：神经网络定义函数
│  │      bpnn.h：神经网络定义函数头文件
│  │      bpnn_config.h：神经网络参数配置
│  │      bpnn_fit.c：神经网络训练函数
│  │      bpnn_fit.h：神经网络训练函数头文件
│  │      
│  └─dataset
│          bpnn_param.txt：神经网络输出参数
│          in.txt：测试集数据
│          out.txt：测试集标签
│          test_in copy_11.txt
│          test_in copy_17.txt
│          test_in copy_18.txt
│          test_in copy_22.txt
│          test_in copy_36.txt
│          test_in copy_40.txt
│          test_in copy_8.txt
│          test_in.txt：训练集数据（数字为数据规模）
│          test_out copy_11.txt
│          test_out copy_17.txt
│          test_out copy_18.txt
│          test_out copy_22.txt
│          test_out copy_36.txt
│          test_out copy_40.txt
│          test_out copy_8.txt
│          test_out.txt：训练集标签（数字为数据规模）
│          
└─MCU files
        bpnn_config.h：神经网络参数配置
        bpnn_fit.c
        bpnn_fit.h：神经网络函数
        display.c：显示屏函数
        display.h：显示屏函数头文件
        ImgProcess.c：图像处理函数
        ImgProcess.h：图像处理函数头文件
        seqQueue.h：特征提取队列数据结构定义
```



### 参考资料

神经网络模型参考：https://github.com/ThreeClassMrWang/c-bpnn

部分视觉模型参考：十六届智能车-山魂七队-技术报告
