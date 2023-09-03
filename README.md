# MAX30102_hal-_cubex
需要配置一个中断引脚
![image](https://github.com/DOUGIE2111/MAX30102_hal-_cubex/assets/123169001/a7298338-7c21-40d5-a061-be43bbd9c037)





I2正常配置即可，中断等级最后拉到最高，防止其它的中断将它抢断，hal库delay的中断要拉到0或者比15小。
在主程序的初始写如下代码
![image](https://github.com/DOUGIE2111/MAX30102_hal-_cubex/assets/123169001/613c3a8e-7614-4714-af88-0846bf815787)



中断代码如下，可以用bool型的标志位也可。
![image](https://github.com/DOUGIE2111/MAX30102_hal-_cubex/assets/123169001/04ebe536-42a0-4f94-a93b-369d9d2e7379)


