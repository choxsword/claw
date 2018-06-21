#pragma once

#define NOTE_D0 -1
#define NOTE_D1 294
#define NOTE_D2 330
#define NOTE_D3 350
#define NOTE_D4 393
#define NOTE_D5 441
#define NOTE_D6 495
#define NOTE_D7 556

#define NOTE_DL1 147
#define NOTE_DL2 165
#define NOTE_DL3 175
#define NOTE_DL4 196
#define NOTE_DL5 221
#define NOTE_DL6 248
#define NOTE_DL7 278

#define NOTE_DH1 589
#define NOTE_DH2 661
#define NOTE_DH3 700
#define NOTE_DH4 786
#define NOTE_DH5 882
#define NOTE_DH6 990
#define NOTE_DH7 112
//以上部分是定义是把每个音符和频率值对应起来，其实不用打这么多，但是都打上了，后面可以随意编写D调的各种歌，我这里用NOTE_D+数字表示音符，NOTE_DH+数字表示上面有点的那种音符，NOTE_DL+数字表示下面有点的那种音符。这样后面写起来比较好识别。
#define WHOLE 1
#define HALF 0.5
#define QUARTER 0.25
#define EIGHTH 0.25
#define SIXTEENTH 0.625
//这部分是用英文对应了拍子，这样后面也比较好看

static int tune_start[] = 
{
  NOTE_DH1,NOTE_D6,NOTE_D5,NOTE_D6,NOTE_D0,
  NOTE_DH1,NOTE_D6,NOTE_D5,NOTE_DH1,NOTE_D6,NOTE_D0,
};

static int tune_success[]={
  NOTE_DH1,NOTE_D0,NOTE_D6,NOTE_D6,NOTE_D5,NOTE_D5,NOTE_D6,NOTE_D6,
  NOTE_D0,NOTE_D5,NOTE_D1,NOTE_D2,NOTE_D0,

  NOTE_DH1,NOTE_D0,NOTE_D7,NOTE_D5,
  NOTE_D6,
};//这部分就是整首曲子的音符部分，用了一个序列定义为tune，整数

static float duration_start[]=
{
  1,1,0.5,0.5,1,
  0.5,0.5,0.5,0.5,1,0.5,
};
static float duration_success[]={
  0.5,0.5,0.5+0.25,0.25,0.5+0.25,0.25,0.5+0.25,0.25,
  0.5,1,0.5,1,1,
  
  1+1,0.5,0.5,1,
  1+1+1+1,

};//这部分是整首曲子的接拍部分，也定义个序列duration，浮点（数组的个数和前面音符的个数是一样的，一一对应么）

namespace xzj{


class Alarm
{
    int tonePin;
    public:
    Alarm(int _tonPin):tonePin(_tonPin){
        pinMode(tonePin,OUTPUT);//设置蜂鸣器的pin为输出模式
    }

    void start(){

        
        for(int i=0;i<sizeof(tune_start)/sizeof(tune_start[0]);++i){
             tone(tonePin,tune_start[i]);//此函数依次播放tune序列里的数组，即每个音符
             delay(400*duration_start[i]);//每个音符持续的时间，即节拍duration，400是调整时间的越大，曲子速度越慢，越小曲子速度越快，自己掌握吧
             noTone(tonePin);//停止当前音符，进入下一音符
        }
        

    }

    void success(){
        for(int i=0;i<sizeof(tune_success)/sizeof(tune_success[0]);++i){
             tone(tonePin,tune_success[i]);//此函数依次播放tune序列里的数组，即每个音符
             delay(400*duration_success[i]);//每个音符持续的时间，即节拍duration，400是调整时间的越大，曲子速度越慢，越小曲子速度越快，自己掌握吧
             noTone(tonePin);//停止当前音符，进入下一音符
        }
    }

    void warning()
    {
        for (int i = 200; i <= 600; i++) //用循环的方式将频率从200HZ 增加到800HZ
        {
            tone(tonePin, i); //在四号端口输出频率
            delay(5);   //该频率维持5毫秒
        }
        delay(1500); //最高频率下维持4秒钟
        for (int i = 600; i >= 200; i--)
        {
            tone(tonePin, i);
            delay(5);
        }
        noTone(tonePin);
    }
};

}