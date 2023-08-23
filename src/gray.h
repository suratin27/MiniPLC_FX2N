#ifndef Gray_h
#define Gray_h
#include <stdio.h>

#ifdef __cplusplus
extern "C"  {
#endif
/**
*Convert binary to gray code
*@param lStart lValue lower bound of the interval
*@param lEnd lValue upper bound of the interval
*@param lValue The real value of the binary number to convert
*@return returns the real value of the binary number corresponding to the gray code
*@see g2b() g2b Gray code to binary
*@see BtoG() BtoG binary conversion gray code
*@see GtoB() BtoG Gray code to binary
*@useage a=b2g(0,15,4); //Get the binary value of the Gray code corresponding to 4, the result a is equal to 6
*The value of @memo lValue must be in the interval [lStart, lEnd], otherwise the required result cannot be obtained. Correspondingly, if the interval is smaller, the result can be obtained
*The less time it takes. And the value of lStart, lEnd must be 2 to the Nth power minus 1. Usually lStart is 0. For convenience
*Its value, it is recommended to use the BtoG() function to operate. However, this will increase the calculation time to 120%~180% of the original.
*/
unsigned int b2g(unsigned int lStart,unsigned int lEnd,unsigned int lValue)
{
 unsigned int Start=lStart,End=lEnd,Temp=0,Counter=0;
 unsigned char Type=1;
 while(Start<End)
  {
   Temp=(End+Start-1)>>1;
   if (lValue<=Temp)
   {
    if(!Type)
     Counter+=((End-Start+1)>>1);
    End=Temp;
    Type=1;
   }
   else
   {
    if(Type)
     Counter+=((End-Start+1)>>1);
    Start=++Temp;
    Type=0;
   }
  }
 return Counter;
}
/**
 * 格雷码转换成二进制
 * @param lStart lValue对应二进制数所在区间下界
 * @param lEnd lValue对应二进制数所在区间上界
 * @param lValue 要转换的格雷码的实值
 * @return 返回二进制数对应的格雷码的实值
 * @see b2g()   b2g  二进制转换格雷码
 * @see BtoG()  BtoG 二进制转换格雷码
 * @see GtoB()  BtoG 格雷码转换二进制
 * @useage a=b2g(0,15,6); //取得6所对应二进制值的格雷码 结果a等于4
 * @memo lValue对应二进制数的值必须在区间[lStart,lEnd]里,否则无法求得所求结果.相应地,如果区
 *       间越小,求得结果所用的时间就越少.而且lStart,lEnd的值必须为2的N次方减1. 通常lStart为0.
 *       为了方便求得其值,建议使用GtoB()函数来进行操作.但会使计算时间加长到原来的105%~140%.
 */
unsigned int g2b(unsigned int lStart,unsigned int lEnd,unsigned int lValue)
{
 unsigned int Start=lStart,End=lEnd,Counter=0,Temp=0;
 unsigned char Type=1;
 while(Start<End)
  {
   Temp=Counter+((End-Start+1)>>1);
   if(Type^(lValue<Temp))
   { 
    if(Type) Counter=Temp;
    Start=(Start+End+1)>>1;
    Type=0;
   }
   else
   {
    if(!Type) Counter=Temp;
    End=(Start+End-1)>>1;
    Type=1;
   }
  }
 return Start;
}

//=======================================================================================================
// 函数名称:  int BtoG(unsigned int lValue)
// 功能描述： 二进制转换成格雷码
// 输　入:  输入二进制      
// 输　出:  输出格雷码     
// 全局变量:  
// 调用模块: 
// 作　者:  
// 日　期:  2014年8月4日
// 备  注:  
//=======================================================================================================
signed int BtoG(unsigned int lValue)
{
 register unsigned int lV=lValue,lMax=1;
 while (lV>0)
 {
  lV>>=1;
  lMax<<=1;
 }
 if (lMax==0) return -1;
 return b2g(0,--lMax,lValue);
}

//=======================================================================================================
// 函数名称:  int GtoB(unsigned int lValue)
// 功能描述：格雷码转换成二进制
// 输　入:  输入格雷码    
// 输　出:  输出二进制     
// 全局变量:  
// 调用模块: 
// 作　者:  
// 日　期:   2014年8月4日
// 备  注:  
//=======================================================================================================
signed int GtoB(unsigned int lValue)
{
 register unsigned int lV=lValue,lMax=1;
 while (lV>0)
 {
  lV>>=1;
  lMax<<=1;
 }
 if (lMax==0) return -1;
 return g2b(0,--lMax,lValue);
}	
#ifdef __cplusplus
}
#endif

#endif
