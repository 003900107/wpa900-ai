#include "dft.h"
#include <math.h>
#include <stdarg.h>
#include "adc_calc.h"
#include "bsp.h"

extern int16_t PeriodCycleTab[13][SAMP_POINT_NBR];
extern Setting SetCurrent;
//extern uint16_t Amp_Coef[10];
extern uint32_t Ang_Coef[10];
extern int32_t FreqCounter;
 
//旋转因子查表

//扩大1024倍
complex twiddle[SAMP_POINT_NBR/2]={{1024,0},{1019,-100},{1004,-200},{980,-297},{946,-392},{903,-483},
                                   {851,-569},{792,-650},{724,-724},{650,-792},{569,-851},{483,-903},
								   {392,-946},{297,-980},{200,-1004},{100,-1019},{0,-1024},{-100,-1019},
								   {-200,-1004},{-297,-980},{-392,-946},{-483,-903},{-569,-851},{-650,-792},
								   {-724,-724},{-792,-650},{-851,-569},{-903,-483},{-946,-392},{-980,-297},
								   {-1004,-200},{-1019,-100}} ;



double FFT_SCALE[21]={1,1,1,1000,1,1000,1000,1000000,1,1000,1000,
                            1000000,1000,1000000,1000000,1000000000,1,1000,1000,1000000,1000};


//计算有效值（均方根算法）
void FUN_JF(int phase_index,MEA_DFT *dft)
{

	int i,j;
	float avg=0;
		
	for(i=0;i<SAMP_POINT_NBR;i++)
	{
	//	(dft->zero)+=PeriodCycleTab[phase][i];
          avg += (PeriodCycleTab[phase_index][i]- avg) / (i+1);
		  
	}
     
//	dft->zero=(dft->zero)/64; 
 	dft->zero=avg; 

	for(j=0;j<SAMP_POINT_NBR;j++)
		{
         (dft->am)+=(PeriodCycleTab[phase_index][j]-dft->zero)*(PeriodCycleTab[phase_index][j]-dft->zero);
	    }
	dft->am=((dft->am)*2)/64000000;     
	dft->am=sqrtf(dft->am);     // 最大值
} 



void FUN_DFT_64(int phase,MEA_DFT *dft)
{
    
	float Angcoef=(float)(Ang_Coef[phase]);
	float adjreal,adjimag,avg;
	int i;
	adjreal= cosf(Angcoef*PI/180);
	adjimag= sinf(Angcoef*PI/180);
    

	(dft->real)=1019*(PeriodCycleTab[phase][1]-PeriodCycleTab[phase][31]-PeriodCycleTab[phase][33]+PeriodCycleTab[phase][63]);
    (dft->real)=(dft->real)+1004*(PeriodCycleTab[phase][2]-PeriodCycleTab[phase][30]-PeriodCycleTab[phase][34]+PeriodCycleTab[phase][62]);
	(dft->real)=(dft->real)+980*(PeriodCycleTab[phase][3]-PeriodCycleTab[phase][29]-PeriodCycleTab[phase][35]+PeriodCycleTab[phase][61]);
	(dft->real)=(dft->real)+946*(PeriodCycleTab[phase][4]-PeriodCycleTab[phase][28]-PeriodCycleTab[phase][36]+PeriodCycleTab[phase][60]);
	(dft->real)=(dft->real)+903*(PeriodCycleTab[phase][5]-PeriodCycleTab[phase][27]-PeriodCycleTab[phase][37]+PeriodCycleTab[phase][59]);
	(dft->real)=(dft->real)+851*(PeriodCycleTab[phase][6]-PeriodCycleTab[phase][26]-PeriodCycleTab[phase][38]+PeriodCycleTab[phase][58]);
	(dft->real)=(dft->real)+792*(PeriodCycleTab[phase][7]-PeriodCycleTab[phase][25]-PeriodCycleTab[phase][39]+PeriodCycleTab[phase][57]);
	(dft->real)=(dft->real)+724*(PeriodCycleTab[phase][8]-PeriodCycleTab[phase][24]-PeriodCycleTab[phase][40]+PeriodCycleTab[phase][56]);
	(dft->real)=(dft->real)+650*(PeriodCycleTab[phase][9]-PeriodCycleTab[phase][23]-PeriodCycleTab[phase][41]+PeriodCycleTab[phase][55]);
	(dft->real)=(dft->real)+569*(PeriodCycleTab[phase][10]-PeriodCycleTab[phase][22]-PeriodCycleTab[phase][42]+PeriodCycleTab[phase][54]);
	(dft->real)=(dft->real)+483*(PeriodCycleTab[phase][11]-PeriodCycleTab[phase][21]-PeriodCycleTab[phase][43]+PeriodCycleTab[phase][53]);
	(dft->real)=(dft->real)+392*(PeriodCycleTab[phase][12]-PeriodCycleTab[phase][20]-PeriodCycleTab[phase][44]+PeriodCycleTab[phase][52]);
	(dft->real)=(dft->real)+297*(PeriodCycleTab[phase][13]-PeriodCycleTab[phase][19]-PeriodCycleTab[phase][45]+PeriodCycleTab[phase][51]);
	(dft->real)=(dft->real)+200*(PeriodCycleTab[phase][14]-PeriodCycleTab[phase][18]-PeriodCycleTab[phase][46]+PeriodCycleTab[phase][50]);
	(dft->real)=(dft->real)+100*(PeriodCycleTab[phase][15]-PeriodCycleTab[phase][17]-PeriodCycleTab[phase][47]+PeriodCycleTab[phase][49]);
	dft->real=(dft->real)/1024;
	(dft->real)=(dft->real)+PeriodCycleTab[phase][0]-PeriodCycleTab[phase][32];
	dft->imag=1019*(PeriodCycleTab[phase][47]-PeriodCycleTab[phase][15]-PeriodCycleTab[phase][17]+PeriodCycleTab[phase][49]);
	(dft->imag)=(dft->imag)+1004*(PeriodCycleTab[phase][46]-PeriodCycleTab[phase][14]-PeriodCycleTab[phase][18]+PeriodCycleTab[phase][50]);
	(dft->imag)=(dft->imag)+980*(PeriodCycleTab[phase][45]-PeriodCycleTab[phase][13]-PeriodCycleTab[phase][19]+PeriodCycleTab[phase][51]);
	(dft->imag)=(dft->imag)+946*(PeriodCycleTab[phase][44]-PeriodCycleTab[phase][12]-PeriodCycleTab[phase][20]+PeriodCycleTab[phase][52]);
	(dft->imag)=(dft->imag)+903*(PeriodCycleTab[phase][43]-PeriodCycleTab[phase][11]-PeriodCycleTab[phase][21]+PeriodCycleTab[phase][53]);
	(dft->imag)=(dft->imag)+851*(PeriodCycleTab[phase][42]-PeriodCycleTab[phase][10]-PeriodCycleTab[phase][22]+PeriodCycleTab[phase][54]);
	(dft->imag)=(dft->imag)+792*(PeriodCycleTab[phase][41]-PeriodCycleTab[phase][9]-PeriodCycleTab[phase][23]+PeriodCycleTab[phase][55]);
	(dft->imag)=(dft->imag)+724*(PeriodCycleTab[phase][40]-PeriodCycleTab[phase][8]-PeriodCycleTab[phase][24]+PeriodCycleTab[phase][56]);
	(dft->imag)=(dft->imag)+650*(PeriodCycleTab[phase][39]-PeriodCycleTab[phase][7]-PeriodCycleTab[phase][25]+PeriodCycleTab[phase][57]);
	(dft->imag)=(dft->imag)+569*(PeriodCycleTab[phase][38]-PeriodCycleTab[phase][6]-PeriodCycleTab[phase][26]+PeriodCycleTab[phase][58]);
	(dft->imag)=(dft->imag)+483*(PeriodCycleTab[phase][37]-PeriodCycleTab[phase][5]-PeriodCycleTab[phase][27]+PeriodCycleTab[phase][59]);
	(dft->imag)=(dft->imag)+392*(PeriodCycleTab[phase][36]-PeriodCycleTab[phase][4]-PeriodCycleTab[phase][28]+PeriodCycleTab[phase][60]);
	(dft->imag)=(dft->imag)+297*(PeriodCycleTab[phase][35]-PeriodCycleTab[phase][3]-PeriodCycleTab[phase][29]+PeriodCycleTab[phase][62]);
	(dft->imag)=(dft->imag)+100*(PeriodCycleTab[phase][33]-PeriodCycleTab[phase][1]-PeriodCycleTab[phase][31]+PeriodCycleTab[phase][63]);
	dft->imag=(dft->imag)/1024;
	(dft->imag)=(dft->imag)+PeriodCycleTab[phase][48]-PeriodCycleTab[phase][16];
	dft->real=(dft->real)/32000;
	dft->imag=(dft->imag)/32000;
	(dft->real)=adjreal*(dft->real) - adjimag*(dft->imag);
    (dft->imag)=adjreal*(dft->imag) + adjimag*(dft->real);

	dft->am=(dft->real)*(dft->real)+(dft->imag)*(dft->imag);
    dft->am=sqrtf(dft->am);
	
	if (dft->am>0)
	{
	dft->phase=acosf(dft->real/dft->am)*180/PI;
	if(dft->imag<0) dft->phase = 360 - dft->phase;	
	

	}
	else
	dft->phase=0;
	for(i=0;i<SAMP_POINT_NBR;i++)
	{
	  avg += (PeriodCycleTab[phase][i]- avg) / (i+1);
	}
     

 	dft->zero=avg;		 
}

void FUN_ANGLE_REF(int number,MEA_DFT *first,...)
{
	int i;
	MEA_DFT *next;
	va_list argp;
	va_start(argp, first);

 	if( (first->am) > 0.1 )
	{
		for( i=0;i<(number-1);i++ )
		{  
			next=va_arg(argp,MEA_DFT *);
			if((next->am)>0.1 )
			{
				if( (next->phase) > (first->phase) )
				{
					(next->phase)=(next->phase)-(first->phase);
				}
				else (next->phase)=360-(first->phase) + (next->phase);
			}
		}
		first->phase=0;
	}
	va_end(argp);
}


void DEFINE_SAMPLE_SUB(int phase_a,int phase_b,int phase_ab)
{
	uint8_t i;
    for(i=0;i<SAMP_POINT_NBR;i++)
    	{
    	 PeriodCycleTab[phase_ab][i]=PeriodCycleTab[phase_a][i] - PeriodCycleTab[phase_b][i];
    	}
}

void FUN_Reverse(complex *h)		  //FFT结果序列倒置
{
  int i,j,k;
  complex t;

  for(i=1,j=SAMP_POINT_NBR/2;i<=SAMP_POINT_NBR-2;i++)
	  {
		  if(i<j)
		  {
			  fun_complex_assign(&h[j],&t);     //t=h[j];
			  fun_complex_assign(&h[i],&h[j]);     //h[j]=h[i];
			  fun_complex_assign(&t,&h[i]);     //h[i]=t;
		  }
		  k=SAMP_POINT_NBR/2;
		  while(k<=j)
		  {
			  j=j-k;
			  k=k/2;
		  }
		  j=j+k;
	  }
}



void fun_complex_multiply(complex *f1,complex *f2,complex *f3)	  //复数乘法运算
{

  f3->real=(f1->real)*(f2->real)-(f1->imag)*(f2->imag);
  f3->imag=(f1->real)*(f2->imag)+(f1->imag)*(f2->real);
 
}


void fun_complex_add(complex *f1,complex *f2,complex *f3)		 //	复数加法运算
{

   f3->real=(f1->real)+(f2->real);
   f3->imag=(f1->imag)+(f2->imag);

}



void fun_complex_sub(complex *f1,complex *f2,complex *f3)		 //	复数减法运算
{

   f3->real=(f1->real)-(f2->real);
   f3->imag=(f1->imag)-(f2->imag);

}



void fun_complex_assign(complex *f1,complex *f2)			     //复数赋值运算
{
   f2->real=f1->real;
   f2->imag=f1->imag;

}


void FUN_FFT_64(int phase,MEA_DFT *FFT)		 //64点FFT
{

   complex f[SAMP_POINT_NBR];
   complex t,s;
   int j,m,i,n,a,b;
   double am;
   float thd;
   int k=log(SAMP_POINT_NBR)/log(2);

  
   for(j=0;j<SAMP_POINT_NBR;j++)        //定义时域采样数组
   	{
     f[j].real=PeriodCycleTab[phase][j];
     f[j].imag=0;
    }
   for(m=1;m<=k;m++)        //分级
   	{
       int node=pow(2,k+1-m);   //M级每个分租所含节点数
	   int unit=node/2;         //每个分组所含蝶形单元数
	   for(i=1;i<=unit;i++)
	   	{
          int r=(i-1)*pow(2,m-1);
	      for(n=i-1;n<SAMP_POINT_NBR-1;n=n+node) //遍历每个分组
	      	{
              int number=n+unit;	   //number代表每个蝶形单元的上下编号
			  fun_complex_add(&f[n],&f[number],&t);					  //t=f[n]+f[number];
			  fun_complex_sub(&f[n],&f[number],&s);                //s=f[n]-f[number];	  
			  fun_complex_multiply(&s,&twiddle[r],&f[number]);	  //f[number]=s[n]*twiddle[r];
              fun_complex_assign(&t,&f[n]);							  //f[n]=t;
		    }
	    }
    }
  FUN_Reverse(f);      //结果倒置
  for(a=0;a<=20;a++)		//计算20次谐波幅值与相角,20次以上谐波已经过150欧电阻，1NF电容滤掉
  {

	  f[a].real=(f[a].real)/1024;
	  f[a].real=(f[a].real)/32000;
	  f[a].real=(f[a].real)*SetCurrent.ChannelCoef[phase]/1000.0;    
	  f[a].imag=(f[a].imag)/1024;
	  f[a].imag=(f[a].imag)/32000;
	  f[a].imag=(f[a].imag)*SetCurrent.ChannelCoef[phase]/1000.0; 
	  am=sqrtf((f[a].real)*(f[a].real)+(f[a].imag)*(f[a].imag));
      am=am/FFT_SCALE[a];   //乘以运放的放大倍数,再除以FFT的谐波系数
	  FFT[a].am=am;
	  FFT[a].real=f[a].real;
	  FFT[a].imag=f[a].imag;
       if ((FFT[a].am)>0)
    	{
          FFT[a].phase=acosf((f[a].real)/(FFT[a].am))*180/PI;
	      if(f[a].imag<0) 
	      FFT[a].phase = 360 - (FFT[a].phase);	
	    }
	   else
	      FFT[a].phase=0;

	}
//计算谐波畸变率
  for(b=2;b<21;b++)
  {
  	 thd+=(FFT[b].am)*(FFT[b].am);
  }
  thd=sqrtf(thd) ;
  thd=thd/(FFT[1].am);
  FFT[1].thd=thd;
}

//用采样点计算有功功率
 void FUN_Power_CAL(Mea_Para *p,int phase_u,MEA_DFT *u,int phase_i,MEA_DFT *i)
{
  int j;
  
     for(j=0;j<SAMP_POINT_NBR;j++)
     {
       (p->pwr)+= (PeriodCycleTab[phase_u][j]-(u->zero))*(PeriodCycleTab[phase_i][j]-(i->zero))	;
     }
  	 
	 
}


void TOTAL_MEASURE(Mea_Para *measure)
{
	float p,q,sample_time;
	/*double uscale,iscale;
	uscale=SetCurrent.ChannelCoef[0]*SetCurrent.ChannelCoef[1]*SetCurrent.ChannelCoef[2]/1000000000.0;
	iscale=SetCurrent.ChannelCoef[3]*SetCurrent.ChannelCoef[4]*SetCurrent.ChannelCoef[5]/1000000000.0; */

   	  DEFINE_SAMPLE_SUB(UA,UB,UAB);
   	  DEFINE_SAMPLE_SUB(UB,UC,UBC);
   	  DEFINE_SAMPLE_SUB(UC,UA,UCA);

	 //用FFT算谐波和角度
	  FUN_FFT_64(UA,measure->mua);
   	  FUN_FFT_64(UB,measure->mub);
   	  FUN_FFT_64(UC,measure->muc);
   	  FUN_FFT_64(UAB,measure->muab);
   	  FUN_FFT_64(UBC,measure->mubc);
	  FUN_FFT_64(UCA,measure->muca);
	  FUN_FFT_64(IA,measure->mia);
	  //if(measure->mia[1].am>1.570) FUN_FFT_64(IAP,measure->mia);
	  FUN_FFT_64(IB,measure->mib);
	  //if(measure->mib[1].am>1.570) FUN_FFT_64(IBP,measure->mib);
	  FUN_FFT_64(IC,measure->mic);
	  //if(measure->mic[1].am>1.570) FUN_FFT_64(ICP,measure->mic);

	  //用均方根算基波幅值
   	  FUN_JF(UA,&measure->mua[1]);
   	  FUN_JF(UB,&measure->mub[1]);
   	  FUN_JF(UC,&measure->muc[1]);
   	  FUN_JF(UAB,&measure->muab[1]);
   	  FUN_JF(UBC,&measure->mubc[1]);
	  FUN_JF(UCA,&measure->muca[1]);
	  FUN_JF(IA,&measure->mia[1]);
	  //if(measure->mia[1].am>1.570) FUN_JF(IAP,&measure->mia[1]);
	  FUN_JF(IB,&measure->mib[1]);
	 // if(measure->mib[1].am>1.570) FUN_JF(IBP,&measure->mib[1]);
	  FUN_JF(IC,&measure->mic[1]);
	  //if(measure->mic[1].am>1.570) FUN_JF(ICP,&measure->mic[1]);
	 // if(measure->mic.am>6.570) FUN_DFT_64(ICP,&measure->mic);

	  
   	  measure->frequence=100000.0/FreqCounter;
   
   
   FUN_ANGLE_REF(9,&measure->mua[1],&measure->mub[1],&measure->muc[1],&measure->muab[1],&measure->mubc[1],\
   &measure->muca[1],&measure->mia[1],&measure->mib[1],&measure->mic[1]);

  /*//三瓦法计算有功功率
 
   FUN_Power_CAL(measure,UA,&measure->mua[1],IA,&measure->mia[1]);
   FUN_Power_CAL(measure,UB,&measure->mub[1],IB,&measure->mib[1]);
   FUN_Power_CAL(measure,UC,&measure->muc[1],IC,&measure->mic[1]);
   
   measure->pwr=(measure->pwr)*2/1000000/SAMP_POINT_NBR ;
   (measure->pwr)*=uscale*iscale;
   p=measure->pwr;*/

   p=(measure->mua[1].real)*(measure->mia[1].real)+(measure->mua[1].imag)*(measure->mia[1].imag)+
		(measure->mub[1].real)*(measure->mib[1].real)+(measure->mub[1].imag)*(measure->mib[1].imag)+
		(measure->muc[1].real)*(measure->mic[1].real)+(measure->muc[1].imag)*(measure->mic[1].imag);

   q=(measure->mua[1].imag)*(measure->mia[1].real)-(measure->mua[1].real)*(measure->mia[1].imag)+
		(measure->mub[1].imag)*(measure->mib[1].real)-(measure->mub[1].real)*(measure->mib[1].imag)+
		(measure->muc[1].imag)*(measure->mic[1].real)-(measure->muc[1].real)*(measure->mic[1].imag);

   
   measure->pwr=p;
   measure->rpwr=q;
   
		   //电压或电流小时，不计算功率。
		   if( ( measure->mua[1].am > 0.18 ) || ( measure->mub[1].am > 0.18 ) || ( measure->muc[1].am > 0.18 ) )
		   {
			   if( measure->ie >5 )
			   {
				   if( ( measure->mia[1].am > 0.01*measure->ie ) || ( measure->mib[1].am >  0.01*measure->ie ) || \
				    ( measure->mic[1].am >	0.01*measure->ie ) )
				   {
   
				   }
				   else  //ie>5 时，电流大于1%计算电度 
				   {
					   measure->cos=0;
					   return;
				   }
			   }
			   else if( ( measure->mia[1].am > 0.03 ) || ( measure->mib[1].am >  0.03 ) ||  ( measure->mic[1].am >  0.03 ) ) 
			   {
   
			   }
			   else   //ie<=5 时，电流大于0.03计算电度 
			   {
					   measure->cos=0;
					   return;
			   }
   
			   //计算功率因数
			   sample_time=8*(TIM2->ARR+1)/700;
			   measure->cos=sqrtf( p*p/( p*p+q*q ) );
			   if(	(p*p+q*q) < 1 ) measure->cos=0;
	   
			   //电度量计算
			   if( p > 2 ) 
			   {
				   measure->pwh+=p*sample_time*100/3600000;
			   }
			   else if( p < (-2) ) 
			   {
				   measure->nwh-=p*sample_time*100/3600000;
			   }
	   
			   if( q > 2 ) 
			   {
				   measure->pvarh+=q*sample_time*100/3600000;
			   }
			   else if( q < (-2) ) 
			   {
				   measure->nvarh-=q*sample_time*100/3600000;
			   }
		   
			   measure->wh=measure->pwh+measure->nwh;
			   measure->varh=measure->pvarh+measure->nvarh;
				   
			   //Pulse_save();
		   }
		   else measure->cos=0;
}
//零序电流
void SequenceFilter_0(Mea_Para *measure)
{
  measure->mi0.real=measure->mia[1].real+measure->mib[1].real+measure->mic[1].real;
  measure->mi0.real/=3;
  measure->mi0.imag=measure->mia[1].imag+measure->mib[1].imag+measure->mic[1].imag;
  measure->mi0.imag/=3;
  measure->mi0.am=sqrtf((measure->mi0.imag)*(measure->mi0.imag)+(measure->mi0.real)*(measure->mi0.real));
 
}
//负序电流
void SequenceFilter_2(Mea_Para *measure)
{
  measure->mi2.real=measure->mia[1].real-(measure->mib[1].real+measure->mic[1].real)/2+0.866*(measure->mib[1].imag-measure->mic[1].imag);
  measure->mi2.real/=3;
  measure->mi2.imag=measure->mia[1].imag-(measure->mib[1].imag+measure->mic[1].imag)/2-0.866*(measure->mib[1].real-measure->mic[1].real);
  measure->mi2.imag/=3;
  measure->mi2.am=sqrtf((measure->mi2.imag)*(measure->mi2.imag)+(measure->mi2.real)*(measure->mi2.real));
}

float PhasePwr(float *p)
{	
	float result;
	float cosine;
	float angle;
	angle=*(p+1)-*(p+7) ;
	cosine=cosf(angle*PI/180);
	result=(*p)*(*(p+6))*cosine;
	return result;
}

float PhaseVar(float *p)
{	
	float result; 
	result=(*p)*(*(p+6))*(sinf((*(p+1)-*(p+7))*PI/180));
	return result;
}

void ValueScaling(float *pMeaTab,Mea_Para *measure)
{
	float *pBaseMeaTab;
    pBaseMeaTab=pMeaTab;

    /*1*/*(pMeaTab++)=measure->mua[1].am*SetCurrent.ChannelCoef[0]*2/45;
    /*2*/*(pMeaTab++)=measure->mua[1].phase;
    /*3*/*(pMeaTab++)=measure->mub[1].am*SetCurrent.ChannelCoef[1]*2/45;
    /*4*/*(pMeaTab++)=measure->mub[1].phase;
    /*5*/*(pMeaTab++)=measure->muc[1].am*SetCurrent.ChannelCoef[2]*2/45;
    /*6*/*(pMeaTab++)=measure->muc[1].phase;
    
    /*7*/*(pMeaTab++)=measure->mia[1].am*SetCurrent.ChannelCoef[3]*25/7000;
    /*8*/*(pMeaTab++)=measure->mia[1].phase;
    /*9*/*(pMeaTab++)=measure->mib[1].am*SetCurrent.ChannelCoef[4]*25/7000;
   /*10*/*(pMeaTab++)=measure->mib[1].phase;
   /*11*/*(pMeaTab++)=measure->mic[1].am*SetCurrent.ChannelCoef[5]*25/7000;
   /*12*/*(pMeaTab++)=measure->mic[1].phase;
    
   /*13*/*(pMeaTab++)=measure->muab[1].am*SetCurrent.ChannelCoef[6]*2/45;
   /*14*/*(pMeaTab++)=measure->mubc[1].am*SetCurrent.ChannelCoef[7]*2/45;
   /*15*/*(pMeaTab++)=measure->muca[1].am*SetCurrent.ChannelCoef[8]*2/45;
   if(*pBaseMeaTab>10.0)
   /*16*/*(pMeaTab++)=measure->frequence;
   else
	     *(pMeaTab++)=0.0;
   /*17*/*(pMeaTab++)=measure->pwr*10000/63;//PhasePwr(pBaseMeaTab)+PhasePwr(pBaseMeaTab+2)+PhasePwr(pBaseMeaTab+4);//
   /*18*/*(pMeaTab++)=measure->rpwr*10000/63;//PhaseVar(pBaseMeaTab)+PhaseVar(pBaseMeaTab+2)+PhaseVar(pBaseMeaTab+4);//
   /*19*/*(pMeaTab++)=measure->cos;
   /*20*/*(pMeaTab++)=measure->mi0.am*25/7;
   /*21*/*(pMeaTab)=measure->mi2.am*25/7;

 
}

