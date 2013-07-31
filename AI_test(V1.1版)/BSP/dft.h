#ifndef DFT_H
#define DFT_H
//������Чֵ���������㷨��

typedef struct 
{

	double 	am;         //��ֵ
	float 	phase;	  	//���
	float an_real;			//�Ƕȵ�ʵ��
	float an_imag;			//�Ƕȵ��鲿
	float real;		//У����ʵ��
	float imag;		//У�����鲿
	short zero;				//��Ư
	unsigned short count;   //ƽ������
	float thd;     //����г������
}MEA_DFT;





typedef struct
{
	unsigned short ie;
	MEA_DFT mua[21];
	MEA_DFT mub[21];
	MEA_DFT muc[21];
    MEA_DFT mia[21];
	MEA_DFT mib[21];
	MEA_DFT mic[21];
	MEA_DFT muab[21];
	MEA_DFT mubc[21];
	MEA_DFT muca[21];

   
	MEA_DFT mi2;
    MEA_DFT mi0;
    float pwr;
    float rpwr;
	double pwh;
	double nwh;
	double wh;
    double pvarh;
	double nvarh;
	double varh;
	float cos;
	float frequence;
} Mea_Para;



typedef struct 		   //�����ṹ��
{
	float real;
	float imag;
}complex;



void fun_complex_multiply(complex *f1,complex *f2,complex *f3);	  //�����˷�����
void fun_complex_add(complex *f1,complex *f2,complex *f3);		 //	�����ӷ�����
void fun_complex_sub(complex *f1,complex *f2,complex *f3);		 //������������
void fun_complex_assign(complex *f1,complex *f2);			     //������ֵ����
void FUN_JF(int phase_index,MEA_DFT *dft) ;



//DFT
//short TAB_DFT_64[15]={1019,1004,980,946,903,851,792,724,
 //                     650,569,483,392,297,200,100}��
//���ϼ������ṹ��



void FUN_DFT_64(int phase,MEA_DFT *dft);
void FUN_ANGLE_REF(int number,MEA_DFT *first,...);
void DEFINE_SAMPLE_SUB(int phase_a,int phase_b,int phase_ab);
void TOTAL_MEASURE(Mea_Para *measure);
void SequenceFilter_2(Mea_Para *measure);
void SequenceFilter_0(Mea_Para *measure);
void ValueScaling(float *pMeaTab,Mea_Para *measure);
void FUN_FFT_64(int phase,MEA_DFT *FFT);
void FUN_Reverse(complex *h);
void FUN_Power_CAL(Mea_Para *p,int phase_u,MEA_DFT *u,int phase_i,MEA_DFT *i);
#endif


