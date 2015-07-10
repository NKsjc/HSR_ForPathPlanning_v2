#include "HICDataHelper.h"
#include <string.h>
#include <stdio.h>
#include<stdlib.h>

//创建一个数据包
HICDataPack *HICCreatePack()
{
	//创建一个结构并初始化参数
	HICDataPack *pPack = (HICDataPack *)malloc(sizeof(HICDataPack));

	pPack->nDeviceType = 0;
	pPack->nDeviceName = 0;
	pPack->nDataType = 0;
	pPack->nDataLength = 0;
	pPack->nHeader = 1;
	pPack->nContentRealLength = 0;

	pPack->pContent = 0;

	return pPack;
}

//此函数调用后保证一件事情：nDataLength + nAddSize <= nContentRealLength
void ResizeData(HICDataPack *pPack, int nAddSize)
{
	unsigned char *pOldPack;

	//如果空间足够，那么不做任何事情
	if(pPack->nDataLength + nAddSize <= pPack->nContentRealLength)
		return;

	//如果还没有空间，直接new nAddSize大小的一个数组
	if(pPack->pContent == 0)
	{
		//保证new的空间为2的n次方
		int nNewSize = 2;
		while(nNewSize < nAddSize)
		{
			nNewSize *= 2;
		}

		//创建空间
		pPack->pContent = (unsigned char *)malloc(nNewSize * sizeof(char));
		pPack->nContentRealLength = nNewSize;
		return;
	}

	//保存之前的数据指针
	pOldPack = pPack->pContent;

	//不断2倍扩大，直至有足够空间
	do
	{
		pPack->nContentRealLength *= 2;
	}while(pPack->nContentRealLength < pPack->nDataLength + nAddSize);
	
	//生成足够大的空间
	pPack->pContent = (unsigned char *)malloc(pPack->nContentRealLength * sizeof(unsigned char));

	//拷贝有效数据	
// 	for(int i = 0; i<pPack->nDataLength; i++)
// 	{
// 		pPack->pContent[i] = pOldPack[i];
// 	}

	memcpy(pPack->pContent, pOldPack, pPack->nDataLength);

	free(pOldPack);
}

//释放数据包
void HICReleasePack(HICDataPack *pPack)
{
	free(pPack->pContent);
	free(pPack);
}

//包头相关操作
void HICSetDeviceType(HICDataPack *pPack, unsigned char nValue)
{
	pPack->nDeviceType = nValue;
}

void HICSetDeviceName(HICDataPack *pPack, short nValue)
{
	pPack->nDeviceName = nValue;
}

void HICSetDataType(HICDataPack *pPack, unsigned char nValue)
{
	pPack->nDataType = nValue;
}
//Data操作

//单值操作
void HICAddChar(HICDataPack *pPack, char nValue)
{
	ResizeData(pPack, sizeof(char));

	pPack->pContent[pPack->nDataLength] = nValue;

	pPack->nDataLength += sizeof(char);
}

void HICAddShort(HICDataPack *pPack, short nValue)
{
	ResizeData(pPack, sizeof(short));

	memcpy(&(pPack->pContent[pPack->nDataLength]), &nValue, sizeof(short));

	pPack->nDataLength += sizeof(short);
}

void HICAddInt(HICDataPack *pPack, int nValue)
{
	ResizeData(pPack, sizeof(int));
	
	memcpy(&(pPack->pContent[pPack->nDataLength]), &nValue, sizeof(int));

	pPack->nDataLength += sizeof(int);
}

void HICAddFloat(HICDataPack *pPack, float fValue)
{
	ResizeData(pPack, sizeof(float));
	
	memcpy(&(pPack->pContent[pPack->nDataLength]), &fValue, sizeof(float));

	pPack->nDataLength += sizeof(float);
}

//数组操作
void HICAddCharArray(HICDataPack *pPack, char *pValue, int nLength)
{
	int nAddLength = sizeof(char) * nLength;

	ResizeData(pPack, nAddLength);

	memcpy(&(pPack->pContent[pPack->nDataLength]), pValue, nAddLength);

	pPack->nDataLength += nAddLength;
}

void HICAddShortArray(HICDataPack *pPack, short *pValue, int nLength)
{
	int nAddLength = sizeof(short) * nLength;

	ResizeData(pPack, nAddLength);

	memcpy(&(pPack->pContent[pPack->nDataLength]), pValue, nAddLength);

	pPack->nDataLength += nAddLength;
}

void HICAddIntArray(HICDataPack *pPack, int *pValue, int nLength)
{
	int nAddLength = sizeof(int) * nLength;

	ResizeData(pPack, nAddLength);

	memcpy(&(pPack->pContent[pPack->nDataLength]), pValue, nAddLength);

	pPack->nDataLength += nAddLength;
}

void HICAddFloatArray(HICDataPack *pPack, float *pValue, int nLength)
{
	int nAddLength = sizeof(float) * nLength;

	ResizeData(pPack, nAddLength);

	memcpy(&(pPack->pContent[pPack->nDataLength]), pValue, nAddLength);

	pPack->nDataLength += nAddLength;
}

unsigned short CheckSum(unsigned short *buffer, int size)
{
	int cksum = 0;
	int counter = 0;

	while (size > 0)
	{
		unsigned short val = buffer[counter];
		cksum += (int)buffer[counter];
		counter += 1;
		size -= 1;
	}

	cksum = (cksum >> 16) + (cksum & 0xffff);
	cksum += (cksum >> 16);
	return (unsigned short)(~cksum);
}

unsigned char *HICPackToByte(HICDataPack *pPack, int *nDataLength)
{
	int nPackTotalLength;
	//是否需要补0
	int bNeedAddZero = 0;

	unsigned char *pContext;

	int nDevicePos;
	int nDeviceTypePos;
	int nDataTypePos;
	int nDataLengthPos;
	int nContentPos;

	unsigned short *pCheckData;

	unsigned short nCheckSum;

	if(pPack->nHeader == 0 ||
		pPack->nDeviceType == 0 ||
		pPack->nDeviceName == 0 ||
		pPack->nDataType == 0 ||
		pPack->nDataLength == 0)
	{
		nDataLength = 0;
		return 0;
	}

	nPackTotalLength = sizeof(short) +	//1.包头
		sizeof(unsigned char) +				//2.设备类型
		sizeof(short) + 					//3.设备编号
		sizeof(unsigned char) +				//4.数据类型
		sizeof(short) +						//5.数据长度
		pPack->nDataLength +				//6.数据内容长度 
		sizeof(short);						//7.校验和

	//保证是16位的整数倍
	if(nPackTotalLength % 2 != 0)
	{
		nPackTotalLength += 1;
		bNeedAddZero = 1;
	}

	//数据包内容
	pContext = (unsigned char*)malloc(nPackTotalLength * sizeof(unsigned char));

	nDevicePos = sizeof(short);
	nDeviceTypePos = sizeof(short) + sizeof(unsigned char);
	nDataTypePos = nDeviceTypePos + sizeof(short);
	nDataLengthPos = nDataTypePos + sizeof(unsigned char);
	nContentPos = nDataLengthPos + sizeof(short);

	//拷贝各个数据
	memcpy(pContext, &(pPack->nHeader), sizeof(short));
	memcpy(&(pContext[nDevicePos]), &(pPack->nDeviceType), sizeof(unsigned char));
	memcpy(&(pContext[nDeviceTypePos]), &(pPack->nDeviceName), sizeof(short));
	memcpy(&(pContext[nDataTypePos]), &(pPack->nDataType), sizeof(unsigned char));
	memcpy(&(pContext[nDataLengthPos]), &(pPack->nDataLength), sizeof(short));
	memcpy(&(pContext[nContentPos]), pPack->pContent, pPack->nDataLength);

	//凑够16byte，最后一个byte补加0
	if(bNeedAddZero == 1)
		pContext[nPackTotalLength - 3] = 0;

	//CheckSum首先置0
	pContext[nPackTotalLength - 2] = 0;
	pContext[nPackTotalLength - 1] = 0;

	pCheckData = (unsigned short *)pContext;

	nCheckSum = CheckSum(pCheckData, nPackTotalLength/2);

	//写计算出的CheckSum
	pContext[nPackTotalLength - 2] = (unsigned char)nCheckSum;
	pContext[nPackTotalLength - 1] = nCheckSum >> 8;

	*nDataLength = nPackTotalLength;

	return pContext;
}

void HICResetPack(HICDataPack *pPack)
{
	pPack->nDeviceType = 0;
	pPack->nDeviceName = 0;
	pPack->nDataType = 0;
	pPack->nDataLength = 0;
	pPack->nHeader = 0;
	//pPack->nContentRealLength = 0;

	pPack->pContent = 0;
}

unsigned int g_nCurID = 0;

//注：最多只能同时注册255个数据包，多了的话将不恩能够创建
unsigned int g_nMaxID = 255;
HICDataPack **g_pPackBlank = 0;

unsigned int AutoGenerateID()
{
	unsigned int i = 0;

	//第一次初始化
	if(g_pPackBlank == 0)
	{
		g_pPackBlank = (HICDataPack **)malloc(g_nMaxID * sizeof(HICDataPack *));

		for(i = 0; i<g_nMaxID; i++)
		{
			g_pPackBlank[i] = 0;
		}
	}

	//如果超过最大值，从之前查找
	if(g_nCurID >= g_nMaxID)
	{
		for(i = 0; i<g_nCurID; i++)
		{
			if(g_pPackBlank[i] == 0)
			{
				g_nCurID = i;
			}
		}
	}
	else
	{
		do
		{
			g_nCurID++;
		}
		while(g_pPackBlank[g_nCurID] != 0);
	}

	return g_nCurID;
}

HICDataPack *GetSafePack(unsigned int nID)
{
	if(nID > g_nMaxID)
		return 0;

	return g_pPackBlank[nID];
}

unsigned int HICCreatePackS()
{
	unsigned int nID = AutoGenerateID();

	if(nID < g_nMaxID)
	{
		g_pPackBlank[nID] = HICCreatePack();
		return nID;
	}
	else
		return g_nMaxID;
}

void HICReleasePackS(unsigned int nPackID)
{
	HICDataPack *pPack;

	if(nPackID > g_nMaxID)
		return;

	pPack = g_pPackBlank[nPackID];

	if(pPack)
	{
		HICReleasePack(pPack);
		g_pPackBlank[nPackID] = 0;
	}
}

void HICSetDeviceTypeS(unsigned int nPackID, unsigned char nValue)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICSetDeviceType(pPack, nValue);
	}
}

void HICSetDeviceNameS(unsigned int nPackID, short nValue)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICSetDeviceName(pPack, nValue);
	}
}

void HICSetDataTypeS(unsigned int nPackID, unsigned char nValue)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICSetDataType(pPack, nValue);
	}
}

void HICAddCharS(unsigned int nPackID, char nValue)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICAddChar(pPack, nValue);
	}
}

void HICAddShortS(unsigned int nPackID, short nValue)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICAddShort(pPack, nValue);
	}
}

void HICAddIntS(unsigned int nPackID, int nValue)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICAddInt(pPack, nValue);
	}
}

void HICAddFloatS(unsigned int nPackID, float fValue)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICAddFloat(pPack, fValue);
	}
}

void HICAddCharArrayS(unsigned int nPackID, char *pValue, int nLength)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICAddCharArray(pPack, pValue, nLength);
	}
}

void HICAddShortArrayS(unsigned int nPackID, short *pValue, int nLength)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICAddShortArray(pPack, pValue, nLength);
	}
}

void HICAddIntArrayS(unsigned int nPackID, int *pValue, int nLength)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICAddIntArray(pPack, pValue, nLength);
	}
}

void HICAddFloatArrayS(unsigned int nPackID, float *pValue, int nLength)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICAddFloatArray(pPack, pValue, nLength);
	}
}

unsigned char *HICPackToByteS(unsigned int nPackID, int *nDataLength)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		return HICPackToByte(pPack, nDataLength);
	}
	else
		return 0;
}

void HICResetPackS(unsigned int nPackID)
{
	HICDataPack *pPack = GetSafePack(nPackID);

	if(pPack)
	{
		HICResetPack(pPack);
	}
}

int HICPackServerData(unsigned char *cBuf, unsigned int nLength, unsigned char *cBufOut)
{
	int nOutLength = 0;
	unsigned char *pRet;

	HICDataPack *pPack = HICCreatePack();

	pPack->nHeader = 1;

	//服务器的类型规定为255
	HICSetDeviceType(pPack, 255);

	//设备名称也规定为65535
	HICSetDeviceName(pPack, 255);

	//数据类型为1
	HICSetDataType(pPack, 2);

	pPack->pContent = (unsigned char *)malloc(nLength * sizeof(unsigned char));

	memcpy(pPack->pContent, cBuf, nLength);

	pPack->nDataLength = nLength;

	pRet = HICPackToByte(pPack, &nOutLength);

	//拷贝数据
	memcpy(cBufOut, pRet, nOutLength);

	//释放资源
	HICReleasePack(pPack);
	free(pRet);

	return nOutLength;
}

HICVideoPackTemp *g_pVideoTemp = 0;
unsigned short g_sFrameID = 0;

//转换处理图片的数据
HICVideoPackTemp *HICPackPicData(unsigned char *cBuf, int nLength, int nChannel, int nThreadID)
{
	//首先创建缓存用的数据
	if(g_pVideoTemp == 0)
	{
		//能够处理最大的一张图1024 * 1024
		g_pVideoTemp = (HICVideoPackTemp *)malloc(sizeof(HICVideoPackTemp));

		g_pVideoTemp->m_cVideoTemp = (char**)malloc(1024*sizeof(char*));
		for(int i = 0;i < 1024; i++)
		{
			g_pVideoTemp->m_cVideoTemp[i] = (char *)malloc(DEV_DEFAULT_VIDEOPACK_SIZE * sizeof(char));
		}
	}

	//数据包的数量
	int nPackNum = nLength / (DEV_DEFAULT_VIDEOPACK_SIZE - 12) + 1;

	//最后一个包的有效数据长度
	int nLastDataLength = nLength % ( DEV_DEFAULT_VIDEOPACK_SIZE -12);

	//1.Header
	unsigned short sHeader = 101;

	//2.Channel
	unsigned short sChannel = nChannel;

	//3.FrameID 当前策略下此值保留
	unsigned short sFrameID = g_sFrameID;
	g_sFrameID++;

	//4.PackNum
	unsigned short sShortPackNum = nPackNum;

	//5.PackID
	unsigned short sPackID = 0;

	//6.DataLength
	unsigned short sDataLength = 0;

	//拷贝前nPackNum - 1个数据
	for(int i = 0; i<nPackNum - 1; i++)
	{
		//更新包的ID，为包编一个序号
		sPackID = i;

		//数据长度是固定的，填满一个数据包
		sDataLength = 1024;

		//拷贝数据
		char *pContext = g_pVideoTemp->m_cVideoTemp[i];
		memcpy(pContext, &sHeader, 2);
		memcpy(&(pContext[2]), &sChannel, 2);
		memcpy(&(pContext[4]), &sFrameID, 2);
		memcpy(&(pContext[6]), &sShortPackNum, 2);
		memcpy(&(pContext[8]), &sPackID, 2);
		memcpy(&(pContext[10]), &sDataLength, 2);
		memcpy(&(pContext[12]), &(cBuf[i * 1024]), 1024);
	}

	//拷贝最后一个包的数据
	sPackID = nPackNum - 1;
	sDataLength = nLastDataLength;
	char *pContext = g_pVideoTemp->m_cVideoTemp[nPackNum - 1];
	memcpy(pContext, &sHeader, 2);
	memcpy(&(pContext[2]), &sChannel, 2);
	memcpy(&(pContext[4]), &sFrameID, 2);
	memcpy(&(pContext[6]), &sShortPackNum, 2);
	memcpy(&(pContext[8]), &sPackID, 2);
	memcpy(&(pContext[10]), &sDataLength, 2);
	memcpy(&(pContext[12]), &(cBuf[(nPackNum - 1) * 1024]), sDataLength);

	//保存数据包
	g_pVideoTemp->nPackNum = nPackNum;

	return g_pVideoTemp;
}

//////////////////////////////////////////////////////////////////////////
//获取数据包第一步，解析数据包
//////////////////////////////////////////////////////////////////////////

int HICCheckPack(unsigned char *cData, int nLength)
{
	unsigned short *pData16;

	//数据长度必须是2的整数倍
	if(nLength%2 != 0)
		return 0;

	//首先校验checksum，如果不为0那么即为无效数据
	pData16 = (unsigned short *)cData;
	if(CheckSum(pData16, nLength/2) != 0)
		return 0;
	return 1;
}

//获取通用数据部分
short HICGetPackVersion(unsigned char *cData)
{
	return ((short *)cData)[0];
}

unsigned char HICGetDeviceType(unsigned char *cData)
{
	return cData[2];
}

short HICGetDeviceName(unsigned char *cData)
{
	return ((short *)(&cData[3]))[0];
}

unsigned char HICGetDataType(unsigned char *cData)
{
	return cData[5];
}

short HICGetDataLength(unsigned char *cData)
{
	return ((short *)(&cData[6]))[0];
}

unsigned char *HICGetData(unsigned char *cData)
{
	return &(cData[8]);
}
