/*
 * bitptr_test.cpp
 *
 *  Created on: Jul 11, 2014
 *      Author: dnad
 */

#include <bitset>
#include <iostream>

#include <time.h>

#include <vector>
#include <fstream>
#include <set>

#include <rtklib.h>

#pragma pack(1)

struct rtcm2_msg_t {
	struct rtcm2_msghw1 {	/* header word 1 */
		uint            parity:6;
		uint            refstaid:10;	/* reference station ID */
		uint            msgtype:6;		/* RTCM message type */
		uint            preamble:8;		/* fixed at 01100110 */
		uint            _pad:2;
	} w1;

	struct rtcm2_msghw2 {			/* header word 2 */
		uint            parity:6;
		uint            stathlth:3;		/* station health */
		uint            frmlen:5;
		uint            sqnum:3;
		uint            zcnt:13;
		uint            _pad:2;
	} w2;
};

int decode_word(unsigned int word)
{
	const unsigned int hamming[]={
			0xBB1F3480,0x5D8F9A40,0xAEC7CD00,0x5763E680,0x6BB1F340,0x8B7A89C0
	};
	unsigned int parity=0,w;
	int i;

	if (word&0x40000000) word^=0x3FFFFFC0;

	for (i=0;i<6;i++) {
		parity<<=1;
		for (w=(word&hamming[i])>>6;w;w>>=1) parity^=w&1;
	}
	if (parity!=(word&0x3F)) return 0;

	//for (i=0;i<3;i++) data[i]=(unsigned char)(word>>(22-i*8));
	return 1;
}

int main(int argc, char* argv[])
{
	//std::ifstream file("/home/dnad/rtcm2.3.dat");
	//std::ifstream file("/home/dnad/Downloads/rtcmout.bin");
	std::ifstream file("/home/dnad/rtcmout2_d.bin");
//	std::ofstream fileO("/home/dnad/rtcmout2_rewrite.bin");

	enum {headerSize=10, RTCM2PREAMB=0x66};

	std::vector<uint8_t> ringBuffer(headerSize,0);
	std::vector<uint8_t> dataBuffer;

	std::set<int> msgTypes;

	int nextRead(headerSize);
	int skipped(0);

	uint32_t word1(0),word2(0);

	rtcm_t rtcm;
	init_rtcm(&rtcm);

	std::vector<uint8_t> dataOut;

		//while (!file.eof())
		{
			std::vector<uint8_t> data(10,0);
			//file.read(reinterpret_cast<char*>(&data[0]), 10);
	    uint8_t dataa[] = {102, 105, 65, 64, 72, 92, 106, 73, 94, 103, 74, 82, 126, 98, 104, 95, 106, 106, 106, 113, 127, 127, 127, 127, 127, 89, 126, 125, 127, 113, 65, 76, 122, 126, 98, 119, 76, 69, 111, 106, 112, 86, 111, 79, 95, 79, 113, 125, 103, 116, 89, 105, 106, 106, 82};
	    std::cout<<"Size:"<<sizeof(dataa)<<std::endl;
	    data.assign(&dataa[0], &dataa[0] + sizeof(dataa));
			while (!data.empty())
			{
				int ret = 0;

				if ((ret = input_rtcm2(&rtcm, data[0])) > 0)
				{
					std::cout<<"Got message:"<<ret<<std::endl;

				}
				else
				{
					//std::cout<<"Failed message:"<<ret<<std::endl;
				}

				data.erase(data.begin());
			}
		}

	file.seekg(0,std::ios::beg);
	file.clear(std::ios::eofbit);

	while (!file.eof())
	{
		if (nextRead == headerSize)
		{
			std::cout<<"Skipped:"<<skipped<<std::endl;
			file.read(reinterpret_cast<char*>(&ringBuffer[0]), nextRead);
			skipped = 0;
		}
		else
		{
			++skipped;
			char a;
			file.read(&a,1);
			ringBuffer.push_back(a);
		}

		if ((ringBuffer[0]&0xC0) == 0x40)
		{
			for (int i=0; i<5; ++i)
			{
				uint8_t data = ringBuffer[i];
				for (int j=0;j<6;j++,data>>=1)
				{
					word1=(word1<<1)+(data&1);
				}
			}

			uint8_t preamb=uint8_t(word1>>22);
			if (word1&0x40000000) preamb^=0xFF;
			if ((preamb == RTCM2PREAMB) && (decode_word(word1)))
			{
				//std::cout<<"Found one."<<std::endl;
				uint8_t* pt2 = reinterpret_cast<uint8_t*>(&word1);
				rtcm2_msg_t::rtcm2_msghw1* pw1 = reinterpret_cast<rtcm2_msg_t::rtcm2_msghw1*>(&word1);
				if (pw1->msgtype > 0)
				{
					//std::cout<<std::bitset<32>(word1)<<std::endl;
					//std::cout<<"Type:"<<pw1->msgtype<<std::endl;
				}
				else
				{
					nextRead = 1;
					//word1 = 0;
				}

				/*std::cout<<"Word1:";
				for (int i=0; i<4; ++i)
				{
					std::cout<<std::bitset<8>(pt2[i]);
				}
				std::cout<<std::endl;*/

				word2 = word1 & 0x3;

				for (int i=0; i<5; ++i)
				{
					uint8_t data = ringBuffer[i+5];
					for (int j=0;j<6;j++,data>>=1)
					{
						word2=(word2<<1)+(data&1);
					}
				}

				if (decode_word(word2))
				{
					rtcm2_msg_t::rtcm2_msghw2* pw2 = reinterpret_cast<rtcm2_msg_t::rtcm2_msghw2*>(&word2);
					std::cout<<"Type:"<<pw1->msgtype<<", size:"<<pw2->frmlen*3+6<<std::endl;
					dataBuffer.resize(pw2->frmlen*5);
					file.read(reinterpret_cast<char*>(&dataBuffer[0]), pw2->frmlen*5);
					if (pw2->frmlen != 0)
					{
						word1 = dataBuffer[pw2->frmlen*5-1] & 0x3;
					}
					else
					{
						word1 = word2 & 0x3;
					}
					//exit(0);
				}

				//
				//				rtcm2_msg_t::rtcm2_msghw1* pw1 = reinterpret_cast<rtcm2_msg_t::rtcm2_msghw1*>(&word1);
				//				rtcm2_msg_t::rtcm2_msghw2* pw2 = reinterpret_cast<rtcm2_msg_t::rtcm2_msghw2*>(&word2);
				//				//std::cout<<"preamble:"<<pw1->preamble<<std::endl;
				//				std::cout<<"msgtpye:"<<pw1->msgtype<<" : "<<pw2->frmlen<<", "<<pw1->preamble<<", seq:"<<pw2->sqnum<<std::endl;
				//
				//				msgTypes.insert(pw1->msgtype);
				//
				//				if ((decode_word(word1) && decode_word(word2)) && pw2->sqnum)
				//				{
				//					msgTypes.insert(pw1->msgtype);
				//					dataBuffer.resize(pw2->frmlen*5);
				//					file.read(reinterpret_cast<char*>(&dataBuffer[0]), pw2->frmlen*5);
				//					if (pw2->frmlen != 0)
				//					{
				//						word1 = dataBuffer[pw2->frmlen*5-1] & 0x3;
				//					}
				//					else
				//					{
				//						word1 = word2 & 0x3;
				//					}
				//
				//					nextRead = headerSize;
				//				}
				//				else
				//				{
				//					nextRead = 1;
				//					word1 = 0;
				//				}
			}
			else
			{
				nextRead = 1;
				//word1 = 0;
			}
		}
		else
		{
			nextRead = 1;
			//word1 = 0;
		}
		ringBuffer.erase(ringBuffer.begin());
	}

	std::cout<<"Message types:";
	for (std::set<int>::const_iterator it=msgTypes.begin(); it!=msgTypes.end();++it)
	{
		std::cout<<*it<<", ";
	}
	std::cout<<std::endl;


	return 0;
}

