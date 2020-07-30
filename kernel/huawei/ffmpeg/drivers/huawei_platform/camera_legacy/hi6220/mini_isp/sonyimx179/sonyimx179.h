/*
 *  sonyimx179 camera driver head file
 *
 *  Author: 	Zhoujie (zhou.jie1981@163.com)
 *  Date:  	2013/01/05
 *  Version:	1.0
 *  History:	2013/01/05      Frist add driver for sonyimx179 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _SONYIMX179_H
#define _SONYIMX179_H

/***********************************************************************
 *
 * sonyimx179 init sensor registers list
 *
 ***********************************************************************/
static const struct _mini_sensor_reg_t sonyimx179_init_regs[] = {				
	{	0x0101	,	0x00	},
	{	0x0105	,	0x01	},
	{	0x0110	,	0x00	},
	{	0x0220	,	0x01	},
	{	0x3302	,	0x11	},
	{	0x3833	,	0x20	},
	{	0x3893	,	0x00	},
	{	0x3906	,	0x08	},
	{	0x3907	,	0x01	},
	{	0x391B	,	0x01	},
	{	0x3C09	,	0x01	},
	{	0x600A	,	0x00	},
	{	0x3008	,	0xB0	},
	{	0x320A	,	0x01	},
	{	0x320D	,	0x10	},
	{	0x3216	,	0x2E	},
	{	0x322C	,	0x02	},
	{	0x3409	,	0x0C	},
	{	0x340C	,	0x2D	},
	{	0x3411	,	0x39	},
	{	0x3414	,	0x1E	},
	{	0x3427	,	0x04	},
	{	0x3480	,	0x1E	},
	{	0x3484	,	0x1E	},
	{	0x3488	,	0x1E	},
	{	0x348C	,	0x1E	},
	{	0x3490	,	0x1E	},
	{	0x3494	,	0x1E	},
	{	0x3511	,	0x8F	},
	{	0x3617	,	0x2D	},

    //Defect Correction Recommended Setting				
    {	0x380A	,	0x00	},
    {	0x380B	,	0x00	},
    {	0x4103	,	0x00	},

    //Color Artifact Recommended Setting				
    {	0x4243	,	0x9A	},
    {	0x4330	,	0x01	},
    {	0x4331	,	0x90	},
    {	0x4332	,	0x02	},
    {	0x4333	,	0x58	},
    {	0x4334	,	0x03	},
    {	0x4335	,	0x20	},
    {	0x4336	,	0x03	},
    {	0x4337	,	0x84	},
    {	0x433C	,	0x01	},
    {	0x4340	,	0x02	},
    {	0x4341	,	0x58	},
    {	0x4342	,	0x03	},
    {	0x4343	,	0x52	},

    //Moir�� reduction Parameter Setting				
    {	0x4364	,	0x0B	},
    {	0x4368	,	0x00	},
    {	0x4369	,	0x0F	},
    {	0x436A	,	0x03	},
    {	0x436B	,	0xA8	},
    {	0x436C	,	0x00	},
    {	0x436D	,	0x00	},
    {	0x436E	,	0x00	},
    {	0x436F	,	0x06	},
    //CNR parameter setting				
    {	0x4281	,	0x21	},
    {	0x4282	,	0x18	},
    {	0x4283	,	0x04	},
    {	0x4284	,	0x08	},
    {	0x4287	,	0x7F	},
    {	0x4288	,	0x08	},
    {	0x428B	,	0x7F	},
    {	0x428C	,	0x08	},
    {	0x428F	,	0x7F	},
    {	0x4297	,	0x00	},
    {	0x4298	,	0x7E	},
    {	0x4299	,	0x7E	},
    {	0x429A	,	0x7E	},
    {	0x42A4	,	0xFB	},
    {	0x42A5	,	0x7E	},
    {	0x42A6	,	0xDF	},
    {	0x42A7	,	0xB7	},
    {	0x42AF	,	0x03	},
    //ARNR Parameter Setting				
    {	0x4207	,	0x03	},
    {	0x4216	,	0x08	},
    {	0x4217	,	0x08	},
    //DLC Parameter Setting				
    {	0x4218	,	0x00	},
    {	0x421B	,	0x20	},
    {	0x421F	,	0x04	},
    {	0x4222	,	0x02	},
    {	0x4223	,	0x22	},
    {	0x422E	,	0x54	},
    {	0x422F	,	0xFB	},
    {	0x4230	,	0xFF	},
    {	0x4231	,	0xFE	},
    {	0x4232	,	0xFF	},
    {	0x4235	,	0x58	},
    {	0x4236	,	0xF7	},
    {	0x4237	,	0xFD	},
    {	0x4239	,	0x4E	},
    {	0x423A	,	0xFC	},
    {	0x423B	,	0xFD	},
    //HDR Setting				
    {	0x4300	,	0x00	},
    {	0x4316	,	0x12	},
    {	0x4317	,	0x22	},
    {	0x4318	,	0x00	},
    {	0x4319	,	0x00	},
    {	0x431A	,	0x00	},
    {	0x4324	,	0x03	},
    {	0x4325	,	0x20	},
    {	0x4326	,	0x03	},
    {	0x4327	,	0x84	},
    {	0x4328	,	0x03	},
    {	0x4329	,	0x20	},
    {	0x432A	,	0x03	},
    {	0x432B	,	0x20	},
    {	0x432C	,	0x01	},
    {	0x432D	,	0x01	},
    {	0x4338	,	0x02	},
    {	0x4339	,	0x00	},
    {	0x433A	,	0x00	},
    {	0x433B	,	0x02	},
    {	0x435A	,	0x03	},
    {	0x435B	,	0x84	},
    {	0x435E	,	0x01	},
    {	0x435F	,	0xFF	},
    {	0x4360	,	0x01	},
    {	0x4361	,	0xF4	},
    {	0x4362	,	0x03	},
    {	0x4363	,	0x84	},
    {	0x437B	,	0x01	},
    {	0x4401	,	0x3F	},
    {	0x4402	,	0xFF	},
    {	0x4404	,	0x13	},
    {	0x4405	,	0x26	},
    {	0x4406	,	0x07	},
    {	0x4408	,	0x20	},
    {	0x4409	,	0xE5	},
    {	0x440A	,	0xFB	},
    {	0x440C	,	0xF6	},
    {	0x440D	,	0xEA	},
    {	0x440E	,	0x20	},
    {	0x4410	,	0x00	},
    {	0x4411	,	0x00	},
    {	0x4412	,	0x3F	},
    {	0x4413	,	0xFF	},
    {	0x4414	,	0x1F	},
    {	0x4415	,	0xFF	},
    {	0x4416	,	0x20	},
    {	0x4417	,	0x00	},
    {	0x4418	,	0x1F	},
    {	0x4419	,	0xFF	},
    {	0x441A	,	0x20	},
    {	0x441B	,	0x00	},
    {	0x441D	,	0x40	},
    {	0x441E	,	0x1E	},
    {	0x441F	,	0x38	},
    {	0x4420	,	0x01	},
    {	0x4444	,	0x00	},
    {	0x4445	,	0x00	},
    {	0x4446	,	0x1D	},
    {	0x4447	,	0xF9	},
    {	0x4452	,	0x00	},
    {	0x4453	,	0xA0	},
    {	0x4454	,	0x08	},
    {	0x4455	,	0x00	},
    {	0x4456	,	0x0F	},
    {	0x4457	,	0xFF	},
    {	0x4458	,	0x18	},
    {	0x4459	,	0x18	},
    {	0x445A	,	0x3F	},
    {	0x445B	,	0x3A	},
    {	0x445C	,	0x00	},
    {	0x445D	,	0x28	},
    {	0x445E	,	0x01	},
    {	0x445F	,	0x90	},
    {	0x4460	,	0x00	},
    {	0x4461	,	0x60	},
    {	0x4462	,	0x00	},
    {	0x4463	,	0x00	},
    {	0x4464	,	0x00	},
    {	0x4465	,	0x00	},
    {	0x446C	,	0x00	},
    {	0x446D	,	0x00	},
    {	0x446E	,	0x00	},
    //LSC Setting				
    {	0x452A	,	0x02	},
    //White Balance Setting				
    {	0x0712	,	0x01	},
    {	0x0713	,	0x00	},
    {	0x0714	,	0x01	},
    {	0x0715	,	0x00	},
    {	0x0716	,	0x01	},
    {	0x0717	,	0x00	},
    {	0x0718	,	0x01	},
    {	0x0719	,	0x00	},
    //Shading setting				
    {	0x4500	,	0x1F	}
};


/*1600x1200.4-lane,30fps*/
static const struct _mini_sensor_reg_t sonyimx179_framesize_1600x1200[] = {
	{	0x0100	,	0x00	}, //stand by
	//1600x1200 30fps				
	//Clock Setting				
	{	0x011E	,	0x14	},
	{	0x011F	,	0x00	},
	{	0x0301	,	0x05	},
	{	0x0303	,	0x01	},
	{	0x0305	,	0x02	},
	{	0x0309	,	0x05	},
	{	0x030B	,	0x01	},
	{	0x030C	,	0x00	},
	{	0x030D	,	0x28	},
	{	0x030E	,	0x01	},
	{	0x3A06	,	0x11	},
	//Mode setting				
	{	0x0108	,	0x03	},
	{	0x0112	,	0x0A	},
	{	0x0113	,	0x0A	},
	{	0x0381	,	0x01	},
	{	0x0383	,	0x01	},
	{	0x0385	,	0x01	},
	{	0x0387	,	0x01	},
	{	0x0390	,	0x01	},
	{	0x0391	,	0x22	},
	{	0x0392	,	0x00	},
	{	0x0401	,	0x00	},
	{	0x0404	,	0x00	},
	{	0x0405	,	0x10	},
	{	0x4082	,	0x00	},
	{	0x4083	,	0x01	},
	{	0x7006	,	0x04	},
	//OptionnalFunction setting
	#ifndef IMX179_OTP
	{	0x0700	,	0x00	},
	{	0x3A63	,	0x00	},
	#endif
	{	0x4100	,	0xF8	},
	{	0x4203	,	0xFF	},
	{	0x4344	,	0x00	},
	{	0x441C	,	0x01	},
	//Size setting				
	{	0x0340	,	0x05	},
	{	0x0341	,	0xC8	},
	{	0x0342	,	0x0E	},
	{	0x0343	,	0x10	},
	{	0x0344	,	0x00	},
	{	0x0345	,	0x00	},
	{	0x0346	,	0x00	},
	{	0x0347	,	0x00	},
	{	0x0348	,	0x0C	},
	{	0x0349	,	0xCF	},
	{	0x034A	,	0x09	},
	{	0x034B	,	0x9F	},
	{	0x034C	,	0x06	},
	{	0x034D	,	0x40	},
	{	0x034E	,	0x04	},
	{	0x034F	,	0xB0	},
	{	0x0350	,	0x00	},
	{	0x0351	,	0x14	},
	{	0x0352	,	0x00	},
	{	0x0353	,	0x10	},
	{	0x0354	,	0x06	},
	{	0x0355	,	0x40	},
	{	0x0356	,	0x04	},
	{	0x0357	,	0xB0	},
	{	0x301D	,	0x30	},
	{	0x3310	,	0x06	},
	{	0x3311	,	0x40	},
	{	0x3312	,	0x04	},
	{	0x3313	,	0xB0	},
	{	0x331C	,	0x03	},
	{	0x331D	,	0xF2	},
	{	0x4084	,	0x00	},
	{	0x4085	,	0x00	},
	{	0x4086	,	0x00	},
	{	0x4087	,	0x00	},
	{	0x4400	,	0x00	},
	//Global Timing Setting				
	{	0x0830	,	0x67	},
	{	0x0831	,	0x1F	},
	{	0x0832	,	0x47	},
	{	0x0833	,	0x1F	},
	{	0x0834	,	0x1F	},
	{	0x0835	,	0x17	},
	{	0x0836	,	0x77	},
	{	0x0837	,	0x27	},
	{	0x0839	,	0x1F	},
	{	0x083A	,	0x17	},
	{	0x083B	,	0x02	},
	//Integration Time Setting				
	//{	0x0202	,	0x05	},
	//{	0x0203	,	0xC4	},
	//Gain Setting				
	//{	0x0205	,	0x00	},
	//{	0x020E	,	0x01	},
	//{	0x020F	,	0x00	},
	//{	0x0210	,	0x01	},
	//{	0x0211	,	0x00	},
	//{	0x0212	,	0x01	},
	//{	0x0213	,	0x00	},
	//{	0x0214	,	0x01	},
	//{	0x0215	,	0x00	},
	//HDR Setting				
	{	0x0230	,	0x00	},
	{	0x0231	,	0x00	},
	{	0x0233	,	0x00	},
	{	0x0234	,	0x00	},
	{	0x0235	,	0x40	},
	{	0x0238	,	0x00	},
	{	0x0239	,	0x04	},
	{	0x023B	,	0x00	},
	{	0x023C	,	0x01	},
	{	0x33B0	,	0x04	},
	{	0x33B1	,	0x00	},
	{	0x33B3	,	0x00	},
	{	0x33B4	,	0x01	},
	{	0x3800	,	0x00	},
	{	0x34A9	,	0x02	},//-6db
	{	0x3A43	,	0x01	},//init pd
	{	0x0100	,	0x01	}

};

/* 1920x1088, 30fps, 4-lane */
static const struct _mini_sensor_reg_t sonyimx179_framesize_1920x1088[] = {
	{	0x0100	,	0x00	},
	//	FHD mode  *30fps	
	//	H : 1920
	//	V : 1088
	//	Clock Setting
	{	0x011E	,	0x14	},
	{	0x011F	,	0x00	},
	{	0x0301	,	0x05	},
	{	0x0303	,	0x01	},
	{	0x0305	,	0x0F	},
	{	0x0309	,	0x05	},
	{	0x030B	,	0x01	},
	{	0x030C	,	0x02	},
	{	0x030D	,	0x0D	},
	{	0x030E	,	0x01	},
	{	0x3A06	,	0x11	},
	//	Mode setting
	{	0x0108	,	0x03	},
	{	0x0112	,	0x0A	},
	{	0x0113	,	0x0A	},
	{	0x0381	,	0x01	},
	{	0x0383	,	0x01	},
	{	0x0385	,	0x01	},
	{	0x0387	,	0x01	},
	{	0x0390	,	0x00	},
	{	0x0391	,	0x11	},
	{	0x0392	,	0x00	},
	{	0x0401	,	0x02	},
	{	0x0404	,	0x00	},
	{	0x0405	,	0x1B	},
	{	0x4082	,	0x00	},
	{	0x4083	,	0x00	},
	{	0x7006	,	0x04	},
	//OptionnalFunction setting
	#ifndef IMX179_OTP
	{	0x0700	,	0x00	},
	{	0x3A63	,	0x00	},
	#endif
	{	0x4100	,	0xF8	},
	{	0x4203	,	0xFF	},
	{	0x4344	,	0x00	},
	{	0x441C	,	0x01	},
	//Size setting	
	{	0x0340	,	0x0A	},
	{	0x0341	,	0x20	},
	{	0x0342	,	0x0E	},
	{	0x0343	,	0x10	},
	{	0x0344	,	0x00	},
	{	0x0345	,	0x14	},
	{	0x0346	,	0x01	},
	{	0x0347	,	0x3C	},
	{	0x0348	,	0x0C	},
	{	0x0349	,	0xBB	},
	{	0x034A	,	0x08	},
	{	0x034B	,	0x67	},
	{	0x034C	,	0x07	},
	{	0x034D	,	0x80	},
	{	0x034E	,	0x04	},
	{	0x034F	,	0x40	},
	{	0x0350	,	0x00	},
	{	0x0351	,	0x00	},
	{	0x0352	,	0x00	},
	{	0x0353	,	0x00	},
	{	0x0354	,	0x0C	},
	{	0x0355	,	0xA8	},
	{	0x0356	,	0x07	},
	{	0x0357	,	0x2C	},
	{	0x301D	,	0x30	},
	{	0x3310	,	0x07	},
	{	0x3311	,	0x80	},
	{	0x3312	,	0x04	},
	{	0x3313	,	0x40	},
	{	0x331C	,	0x04	},
	{	0x331D	,	0x1E	},
	{	0x4084	,	0x07	},
	{	0x4085	,	0x80	},
	{	0x4086	,	0x04	},
	{	0x4087	,	0x40	},
	{	0x4400	,	0x00	},
	//Global Timing Setting	
	{	0x0830	,	0x77	},
	{	0x0831	,	0x2F	},
	{	0x0832	,	0x4F	},
	{	0x0833	,	0x37	},
	{	0x0834	,	0x2F	},
	{	0x0835	,	0x37	},
	{	0x0836	,	0xAF	},
	{	0x0837	,	0x37	},
	{	0x0839	,	0x1F	},
	{	0x083A	,	0x17	},
	{	0x083B	,	0x02	},
	//	Integration Time Setting	
	//{	0x0202	,	0x0A	},
	//{	0x0203	,	0x1C	},
	//	Gain Setting	
	//{	0x0205	,	0x00	},
	//{	0x020E	,	0x01	},
	//{	0x020F	,	0x00	},
	//{	0x0210	,	0x01	},
	//{	0x0211	,	0x00	},
	//{	0x0212	,	0x01	},
	//{	0x0213	,	0x00	},
	//{	0x0214	,	0x01	},
	//{	0x0215	,	0x00	},
	//	HDR Setting
	{	0x0230	,	0x00	},
	{	0x0231	,	0x00	},
	{	0x0233	,	0x00	},
	{	0x0234	,	0x00	},
	{	0x0235	,	0x40	},
	{	0x0238	,	0x00	},
	{	0x0239	,	0x04	},
	{	0x023B	,	0x00	},
	{	0x023C	,	0x01	},
	{	0x33B0	,	0x04	},
	{	0x33B1	,	0x00	},
	{	0x33B3	,	0x00	},
	{	0x33B4	,	0x01	},
	{	0x3800	,	0x00	},
	//{	0x34A9	,	0x02	},//-6db
	{	0x3A43	,	0x01	},//init pd

    //color bar:100% color bar
    //{0x0601	,	0x02},
	{	0x0100	,	0x01	}
};

static const struct _mini_sensor_reg_t sonyimx179_rgb_framesize_1280x720_HDR[] = {
{0x0100,0x00},
{0x011E,0x14},
{0x011F,0x00},
{0x0301,0x05},
{0x0303,0x01},
{0x0305,0x0A},
{0x0309,0x05},
{0x030B,0x02},
{0x030C,0x01},
{0x030D,0x54},
{0x030E,0x01},
{0x3A06,0x12},

{0x0108,0x03},
{0x0112,0x0E},
{0x0113,0x0A},
{0x0381,0x01},
{0x0383,0x01},
{0x0385,0x01},
{0x0387,0x01},
{0x0390,0x00},
{0x0391,0x11},
{0x0392,0x00},
{0x0401,0x02},
{0x0404,0x00},
{0x0405,0x14},
{0x4082,0x00},
{0x4083,0x00},
{0x7006,0x04},

{0x0700,0x00},
{0x3A63,0x00},
{0x4100,0xF8},
{0x4203,0xFF},
{0x4344,0x00},
{0x441C,0x01},

{0x0340,0x09},
{0x0341,0xD6},
{0x0342,0x0E},
{0x0343,0x10},
{0x0344,0x00},
{0x0345,0x28},
{0x0346,0x01},
{0x0347,0x4C},
{0x0348,0x0C},
{0x0349,0xA7},
{0x034A,0x08},
{0x034B,0x53},
{0x034C,0x05},
{0x034D,0x00},
{0x034E,0x02},
{0x034F,0xD0},
{0x0350,0x00},
{0x0351,0x00},
{0x0352,0x00},
{0x0353,0x00},
{0x0354,0x06},
{0x0355,0x40},
{0x0356,0x03},
{0x0357,0x84},
{0x301D,0x30},
{0x3310,0x05},
{0x3311,0x00},
{0x3312,0x02},
{0x3313,0xD0},
{0x331C,0x01},
{0x331D,0x10},
{0x4084,0x05},
{0x4085,0x00},
{0x4086,0x02},
{0x4087,0xD0},
{0x4400,0x00},

{0x0830,0x5F},
{0x0831,0x1F},
{0x0832,0x3F},
{0x0833,0x1F},
{0x0834,0x1F},
{0x0835,0x17},
{0x0836,0x67},
{0x0837,0x27},
{0x0839,0x1F},
{0x083A,0x17},
{0x083B,0x02},

//{0x0202,0x09},
//{0x0203,0xD2},

//{0x0205,0x00},
//{0x020E,0x01},
//{0x020F,0x00},
//{0x0210,0x01},
//{0x0211,0x00},
//{0x0212,0x01},
//{0x0213,0x00},
//{0x0214,0x01},
//{0x0215,0x00},

{0x0230,0x00},
{0x0231,0x00},
{0x0233,0x00},
{0x0234,0x00},
{0x0235,0x40},
{0x0238,0x01},
{0x0239,0x04},
{0x023B,0x03},
{0x023C,0x01},
{0x33B0,0x04},
{0x33B1,0x00},
{0x33B3,0x02},
{0x33B4,0x00},
{0x3800,0x00},
{0x3A43,0x01},//init pd
{0x0100,0x01},
};

static const struct _mini_sensor_reg_t sonyimx179_rgb_framesize_1600x1200_HDR[] = {
{0x0100,0x00},

{0x011E,0x14},
{0x011F,0x00},
{0x0301,0x05},
{0x0303,0x01},
{0x0305,0x02},
{0x0309,0x05},
{0x030B,0x02},
{0x030C,0x00},
{0x030D,0x44},
{0x030E,0x01},
{0x3A06,0x12},

{0x0108,0x03},
{0x0112,0x0E},
{0x0113,0x0A},
{0x0381,0x01},
{0x0383,0x01},
{0x0385,0x01},
{0x0387,0x01},
{0x0390,0x00},
{0x0391,0x11},
{0x0392,0x00},
{0x0401,0x00},
{0x0404,0x00},
{0x0405,0x10},
{0x4082,0x01},
{0x4083,0x01},
{0x7006,0x04},

{0x0700,0x00},
{0x3A63,0x00},
{0x4100,0xF8},
{0x4203,0xFF},
{0x4344,0x00},
{0x441C,0x01},

{0x0340,0x09},
{0x0341,0xD6},
{0x0342,0x0E},
{0x0343,0x10},
{0x0344,0x00},
{0x0345,0x28},
{0x0346,0x00},
{0x0347,0x20},
{0x0348,0x0C},
{0x0349,0xA7},
{0x034A,0x09},
{0x034B,0x7F},
{0x034C,0x06},
{0x034D,0x40},
{0x034E,0x04},
{0x034F,0xB0},
{0x0350,0x00},
{0x0351,0x00},
{0x0352,0x00},
{0x0353,0x00},
{0x0354,0x06},
{0x0355,0x40},
{0x0356,0x04},
{0x0357,0xB0},
{0x301D,0x30},
{0x3310,0x06},
{0x3311,0x40},
{0x3312,0x04},
{0x3313,0xB0},
{0x331C,0x03},
{0x331D,0x58},
{0x4084,0x00},
{0x4085,0x00},
{0x4086,0x00},
{0x4087,0x00},
{0x4400,0x00},

{0x0830,0x5F},
{0x0831,0x1F},
{0x0832,0x3F},
{0x0833,0x1F},
{0x0834,0x1F},
{0x0835,0x17},
{0x0836,0x67},
{0x0837,0x27},
{0x0839,0x1F},
{0x083A,0x17},
{0x083B,0x02},

//{0x0202,0x09},
//{0x0203,0xD2},

//{0x0205,0x00},
//{0x020E,0x01},
//{0x020F,0x00},
//{0x0210,0x01},
//{0x0211,0x00},
//{0x0212,0x01},
//{0x0213,0x00},
//{0x0214,0x01},
//{0x0215,0x00},

{0x0230,0x00},
{0x0231,0x00},
{0x0233,0x00},
{0x0234,0x00},
{0x0235,0x40},
{0x0238,0x01},
{0x0239,0x04},
{0x023B,0x03},
{0x023C,0x01},
{0x33B0,0x04},
{0x33B1,0x00},
{0x33B3,0x02},
{0x33B4,0x00},
{0x3800,0x00},
{0x3A43,0x01},//init pd
{0x0100,0x01},
};

static const struct _mini_sensor_reg_t sonyimx179_framesize_full[] = {
	{	0x0100	,	0x00	},
    //3264x2448 15fps				
    //Clock Setting				
    {	0x011E	,	0x14	},
    {	0x011F	,	0x00	},
    {	0x0301	,	0x05	},
    {	0x0303	,	0x01	},
    {	0x0305	,	0x02	},
    {	0x0309	,	0x05	},
    {	0x030B	,	0x01	},
    {	0x030C	,	0x00	},
    {	0x030D	,	0x28	},
    {	0x030E	,	0x01	},
    {	0x3A06	,	0x11	},
    //Mode setting				
    {	0x0108	,	0x03	},
    {	0x0112	,	0x0A	},
    {	0x0113	,	0x0A	},
    {	0x0381	,	0x01	},
    {	0x0383	,	0x01	},
    {	0x0385	,	0x01	},
    {	0x0387	,	0x01	},
    {	0x0390	,	0x00	},
    {	0x0391	,	0x11	},
    {	0x0392	,	0x00	},
    {	0x0401	,	0x00	},
    {	0x0404	,	0x00	},
    {	0x0405	,	0x10	},
    {	0x4082	,	0x01	},
    {	0x4083	,	0x01	},
    {	0x7006	,	0x04	},
    //OptionnalFunction setting				
	#ifndef IMX179_OTP
	{	0x0700	,	0x00	},
	{	0x3A63	,	0x00	},
	#endif
    {	0x4100	,	0xF8	},
    {	0x4203	,	0xff	},
    {	0x4344	,	0x00	},
    {	0x441C	,	0x01	},
    //Size setting				
    {	0x0340	,	0x0B	},
    {	0x0341	,	0x90	},
    {	0x0342	,	0x10	},
    {	0x0343	,	0x36	},
    {	0x0344	,	0x00	},
    {	0x0345	,	0x00	},
    {	0x0346	,	0x00	},
    {	0x0347	,	0x00	},
    {	0x0348	,	0x0C	},
    {	0x0349	,	0xBF	},
    {	0x034A	,	0x09	},//09
    {	0x034B	,	0x8F	},//9f
    {	0x034C	,	0x0C	},
    {	0x034D	,	0xC0	},
    {	0x034E	,	0x09	},
    {	0x034F	,	0x90	},//A0
    {	0x0350	,	0x00	},
    {	0x0351	,	0x00	},
    {	0x0352	,	0x00	},
    {	0x0353	,	0x00	},
    {	0x0354	,	0x0C	},
    {	0x0355	,	0xC0	},
    {	0x0356	,	0x09	},
    {	0x0357	,	0x90	},//A0
    {	0x301D	,	0x30	},
    {	0x3310	,	0x0C	},
    {	0x3311	,	0xC0	},
    {	0x3312	,	0x09	},
    {	0x3313	,	0x90	},//A0
    {	0x331C	,	0x01	},
    {	0x331D	,	0xAE	},
    {	0x4084	,	0x00	},
    {	0x4085	,	0x00	},
    {	0x4086	,	0x00	},
    {	0x4087	,	0x00	},
    {	0x4400	,	0x00	},
    //Global Timing Setting				
    {	0x0830	,	0x67	},
    {	0x0831	,	0x1F	},
    {	0x0832	,	0x47	},
    {	0x0833	,	0x1F	},
    {	0x0834	,	0x1F	},
    {	0x0835	,	0x17	},
    {	0x0836	,	0x77	},
    {	0x0837	,	0x27	},
    {	0x0839	,	0x1F	},
    {	0x083A	,	0x17	},
    {	0x083B	,	0x02	},
    //Integration Time Setting				
    //{	0x0202	,	0x0B	},
    //{	0x0203	,	0x8C	},
    //Gain Setting				
    //{	0x0205	,	0x00	},
    //{	0x020E	,	0x01	},
    //{	0x020F	,	0x00	},
    //{	0x0210	,	0x01	},
    //{	0x0211	,	0x00	},
    //{	0x0212	,	0x01	},
    //{	0x0213	,	0x00	},
    //{	0x0214	,	0x01	},
    //{	0x0215	,	0x00	},
    //HDR Setting				
    {	0x0230	,	0x00	},
    {	0x0231	,	0x00	},
    {	0x0233	,	0x00	},
    {	0x0234	,	0x00	},
    {	0x0235	,	0x40	},
    {	0x0238	,	0x00	},
    {	0x0239	,	0x04	},
    {	0x023B	,	0x00	},
    {	0x023C	,	0x01	},
    {	0x33B0	,	0x04	},
    {	0x33B1	,	0x00	},
    {	0x33B3	,	0x00	},
    {	0x33B4	,	0x01	},
    {	0x3800	,	0x00	},
    {	0x34A9	,	0x02	},//-6db
    {	0x3A43	,	0x01	},
    {	0x0100	,	0x01	} 
};


static const struct _mini_sensor_reg_t sonyimx179_framesize_zsl_full[] = {
	//these registers for zsl mode
	{	0x0100	,	0x00	},
    //3264x2448 15fps				
    //Clock Setting				
	{	0x011E	,	0x14	},
	{	0x011F	,	0x00	},
	{	0x0301	,	0x05	},
	{	0x0303	,	0x01	},
	{	0x0305	,	0x0C	},
	{	0x0309	,	0x05	},
	{	0x030B	,	0x01	},
	{	0x030C	,	0x01	},
	{	0x030D	,	0xC2	},
	{	0x030E	,	0x01	},
	{	0x3A06	,	0x11	},
	//Mode setting				
	{	0x0108	,	0x03	},
	{	0x0112	,	0x0A	},
	{	0x0113	,	0x0A	},
	{	0x0381	,	0x01	},
	{	0x0383	,	0x01	},
	{	0x0385	,	0x01	},
	{	0x0387	,	0x01	},
	{	0x0390	,	0x00	},
	{	0x0391	,	0x11	},
	{	0x0392	,	0x00	},
	{	0x0401	,	0x00	},
	{	0x0404	,	0x00	},
	{	0x0405	,	0x10	},
	{	0x4082	,	0x01	},
	{	0x4083	,	0x01	},
	{	0x7006	,	0x04	},
    //OptionnalFunction setting				
	#ifndef IMX179_OTP
	{	0x0700	,	0x00	},
	{	0x3A63	,	0x00	},
	#endif
    {	0x4100	,	0xF8	},
	{	0x4203	,	0xFF	},
	{	0x4344	,	0x00	},
	{	0x441C	,	0x01	},
    //Size setting				
    {	0x0340	,	0x0A	},
	{	0x0341	,	0xD6	},
	{	0x0342	,	0x0E	},
	{	0x0343	,	0x10	},
	{	0x0344	,	0x00	},
	{	0x0345	,	0x08	},
	{	0x0346	,	0x00	},
	{	0x0347	,	0x08	},
	{	0x0348	,	0x0C	},
	{	0x0349	,	0xC7	},
	{	0x034A	,	0x09	},
	{	0x034B	,	0x97	},
	{	0x034C	,	0x0C	},
	{	0x034D	,	0xC0	},
	{	0x034E	,	0x09	},
	{	0x034F	,	0x90	},
	{	0x0350	,	0x00	},
	{	0x0351	,	0x00	},
	{	0x0352	,	0x00	},
	{	0x0353	,	0x00	},
	{	0x0354	,	0x0C	},
	{	0x0355	,	0xC0	},
	{	0x0356	,	0x09	},
	{	0x0357	,	0x90	},
	{	0x301D	,	0x30	},
	{	0x3310	,	0x0C	},
	{	0x3311	,	0xC0	},
	{	0x3312	,	0x09	},
	{	0x3313	,	0x90	},
	{	0x331C	,	0x01	},
	{	0x331D	,	0xAE	},
	{	0x4084	,	0x00	},
	{	0x4085	,	0x00	},
	{	0x4086	,	0x00	},
	{	0x4087	,	0x00	},
	{	0x4400	,	0x00	},
		
    //Global Timing Setting				
    {	0x0830	,	0x77	},
	{	0x0831	,	0x2F	},
	{	0x0832	,	0x5F	},
	{	0x0833	,	0x37	},
	{	0x0834	,	0x37	},
	{	0x0835	,	0x37	},
	{	0x0836	,	0xBF	},
	{	0x0837	,	0x3F	},
	{	0x0839	,	0x1F	},
	{	0x083A	,	0x17	},
	{	0x083B	,	0x02	},

    //Integration Time Setting				
	//{	0x0202	,	0x0A	},
	//{	0x0203	,	0xD2	},

    //Gain Setting				
    //{	0x0205	,	0x00		},
	//{	0x020E	,	0x01		},
	//{	0x020F	,	0x00		},
	//{	0x0210	,	0x01		},
	//{	0x0211	,	0x00		},
	//{	0x0212	,	0x01		},
	//{	0x0213	,	0x00		},
	//{	0x0214	,	0x01		},
	//{	0x0215	,	0x00		},

    //HDR Setting				
    {	0x0230	,	0x00	},
	{	0x0231	,	0x00	},
	{	0x0233	,	0x00	},
	{	0x0234	,	0x00	},
	{	0x0235	,	0x40	},
	{	0x0238	,	0x00	},
	{	0x0239	,	0x04	},
	{	0x023B	,	0x00	},
	{	0x023C	,	0x01	},
	{	0x33B0	,	0x04	},
	{	0x33B1	,	0x00	},
	{	0x33B3	,	0x00	},
	{	0x33B4	,	0x01	},
	{	0x3800	,	0x00	},

	{	0x3A43	,	0x01	},
	{	0x0100	,	0x01	}
};

/* 3280x2464@29fps base 3542x2598@690M. */
static const struct _mini_sensor_reg_t sonyimx179_framesize_3280x2464_690M[] = {
	{	0x0100	,	0x00	},
	//	Clock Setting

	{	0x011E	,	0x14	},
	{	0x011F	,	0x00	},
	{	0x0301	,	0x05	},
	{	0x0303	,	0x01	},
	{	0x0305	,	0x0A	},
	{	0x0309	,	0x05	},
	{	0x030B	,	0x01	},
	{	0x030C	,	0x01	},
	{	0x030D	,	0x59	},
	{	0x030E	,	0x01	},
	{	0x3A06	,	0x11	},


	//	Mode setting
	{	0x0108	,	0x03	},
	{	0x0112	,	0x0A	},
	{	0x0113	,	0x0A	},
	{	0x0381	,	0x01	},
	{	0x0383	,	0x01	},
	{	0x0385	,	0x01	},
	{	0x0387	,	0x01	},
	{	0x0390	,	0x00	},
	{	0x0391	,	0x11	},
	{	0x0392	,	0x00	},
	{	0x0401	,	0x00	},
	{	0x0404	,	0x00	},
	{	0x0405	,	0x10	},
	{	0x4082	,	0x01	},
	{	0x4083	,	0x01	},
	{	0x7006	,	0x04	},
	//OptionnalFunction setting
	#ifndef IMX179_OTP
	{	0x0700	,	0x00	},
	{	0x3A63	,	0x00	},
	#endif
	{	0x4100	,	0xF8	},
	{	0x4203	,	0xFF	},
	{	0x4344	,	0x00	},
	{	0x441C	,	0x01	},
	//Size setting
//3542x2598
//	{	0x0340	,	0x0A	},
//	{	0x0341	,	0x26	},
//	{	0x0342	,	0x0D	},
//	{	0x0343	,	0xD6	},
	{	0x0340	,	0x0A	},
	{	0x0341	,	0x9E	},
	{	0x0342	,	0x0E	},
	{	0x0343	,	0x10	},

	{	0x0344	,	0x00	},
	{	0x0345	,	0x00	},
	{	0x0346	,	0x00	},
	{	0x0347	,	0x00	},
	{	0x0348	,	0x0C	},
	{	0x0349	,	0xCF	},
	{	0x034A	,	0x09	},
	{	0x034B	,	0x9F	},
	{	0x034C	,	0x0C	},
	{	0x034D	,	0xD0	},
	{	0x034E	,	0x09	},
	{	0x034F	,	0xA0	},
	{	0x0350	,	0x00	},
	{	0x0351	,	0x00	},
	{	0x0352	,	0x00	},
	{	0x0353	,	0x00	},
	{	0x0354	,	0x0C	},
	{	0x0355	,	0xD0	},
	{	0x0356	,	0x09	},
	{	0x0357	,	0xA0	},
	{	0x301D	,	0x30	},
	{	0x3310	,	0x0C	},
	{	0x3311	,	0xD0	},
	{	0x3312	,	0x09	},
	{	0x3313	,	0xA0	},
	{	0x331C	,	0x01	},
	{	0x331D	,	0xAE	},
	{	0x4084	,	0x00	},
	{	0x4085	,	0x00	},
	{	0x4086	,	0x00	},
	{	0x4087	,	0x00	},
	{	0x4400	,	0x00	},
	//Global Timing Setting
	{	0x0830	,	0x77	},
	{	0x0831	,	0x2F	},
	{	0x0832	,	0x4F	},
	{	0x0833	,	0x37	},
	{	0x0834	,	0x2F	},
	{	0x0835	,	0x37	},
	{	0x0836	,	0xAF	},
	{	0x0837	,	0x37	},
	{	0x0839	,	0x1F	},
	{	0x083A	,	0x17	},
	{	0x083B	,	0x02	},
	//	Integration Time Setting
//	{	0x0202	,	0x0A	},
//	{	0x0203	,	0xD6	},
	//	Gain Setting
//	{	0x0205	,	0x00	},
//	{	0x020E	,	0x01	},
//	{	0x020F	,	0x00	},
//	{	0x0210	,	0x01	},
//	{	0x0211	,	0x00	},
//	{	0x0212	,	0x01	},
//	{	0x0213	,	0x00	},
//	{	0x0214	,	0x01	},
//	{	0x0215	,	0x00	},
	//	HDR Setting
	{	0x0230	,	0x00	},
	{	0x0231	,	0x00	},
	{	0x0233	,	0x00	},
	{	0x0234	,	0x00	},
	{	0x0235	,	0x40	},
	{	0x0238	,	0x00	},
	{	0x0239	,	0x04	},
	{	0x023B	,	0x00	},
	{	0x023C	,	0x01	},
	{	0x33B0	,	0x04	},
	{	0x33B1	,	0x00	},
	{	0x33B3	,	0x00	},
	{	0x33B4	,	0x01	},
	{	0x3800	,	0x00	},
	//{	0x34A9	,	0x02	},//-6db
	{	0x3A43	,	0x01	},//init pd

    //color bar:100% color bar
    //{0x0601	,	0x02},
	{	0x0100	,	0x01	}
};

/* 3280x1960@30fps base 3600x2148@580M. */
static const struct _mini_sensor_reg_t sonyimx179_framesize_3280x1960_580M[] = {
	{	0x0100	,	0x00	},
	//	Clock Setting

	{	0x011E	,	0x14	},
	{	0x011F	,	0x00	},
	{	0x0301	,	0x05	},
	{	0x0303	,	0x01	},
	{	0x0305	,	0x0A	},
	{	0x0309	,	0x05	},
	{	0x030B	,	0x01	},
	{	0x030C	,	0x01	},
	{	0x030D	,	0x22	},
	{	0x030E	,	0x01	},
	{	0x3A06	,	0x11	},


	//	Mode setting
	{	0x0108	,	0x03	},
	{	0x0112	,	0x0A	},
	{	0x0113	,	0x0A	},
	{	0x0381	,	0x01	},
	{	0x0383	,	0x01	},
	{	0x0385	,	0x01	},
	{	0x0387	,	0x01	},
	{	0x0390	,	0x00	},
	{	0x0391	,	0x11	},
	{	0x0392	,	0x00	},
	{	0x0401	,	0x00	},
	{	0x0404	,	0x00	},
	{	0x0405	,	0x10	},
	{	0x4082	,	0x01	},
	{	0x4083	,	0x01	},
	{	0x7006	,	0x04	},
	//OptionnalFunction setting
	#ifndef IMX179_OTP
	{	0x0700	,	0x00	},
	{	0x3A63	,	0x00	},
	#endif
	{	0x4100	,	0xF8	},
	{	0x4203	,	0xFF	},
	{	0x4344	,	0x00	},
	{	0x441C	,	0x01	},
	//Size setting
//3280x1960
	{	0x0340	,	0x08	},//vts
	{	0x0341	,	0x64	},
	{	0x0342	,	0x0E	},//vts
	{	0x0343	,	0x10	},
	{	0x0344	,	0x00	},//start_x
	{	0x0345	,	0x00	},
	{	0x0346	,	0x00	},//start_y
	{	0x0347	,	0xFC	},
	{	0x0348	,	0x0C	},//end_x
	{	0x0349	,	0xCF	},
	{	0x034A	,	0x08	},//end_y
	{	0x034B	,	0xA3	},
	{	0x034C	,	0x0C	},//x_out_size
	{	0x034D	,	0xD0	},
	{	0x034E	,	0x07	},//y_out_size
	{	0x034F	,	0xA8	},
	{	0x0350	,	0x00	},//crop_x_stat
	{	0x0351	,	0x00	},
	{	0x0352	,	0x00	},//crop_y_stat
	{	0x0353	,	0x00	},
	{	0x0354	,	0x0C	},//crop_x_size
	{	0x0355	,	0xD0	},
	{	0x0356	,	0x07	},//crop_x_size
	{	0x0357	,	0xA8	},
	{	0x301D	,	0x30	},
	{	0x3310	,	0x0C	},//write_h_size
	{	0x3311	,	0xD0	},
	{	0x3312	,	0x07	},//write_v_size
	{	0x3313	,	0xA8	},
	{	0x331C	,	0x01	},
	{	0x331D	,	0xAE	},
	{	0x4084	,	0x00	},
	{	0x4085	,	0x00	},
	{	0x4086	,	0x00	},
	{	0x4087	,	0x00	},
	{	0x4400	,	0x00	},
	//Global Timing Setting
	{	0x0830	,	0x6F	},
	{	0x0831	,	0x27	},
	{	0x0832	,	0x4F	},
	{	0x0833	,	0x2F	},
	{	0x0834	,	0x2F	},
	{	0x0835	,	0x2F	},
	{	0x0836	,	0x9F	},
	{	0x0837	,	0x37	},
	{	0x0839	,	0x1F	},
	{	0x083A	,	0x17	},
	{	0x083B	,	0x02	},
	//	Integration Time Setting
//	{	0x0202	,	0x07	},
//	{	0x0203	,	0xCC	},
	//	Gain Setting
//	{	0x0205	,	0x00	},
//	{	0x020E	,	0x01	},
//	{	0x020F	,	0x00	},
//	{	0x0210	,	0x01	},
//	{	0x0211	,	0x00	},
//	{	0x0212	,	0x01	},
//	{	0x0213	,	0x00	},
//	{	0x0214	,	0x01	},
//	{	0x0215	,	0x00	},
	//	HDR Setting
	{	0x0230	,	0x00	},
	{	0x0231	,	0x00	},
	{	0x0233	,	0x00	},
	{	0x0234	,	0x00	},
	{	0x0235	,	0x40	},
	{	0x0238	,	0x00	},
	{	0x0239	,	0x04	},
	{	0x023B	,	0x00	},
	{	0x023C	,	0x01	},
	{	0x33B0	,	0x04	},
	{	0x33B1	,	0x00	},
	{	0x33B3	,	0x00	},
	{	0x33B4	,	0x01	},
	{	0x3800	,	0x00	},
	//{	0x34A9	,	0x02	},//-6db
	{	0x3A43	,	0x01	},//init pd

    //color bar:100% color bar
    //{0x0601	,	0x02},
	{	0x0100	,	0x01	}
};

#endif /* SONYIMX179_H_INCLUDED */
/************************** END ***************************/

