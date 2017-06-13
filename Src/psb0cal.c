/*
 * psb0cal.c
 *
 *  Created on: Jun 13, 2017
 *      Author: jamesliu
 */
#include "psb0cal.h"

// psb0ch0Map formula: (raw+20783)/33038
// the below returns microVolts
int32_t psb0ch0Map(int32_t raw){
    if(raw&0x800000) raw -= 0x1000000;
	return (int32_t)roundivide(((int64_t)raw*1000000000-20783000000000),33038000);
//    return (int32_t)(((int64_t)raw*1000000000-69080970588235)/(int64_t)165597927);
}

//psb1ch1Map formula: (raw + 30217.2042253521) / 163576.265422321;
//the below returns microAmps
int32_t psb0ch1Map(int32_t raw){
    if(raw&0x800000) raw -= 0x1000000;
	return (int32_t)roundivide(((int64_t)raw*1000000000+30217204225352),163576265);
}

// psb0ch2Map formula: TODO not calibrated yet
// the below returns microVolts
int32_t psb0ch2Map(int32_t raw){
    return psb0ch4Map(raw);
}

//psb1ch3Map formula: TODO not calibrated yet
//the below returns microAmps
int32_t psb0ch3Map(int32_t raw){
    return psb0ch5Map(raw);
}

// psb0ch4Map formula: (raw+17051.8328322493)/33134.5112988709
// the below returns microVolts
int32_t psb0ch4Map(int32_t raw){
    if(raw&0x800000) raw -= 0x1000000;
	return (int32_t)roundivide(((int64_t)raw*1000000000+17051832832249),33134511);
//    return (int32_t)(((int64_t)raw*1000000000-69080970588235)/(int64_t)165597927);
}

//psb1ch5Map formula: (raw-12693.6707317073)/324921.553523035
//the below returns microAmps
int32_t psb0ch5Map(int32_t raw){
    if(raw&0x800000) raw -= 0x1000000;
	return (int32_t)roundivide(((int64_t)raw*1000000000-12693670731707),324921554);
}
