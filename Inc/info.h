/*
 * info.h
 *
 *  Created on: Nov 8, 2023
 *      Author: Quinn Leydon
 */

#ifndef INC_INFO_H_
#define INC_INFO_H_

enum attribute{
	gen,
	channel,
	type,
	freq,
	minv,
	maxv,
	noise,
	success
};

typedef struct COMMANDS_STRUCT{
	int gen;
	int channel;
	char type;
	float freq;
	float minv;
	float maxv;
	int noise;
}COMMAND;

uint32_t ekg[] = {
	1690, 1680, 1680, 1669, 1648, 1648, 1648, 1680, 1680, 1690, 1680, 1680, 1680, 1680, 1680, 1658,
	1690, 1690, 1712, 1690, 1690, 1680, 1669, 1669, 1680, 1690, 1690, 1680, 1669, 1669, 1669, 1680,
	1680, 1680, 1669, 1669, 1680, 1690, 1690, 1680, 1690, 1690, 1680, 1690, 1690, 1712, 1680, 1680,
	1658, 1648, 1648, 1648, 1669, 1669, 1680, 1690, 1690, 1701, 1680, 1680, 1669, 1680, 1680, 1680,
	1701, 1701, 1701, 1690, 1690, 1701, 1712, 1712, 1722, 1712, 1712, 1690, 1669, 1669, 1680, 1690,
	1690, 1690, 1733, 1733, 1765, 1776, 1861, 1882, 1936, 1936, 1968, 1989, 1989, 2032, 2053, 2053,
	2085, 2149, 2069, 2080, 2058, 2058, 1930, 1930, 1845, 1824, 1792, 1872, 1840, 1754, 1754, 1722,
	1680, 1680, 1680, 1637, 1637, 1637, 1637, 1637, 1626, 1648, 1648, 1637, 1605, 1605, 1616, 1680,
	1680, 1765, 1776, 1861, 2042, 2106, 2021, 1776, 2480, 2400, 2176, 1632, 1637, 1360, 933, 928,
	1962, 1962, 2042, 2149, 3141, 3141, 2320, 1200, 1200, 1392, 1669, 1669, 1658, 1701, 1701, 1701,
	1701, 1701, 1722, 1690, 1690, 1690, 1680, 1680, 1690, 1690, 1690, 1669, 1669, 1669, 1701, 1733,
	1733, 1754, 1744, 1744, 1733, 1733, 1733, 1722, 1765, 1765, 1765, 1733, 1733, 1733, 1722, 1722,
	1701, 1690, 1690, 1701, 1690, 1690, 1701, 1701, 1701, 1701, 1722, 1722, 1712, 1722, 1722, 1733,
	1733, 1733, 1733, 1712, 1712, 1712, 1733, 1733, 1733, 1733, 1733, 1733, 1744, 1744, 1744, 1744,
	1744, 1744, 1733, 1733, 1722, 1722, 1722, 1722, 1722, 1722, 1733, 1722, 1722, 1722, 1722, 1722,
	1701, 1669, 1669, 1680, 1690, 1690, 1690, 1701, 1701, 1712, 1712, 1712, 1690, 1669, 1669, 1680,
};

#endif /* INC_INFO_H_ */