#ifndef BITOPER_H
#define BITOPER_H

#define SETBIT(a,pos)				(a |= (1 << pos))
#define CLEARBIT(a,pos) 		(a &= ~(1 << pos))
#define TOGGLEBIT(a,pos)		(a ^= (1 << pos))
#define GETBIT(a,pos)				(a & (1 << pos))

#endif