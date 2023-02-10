#ifndef ENGAUTO_H
#define ENGAUTO_H_H

// logo engauto em bitmap
const uint16_t engauto[] PROGMEM ={
0x3841, 0x9862, 0x9862, 0x9882, 0xa082, 0xa062, 0xa062, 0xa082, 0xa062, 0xa062, 0xa062, 0xa082, 0xa062, 0xa082, 0xa062, 0xa082, 0xa062, 0xa062, 0xa062, 0xa082, 0xa062, 0xa062, 0xa062, 0xa062, 0xa062, 0xa062, 0xa082, 0xa062, 0xa062, 0xa082, 0xa062, 0xa062, 0xa062, 0xa082, 0xa062, 0xa062, 0xa062, 0xa082, 0xa062, 0xa062, 0xa062, 0xa062, 0xa062, 0xa062, 0x9062, 0x9062, 0x9062, 0xb882, 0x9082, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x8062, 0xb882, 0xa883, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa082, 0xa082, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa882, 0xa082, 0xa082, 0xa082, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa882, 0xa082, 0xa882, 0xa082, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa082, 0xa882, 0xa882, 0xa082, 0xa882, 0xa082, 0xa862, 0xa882, 0xa082, 0x9882, 0xa082, 0x3862, 
0xb882, 0xf8c4, 0xf8c4, 0xf8a3, 0xf0a3, 0xf0a4, 0xf0a4, 0xf0a3, 0xf0a4, 0xf0a3, 0xe883, 0xe883, 0xe883, 0xe863, 0xe883, 0xe883, 0xe883, 0xe883, 0xe883, 0xf0a3, 0xe883, 0xe021, 0xe042, 0xe042, 0xe042, 0xe8a4, 0xf0c4, 0xf0a4, 0xf0c4, 0xe8a3, 0xe042, 0xe042, 0xe862, 0xf0a4, 0xf0c4, 0xf0a3, 0xf0a3, 0xe8a4, 0xf0a3, 0xf0a4, 0xf0a4, 0xe8a3, 0xf0a3, 0xd8a3, 0xe8a4, 0xf8a3, 0xf8c4, 0xe0a4, 0x3021, 0x0000, 0x0000, 0x0000, 0x0000, 0x5882, 0xf0c4, 0xf0a4, 0xe8a3, 0xe8c4, 0xe8a3, 0xe8a4, 0xe8a4, 0xe0a4, 0xe0a3, 0xe8a4, 0xe8a3, 0xe8a4, 0xe8a4, 0xe8a3, 0xe0c4, 0xe8a4, 0xe0a4, 0xe0a4, 0xe0a3, 0xe0a3, 0xe083, 0xe083, 0xe083, 0xe0a3, 0xe083, 0xe083, 0xe0a3, 0xe083, 0xe083, 0xe0a3, 0xe8c3, 0xe8a4, 0xe083, 0xe083, 0xe083, 0xe083, 0xe083, 0xe083, 0xe083, 0xe083, 0xe8a3, 0xe8a4, 0xe8c3, 0xe0a3, 0xe083, 0xe083, 0xe0a3, 0xe0a3, 0xe8a3, 0xe8c3, 0xe8c3, 0xe8a3, 0xe8c4, 0xe0a3, 0xe063, 0xe083, 0xe083, 0xe083, 0xe0a3, 0xe8c4, 0xe0c4, 0xe083, 0xe083, 0xe083, 0xe083, 0xe083, 0xe083, 0xe8a3, 0xe8a4, 0xe8a3, 0xe8a3, 0xe0a3, 0xe083, 0xe083, 0xe0a3, 0xe0c4, 0xe8a4, 0xe8a3, 0xe8a4, 0xe8a3, 0xe8a3, 0xe083, 0xe083, 0xe083, 0xe8a3, 0xe083, 0xe083, 0xe083, 0xe083, 0xe083, 0xe083, 0xe083, 0xe083, 0xe083, 0xe8a3, 0xe8a3, 0xe083, 0xe0a3, 0xe8a4, 0xe8c3, 0xe8a4, 0xe8a3, 0xf0c4, 0xf8c4, 0xf8a4, 0xc0a3, 
0xb082, 0xf8a3, 0xa083, 0x1000, 0x1000, 0x1800, 0x1800, 0x1800, 0x1800, 0x0800, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1021, 0x0800, 0x1000, 0x0800, 0x1000, 0x1000, 0x1820, 0x1000, 0x1020, 0x1000, 0x0800, 0x0800, 0x1020, 0x28a2, 0x1000, 0x1000, 0x0800, 0x0800, 0x0800, 0x0800, 0x0020, 0x0800, 0x0820, 0x0000, 0x3821, 0xd8a3, 0xf8c4, 0xc8a3, 0x0800, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0820, 0x0000, 0x0000, 0x1020, 0x1020, 0x1020, 0x0800, 0x0000, 0x0000, 0x0000, 0x1000, 0x1020, 0x1020, 0x1000, 0x1020, 0x0800, 0x0000, 0x0000, 0x0800, 0x0800, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1020, 0x0800, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0800, 0x1020, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0820, 0x0800, 0x1020, 0x1020, 0x0800, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x1841, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0800, 0x0000, 0x0000, 0x0800, 0x0800, 0x0000, 0x0000, 0x0800, 0x1020, 0x1000, 0x0820, 0x0820, 0x1020, 0x0800, 0x0000, 0x0000, 0x0841, 0x0820, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0821, 0x0000, 0x0000, 0x0820, 0x1020, 0x0820, 0x0820, 0x0800, 0x0020, 0x8883, 0xf8a4, 0xc0a3, 
0xa882, 0xf8c4, 0x6041, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4269, 0xc6ba, 0xae17, 0xadf7, 0xadf7, 0xadf7, 0xadf7, 0xadf7, 0xae18, 0xc6db, 0x1945, 0x7430, 0xe7be, 0xc6ba, 0xd73c, 0xbe79, 0x1082, 0x0000, 0x0000, 0x0000, 0x530c, 0xe7be, 0xf7ff, 0x4aeb, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xb883, 0xf8a4, 0xd8a3, 0x2000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3842, 0x9882, 0x7861, 0x8062, 0x6062, 0x0000, 0x0000, 0x0000, 0x0000, 0x5b4c, 0x9d95, 0x7430, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0882, 0x7430, 0xa5b6, 0x4aaa, 0x0000, 0x9d54, 0xadf7, 0xa596, 0xa596, 0xa596, 0xa596, 0xa596, 0xa596, 0x9d75, 0xcefb, 0x52aa, 0x0000, 0x5b6d, 0xadf7, 0xb618, 0xadf7, 0xadd7, 0xadf7, 0xadf7, 0xadf7, 0xb659, 0x8cb2, 0x0021, 0x0000, 0x6bef, 0xb638, 0x9d55, 0xa5b6, 0x9d55, 0x10a2, 0x0000, 0x0000, 0x0000, 0x0000, 0x8492, 0xc69a, 0xadd7, 0xb638, 0xbe79, 0x1082, 0x0000, 0x63ae, 0xadf7, 0xb618, 0xadd7, 0xadd7, 0xb658, 0xa596, 0x0021, 0x3208, 0xadf7, 0x0861, 0x39e8, 0xc6db, 0xcedb, 0x4aca, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x10a2, 0x9d95, 0xefff, 0x638e, 0x3a28, 0xc69a, 0xadd7, 0xadd7, 0xadb6, 0xadd7, 0xadd6, 0xadd7, 0xb617, 0xb638, 0x2145, 0x6bcf, 0xcefb, 0x6bcf, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4841, 0xf8a4, 0xc083, 
0xb062, 0xf8c4, 0x6041, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0861, 0xe73c, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x94b2, 0x4a69, 0xffff, 0xffff, 0xffff, 0xffff, 0xef5d, 0x0841, 0x0000, 0x0000, 0x2965, 0xffff, 0xffff, 0xbe38, 0x0000, 0x4000, 0xa862, 0xc0a3, 0xc883, 0xc883, 0xc883, 0xc083, 0xc083, 0xb882, 0xd8a3, 0xf8a4, 0xc883, 0x1020, 0x0000, 0x0000, 0x0000, 0x0000, 0x3041, 0xe8c3, 0xf8a4, 0xf8a3, 0xf8a4, 0xc8a3, 0x0000, 0x0000, 0x0000, 0x39e7, 0xffff, 0xffff, 0x94d2, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7bcf, 0xffff, 0xffff, 0x3186, 0x5aeb, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xef7d, 0x2103, 0xb595, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x3166, 0x1082, 0xffff, 0xffff, 0xffff, 0xffff, 0xef7d, 0x0841, 0x0000, 0x0000, 0x0000, 0xb596, 0xffff, 0xffff, 0xffff, 0xffff, 0x94b2, 0x0000, 0xd69a, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xef7d, 0xc658, 0xffff, 0xffff, 0x3186, 0x7bcf, 0xffff, 0xffff, 0x5acb, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xce79, 0xffff, 0xe71c, 0x31a6, 0xd6ba, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x9cd3, 0x528a, 0xffff, 0xffff, 0x5acb, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5061, 0xf8a4, 0xb8a3, 
0xa882, 0xf8c4, 0x6041, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x94b2, 0xffff, 0xf79e, 0x7bef, 0x73ae, 0x7bef, 0x7bef, 0x7bef, 0x8c51, 0x7bcf, 0x1082, 0xce9a, 0xffff, 0xef9e, 0xffff, 0xffff, 0xdefb, 0x0000, 0x0000, 0x0000, 0xad76, 0xffff, 0xf7ff, 0x00e3, 0x5000, 0xf8c4, 0xf8c4, 0xf8c3, 0xf8c4, 0xf8a4, 0xf8c3, 0xf8c4, 0xf8c3, 0xf8c4, 0xf8c4, 0xd0a3, 0x1020, 0x0000, 0x0000, 0x0000, 0x0000, 0x3041, 0xf8a4, 0xf8a4, 0x9883, 0xd083, 0xf0a3, 0xa082, 0x0000, 0x0000, 0x0000, 0xb5b6, 0xffff, 0xe73d, 0x0841, 0x0000, 0x0000, 0x0000, 0x0000, 0x0861, 0xef5d, 0xffff, 0x9492, 0x0000, 0xdefb, 0xffff, 0xd69a, 0xe73c, 0xffff, 0xffff, 0xef5d, 0xce79, 0xdedb, 0xf7be, 0x2945, 0x6b4d, 0xffff, 0xffff, 0xef7e, 0xd69a, 0xdedb, 0xdedb, 0xd69a, 0xef5d, 0xffff, 0xffff, 0xf79e, 0x0020, 0x9cf3, 0xffff, 0xf7be, 0xf7df, 0xffff, 0xce59, 0x0000, 0x0000, 0x0000, 0x9cf3, 0xffff, 0xf7be, 0xf7be, 0xffff, 0xf7be, 0x0841, 0x94b2, 0xffff, 0xffff, 0xef3d, 0xd6ba, 0xdedb, 0xdebb, 0xd69a, 0xef7d, 0xffff, 0xffff, 0xf79e, 0x0841, 0xb596, 0xffff, 0xffff, 0x2124, 0x0000, 0x0000, 0x0000, 0x0000, 0xce59, 0xffff, 0xe73c, 0x0000, 0x632c, 0xffff, 0xffdf, 0xbdd7, 0xbdd7, 0xbdd7, 0xbdd7, 0xbdd7, 0xc618, 0xc618, 0x2965, 0xbdd7, 0xffff, 0xbdd7, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5041, 0xf8a4, 0xb883, 
0xa882, 0xf8c4, 0x6041, 0x0000, 0x0000, 0x0000, 0x0000, 0x4a6a, 0xffff, 0xffff, 0x4228, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9cd3, 0xffff, 0xbdd7, 0x528a, 0xffff, 0xffff, 0xce79, 0x0000, 0x0000, 0x4a49, 0xffff, 0xffff, 0x53ce, 0x2000, 0xf8a4, 0xf8c3, 0x8863, 0x4841, 0x4021, 0x4021, 0x4821, 0x4841, 0x4841, 0x4821, 0x5841, 0x2001, 0x0000, 0x0000, 0x0000, 0x0000, 0x1821, 0xe8a3, 0xf8c4, 0x5061, 0x3042, 0xe8a3, 0xf8a3, 0x8882, 0x0000, 0x0000, 0x4228, 0xffff, 0xffff, 0x6b6d, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x7bcf, 0xffff, 0xffff, 0x2945, 0x0000, 0x1082, 0x0000, 0x0000, 0xd69a, 0xffff, 0xffff, 0x39c7, 0x0000, 0x0000, 0x0000, 0x2945, 0xffff, 0xffff, 0xc638, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4a49, 0xffff, 0xffff, 0x6b4d, 0x3186, 0xffff, 0xffff, 0x9cd3, 0xdefb, 0xffff, 0xb596, 0x0000, 0x0000, 0x8410, 0xffff, 0xffff, 0xad55, 0xef5d, 0xffff, 0x632c, 0x39c7, 0xffff, 0xffff, 0x9cb3, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4228, 0xffff, 0xffff, 0x7c0f, 0x0000, 0xe71c, 0xffff, 0xe71c, 0x0000, 0x0000, 0x0000, 0x0000, 0xb596, 0xffff, 0xf79e, 0x0881, 0x0000, 0xf79e, 0xffff, 0x8430, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x632c, 0xffff, 0xffff, 0x4a49, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5041, 0xf8a4, 0xb8a3, 
0xa862, 0xf8c4, 0x6042, 0x0000, 0x0000, 0x0000, 0x0820, 0xe71b, 0xffff, 0xf79e, 0xad75, 0xc638, 0xce59, 0xc659, 0xdedb, 0xdedb, 0x0000, 0x41e7, 0xffff, 0xffff, 0x2945, 0x2945, 0xffff, 0xffff, 0xc638, 0x0000, 0x0000, 0xd6ba, 0xffff, 0xb679, 0x0000, 0xc822, 0xf8c4, 0x8062, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x2041, 0xe8c3, 0xf8c4, 0x6062, 0x0000, 0x5041, 0xf0a3, 0xf8a4, 0x7862, 0x0000, 0x0000, 0xce99, 0xffff, 0xce38, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1082, 0xf79e, 0xffff, 0x8c51, 0x0000, 0x0000, 0x0000, 0x0000, 0x6b6d, 0xffff, 0xffff, 0x9cd3, 0x0000, 0x0000, 0x0000, 0x0000, 0xce59, 0xffff, 0xef5d, 0x1082, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x8c51, 0xffff, 0xef5d, 0x1082, 0xbdb7, 0xffff, 0xad55, 0x2965, 0xffff, 0xffff, 0x8c71, 0x0000, 0x8c51, 0xffff, 0xffff, 0x52aa, 0x7bcf, 0xffff, 0xef5d, 0x10a2, 0xbdf7, 0xffff, 0xce79, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x738e, 0xffff, 0xf79e, 0x0000, 0x0020, 0xffff, 0xffff, 0xc638, 0x0000, 0x0000, 0x0000, 0xa514, 0xffff, 0xffff, 0x2965, 0x0000, 0x8c71, 0xffff, 0xffff, 0xad75, 0x94b2, 0xad55, 0xa534, 0xa514, 0xce79, 0x5acb, 0x0000, 0xe71c, 0xffff, 0xbdf7, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5061, 0xf8a3, 0xc083, 
0xa882, 0xf8c4, 0x6041, 0x0000, 0x0000, 0x0000, 0x8c51, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x8430, 0x0000, 0xdedb, 0xffff, 0xb596, 0x0000, 0x5aeb, 0xffff, 0xffff, 0xb596, 0x0000, 0x8c50, 0xffff, 0xf7ff, 0x0104, 0x7800, 0xf8c4, 0xc083, 0x0000, 0x0000, 0x5021, 0x9882, 0x8062, 0x9062, 0x3841, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1821, 0xe0c3, 0xf8a4, 0x4041, 0x0000, 0x0000, 0x6061, 0xf8a4, 0xf8a3, 0x6061, 0x0000, 0x5acb, 0xffff, 0xffff, 0x528a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9cf3, 0xffff, 0xdefb, 0x0000, 0x0000, 0x0000, 0x0000, 0x10a2, 0xf7be, 0xffff, 0xef5d, 0x0841, 0x0000, 0x0000, 0x0000, 0x632c, 0xffff, 0xffff, 0x632c, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x2945, 0xffff, 0xffff, 0x528a, 0x31a6, 0xffff, 0xffff, 0x2104, 0x39e7, 0xffff, 0xffff, 0x4208, 0x4a29, 0xffff, 0xffff, 0x6b8e, 0x0020, 0xffff, 0xffff, 0x52aa, 0x39c7, 0xffff, 0xffff, 0x4228, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x2945, 0xffff, 0xffff, 0x632c, 0x0000, 0x31a6, 0xffff, 0xffff, 0xad55, 0x0000, 0x0000, 0x9cf3, 0xffff, 0xffff, 0x2986, 0x0000, 0x31a6, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x18e3, 0x6b6d, 0xffff, 0xffff, 0x3186, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5041, 0xf8a4, 0xb8a3, 
0xb082, 0xf8c4, 0x6041, 0x0000, 0x0000, 0x2965, 0xffff, 0xffff, 0xc638, 0x8c71, 0x9cf3, 0x9cf3, 0x9cf3, 0xad55, 0x9cd3, 0x0000, 0x73ae, 0xffff, 0xffff, 0x3186, 0x0000, 0x632c, 0xffff, 0xffff, 0x8430, 0x0862, 0xffff, 0xffff, 0x53cf, 0x2000, 0xf8a3, 0xe8a3, 0x2821, 0x0000, 0x5041, 0xf8c4, 0xf8e4, 0xf0a4, 0xf8e5, 0x8882, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0820, 0xc8c4, 0xf8a4, 0xb883, 0x5062, 0x5041, 0x5842, 0xa083, 0xe883, 0xf8a4, 0x4800, 0x0000, 0xce79, 0xffff, 0xc618, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3186, 0xffff, 0xffff, 0x632c, 0x0000, 0x0000, 0x0000, 0x0000, 0x8430, 0xffff, 0xffff, 0x738e, 0x0000, 0x0000, 0x0000, 0x0000, 0xdefb, 0xffff, 0xb5b6, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xbdd7, 0xffff, 0xce79, 0x0000, 0xc618, 0xffff, 0xb596, 0x0000, 0x8430, 0xffff, 0xffff, 0xa514, 0xef5d, 0xffff, 0x94b2, 0x0000, 0xb5b6, 0xffff, 0xce79, 0x0000, 0xc618, 0xffff, 0x9cd3, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xa514, 0xffff, 0xd69a, 0x0000, 0x0000, 0x528a, 0xffff, 0xffff, 0x8410, 0x0000, 0x8410, 0xffff, 0xffff, 0x4228, 0x0000, 0x0000, 0xad55, 0xffff, 0xe73c, 0x9492, 0x9cf3, 0x9cf3, 0x9cf3, 0x9cf3, 0xb5b6, 0x528a, 0x1082, 0xffdf, 0xffff, 0x9cf3, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5041, 0xf8a4, 0xb8a3, 
0xa882, 0xf8c4, 0x6041, 0x0000, 0x0000, 0xd6ba, 0xffff, 0x9cf3, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4a49, 0xffff, 0xffff, 0x6b6d, 0x0000, 0x0000, 0x738e, 0xffff, 0xffff, 0x8c51, 0x94b3, 0xffff, 0xae79, 0x1000, 0xe042, 0xf0a3, 0x3021, 0x0000, 0x0000, 0x3840, 0x3020, 0x5841, 0xf8a4, 0xc8a3, 0x1000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1020, 0xd0c3, 0xf8a3, 0xe083, 0xe0a3, 0xf8a4, 0xf8c4, 0xf8c4, 0xe8a3, 0xd0a3, 0xe8a3, 0x1800, 0x3aca, 0xffff, 0xffff, 0x6b4d, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xc638, 0xffff, 0xce79, 0x0000, 0x0000, 0x0000, 0x0000, 0x18c3, 0xffdf, 0xffff, 0xd6ba, 0x0000, 0x0000, 0x0000, 0x0000, 0x7bef, 0xffff, 0xffff, 0x6b4d, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x73ae, 0xffff, 0xffff, 0x31a6, 0x4208, 0xffff, 0xffdf, 0x18e3, 0x0000, 0xa514, 0xffff, 0xffff, 0xffff, 0xffff, 0x8c51, 0x0000, 0x4a69, 0xffff, 0xffff, 0x39c7, 0x39c7, 0xffff, 0xffff, 0x5aeb, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5aeb, 0xffff, 0xffff, 0x52aa, 0x0000, 0x0000, 0x6b6d, 0xffff, 0xffff, 0x4a48, 0x4a49, 0xffff, 0xffff, 0x52aa, 0x0000, 0x0000, 0x41e7, 0xffff, 0xffff, 0x4a69, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9cf3, 0xffff, 0xffff, 0x3186, 0x0000, 0x0020, 0x0020, 0x0000, 0x1082, 0x0020, 0x0000, 0x0000, 0x5041, 0xf8a4, 0xc0a3, 
0xa882, 0xf8c4, 0x6041, 0x0000, 0x7bef, 0xffff, 0xffff, 0xa514, 0x73ae, 0x9cf3, 0x94b2, 0x94b2, 0xa534, 0x9cf3, 0x0841, 0xc618, 0xffff, 0xce59, 0x0000, 0x0000, 0x0000, 0x8c51, 0xffff, 0xffff, 0xf7be, 0xffff, 0xffff, 0x2a28, 0x8000, 0xf8c4, 0xb883, 0x2820, 0x3001, 0x3820, 0x1820, 0x0000, 0xa062, 0xf8c3, 0x1821, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0xc0a3, 0xf8c4, 0xd8a3, 0xc0a3, 0xc8a3, 0xc8a3, 0xc8a3, 0xc8a3, 0xd083, 0xd883, 0xd022, 0x3061, 0xbeba, 0xffff, 0xffff, 0xd69a, 0x9472, 0x9cf3, 0x9cd3, 0x94b2, 0xdedb, 0xffff, 0xffff, 0x528a, 0x0000, 0x0000, 0x0000, 0x0000, 0x8c71, 0xffff, 0xffff, 0x630c, 0x0000, 0x0000, 0x0000, 0x10a2, 0xf79e, 0xffff, 0xffff, 0xf79e, 0xdedb, 0xdedb, 0xdedb, 0xd6ba, 0xe71c, 0xffff, 0xffff, 0xad75, 0x0000, 0xe73c, 0xffff, 0x7bcf, 0x0000, 0x0000, 0xbdd7, 0xffff, 0xffff, 0xffff, 0x9cf3, 0x0000, 0x0000, 0xc638, 0xffff, 0xc618, 0x0020, 0xdedb, 0xffff, 0xffff, 0xef5d, 0xc618, 0xc638, 0xce59, 0xc618, 0xd6ba, 0xffff, 0xffff, 0xc638, 0x0000, 0x0000, 0x0000, 0x8c51, 0xffff, 0xffff, 0xd6ba, 0xffff, 0xffff, 0x5acb, 0x0000, 0x0000, 0x0000, 0xb5b6, 0xffff, 0xffdf, 0x8410, 0x7bcf, 0x8410, 0x8410, 0x8410, 0x9cf3, 0x528a, 0x18e3, 0xffff, 0xffff, 0xf79e, 0xd6ba, 0xe73c, 0xe73c, 0xe71c, 0xffff, 0xad75, 0x0000, 0x0000, 0x0000, 0x5041, 0xf8c4, 0xb8a3, 
0xa882, 0xf8c4, 0x4000, 0x11a6, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x8430, 0x73ae, 0xffff, 0xffff, 0x39e7, 0x0000, 0x0000, 0x0000, 0x8c71, 0xffff, 0xffff, 0xffff, 0xffff, 0x6451, 0x3800, 0xf8a4, 0xe0a3, 0xe0a3, 0xf8c4, 0xf8a4, 0xf8c4, 0xf8a3, 0xf8e4, 0xe0a3, 0x1800, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xa8a3, 0xf8c4, 0xc8a4, 0x1821, 0x0000, 0x1020, 0x0800, 0x0000, 0x0020, 0xc8a3, 0xf8a4, 0xd000, 0x6249, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x94b2, 0x0000, 0x0000, 0x0000, 0x0000, 0x4a69, 0xffff, 0xffff, 0xc618, 0x0000, 0x0000, 0x0000, 0x0000, 0x52aa, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xdefb, 0x0000, 0x8410, 0xffff, 0xdedb, 0x0000, 0x0000, 0x0000, 0xd6db, 0xffff, 0xffff, 0xa514, 0x0000, 0x0000, 0x6b4d, 0xffff, 0xffff, 0x31c6, 0x2124, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xe71c, 0x18e3, 0x0000, 0x0000, 0x0000, 0xb596, 0xffff, 0xffff, 0xffff, 0xffff, 0x6b4d, 0x0000, 0x0000, 0x0000, 0x52aa, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x7bcf, 0x9cf3, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x52aa, 0x0000, 0x0000, 0x0000, 0x5041, 0xf8a4, 0xb8a2, 
0xa882, 0xf8a3, 0x4000, 0x7513, 0xffff, 0xd69a, 0xd6db, 0xd6db, 0xd6db, 0xd6db, 0xd6db, 0xef9e, 0x9cf3, 0x0041, 0xd6ba, 0xffff, 0x8430, 0x0000, 0x0000, 0x0000, 0x0000, 0x8430, 0xffff, 0xdf1c, 0xf7df, 0xbe18, 0x0000, 0x4000, 0xe083, 0xc882, 0xc083, 0xc883, 0xc883, 0xc083, 0xc883, 0xa063, 0x1820, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0xb8a3, 0xf8c4, 0xb083, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x6062, 0x9863, 0x6800, 0x0020, 0x9575, 0xffff, 0xffff, 0xf7ff, 0xf7ff, 0xefdf, 0xe79e, 0xf7ff, 0xe77d, 0x634d, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x6bce, 0xcefb, 0xadb6, 0x2145, 0x0000, 0x0000, 0x0000, 0x0000, 0x18e3, 0xbe59, 0xe79e, 0xdf3c, 0xdf3c, 0xdf3c, 0xd71c, 0xdf3c, 0xdf5d, 0xdf3c, 0x7c10, 0x0000, 0x0000, 0xb618, 0xdf7d, 0x4269, 0x0000, 0x0000, 0x0020, 0xa555, 0xdf3c, 0x7c30, 0x0000, 0x0000, 0x0000, 0x9d14, 0xe7be, 0x94f3, 0x0000, 0x0000, 0x9d14, 0xd73c, 0xd6fb, 0xcedb, 0xcedb, 0xcedb, 0xcedb, 0xd71c, 0xd6fb, 0x8471, 0x0041, 0x0000, 0x0000, 0x0000, 0x0000, 0x8c92, 0xceba, 0xb5f7, 0xcedb, 0x636d, 0x0000, 0x0000, 0x0000, 0x0000, 0xa575, 0xe79e, 0xbe59, 0xc679, 0xceba, 0xceba, 0xceba, 0xceba, 0xdf5d, 0x9d34, 0x29c7, 0xbe59, 0xb5f7, 0xadd7, 0xadd7, 0xadd7, 0xadd7, 0xadd7, 0xc69a, 0x8cb2, 0x0000, 0x0000, 0x0000, 0x0000, 0x5041, 0xf8a3, 0xc0a3, 
0xa882, 0xf8a4, 0x8083, 0x0061, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0061, 0x0061, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0000, 0x0000, 0x0800, 0x1000, 0x1000, 0x1000, 0x0800, 0x1820, 0x1000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xa0a3, 0xf8a4, 0xc8a3, 0x1841, 0x0000, 0x0820, 0x0020, 0x0820, 0x0800, 0x0820, 0x0821, 0x0020, 0x0000, 0x0000, 0x0000, 0x0000, 0x2104, 0x3165, 0x2945, 0x2965, 0x2904, 0x18a2, 0x18c3, 0x0841, 0x0000, 0x0000, 0x0020, 0x0000, 0x0000, 0x0020, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0000, 0x0000, 0x0841, 0x0841, 0x0841, 0x0020, 0x0841, 0x0841, 0x0841, 0x0020, 0x0000, 0x0000, 0x0841, 0x0020, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0841, 0x0841, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0000, 0x0841, 0x0861, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x0020, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x8883, 0xf8a3, 0xc083, 
0xc083, 0xf8c4, 0xe883, 0xa801, 0x9000, 0xa000, 0xa001, 0xa001, 0xa801, 0xa801, 0xa801, 0xa800, 0xb041, 0xb062, 0xa801, 0xa801, 0xb042, 0xb883, 0xb862, 0xb862, 0xb883, 0xb021, 0xa801, 0xa801, 0xa800, 0xb021, 0xb8a3, 0xb882, 0xa862, 0xb062, 0xa862, 0xb062, 0xb863, 0xb082, 0x1820, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xa8a3, 0xf8a4, 0xe883, 0xc883, 0xb883, 0xd0a3, 0xd083, 0xd083, 0xd083, 0xd083, 0xd083, 0xd0a3, 0xd083, 0xd082, 0xd083, 0xd0a3, 0xc862, 0xb800, 0xb000, 0xb800, 0xb800, 0xb800, 0xc000, 0xb800, 0xc021, 0xd083, 0xd083, 0xc883, 0xd083, 0xd083, 0xc883, 0xc863, 0xc862, 0xc862, 0xd083, 0xc883, 0xc883, 0xd083, 0xc883, 0xd0a3, 0xc862, 0xb801, 0xb801, 0xb801, 0xb801, 0xb801, 0xb801, 0xb800, 0xc021, 0xc883, 0xd0a3, 0xc883, 0xc842, 0xc842, 0xc883, 0xd083, 0xc883, 0xd083, 0xc862, 0xc842, 0xc863, 0xc8a3, 0xd083, 0xc883, 0xc842, 0xc042, 0xc862, 0xd083, 0xd083, 0xc863, 0xc021, 0xc001, 0xc021, 0xc001, 0xc021, 0xc021, 0xc001, 0xc022, 0xc883, 0xd083, 0xc883, 0xc883, 0xd083, 0xc883, 0xc862, 0xc062, 0xc062, 0xc842, 0xc882, 0xd083, 0xd083, 0xc883, 0xc883, 0xc042, 0xc021, 0xc041, 0xc022, 0xc022, 0xc021, 0xc021, 0xc021, 0xb821, 0xc042, 0xc883, 0xc042, 0xc062, 0xc062, 0xc042, 0xc862, 0xc042, 0xc062, 0xc042, 0xc862, 0xc883, 0xd083, 0xc083, 0xd8a3, 0xf0a3, 0xf883, 0xc8c3, 
0x6082, 0xc883, 0xc0a3, 0xd8a4, 0xd8a4, 0xd8a3, 0xd8c4, 0xd8a3, 0xd8a3, 0xd8a3, 0xd8a3, 0xd8a3, 0xd8a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xe8c4, 0x6862, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x88a3, 0xf0c4, 0xb8a3, 0xb8a3, 0xc0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xd0a3, 0xc8a3, 0xc0a3, 0xc8a3, 0x5062
};

#endif