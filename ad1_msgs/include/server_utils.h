/*
 * uart1_utils.h
 *
 *  Created on: Jul 30, 2021
 *      Author: swpark
 */

#ifndef INC_SERVER_UTILS_H_
#define INC_SERVER_UTILS_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

using namespace std;

#define CHECK_SIZE  4
#define CHECK_START 0
#define ID_SIZE     4
#define ID_START    0//CHECK_SIZE, check문자를 제외하고 parsing을 해서 ID_START를 0으로
#define COUNT_SIZE  8
#define COUNT_START (ID_START+ID_SIZE)
#define E_SIZE      16
#define E_START     (COUNT_START+COUNT_SIZE)
#define Q_SIZE      32
#define Q_START     (E_START+E_SIZE)
#define G_SIZE      32
#define G_START     (Q_START+Q_SIZE)
#define A_SIZE      32
#define A_START     (G_START+G_SIZE)
#define GPS_SIZE    64
#define GPS_START   (A_START+A_SIZE)

//3000초 저장량
//module로 부터 받는 데이터 총 크기는 192
// 4 / 4 / 8 / 16(12/4:x) / 32 / 32 / 32 / 64(57/7:x)
#define DATASET_SIZE 192
//(CHECK_SIZE+ID_SIZE+COUNT_SIZE+E_SIZE+Q_SIZE+G_SIZE+A_SIZE+GPS_SIZE)
//#define DATASET_SIZE GPS_SIZE+GPS_START
#define MAX_BUFFER_SIZE DATASET_SIZE*30000


typedef struct{
  uint32_t head;
  uint32_t tail;
  uint8_t buffer[MAX_BUFFER_SIZE];
}serverQ_t;

uint8_t serverutils_isEmpty(serverQ_t* u)
{
    return u->head == u->tail;
}

//void serverutils_pop(serverQ_t* u, char* data)
//{
//    if(u->tail+DATASET_SIZE+5 <= MAX_BUFFER_SIZE)
//    {//not overflow
//        memcpy(data, &u->buffer[u->tail], DATASET_SIZE+5);
//    }
//    else {
//        //overflow
//        memcpy(data, &u->buffer[u->tail], MAX_BUFFER_SIZE-u->tail);
//        memcpy(&data[MAX_BUFFER_SIZE-u->tail], &u->buffer[0], DATASET_SIZE+5 - (MAX_BUFFER_SIZE-u->tail));
//    }

//  //start character check
//  char check[6];
//  for(int i=0 ; i<DATASET_SIZE ; i++)
//  {
//      memcpy(check, &data[i], 5);

//      check[5] = 0;
//      if(!strcmp(&check[0], "Koins"))
//      {
//          if(i!=0)
//          {
//              u->tail += i;

//              if (u->tail >= MAX_BUFFER_SIZE) {
//                  u->tail -= MAX_BUFFER_SIZE;
//              }
//              data[0] = 0;

//              return;
//          }
//      }
//  }

//  u->tail = u->tail + DATASET_SIZE;

//  if (u->tail >= MAX_BUFFER_SIZE) {
//      u->tail -= MAX_BUFFER_SIZE;
//  }
//}

uint8_t serverutils_pop(serverQ_t* u)
{
    uint8_t data = u->buffer[u->tail];
    u->tail++;

    if(u->tail >= MAX_BUFFER_SIZE)
        u->tail=0;

    return data;
}

void serverutils_push(serverQ_t* u, char* data, int size)
{
    for(int i=0 ; i<size ; i++)
        data[i] = data[i]&0x000000ff;

    if(u->head + size <= MAX_BUFFER_SIZE)
    {
        memcpy(&u->buffer[u->head], data, size);
        u->head+=size;
    }
    else {
        memcpy(&u->buffer[u->head], &data[0], MAX_BUFFER_SIZE-u->head);
        memcpy(&u->buffer[0], &data[MAX_BUFFER_SIZE-u->head], size-(MAX_BUFFER_SIZE-u->head));
        u->head = size-(MAX_BUFFER_SIZE-u->head);
    }

    if (u->head >= MAX_BUFFER_SIZE) {
        u->head = 0;
    }
}

void serverutils_init(serverQ_t* u)
{
  u->head = 0;
  u->tail = 0;
  memset(u->buffer, 0, sizeof(u->buffer));
}

#endif /* INC_SERVER_UTILS_H_ */
