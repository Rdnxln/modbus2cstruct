#include "modbus_reqs.h"
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

int compare_regs( const void *i1, const void *i2 )
{
  uint16_t reg1 = *( const uint16_t *)i1;
  uint16_t reg2 = *( const uint16_t *)i2;
  /* return ( (int)reg1 - (int)reg2 ); */
  if( reg1 < reg2 ) return -1;
  if( reg1 > reg2 ) return  1;

  return 0;
}

/* Оптимальная группировка задействованных адресов в запросы Modbus */
int optimize_modbus_requests( uint16_t         *regs,         /* Исходный отсортированный массив адресов */
                              uint16_t          regs_count,   /* Количество элементов в массиве regs */
                              modbus_request_t *out_requests, /* Выходной массив для структуры запросов */
                              uint16_t          gap_in_regs ) /* Интервал-"дыра" из ненужных регистров между нужными  */
{
  uint16_t  gap_in_regs_opt = gap_in_regs;
  if( regs_count == 0 ) return 0;

  qsort( regs, regs_count, sizeof(uint16_t), compare_regs );

  if( gap_in_regs < MAX_RTU_GAP ) gap_in_regs_opt = MAX_RTU_GAP;
  if( gap_in_regs > MAX_TCP_GAP ) gap_in_regs_opt = MAX_TCP_GAP;

  int       req_idx    = 0;
  /* Инициализируем первый запрос первым элементом */
  uint16_t  start_addr = regs[ 0 ];
  uint16_t  last_addr  = regs[ 0 ];

  for( uint16_t i = 1; i < regs_count; i++ ) {
    uint16_t  next_addr = regs[ i ];
    /* Расстояние (пропуск) между текущим и предыдущим регистром */
    uint16_t  gap = next_addr - last_addr - 1;
    /* Потенциальный размер нового гипотетического запроса */
    uint32_t  potential_len = next_addr - start_addr + 1;
    /* Условие разделения: если дыра слишком большая ИЛИ превышен лимит Modbus на один запрос */
    if( gap > gap_in_regs || potential_len > MODBUS_MAX_REGS ) {
      /* Сохраняем текущий сформированный запрос */
      out_requests[ req_idx ].start_addr = start_addr;
      out_requests[ req_idx ].quantity   = last_addr - start_addr + 1;
      req_idx++;
      /* Начинаем новый запрос с текущего регистра */
      start_addr = next_addr;
    }
    /* Обновляем последний обработанный адрес */
    last_addr = next_addr;
    /* Останавливаем анализ, т.к. у нас максимальное число запросов 525 (от 0 до 524) */
    if( req_idx == 524 ) {
      fprintf( stderr, "Достигнуто ограничение по числу запросов, формирование прервано\n" );
      break;
    }
  }
  /* Не забываем сохранить самый последний запрос */
  out_requests[ req_idx ].start_addr = start_addr;
  out_requests[ req_idx ].quantity   = last_addr - start_addr + 1;
  req_idx++;

  return req_idx;
}

/*
 * Считывание регистров от ведомого устройства
 * Результат:
 *  0 - безошибочное чтение
 * -1 - была ошибка чтения
 */
int update_registers
  ( modbus_t          *ctx       /* Контекст modbus */
  , modbus_request_t  *rq        /* Массив запросов */
  , int                rq_count  /* Число запросов в массиве */
  , uint16_t          *regs      /* Массив регистров для сохранения данных */
  , int              (*func)( modbus_t*, int, int, uint16_t* ) /* функция чтения */
  )
{
  int rc;
  if( !ctx || !regs || !rq || !func ) return -1;

  for( int i = 0; i < rq_count; i++ ) {
    rc = func( ctx ,
              (int)rq[ i ].start_addr , /* стартовый адрес регистра */
              (int)rq[ i ].quantity ,   /* количество регистров */
              &regs[ (int)rq[ i ].start_addr ] );
    if( rc == -1 ) {
      fprintf( stderr, "%s\n", modbus_strerror( errno ) );
      return -1;
    }
  }
  return 0;
}
