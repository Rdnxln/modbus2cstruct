#ifndef MODBUS_REQS_H
#define MODBUS_REQS_H

#include <stdint.h>
#include <modbus.h>

#define MODBUS_MAX_REGS 125  /* Максимальное количество регистров в одном запросе
                                согласно */

/* Максимальная "дыра", которую выгодно читать (в регистрах) */
/* Настройка под Modbus TCP */
#define MAX_TCP_GAP          10
/* Настройка под Modbus RTU */
#define MAX_RTU_GAP          6

/* Структура для хранения параметров одного сформированного запроса */
typedef struct {
    uint16_t start_addr; /* начальный адрес запроса */
    uint16_t quantity;   /* количество запрашиваемых регистров подряд */
} modbus_request_t;

/**
 * @brief Оптимальная группировка задействованных адресов в запросы Modbus
 * @param regs           Исходный отсортированный массив адресов
 * @param regs_count     Количество элементов в массиве regs
 * @param out_requests   Выходной массив для структуры запросов
 * @return int           Количество сформированных запросов
 */
int optimize_modbus_requests( uint16_t         *regs,
                              uint16_t          regs_count,
                              modbus_request_t *out_requests,
                              uint16_t          gap_in_regs );
/*
typedef int (*ReadRegistersFunc) (modbus_t *, int, int, uint16_t);
 */
int update_registers( modbus_t          *ctx
                    , modbus_request_t  *rq
                    , int                rq_count
                    , uint16_t          *regs
/*                  , ReadRegistersFunc *func */
                    , int              (*func)( modbus_t*, int, int, uint16_t* )
                    );


#endif
