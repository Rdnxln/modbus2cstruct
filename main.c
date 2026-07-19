#include <errno.h>
#include <inttypes.h>
#include <modbus.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <threads.h>

#include "modbus_expr.h"
#include "modbus_reqs.h"

/* Целевой тип данных приложения "AppStruct",
   в который необходимо преобразовать исходные данные */
typedef struct {
  float      Value; /* значение величины */
  uint16_t   Code;  /* код единиц измерения величины */
  struct {
    uint8_t  A1: 1; /* признак 1  */
    uint8_t  A2: 1; /* признак 2   */
    uint8_t  A3: 1; /* признак 3   */
    uint8_t  A4: 4; /* признак 4   */
    uint8_t    : 1; /* резерв      */
    uint8_t  B1: 2; /* признак N-1 */
    uint8_t  B2: 6; /* признак N   */
  } state ;
  uint64_t   SourceTime;
} AppStruct ;

/* Карта полей структуры "AppStruct" для механизма поиска (mapping-соответствие) */
static const field_map_t field_map[] = {
    /* Имя поля (можно указывать несколько псевдонимов) */
    {"Value",
     /* Смещение в структуре (в байтах) */
                        offsetof(AppStruct, Value),                     /* Это все одно поле (AppStruct*).Value */
     /* Тип поля */
                                                    FTYPE_FLOAT,
     /* Номер бита, количество (ширина поля) в битах */
                                                                 0, 0},
    {"AppStruct.Value", offsetof(AppStruct, Value), FTYPE_FLOAT, 0, 0}, /* Это все одно поле (AppStruct*).Value */
    {"V",               offsetof(AppStruct, Value), FTYPE_FLOAT, 0, 0}, /* Это все одно поле (AppStruct*).Value */
    /* ^^ имена на одно и то же поле */

    {"Code", offsetof(AppStruct, Code), FTYPE_UINT16, 0, 0},
    {"SourceTime", offsetof(AppStruct, SourceTime), FTYPE_UINT64, 0, 0},

    /* Битовые поля */
    {"state.A1", offsetof(AppStruct, state), FTYPE_BITFIELD, 0,  1},
    {"state.A2", offsetof(AppStruct, state), FTYPE_BITFIELD, 1,  1},
    {"state.A3", offsetof(AppStruct, state), FTYPE_BITFIELD, 2,  1},
    {"state.A4", offsetof(AppStruct, state), FTYPE_BITFIELD, 3,  4},
    {"state.B1", offsetof(AppStruct, state), FTYPE_BITFIELD, 8,  2},
    {"state.B2", offsetof(AppStruct, state), FTYPE_BITFIELD, 10, 6},
};
#define FIELD_MAP_SIZE (sizeof(field_map) / sizeof(field_map[0]))

/* Печать полей структуры */
void print_struct(const AppStruct *d) {
  printf("Value: %g\n", d->Value);
  printf("Code: %hd\n", d->Code);

  printf("state.A1: %d\n", d->state.A1);
  printf("state.A2: %d\n", d->state.A2);
  printf("state.A3: %d\n", d->state.A3);
  printf("state.A4: %d\n", d->state.A4);

  printf("state.B1: %d\n", d->state.B1);
  printf("state.B2: %d\n", d->state.B2);

  printf("SourceTime: %" PRId64 "\n", d->SourceTime);
}

/* Печать внутренних переменных */
void print_vars(rules_t *r) {
  if(!r) return;

  for (int i = 0; i < r->runtime_var_count; i++) {
    if (r->runtime_vars[i].val.is_float)
      printf("Var (вещ.) %s: %g\n",          r->runtime_vars[i].name, r->runtime_vars[i].val.f);
    else
      printf("Var (цел.) %s: %" PRId64 "\n", r->runtime_vars[i].name, r->runtime_vars[i].val.i);
  }
}

/* Пересчет регистров в поля согласно правилам */
void update_struct(AppStruct *data,
                   rules_t *r) {
  if(!data) return;
  if(!r) return;

  /* обход правил */
  for (int i = 0; i < r->rule_count; i++) {
    /* выполняем расчет по i-му правилу */
    expr_val_t val = expr_eval(
        r->rules[i].ast, /* дерево вычисления  */
        r,

        data, /* целевая структура, составной тип, куда сохраняются данные */

        field_map, /* карта полей в целевой структуре */
        FIELD_MAP_SIZE);

    /* если цель для результата вычисления - поле структуры, то сохраняем туда
     * результат */
    if (r->rules[i].type == RULE_FIELD) {
      write_field(data, &field_map[r->rules[i].map_idx], val);
    }
    /* иначе, если цель - внутренняя переменная, то сохраняем туда результат */
    else if (r->rules[i].type == RULE_VAR) {
      /* ищем переменную */
      for (int j = 0; j < r->runtime_var_count; j++) {
        if (strcmp(r->runtime_vars[j].name, r->rules[i].var_name) == 0) {
          r->runtime_vars[j].val = val;
          break;
        }
      }
    }
  }
}

/* имитация работы прикладной программы,
  которая обрабатывает целевую структуру с полученными данными */
void PutData(AppStruct *dev_data, time_t t) {
  printf("--- Данные приняты: %s", ctime(&t));
  print_struct(dev_data);
}

/* MVP - прототип
   Входной аргумент - имя конфигурационного файла с правилами и
   опциями для подключения к modbus-серверу */
int main(int argc, char *argv[]) {

  const char *cfg_file = (argc > 1) ? argv[1] : "testconfig.cfg";

  AppStruct my_device = { 0 };

  printf("--- Перед расчетом ---\n");
  print_struct(&my_device);
  printf("\n");


  struct timespec  delay;
  delay.tv_sec  = 0;
  delay.tv_nsec = 200000000;

  int repeat_test = 5;

  do {
    modbus_t *ctx = NULL;
    rules_t *r = NULL;

    r = rules_new();
    if(!r) break;

    printf("Загрузка конфигурации с правилами: %s\n", cfg_file);
    if (load_config(cfg_file, &ctx, r, field_map, FIELD_MAP_SIZE) != 0) {
      return 1;
    }

    if (modbus_connect(ctx) == -1) {
      fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
      free_rules(r);
      r = NULL;
      modbus_free(ctx);
      ctx = NULL;
      return 1;
    }

    printf("Успешно загружено %d правил.\n\n", r->rule_count);

    int count = 5;
    while (count--) {
      /* подготовка структуры к новым данным */
      memset(&my_device, 0, sizeof(AppStruct));
      /* обновление регистров от устройства */
      if (-1 == update_registers(
                    ctx, r->HR_requests, /* массив запросов */
                         r->count_HR_rq, /* количество запросов в массиве */
                         r->modbus_HR_registers, /* регистры для запросов */
                         modbus_read_registers /* 3-я функция библиотеки libmodbus
                                             для чтения HR-регистров */
                    )) {
        goto next_iter;
      }

      if (-1 == update_registers(
                    ctx, r->IR_requests,
                         r->count_IR_rq,
                         r->modbus_IR_registers,
                         modbus_read_input_registers /* 4-я фукцция библиотеки
                                                        для IR-регистров */
                           )) {
        goto next_iter;
      }

      /* пересчет регистров в поля */
      update_struct(&my_device, r);

      /* использование структуры, для передачи данных в систему */
      PutData(&my_device, time(NULL));
      print_vars(r);

    next_iter:
      thrd_sleep(&delay, NULL);
    }

    free_rules(r);
    r = NULL;

    /* Закрытие соединения */
    modbus_close(ctx);
    modbus_free(ctx);
    ctx = NULL;
  }
  while(--repeat_test);

  return 0;
}
