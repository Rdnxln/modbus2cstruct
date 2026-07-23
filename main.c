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
    {"Value",           offsetof(AppStruct, Value), FTYPE_FLOAT, 0, 0},
    {"AppStruct.Value", offsetof(AppStruct, Value), FTYPE_FLOAT, 0, 0},
    {"V",               offsetof(AppStruct, Value), FTYPE_FLOAT, 0, 0},
    /* ^^ это все имена на одно и то же поле */

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
void update_struct(AppStruct *data, rules_t   *r)
{
  if(!data) return;
  if(!r) return;
  /* обход правил */
  for (int i = 0; i < r->rule_count; i++) {
    /* выполняем расчет по i-му правилу */
    expr_val_t val = expr_eval( r->rules[i].ast, /* дерево вычисления  */
                                r, data, /* целевая структура/тип, для сохранения данных */
                                field_map, /* карта полей в целевой структуре */
                                FIELD_MAP_SIZE );
    if (r->rules[i].type == RULE_FIELD) { /* если цель - поле структуры */
      write_field(data, &field_map[r->rules[i].map_idx], val);
    }
    else if (r->rules[i].type == RULE_VAR) { /* если цель - внутренняя переменная ... */
      /* ищем эту переменную */
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
void PutData(AppStruct *dev_data, time_t t)
{
  printf("--- Данные приняты: %s", ctime(&t));
  print_struct(dev_data);
}

/* MVP - прототип
   Входной аргумент - имя конфигурационного файла с правилами и параметрами подключения к modbus-серверу */
int main(int argc, char *argv[]) {
  const char      *cfg_file = (argc > 1) ? argv[1] : "testconfig.cfg";
  struct timespec  delay = { .tv_sec = 0, .tv_nsec = 200000000 };
  AppStruct        my_device = { 0 };
  int repeat_full_test = 50; /* ПОЛНЫЙ ЦИКЛ ДЛЯ ТЕСТИРОВАНИЯ НА УТЕЧКИ ПАМЯТИ */
  do {
/* НАЧАЛО РАБОТЫ ПРОГРАММЫ */
    modbus_t *ctx = NULL;    rules_t  *rules = NULL;
    rules = rules_new();
    if(!rules) break;
/* 1 x ОДНОКРАТНЫЙ РАЗБОР ПРАВИЛ */
    printf("Загрузка конфигурации с правилами: %s\n", cfg_file);
    if (load_config(cfg_file, &ctx, rules, field_map, FIELD_MAP_SIZE) != 0) { return 1; }
    if (modbus_connect(ctx) == -1)  /* Подключение к источнику данных */
    { fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
      free_rules(rules); rules = NULL;   modbus_free(ctx); ctx = NULL;        return 1; }

    printf("Успешно загружено %d правил.\n\n", rules->rule_count);
/* oo x МНОГОКРАТНЫЕ ВЫЧИСЛЕНИЯ */
    int count = 500; /* число итераций алгоритма */
    while (count--) { /* -- ЦИКЛ РАССЧЕТА -- */
      memset(&my_device, 0, sizeof(AppStruct)); /* подготовка структуры к новым данным */
      if (-1 == update_registers( ctx, rules->HR_requests, rules->count_HR_rq, /* HR-регистры из modbus-источника */
           rules->modbus_HR_registers, modbus_read_registers )) { goto next_iter; }
      if (-1 == update_registers( ctx, rules->IR_requests, rules->count_IR_rq, /* IR-регистры ... */
           rules->modbus_IR_registers, modbus_read_input_registers )) { goto next_iter; }
/* ПЕРЕСЧЕТ регистров в поля */
      update_struct(&my_device, rules);
/* ОТПРАВКА ДАННЫХ В ПРИЛОЖЕНИЕ */
      PutData(&my_device, time(NULL));
      print_vars(rules);
    next_iter:
      thrd_sleep(&delay, NULL); /* пауза */
    } /* -- ЦИКЛ РАССЧЕТА -- */
    free_rules(rules); rules = NULL; /* освобождение памяти под правила */
    modbus_close(ctx); modbus_free(ctx); ctx = NULL; /* Закрытие соединения к источнику данных */
  }
  while(--repeat_full_test);
  return 0;
}
