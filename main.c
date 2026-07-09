#include "modbus_expr.h"
#include "modbus_reqs.h"
#include <errno.h>
#include <inttypes.h>
#include <modbus.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* Максимальное количество регистров в карте: 65536 (0x10000 )
   4.4      https://modbus.org/file/secure/modbusprotocolspecification.pdf

   Максимальное количество регистров в запросе: 125
   6.3, 6.4 https://modbus.org/file/secure/modbusprotocolspecification.pdf

   Максимальное число запросов для всей карты:  525 ( = 65536 / 125 )
   (без повторного опроса) */
modbus_request_t HR_requests[525];
int count_HR_rq = 0;

modbus_request_t IR_requests[525];
int count_IR_rq = 0;
/* TODO:
modbus_request_t CL_requests[ 525 ];
modbus_request_t DI_requests[ 525 ]; */

/* Целевой тип данных приложения "AppStruct",
   в который необходимо преобразовать исходные данные */
typedef struct {
  float         lo_value;
  float         hi_value;
  double        dp_value;
  long long     big_param;
  short         parameter2;
  int           parameter3;
  time_t        now;
  unsigned char mode;
  struct __attribute__((packed)) {
    unsigned char A1 : 1;
    unsigned char A2 : 1;
    unsigned char A3 : 1;
    unsigned char A4 : 1;
    unsigned char A5 : 1;
    unsigned char A6 : 1;
    unsigned char A7 : 1;
    unsigned char A8 : 1;

    unsigned char B1 : 3;
    unsigned char B2 : 4;
    unsigned char B3 : 1;
  } state;
} AppStruct;

/* Карта полей структуры "AppStruct" для механизма поиска (mapping-соответствие) */
static const field_map_t field_map[] = {
    /* Имя поля (можно указывать альтернативные имена-синонимы) */
    {"lo_value",
     /* Смещение в структуре (в байтах) */
     offsetof(AppStruct, lo_value),
     /* Тип поля */
     FTYPE_FLOAT,
     /* Номер бита, количество (ширина поля) в битах */
     0, 0},
    {"AppStruct.lo_value", offsetof(AppStruct, lo_value), FTYPE_FLOAT, 0, 0},
    {"synonym", offsetof(AppStruct, lo_value), FTYPE_FLOAT, 0, 0},
    /* ^^ имена на одно и то же поле */

    {"hi_value", offsetof(AppStruct, hi_value), FTYPE_FLOAT, 0, 0},
    {"dp_value", offsetof(AppStruct, dp_value), FTYPE_DOUBLE, 0, 0},
    {"big_param", offsetof(AppStruct, big_param), FTYPE_INT64, 0, 0},
    {"parameter2", offsetof(AppStruct, parameter2), FTYPE_INT16, 0, 0},
    {"parameter3", offsetof(AppStruct, parameter3), FTYPE_INT32, 0, 0},
    {"now", offsetof(AppStruct, now), FTYPE_INT64, 0, 0},
    {"mode", offsetof(AppStruct, mode), FTYPE_UINT8, 0, 0},

    /* Битовые поля */
    {"state.A1", offsetof(AppStruct, state), FTYPE_BITFIELD, 0, 1},
    {"state.A2", offsetof(AppStruct, state), FTYPE_BITFIELD, 1, 1},
    {"state.A3", offsetof(AppStruct, state), FTYPE_BITFIELD, 2, 1},
    {"state.A4", offsetof(AppStruct, state), FTYPE_BITFIELD, 3, 1},
    {"state.A5", offsetof(AppStruct, state), FTYPE_BITFIELD, 4, 1},
    {"state.A6", offsetof(AppStruct, state), FTYPE_BITFIELD, 5, 1},
    {"state.A7", offsetof(AppStruct, state), FTYPE_BITFIELD, 6, 1},
    {"state.A8", offsetof(AppStruct, state), FTYPE_BITFIELD, 7, 1},

    {"state.B1", offsetof(AppStruct, state), FTYPE_BITFIELD, 8, 3},
    {"state.B2", offsetof(AppStruct, state), FTYPE_BITFIELD, 11, 4},
    {"state.B3", offsetof(AppStruct, state), FTYPE_BITFIELD, 15, 1},
};
#define FIELD_MAP_SIZE (sizeof(field_map) / sizeof(field_map[0]))

/* Вычисляемое выражение */
/* ТИП РЕЗУЛЬТАТА для выражения */
typedef enum {
  RULE_FIELD, /* это результат для целевой структы */
  RULE_VAR /* это результат для внутренней переменной */
} rule_type_t;

typedef struct {
  rule_type_t type; /* тип результата */
  union {
    int map_idx; /* порядковый номер поля в целевой структуре (и в таблице
                    field_map)*/
    char var_name[64]; /* имя внутренней переменной */
  };
  expr_node_t *ast; /* дерево вычисления */
} runtime_rule_t;

/* Вычисляемые выражения */
static runtime_rule_t rules[256]; /* слоты для выражений (всего 256) */
static int rule_count = 0; /* число выражений */

/* Внутренние переменные (всего 64) */
static expr_var_t runtime_vars[64];
static int runtime_var_count = 0;

/* Запись значения в целевой тип с использованием карты полей */
static void write_field(void *struct_ptr, const field_map_t *fm,
                        expr_val_t val) {
  uint8_t *bytes = (uint8_t *)struct_ptr;

  if (fm->type == FTYPE_FLOAT) {
    float f = val.is_float ? val.f : (float)val.i;
    memcpy(bytes + fm->offset, &f, sizeof(float));
  } else if (fm->type == FTYPE_DOUBLE) {
    double f = val.is_float ? val.f : (double)val.i;
    memcpy(bytes + fm->offset, &f, sizeof(double));
  } else if (fm->type == FTYPE_INT16) {
    int16_t v = (int16_t)val.i;
    memcpy(bytes + fm->offset, &v, sizeof(int16_t));
  } else if (fm->type == FTYPE_UINT16) {
    uint16_t v = (uint16_t)val.i;
    memcpy(bytes + fm->offset, &v, sizeof(uint16_t));
  } else if (fm->type == FTYPE_INT32) {
    int32_t v = (int32_t)val.i;
    memcpy(bytes + fm->offset, &v, sizeof(int32_t));
  } else if (fm->type == FTYPE_UINT32) {
    uint32_t v = (uint32_t)val.i;
    memcpy(bytes + fm->offset, &v, sizeof(uint32_t));
  } else if (fm->type == FTYPE_INT64) {
    int64_t v = val.i;
    memcpy(bytes + fm->offset, &v, sizeof(int64_t));
  } else if (fm->type == FTYPE_UINT64) {
    uint64_t v = (uint64_t)val.i;
    memcpy(bytes + fm->offset, &v, sizeof(uint64_t));
  } else if (fm->type == FTYPE_UINT8) {
    uint8_t v = (uint8_t)val.i;
    memcpy(bytes + fm->offset, &v, sizeof(uint8_t));
  } else if (fm->type == FTYPE_BITFIELD) {
    uint32_t raw;
    memcpy(&raw, bytes + fm->offset, sizeof(uint32_t));
    uint32_t mask = ((1u << fm->bit_width) - 1) << fm->bit_pos;
    uint32_t new_bits = ((uint32_t)val.i & ((1u << fm->bit_width) - 1))
                        << fm->bit_pos;
    raw = (raw & ~mask) | new_bits;
    memcpy(bytes + fm->offset, &raw, sizeof(uint32_t));
  }
}

/* Загрузка правил из конфигурационного файла
  Ограничения
    - Максимальный размер строки: 511 символов.
    - Число правил: 256 строк
    - Длина имени переменной,
      поля структуры данных,
      строки числа: 63 символа
    - Число внутренних переменных: 64
    - В нотации используются только символы латиницы
  Формат: Цель = Выражение
    - Цель: AppStruct.Поле или Переменная
    - Выражение:
        Аргумент
        Аргумент БинарнаяОперация Аргумент
        УнарнаяОперация Аргумент
        Функция( Аргумент )
    - Аргумент:
        Число
        Modbus-регистр (HR[20])
        Переменная
        ( Выражение )
 */
static int load_config(const char *filename) {
  int line_count = 0;

  FILE *fp = fopen(filename, "r");
  if (!fp) {
    perror("Ошибка загрузки конфигурационного файла");
    return -1;
  }

  char line[512];

  rule_count = 0;
  runtime_var_count = 0;

  while (fgets(line, sizeof(line), fp) && rule_count < 256) {

    line_count++;

    /* ищем начало комментария */
    char *cm = strchr(line, '#');
    if (cm)
      *cm = '\0'; /* начало комментария - это конец разбираемой строки */

    char *eq = strchr(line, '=');
    /* ищем символ присваивания, он разделяет поле (field) и выражение
     * (expr_str)*/
    if (!eq)
      continue; /* переходим к следующей строке */

    *eq = '\0';
    char *field = line;
    char *expr_str = eq + 1;

    /* Убираем незначащие символы вокруг цели (field)...
       начальные пробелы и табуляции пропускаем */
    while (*field == ' ' || *field == '\t')
      field++;

    char *end_field = field + strlen(field) - 1;

    /* подрезаем хвостовые пробелы, табуляции, символы новой строки и возврата
     * каретки */
    while (end_field > field && (*end_field == ' ' || *end_field == '\t' ||
                                 *end_field == '\n' || *end_field == '\r')) {
      *end_field-- = '\0';
    }

    /* Тоже самое - "вокруг" выражения (expr_str)... */
    while (*expr_str == ' ' || *expr_str == '\t')
      expr_str++;

    char *end_expr = expr_str + strlen(expr_str) - 1;

    while (end_expr > expr_str && (*end_expr == ' ' || *end_expr == '\t' ||
                                   *end_expr == '\n' || *end_expr == '\r')) {
      *end_expr-- = '\0';
    }

    /* если что-то не задано, переходим к следующей строке */
    if (!*field || !*expr_str)
      continue;

    expr_node_t *ast = expr_parse(expr_str);
    if (!ast) {
      fprintf(stderr,
              "Ошибка в конфигурации (строка: %d): Ошибка синтаксиса в "
              "выражении '%s'\n",
              line_count, expr_str);
      continue;
    }

    /* Пытаемся найти поле field в структуре AppStruct по field_map */
    int map_idx = -1;
    for (size_t i = 0; i < FIELD_MAP_SIZE; i++) {
      if (strcmp(field, field_map[i].name) == 0) {
        map_idx = (int)i;
        break;
      }
    }

    /* Если поле field в структуре AppStruct найдено */
    if (map_idx != -1) {
      rules[rule_count].type = RULE_FIELD;
      rules[rule_count].map_idx = map_idx;
    } else {
      /* Если не найдено, оно должно быть внутренней переменной */
      rules[rule_count].type = RULE_VAR;
      strncpy(rules[rule_count].var_name, field, 63);
      rules[rule_count].var_name[63] = '\0';

      /* Это новая переменная ? */
      bool found = false;
      for (int i = 0; i < runtime_var_count; i++) {
        if (strcmp(runtime_vars[i].name, field) == 0) {
          found = true;
          break;
        }
      }

      /* Если имя переменной не найдено, то оно - новое */
      if (!found) {
        /* не достигнуто ограничение на число переменных */
        if (runtime_var_count < 64) {
          /* запоминаем имя переменной, как известное */
          runtime_vars[runtime_var_count].name = rules[rule_count].var_name;
          /*runtime_vars[ runtime_var_count ].val = val_int(0);*/
          runtime_vars[runtime_var_count].val =
              (expr_val_t){.is_float = false, .i = 0};
          runtime_var_count++;
        } else {
          fprintf(stderr,
                  "Ошибка в конфигурации: Превышено число переменных (пропуск "
                  "'%s').\n",
                  field);
          expr_free(ast);
          ast = NULL;
          continue;
        }
      }
    }

    rules[rule_count].ast = ast;
    rule_count++;
  }

  fclose(fp);
  return 0;
}

static void free_rules() {
  for (int i = 0; i < rule_count; i++) {
    expr_free(rules[i].ast);
  }
  rule_count = 0;
}

/* Пересчет регистров в поля согласно правилам */
void update_struct(AppStruct *data, const uint16_t *HR_regs,
                   size_t HR_reg_count, const uint16_t *IR_regs,
                   size_t IR_reg_count) {
  /* обход правил */
  for (int i = 0; i < rule_count; i++) {
    /* выполняем расчет по i-му правилу */
    expr_val_t val = expr_eval(
        rules[i].ast, /* дерево вычисления  */

        HR_regs, HR_reg_count, /* исходные данные */
        IR_regs, IR_reg_count, /* исходные данные */

        runtime_vars, /* внутренние переменные */
        runtime_var_count,

        data, /* целевая структура, составной тип, куда сохраняются данные */

        field_map, /* карта полей в целевой структуре */
        FIELD_MAP_SIZE);

    /* если цель для результата вычисления - поле структуры, то сохраняем туда
     * результат */
    if (rules[i].type == RULE_FIELD) {
      write_field(data, &field_map[rules[i].map_idx], val);
    }
    /* иначе, если цель - внутренняя переменная, то сохраняем туда результат */
    else if (rules[i].type == RULE_VAR) {
      /* ищем переменную */
      for (int j = 0; j < runtime_var_count; j++) {
        if (strcmp(runtime_vars[j].name, rules[i].var_name) == 0) {
          runtime_vars[j].val = val;
          break;
        }
      }
    }
  }
}

/* Отладка */
void print_struct(const AppStruct *d) {
  printf("lo_value: %g\n", d->lo_value);
  printf("hi_value: %g\n", d->hi_value);
  printf("dp_value: %g\n", d->dp_value);
  printf("big_param: %lli\n", d->big_param);
  printf("parameter2: %" PRId16 "\n", d->parameter2);
  printf("parameter3: %d\n", d->parameter3);
  printf("now: %" PRId64 "\n", d->now);
  printf("mode: %d\n", (int)d->mode);

  printf("state.A1: %d\n", d->state.A1);
  printf("state.A2: %d\n", d->state.A2);
  printf("state.A3: %d\n", d->state.A3);
  printf("state.A4: %d\n", d->state.A4);
  printf("state.A5: %d\n", d->state.A5);
  printf("state.A6: %d\n", d->state.A6);
  printf("state.A7: %d\n", d->state.A7);
  printf("state.A8: %d\n", d->state.A8);

  printf("state.B1: %d\n", d->state.B1);
  printf("state.B2: %d\n", d->state.B2);
  printf("state.B3: %d\n", d->state.B3);
}

void print_vars(const expr_var_t *vars, size_t var_count) {
  for (size_t i = 0; i < var_count; i++) {
    if (vars[i].val.is_float)
      printf("Var (вещ.) %s: %g\n", vars[i].name, vars[i].val.f);
    else
      printf("Var (цел.) %s: %" PRId64 "\n", vars[i].name, vars[i].val.i);
  }
}

void build_req(expr_node_t *n, uint16_t *req_HR_regs, int *count_HR_regs,
               uint16_t *req_IR_regs, int *count_IR_regs) {
  if (!n)
    return;

  if (!count_HR_regs || !count_IR_regs || !req_HR_regs || !req_IR_regs)
    return;

  int cnt_HR_regs = *count_HR_regs;
  int cnt_IR_regs = *count_IR_regs;

  switch (n->kind) {

  case NODE_REG_HR: {
    if (n->reg_idx > 65535)
      break;
    char found = 0;
    for (int reg = 0; reg < cnt_HR_regs; reg++) {
      if (req_HR_regs[reg] == (uint16_t)n->reg_idx) {
        found = 1;
        break;
      }
    }
    if (found == 1)
      break;
    req_HR_regs[cnt_HR_regs++] = n->reg_idx;
    *count_HR_regs = cnt_HR_regs;
  } break;
  case NODE_REG_IR: {
    if (n->reg_idx > 65535)
      break;
    char found = 0;
    for (int reg = 0; reg < cnt_IR_regs; reg++) {
      if (req_IR_regs[reg] == (uint16_t)n->reg_idx) {
        found = 1;
        break;
      }
    }
    if (found == 1)
      break;
    req_IR_regs[cnt_IR_regs++] = n->reg_idx;
    *count_IR_regs = cnt_IR_regs;
  } break;
  case NODE_UNARY:
    build_req(n->unary.child, req_HR_regs, count_HR_regs, req_IR_regs,
              count_IR_regs);
    break;

  case NODE_BINARY:
    build_req(n->binary.left, req_HR_regs, count_HR_regs, req_IR_regs,
              count_IR_regs);
    build_req(n->binary.right, req_HR_regs, count_HR_regs, req_IR_regs,
              count_IR_regs);
    break;

  case NODE_TERNARY:
    build_req(n->ternary.cond, req_HR_regs, count_HR_regs, req_IR_regs,
              count_IR_regs);
    build_req(n->ternary.yes, req_HR_regs, count_HR_regs, req_IR_regs,
              count_IR_regs);
    build_req(n->ternary.no, req_HR_regs, count_HR_regs, req_IR_regs,
              count_IR_regs);
    break;

  case NODE_CAST:
    build_req(n->cast.child, req_HR_regs, count_HR_regs, req_IR_regs,
              count_IR_regs);
    break;

  case NODE_FLOAT_CONVERT:
    build_req(n->byteconvert.child, req_HR_regs, count_HR_regs, req_IR_regs,
              count_IR_regs);
    break;

  case NODE_DOUBLE_CONVERT:
    build_req(n->byteconvert.child, req_HR_regs, count_HR_regs, req_IR_regs,
              count_IR_regs);
    break;

  case NODE_FUNC:
    build_req(n->func.child, req_HR_regs, count_HR_regs, req_IR_regs,
              count_IR_regs);
    break;

  default:
    break;
  }

  return;
}

/*
void build_requests(  modbus_request_t *HR_rq,
                      int              *count_HR_rq,
                      modbus_request_t *IR_rq,
                      int              *count_IR_rq )
 */
void build_requests() {
  /* Карта упомянутых в правилах регистров, для составления плана запросов */
  uint16_t *req_HR_regs = NULL;
  uint16_t *req_IR_regs = NULL;

  int count_HR_regs = 0;
  int count_IR_regs = 0;

  req_HR_regs = (uint16_t *)calloc(0x10000, sizeof(uint16_t));
  if (!req_HR_regs)
    goto build_reqs_cleanup;

  req_IR_regs = (uint16_t *)calloc(0x10000, sizeof(uint16_t));
  if (!req_IR_regs)
    goto build_reqs_cleanup;

  memset(HR_requests, 0, sizeof(HR_requests));
  memset(IR_requests, 0, sizeof(IR_requests));

  /* Обойдем все правила в поисках выявленных регистров */
  for (int i = 0; i < rule_count; i++) {
    build_req(rules[i].ast, req_HR_regs, &count_HR_regs, req_IR_regs,
              &count_IR_regs);
  }

  count_HR_rq = optimize_modbus_requests(req_HR_regs, count_HR_regs,
                                         HR_requests, MAX_TCP_GAP);

  count_IR_rq = optimize_modbus_requests(req_IR_regs, count_IR_regs,
                                         IR_requests, MAX_TCP_GAP);
  /* Отладка
    for( int i = 0 ; i < count_HR_regs ; i++ ) printf( "%d HR[%d]\n", i,
    req_HR_regs[ i ] ); printf( "Для HR-регистров сформировано итого запросов:
    %d\n", count_HR_rq ); for( int i = 0; i < count_HR_rq; i++ ) { printf( "
    Запрос %d: Старт = %u, Кол-во регистров = %u\n", i + 1,
    HR_requests[i].start_addr, HR_requests[i].quantity );
    }

    for( int i = 0 ; i < count_IR_regs ; i++ ) printf( "%d IR[%d]\n", i,
    req_IR_regs[ i ] ); printf( "Для IR-регистров сформировано итого запросов:
    %d\n", count_IR_rq ); for( int i = 0; i < count_IR_rq; i++ ) { printf( "
    Запрос %d: Старт = %u, Кол-во регистров = %u\n", i + 1,
    IR_requests[i].start_addr, IR_requests[i].quantity );
    }
   */

build_reqs_cleanup:
  if (req_HR_regs)
    free(req_HR_regs);
  if (req_IR_regs)
    free(req_IR_regs);
}

void PutData(AppStruct *dev_data, time_t t) {
  printf("--- Данные приняты: %s", ctime(&t));
  print_struct(dev_data);
  print_vars(runtime_vars, runtime_var_count);
}

/*============================================================================
 * Прототип
 * Входной аргумент - имя конфигурационного файла с правилами
 *============================================================================*/
int main(int argc, char *argv[]) {

  const char *cfg_file = (argc > 1) ? argv[1] : "testconfig.cfg";

  uint16_t modbus_HR_registers[65536];
  memset(modbus_HR_registers, 0, sizeof(modbus_HR_registers));

  uint16_t modbus_IR_registers[65536];
  memset(modbus_IR_registers, 0, sizeof(modbus_IR_registers));

  modbus_t *ctx;

  /*
    ctx = modbus_new_tcp( "192.168.222.244", 1502 );
   */
  ctx = modbus_new_tcp("45.8.248.56", 502);
  modbus_set_slave(ctx, 10);

  if (modbus_connect(ctx) == -1) {
    fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
    modbus_free(ctx);
    return -1;
  }

  /* Тестовые данные: 10.0f in Big Endian = 0x41200000 */
  /*modbus_HR_registers[ 0 ] = 0x4120;
    modbus_HR_registers[ 1 ] = 0x0000;

    modbus_HR_registers[ 2 ] = 100;*/

  /* Тестовые данные: 1.2e+34 in Little Endian = 0x47027D2A59B51735 */
  /*modbus_IR_registers[ 0 ] = 0x1735;
    modbus_IR_registers[ 1 ] = 0x59B5;
    modbus_IR_registers[ 2 ] = 0x7D2A;
    modbus_IR_registers[ 3 ] = 0x4702;

    modbus_IR_registers[ 4 ] = 200;*/

  AppStruct my_device = {0};

  printf("--- Перед расчетом ---\n");
  print_struct(&my_device);
  printf("\n");

  int extc = 1;
  while (extc--) {

    printf("Загрузка конфигурации с правилами: %s\n", cfg_file);
    if (load_config(cfg_file) != 0) {
      return 1;
    }

    printf("Успешно загружено %d правил.\n\n", rule_count);

    build_requests();

    int count = 1000;
    while (count--) {
      /* подготовка структуры к новым данным */
      memset(&my_device, 0, sizeof(AppStruct));
      /* обновление регистров от устройства */
      if (-1 == update_registers(
                    ctx, HR_requests, /* массив запросов */
                    count_HR_rq, /* количество запросов в массиве */
                    modbus_HR_registers, /* регистры для запросов */
                    modbus_read_registers /* 3-я функция библиотеки libmodbus
                                             для чтения HR-регистров */
                    )) {
        goto next_iter;
      }

      if (-1 ==
          update_registers(ctx, IR_requests, count_IR_rq, modbus_IR_registers,
                           modbus_read_input_registers /* 4-я фукцция библиотеки
                                                          для IR-регистров */
                           )) {
        goto next_iter;
      }

      /* пересчет регистров в поля */
      update_struct(&my_device, modbus_HR_registers,
                    sizeof(modbus_HR_registers) / sizeof(uint16_t),
                    modbus_IR_registers,
                    sizeof(modbus_IR_registers) / sizeof(uint16_t));

      /* использование структуры, для передачи данных в систему */
      PutData(&my_device, time(NULL));
    next_iter:
      /* TODO: Можно сделать "адаптивную" паузу,
               учитывающую время запроса и обработки данных  */
      sleep(1);
    }

    free_rules();
  }

  /* Закрытие соединения */
  modbus_close(ctx);
  modbus_free(ctx);

  return 0;
}
