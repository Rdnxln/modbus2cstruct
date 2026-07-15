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

static uint16_t modbus_HR_registers[65536];
static uint16_t modbus_IR_registers[65536];

/* Максимальное количество регистров в карте: 65536 (0x10000 )
   4.4      https://modbus.org/file/secure/modbusprotocolspecification.pdf

   Максимальное количество регистров в запросе: 125
   6.3, 6.4 https://modbus.org/file/secure/modbusprotocolspecification.pdf

   Максимальное число запросов для всей карты:  525 ( = 65536 / 125 )
   (без повторного опроса) */
static modbus_request_t HR_requests[525];
static int count_HR_rq = 0;

static modbus_request_t IR_requests[525];
static int count_IR_rq = 0;
/* TODO:
modbus_request_t CL_requests[ 525 ];
modbus_request_t DI_requests[ 525 ]; */

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
void print_vars(const expr_var_t *vars, size_t var_count) {
  for (size_t i = 0; i < var_count; i++) {
    if (vars[i].val.is_float)
      printf("Var (вещ.) %s: %g\n", vars[i].name, vars[i].val.f);
    else
      printf("Var (цел.) %s: %" PRId64 "\n", vars[i].name, vars[i].val.i);
  }
}

/* удаляем начальные и завершающие пробелы */
void str_trim_spaces(char *str)
{  int i = 0, /* итератор */
       s = 0; /* начало строки (за пробелами) */

  if (str == NULL) return;

  while(str[i] == ' ') /* пропускаем стартовые пробелы (если есть) */
    i++;

  s = i; /* сохраняем индекс начала строки без учета стартовых пробелов */
  do {
    str[i - s] = str[i]; /* сдвигаем всю строку к началу, замещая стартовые пробелы */
  } while(str[i++] != '\0');

  i = i - s - 2;
  while(str[i] == ' ' || str[i] == '\n') {
    str[i] = '\0';
    i--;
  }
}


/* Вычисляемое выражение */
/* Тип результата для выражения: */
typedef enum {
  RULE_FIELD, /* это результат для целевой структы */
  RULE_VAR    /* это результат для внутренней переменной */
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
                        expr_val_t val)
{
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
    write_bitfield(struct_ptr, fm, val);
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
        AppStruct.Поле (в разработке)
 */
static int load_config(const char *filename, modbus_t **p_ctx) {

  int line_count = 0;

  if(!p_ctx) return 0;
  if(*p_ctx != NULL ) return 0; /* уже контекст занят или не инициализирован */

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
    /* ищем символ присваивания, он разделяет поле (field) и выражение (expr_str).
       если нет '=', то это или коментарий, или строка подключения,
       или ошибочная строка, или пустая строка */
    if (!eq)
    {
      /* если нет символа '=' это или комментарий, или строка подключения */
      if (strstr(line, "MODBUS_TCP:"))
      {
        if (*p_ctx!=NULL) {
          fprintf(stderr, "Игнорируется повторная строка подключения: %s\n", line);
          continue;
        }
        /*MODBUS_TCP:127.0.0.1:1502:0*/
        /*MODBUS_TCP:IP-addr:IP-port:AddressID*/
        char proto [512],
             ipaddr[512],
             port  [512],
             addr  [512];
        int count_param = sscanf(line, "%[^:]:%[^:]:%[^:]:%[^:]", proto, ipaddr, port, addr);
        if (count_param == 4) {
          str_trim_spaces(proto);
          if (!strcmp(proto, "MODBUS_TCP")) {
            str_trim_spaces(ipaddr);
            str_trim_spaces(port);
            str_trim_spaces(addr);

            int p = atoi(port);
            if (p < 1 || p > 65535) {
              fprintf(stderr, "Указаный TCP-порт '%s' для modbus-подключения вне допустимого диапазона\n", port);
              continue;
            }
            int unit_id = atoi(addr);
            if (unit_id < 0 || unit_id > 247) {
              fprintf(stderr, "Указаный адрес (UnitID) '%s' для подчиненного устройства вне допустимого диапазона\n", addr);
              continue;
            }
            if(!strlen(ipaddr)) {
              fprintf( stderr, "Не указан IP-адрес или имя хоста для подключения к modbus-серверу\n");
              continue;
            }

            *p_ctx = modbus_new_tcp(ipaddr, p);
            if (*p_ctx == NULL) {
              perror("Невозможно создать modbus-контекст!");
              fprintf(stderr, "Не удалось подключиться! Опция для подключения: %s\n", line);
            }
            else {
              /* если подключились - выставляем адрес подчиненного устройства для опроса */
              if (unit_id)
                modbus_set_slave(*p_ctx, unit_id);
            }
          }
        } else {
          fprintf( stderr, "Недостаточно параметров для подключения %s\n", line );
        }
        continue;
      }
      else if (strstr(line, "MODBUS_RTU:"))
      {
        if (*p_ctx!=NULL) {
          fprintf(stderr, "Игнорируется повторная строка подключения: %s\n", line);
          continue;
        }
        /*MODBUS_RTU:/dev/ttyUSB0:115200:8:N:1:10*/
        /*MODBUS_RTU:serial_tty_file:baud:data-bit:parity_control:stop-bit:AddressID*/
        char proto  [512],
             ttyfile[512],
             baud   [512],
             databit[512],
             cparity[512],
             stopbit[512],
             addr   [512];
        int count_param = sscanf(line, "%[^:]:%[^:]:%[^:]:%[^:]:%[^:]:%[^:]:%[^:]",
                                 proto, ttyfile, baud, databit, cparity, stopbit, addr);
        if (count_param == 7) {
          str_trim_spaces(proto);
          if (!strcmp(proto, "MODBUS_RTU")) {
            str_trim_spaces(ttyfile);
            str_trim_spaces(baud);
            str_trim_spaces(databit);
            str_trim_spaces(cparity);
            str_trim_spaces(stopbit);
            str_trim_spaces(addr);

            int b = atoi(baud);
            if (b < 300) {
              fprintf(stderr, "Указанная скорость передачи данных baudrate '%s' для modbus-подключения вне допустимого диапазона\n", baud);
              continue;
            }

            int db = atoi(databit);
            if (db < 5 || db > 9) {
              fprintf(stderr, "Указанно некорректное количество бит данных '%s'\n", databit);
              continue;
            }

            int sb = atoi(stopbit);
            if (sb != 1 && sb != 2) {
              fprintf(stderr, "Указанно некорректное количество стоповых бит '%s'\n", stopbit);
              continue;
            }

            int unit_id = atoi(addr);
            if (unit_id < 0 || unit_id > 247) {
              fprintf(stderr, "Указаный адрес (UnitID) '%s' для подчиненного устройства вне допустимого диапазона\n", addr);
              continue;
            }
            if(!strlen(ttyfile)) {
              fprintf( stderr, "Не указан файл tty-устройства для подключения к modbus-серверу\n");
              continue;
            }
            *p_ctx = modbus_new_rtu(ttyfile, b, cparity[0], db, sb);
            if (*p_ctx == NULL) {
              perror("Невозможно создать modbus-контекст!");
              fprintf(stderr, "Не удалось подключиться! Опция для подключения: %s\n", line);
            }
            else {
              /* если подключились - выставляем адрес подчиненного устройства для опроса */
              if (unit_id)
                modbus_set_slave(*p_ctx, unit_id);
            }
          }
        } else {
          fprintf( stderr, "Недостаточно параметров для подключения %s\n", line );
        }
        continue;

      }

      continue; /* переходим к следующей строке */
    }
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
      if(strlen(field)<64) {
        strncpy(rules[rule_count].var_name, field, 63);
      } else {
        fprintf(stderr,
                "Имя переменной '%s' превышает ограничение\n", field );
        expr_free(ast);
        ast = NULL;
        continue;
      }
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
                  "Ошибка в конфигурации: Превышено число переменных (пропуск '%s').\n",
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
  /* если опции соединения не были указаны - пытаемся использовать по-умолчанию */
  if (*p_ctx == NULL) {
    fprintf(stderr, "Не найдены корректные опции modbus-подключения\n");
  }

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

/* рекурсивный обход AST-дерева для поиска
   уникальных адресов Modbus-регистров с фиксацией их в массивах */
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

/* построение запросов по регистрам в правилах */
void build_requests() {

  /* Карта упомянутых в правилах регистров, для составления плана запросов */
  int count_HR_regs = 0;
  int count_IR_regs = 0;

  uint16_t *req_HR_regs = NULL;
  uint16_t *req_IR_regs = NULL;

  req_HR_regs = (uint16_t *)calloc(0x10000, sizeof(uint16_t));
  if (!req_HR_regs)
    goto build_reqs_cleanup;

  req_IR_regs = (uint16_t *)calloc(0x10000, sizeof(uint16_t));
  if (!req_IR_regs)
    goto build_reqs_cleanup;

  memset(HR_requests, 0, sizeof(HR_requests));
  memset(IR_requests, 0, sizeof(IR_requests));

  /* Поиск Modbus-регистров в правилах */
  /* Проходим по каждому правилу */
  for (int i = 0; i < rule_count; i++) {
    /* в каждом правиле рекурсивно обходим узлы */
    build_req(rules[i].ast,
              req_HR_regs, &count_HR_regs, /* массивы для хранения встречающихся регистров и их число */
              req_IR_regs, &count_IR_regs);
  }

  /* На основе массива найденых регистров строим запросы */
  count_HR_rq = optimize_modbus_requests(req_HR_regs, count_HR_regs,
                                         HR_requests, MAX_TCP_GAP);
  /* TODO: дописать переключение на MAX_RTU_GAP в зависимости от типа подключения */

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

/* имитация работы прикладной программы,
  которая обрабатывает целевую структуру с полученными данными */
void PutData(AppStruct *dev_data, time_t t) {
  printf("--- Данные приняты: %s", ctime(&t));
  print_struct(dev_data);
  print_vars(runtime_vars, runtime_var_count);
}

/* MVP - прототип
   Входной аргумент - имя конфигурационного файла с правилами и
   опциями для подключения к modbus-серверу */
int main(int argc, char *argv[]) {

  const char *cfg_file = (argc > 1) ? argv[1] : "testconfig.cfg";

//  uint16_t modbus_HR_registers[65536];
  memset(modbus_HR_registers, 0, sizeof(modbus_HR_registers));

//  uint16_t modbus_IR_registers[65536];
  memset(modbus_IR_registers, 0, sizeof(modbus_IR_registers));



  /* Тестовые данные для типа float:
     10.0f in Big Endian = 0x41200000 */
  /*modbus_HR_registers[ 0 ] = 0x4120;
    modbus_HR_registers[ 1 ] = 0x0000;

    modbus_HR_registers[ 2 ] = 100;*/

  /* Тестовые данные для типа double:
     1.2e+34 in Little Endian = 0x47027D2A59B51735 */
  /*modbus_IR_registers[ 0 ] = 0x1735;
    modbus_IR_registers[ 1 ] = 0x59B5;
    modbus_IR_registers[ 2 ] = 0x7D2A;
    modbus_IR_registers[ 3 ] = 0x4702;

    modbus_IR_registers[ 4 ] = 200;*/

  AppStruct my_device = { 0 };

  printf("--- Перед расчетом ---\n");
  print_struct(&my_device);
  printf("\n");

  do {
    modbus_t *ctx = NULL;

    printf("Загрузка конфигурации с правилами: %s\n", cfg_file);
    if (load_config(cfg_file, &ctx) != 0) {
      return 1;
    }

    if (modbus_connect(ctx) == -1) {
      fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
      free_rules();
      modbus_free(ctx);
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

      sleep(1);
    }

    free_rules();

    /* Закрытие соединения */
    modbus_close(ctx);
    modbus_free(ctx);

  }
  while(0);

  return 0;
}
