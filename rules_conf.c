#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <modbus.h>
#include "modbus_expr.h"
#include "modbus_reqs.h"
#include "rules_conf.h"

/* удаляем начальные и завершающие пробелы */
static void str_trim_spaces(char *str)
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

/* Получить память для хранения всех */
rules_t *rules_new()
{
  return (rules_t*)calloc( 1, sizeof(rules_t));
}

void free_rules(rules_t *r) {
  if(!r) return;

  for (int i = 0; i < r->rule_count; i++) {
    expr_free(r->rules[i].ast);
  }
  r->rule_count = 0;
  free(r);
}

/* рекурсивный обход AST-дерева для поиска
   уникальных адресов Modbus-регистров с фиксацией их в массивах */
static void build_req(expr_node_t *n, uint16_t *req_HR_regs, int *count_HR_regs,
               uint16_t *req_IR_regs, int *count_IR_regs)
{
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
static void build_requests(rules_t *r)
{
  if(!r) return;

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

  memset(r->HR_requests, 0, sizeof(r->HR_requests));
  memset(r->IR_requests, 0, sizeof(r->IR_requests));

  /* Поиск Modbus-регистров в правилах */
  /* Проходим по каждому правилу */
  for (int i = 0; i < r->rule_count; i++) {
    /* в каждом правиле рекурсивно обходим узлы */
    build_req(r->rules[i].ast,
              req_HR_regs, &count_HR_regs, /* массивы для хранения встречающихся регистров и их число */
              req_IR_regs, &count_IR_regs);
  }

  /* На основе массива найденых регистров строим запросы */
  r->count_HR_rq = optimize_modbus_requests(req_HR_regs, count_HR_regs,
                                            r->HR_requests, MAX_TCP_GAP);
  /* TODO: дописать переключение на MAX_RTU_GAP в зависимости от типа подключения */

  r->count_IR_rq = optimize_modbus_requests(req_IR_regs, count_IR_regs,
                                            r->IR_requests, MAX_TCP_GAP);
  /* Отладка
  for( int i = 0 ; i < count_HR_regs ; i++ )
    printf( "%d HR[%d]\n", i, req_HR_regs[ i ] );

  printf( "Для HR-регистров сформировано итого запросов: %d\n", r->count_HR_rq );
  for( int i = 0; i < r->count_HR_rq; i++ )
  {
    printf( "Запрос %d: Старт = %u, Кол-во регистров = %u\n", i + 1,
      r->HR_requests[i].start_addr, r->HR_requests[i].quantity );
  }

  for( int i = 0 ; i < count_IR_regs ; i++ )
    printf( "%d IR[%d]\n", i, req_IR_regs[ i ] );

  printf( "Для IR-регистров сформировано итого запросов: %d\n", r->count_IR_rq );
  for( int i = 0; i < r->count_IR_rq; i++ )
  {
    printf( "Запрос %d: Старт = %u, Кол-во регистров = %u\n", i + 1,
      r->IR_requests[i].start_addr, r->IR_requests[i].quantity );
  }
   */

build_reqs_cleanup:
  if (req_HR_regs)
    free(req_HR_regs);
  if (req_IR_regs)
    free(req_IR_regs);
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
int load_config(const char *filename, modbus_t **p_ctx, rules_t *r, const field_map_t *field_map, int field_count)
{
  int line_count = 0;

  if(!p_ctx) return 0;
  if(*p_ctx != NULL ) return 0; /* уже контекст занят или не инициализирован */
  if(!r) return -1;

  FILE *fp = fopen(filename, "r");
  if (!fp) {
    perror("Ошибка загрузки конфигурационного файла");
    return -1;
  }

  char line[512];

  r->rule_count = 0;
  r->runtime_var_count = 0;

  while (fgets(line, sizeof(line), fp) && r->rule_count < 256) {

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
    for (size_t i = 0; i < field_count; i++) {
      if (strcmp(field, field_map[i].name) == 0) {
        map_idx = (int)i;
        break;
      }
    }

    /* Если поле field в структуре AppStruct найдено */
    if (map_idx != -1) {
      r->rules[r->rule_count].type = RULE_FIELD;
      r->rules[r->rule_count].map_idx = map_idx;
    } else {
      /* Если не найдено, оно должно быть внутренней переменной */
      r->rules[r->rule_count].type = RULE_VAR;
      if(strlen(field)<64) {
        strncpy(r->rules[r->rule_count].var_name, field, 63);
      } else {
        fprintf(stderr,
                "Имя переменной '%s' превышает ограничение\n", field );
        expr_free(ast);
        ast = NULL;
        continue;
      }
      /* Это новая переменная ? */
      bool found = false;
      for (int i = 0; i < r->runtime_var_count; i++) {
        if (strcmp(r->runtime_vars[i].name, field) == 0) {
          found = true;
          break;
        }
      }

      /* Если имя переменной не найдено, то оно - новое */
      if (!found) {
        /* не достигнуто ограничение на число переменных */
        if (r->runtime_var_count < 64) {
          /* запоминаем имя переменной, как известное */
          r->runtime_vars[r->runtime_var_count].name = r->rules[r->rule_count].var_name;
          /*runtime_vars[ runtime_var_count ].val = val_int(0);*/
          r->runtime_vars[r->runtime_var_count].val =
              (expr_val_t){.is_float = false, .i = 0};
          r->runtime_var_count++;
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

    r->rules[r->rule_count].ast = ast;
    r->rule_count++;
  }

  fclose(fp);
  /* если опции соединения не были указаны - пытаемся использовать по-умолчанию */
  if (*p_ctx == NULL) {
    fprintf(stderr, "Не найдены корректные опции modbus-подключения\n");
  }

  build_requests(r);
  return 0;
}
