#ifndef RULES_CONF_H
#define RULES_CONF_H

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



typedef struct {
  uint16_t modbus_HR_registers[65536];
  uint16_t modbus_IR_registers[65536];

/* Максимальное количество регистров в карте: 65536 (0x10000 )
   4.4      https://modbus.org/file/secure/modbusprotocolspecification.pdf

   Максимальное количество регистров в запросе: 125
   6.3, 6.4 https://modbus.org/file/secure/modbusprotocolspecification.pdf

   Максимальное число запросов для всей карты:  525 ( = 65536 / 125 )
   (без повторного опроса) */
  modbus_request_t HR_requests[525];
  int count_HR_rq;

  modbus_request_t IR_requests[525];
  int count_IR_rq;
/* TODO:
  modbus_request_t CL_requests[ 525 ];
  modbus_request_t DI_requests[ 525 ]; */


/* Вычисляемые выражения */
  runtime_rule_t rules[256]; /* слоты для выражений (всего 256) */
  int rule_count; /* число выражений */

/* Внутренние переменные (всего 64) */
  expr_var_t runtime_vars[64];
  int runtime_var_count;
} rules_t ;

/* Получить память для хранения всех правил */
rules_t *rules_new();

/* Освободить память правил */
void free_rules(rules_t *r);

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
int load_config(const char *filename, modbus_t **p_ctx, rules_t *r);
 */
int load_config(const char *filename, modbus_t **p_ctx, rules_t *r, const field_map_t *field_map, int field_count);

#endif
