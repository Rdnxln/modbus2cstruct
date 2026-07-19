#ifndef MODBUS_EXPR_H
#define MODBUS_EXPR_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <modbus.h>
#include "modbus_reqs.h"

/* Перечисление поддерживаемых операций операций */
typedef enum {
  /*  *       /       %       +       - */
  OP_MUL, OP_DIV, OP_MOD, OP_ADD, OP_SUB,
  /*  <<      >> */
  OP_SHL, OP_SHR,
  /*  &      |      ^ */
  OP_AND, OP_OR, OP_XOR,
  /*  <      >     ==     !=     <=     >= */
  OP_LT, OP_GT, OP_EQ, OP_NE, OP_LE, OP_GE,
  /* &&       || */
  OP_LAND, OP_LOR
} binop_t;

/* Перечисление унарных операций */
typedef enum {
  UNARY_NOT,  /* Логическая инверсия */
  UNARY_BNOT, /* Битовое отрицание  */
  UNARY_NEG   /* Смена знака */
} unop_t;

/* Результат вычисления выражения */
typedef struct {
  bool       is_float;
  union
  {
    int64_t  i;
    double   f;
  };
} expr_val_t;


/* Функции */
typedef enum {
  /* тригонометрия */
  FN_SIN, FN_COS, FN_TAN, FN_ASIN, FN_ACOS, FN_ATAN,
  /* корень, логарифмы,     модуль, округление... */
  FN_SQRT, FN_LN, FN_LOG10, FN_ABS, FN_FLOOR, FN_CEIL,
  /* секунды с 1970UTC */
  FN_UNIXTIME
} func_id_t;

/* Приведение типа */
typedef enum {
  CAST_NONE=0,
  /* (float)   (double) */
  CAST_FLOAT,  CAST_DOUBLE,
  /* (int8_t)  (uint8_t)    (int16_t)   (uint16_t) */
  CAST_INT8,   CAST_UINT8,  CAST_INT16, CAST_UINT16,
  /* (int32_t) (uint32_t)   (int64_t)   (uint64_t) */
  CAST_INT32,  CAST_UINT32, CAST_INT64, CAST_UINT64
} cast_type_t;

/* Внутренние переменные */
typedef struct {
  const char *name; /* имя переменной */
  expr_val_t val;   /* значение */
} expr_var_t;

/* внутренние типы соответствующие типам языка C */
typedef enum {
  FTYPE_FLOAT, FTYPE_DOUBLE, /* вещественные типы      */
  FTYPE_INT16, FTYPE_UINT16, /* целочисленные 2 байта  */
  FTYPE_INT32, FTYPE_UINT32, /* ------"------ 4 байта  */
  FTYPE_INT64, FTYPE_UINT64, /* ------"------ 8 байт   */
  FTYPE_UINT8,               /* ------"------ 1 байт   */
  FTYPE_BITFIELD             /* 1-бит или битовое поле */
} field_type_t;

/* описание поля целевой структуры, куда сохраняется результат вычисления */
typedef struct {
  const char *name;      /* имя поля */
  size_t      offset;    /* смещение относительно начала целевой структуры/составного типа */
  field_type_t type;     /* тип поля */
  uint8_t     bit_pos;   /* начало поля в битовом поле */
  uint8_t     bit_width; /* количество бит в поле */
} field_map_t;

/* Тип узла */
typedef enum {
  NODE_LIT_INT,        /* целое число */
  NODE_LIT_FLOAT,      /* вещественное число */
/* Достаточно для MVP */
  NODE_REG_HR,         /* ссылка на HR-регистр (Modbus) */
  NODE_REG_IR,         /* ссылка на IR-регистр (Modbus) */
/*  TODO: */
/*  NODE_REG_CL, */    /* ссылка на CL-регистр (Modbus) */
/*  NODE_REG_DI, */    /* ссылка на DI-регистр (Modbus) */
  NODE_VAR,            /* внутренняя переменная */
  NODE_FUNC,           /* функция */
  NODE_UNARY,          /* унарная операция */
  NODE_BINARY,         /* бинарная операция */
  NODE_TERNARY,        /* тернарная операция */
  NODE_CAST,           /* приведение типа */
  NODE_FLOAT_CONVERT,  /* байтовая интерпретация памяти в float */
  NODE_DOUBLE_CONVERT  /* байтовая интерпретация памяти в double */
} node_kind_t;

/* структура ноды - элементарной единицы синтаксиса */
typedef struct expr_node {
  node_kind_t  kind;            /* тип ноды NODE_... */
  union {
    int64_t  lit_int;           /* целое значение */
    double   lit_float;         /* вещественное значение */
    int      reg_idx;           /* индекс для регистра */
    char     var_name[64];      /* имя внутренней переменной */
    struct {                    /* функция */
      func_id_t         fn;        /* идентификатор функции */
      struct expr_node *child;     /* аргумент функции */
           } func;
    struct {                    /* унарная операция */
      unop_t            op;        /* тип унарной операции */
      struct expr_node *child;     /* аргумент для унарной операции */
           } unary;
    struct {                    /* бинарная операция */
      binop_t           op;        /* тип бинарной операции */
      struct expr_node *left,      /* левый аргумент */
                       *right;     /* правый аргумент */
           } binary;
    struct {                    /* тернарная операция */
       struct expr_node *cond,     /* выражение условия тернарной операции */
                        *yes,      /* выражение для TRUE */
                        *no;       /* выражение для FALSE */
           } ternary;
    struct {                    /* приведение типа */
       cast_type_t       ctype;    /* тип */
       struct expr_node *child;    /* приводимый аргумент */
           } cast;
    struct {                    /* реинтерпретация в float/double  */
       struct expr_node *child;    /* аргумент */
           } byteconvert;
  };
} expr_node_t;


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




/* Разбор выражений, правил - составление Abstract Syntax Tree */
expr_node_t *expr_parse ( const char        *str );

/* Вычисление выражений */
expr_val_t   expr_eval  ( const expr_node_t *node,
                          const rules_t     *r,
                          const void        *struct_ptr,
                          const field_map_t *fmap,
                          size_t             fmap_size );

/* Освобождение структур */
void         expr_free  ( expr_node_t       *node );

/* чтение битовых полей */
void         read_bitfield(  const void *struct_ptr, const field_map_t *fm, expr_val_t *result);

/* запись битовых полей */
void         write_bitfield( void       *struct_ptr, const field_map_t *fm, expr_val_t  val   );

/* Запись значения в целевой тип с использованием карты полей */
void         write_field( void *struct_ptr, const field_map_t *fm,          expr_val_t val );

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
 */
int load_config(const char *filename, modbus_t **p_ctx, rules_t *r, const field_map_t *field_map, int field_count);

#endif
