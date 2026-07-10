#ifndef MODBUS_EXPR_H
#define MODBUS_EXPR_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>

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

typedef struct {
    bool       is_float;
    union
    {
      int64_t  i;
      double   f;
    };
} expr_val_t;

typedef enum {
    NODE_LIT_INT,        /* целое число */
    NODE_LIT_FLOAT,      /* вещественное число */
/* Достаточно для MVP */
    NODE_REG_HR,         /* ссылка на HR-регистр (Modbus) */
    NODE_REG_IR,         /* ссылка на IR-регистр (Modbus) */

/*  TODO: */
/*  NODE_REG_CL, */      /* ссылка на CL-регистр (Modbus) */
/*  NODE_REG_DI, */      /* ссылка на DI-регистр (Modbus) */

    NODE_VAR,            /* внутренняя переменная */
    NODE_FUNC,           /* функция */
    NODE_UNARY,          /* унарная операция */
    NODE_BINARY,         /* */
    NODE_TERNARY,        /* тернарная операция */
    NODE_CAST,           /* приведение типа */
    NODE_FLOAT_CONVERT,  /* байтовые интерпретации */
    NODE_DOUBLE_CONVERT
} node_kind_t;

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

/* структура ноды - элементарной единицы синтаксиса */
typedef struct expr_node {
    node_kind_t    kind;  /* тип ноды NODE_... */
    union {
        int64_t    lit_int;
        double     lit_float;
        int        reg_idx;
        char       var_name[64];

        struct {
           func_id_t         fn;
           struct expr_node *child;
               }   func;

        struct {
           unop_t            op;
           struct expr_node *child;
               }   unary;

        struct {
           binop_t           op;
           struct expr_node *left,
                            *right;
               }   binary;
        struct {
           struct expr_node *cond,
                            *yes,
                            *no;
               } ternary;
        struct {
           cast_type_t       ctype;
           struct expr_node *child;
               } cast;
        struct {
           struct expr_node *child;
               } byteconvert;
    };
} expr_node_t;

/* Разбор выражений, правил - составление Abstract Syntax Tree */
expr_node_t *expr_parse ( const char        *str );

/* Вычисление выражений */
expr_val_t   expr_eval  ( const expr_node_t *node,
                          const uint16_t    *HR_regs,
                          size_t             HR_reg_count,
                          const uint16_t    *IR_regs,
                          size_t             IR_reg_count,
                          const expr_var_t  *vars,
                          size_t             var_count,
                          const void        *struct_ptr,
                          const field_map_t *fmap,
                          size_t             fmap_size );

/* Освобождение структур */
void         expr_free  ( expr_node_t       *node );

/* чтение битовых полей */
void         read_bitfield(  const void *struct_ptr, const field_map_t *fm, expr_val_t *result);

/* запись битовых полей */
void         write_bitfield( void       *struct_ptr, const field_map_t *fm, expr_val_t  val   );

#endif
