#include "modbus_expr.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Лексический и синтаксический разбор */

/* Тип токена */
typedef enum {
  T_EOF,       /* \0 */
  T_NUM_INT,   /* [0-9]+ */
  T_NUM_FLOAT, /* .[0-9]+ */
  T_IDENT,     /* A-Za-z_ */
  T_LBRACK,    /* [ */
  T_RBRACK,    /* ] */
  T_LPAREN,    /* ( */
  T_RPAREN,    /* ) */
  T_QUESTION,  /* ? */
  T_COLON,     /* : */
  T_PLUS,      /* + */
  T_MINUS,     /* - */
  T_STAR,      /* * */
  T_SLASH,     /* / */
  T_PERCENT,   /* % */
  T_AMP,       /* & */
  T_PIPE,      /* | */
  T_CARET,     /* ^ */
  T_TILDE,     /* ~ */
  T_LSHIFT,    /* << */
  T_RSHIFT,    /* >> */
  T_EQ,        /* = */
  T_NE,        /* != <> */
  T_LT,        /* < */
  T_GT,        /* > */
  T_LE,        /* <= */
  T_GE,        /* >= */
  T_LAND,      /* && */
  T_LOR,       /* || */
  T_NOT        /* ! */
} tok_t;

/* Токен */
typedef struct {
  tok_t type;     /* Тип токена */
  int64_t ival;   /* Целочисленное значение */
  double fval;    /* Вещественное значение  */
  char ident[64]; /* Переменная/Функция/и т.д. */
} token_t;

typedef struct {
  const char *src;
  int pos;
  token_t cur;
} lexer_t;

static expr_node_t *parse_expr(lexer_t *L);
static expr_node_t *parse_lor(lexer_t *L);
static expr_node_t *parse_land(lexer_t *L);
static expr_node_t *parse_bor(lexer_t *L);
static expr_node_t *parse_bxor(lexer_t *L);
static expr_node_t *parse_band(lexer_t *L);
static expr_node_t *parse_eq(lexer_t *L);
static expr_node_t *parse_rel(lexer_t *L);
static expr_node_t *parse_shift(lexer_t *L);
static expr_node_t *parse_add(lexer_t *L);
static expr_node_t *parse_mul(lexer_t *L);
static expr_node_t *parse_unary(lexer_t *L);
static expr_node_t *parse_primary(lexer_t *L);
/*
static int  parse_error = 0;
 */
/* Функция для установки/сброса и проверки ошибок */
/*
static void set_error()    { parse_error = 1; }
static void reset_parser() { parse_error = 0; }
static int  has_error()    { return parse_error; }
 */
/* Безопасные сдвиги */
static uint64_t safe_shl(int64_t left, int64_t right) {
  /* Ограничиваем сдвиг разумным диапазоном */
  if (right < 0 || right >= 64) return 0;

  /* Приводим к беззнаковому для безопасного сдвига */
  uint64_t uleft = (uint64_t)left;
  uint64_t result = uleft << (uint64_t)right;

  /* Восстанавливаем знак для отрицательных чисел (арифметический сдвиг)
     Но для беззнаковых операций это не нужно - результат всегда беззнаковый */
  return result;
}

static uint64_t safe_shr(int64_t left, int64_t right) {
  if (right < 0 || right >= 64) return 0;

  /* Для отрицательных чисел - арифметический сдвиг вправо */
  if (left < 0) {
    return (uint64_t)(left >> right); /* implementation-defined, но часто арифметический */
  }
  return (uint64_t)left >> (uint64_t)right;
}

/* пропуск пробелов */
static void skip_ws(lexer_t *L) {
  while (isspace((unsigned char)L->src[L->pos]))
    L->pos++;
}

/* вернуть текущий символ, оставшись на нем */
static char peek(lexer_t *L) { return L->src[L->pos]; }

/* вернуть текущий символ, продвинувшись на следующий */
static char advance(lexer_t *L) { return L->src[L->pos++]; }

/* разбор числа */
/* длина лексемы числа не должна превышать 63 символа */
/* +,- начальный символ знака числа в само число не входит, это отдельный токен
 */
static void lex_number(lexer_t *L, token_t *t) {
  char buf[64];
  int i = 0;

  /* Изначально ожидаем целое число */
  bool is_float = false;

  /* 0x[0-9]+ - HEX-нотация */
  if (L->src[L->pos] == '0' &&
      (L->src[L->pos + 1] == 'x' || L->src[L->pos + 1] == 'X')) {
    buf[i++] = advance(L);
    buf[i++] = advance(L);

    while (isxdigit((unsigned char)peek(L))) {
      if( i < 62 )
        buf[i++] = advance(L);
      else
        break;
    }

    buf[i] = '\0';
    t->type = T_NUM_INT;
    t->ival = strtoll(buf, NULL, 16);

    return;
  }

  /* пока числа, собираем буфер */
  while (isdigit((unsigned char)peek(L))) {
    if( i < 62 )
      buf[i++] = advance(L);
    else
      break;
  }

  /* Если дробное число или число в экспоненциальном представлении
     Если встретилась точка,
     или нотация экспоненты 10e5,10e+5, 10e-5 */
  if (peek(L) == '.' || peek(L) == 'e' || peek(L) == 'E') {
    /* вещественное */
    is_float = true;

    /* точку переносим в буфер */
    if (peek(L) == '.')
      if(i<62)
        buf[i++] = advance(L);

    /* дробную часть переносим в буфер */
    while (isdigit((unsigned char)peek(L))) {
      if( i < 62 )
        buf[i++] = advance(L);
      else
        break;
    }

    /* разбор для экспоненциального представления */
    if (peek(L) == 'e' || peek(L) == 'E') {
      if(i<62)
        buf[i++] = advance(L); /* eE -> в буфер */

      /* знак порядка, если есть - тоже в буфер */
      if (peek(L) == '+' || peek(L) == '-')
        if(i<62)
          buf[i++] = advance(L);
      /* значение порядка */
      while (isdigit((unsigned char)peek(L)))
      {
        if( i < 62 )
          buf[i++] = advance(L);
        else
          break;
      }
    }
  }

  buf[i] = '\0';

  if (is_float) {
    t->type = T_NUM_FLOAT;
    t->fval = strtod(buf, NULL);
  } else {
    t->type = T_NUM_INT;
    t->ival = strtoll(buf, NULL, 10);
  }
}

/* разбор переменной */
static void lex_ident(lexer_t *L, token_t *t) {

  int i = 0;

  while (isalnum((unsigned char)peek(L)) || peek(L) == '_') {
    if (i < 62)
      t->ident[i++] = advance(L);
    else
      advance(L);
  }

  t->ident[i] = '\0';
  t->type = T_IDENT;
}

static void lex_next(lexer_t *L) {

  skip_ws(L); /* пропускаем пробелы */

  char c = peek(L);

  if (!c) /* если нулевой байт - завершение строки */
  {
    L->cur.type = T_EOF;
    return;
  }

  /* Если число или точка */
  if (isdigit((unsigned char)c) || (c == '.' && isdigit((unsigned char)L->src[L->pos + 1]))) {
    lex_number(L, &L->cur); /* начинаем разбор числа */
    return;
  }

  /* если символ или подчеркирование */
  if (isalpha(c) || c == '_') {
    lex_ident(L, &L->cur); /* разбор идентификатора */
    return;
  }
  L->pos++;

  switch (c) {
  case '[':
    L->cur.type = T_LBRACK;
    break;
  case ']':
    L->cur.type = T_RBRACK;
    break;
  case '(':
    L->cur.type = T_LPAREN;
    break;
  case ')':
    L->cur.type = T_RPAREN;
    break;
  case '?':
    L->cur.type = T_QUESTION;
    break;
  case ':':
    L->cur.type = T_COLON;
    break;
  case '+':
    L->cur.type = T_PLUS;
    break;
  case '-':
    L->cur.type = T_MINUS;
    break;
  case '*':
    L->cur.type = T_STAR;
    break;
  case '/':
    L->cur.type = T_SLASH;
    break;
  case '%':
    L->cur.type = T_PERCENT;
    break;
  case '~':
    L->cur.type = T_TILDE;
    break;
  case '!':
    L->cur.type = (peek(L) == '=') ? (L->pos++, T_NE) : T_NOT;
    break;
  case '=':
    L->cur.type = (peek(L) == '=') ? (L->pos++, T_EQ) : T_EOF;
    /* одиночный символ '=' должен отсутствовать в правилах,
       т.к. мы не поддерживаем вложенные выражения с присваиванием */
    break;
  case '<':

    if (peek(L) == '<') {
      L->pos++;
      L->cur.type = T_LSHIFT;
    } else if (peek(L) == '=') {
      L->pos++;
      L->cur.type = T_LE;
    } else if (peek(L) == '>') { /* <> трактуем как != */
      L->pos++;
      L->cur.type = T_NE;
    } else
      L->cur.type = T_LT;

    break;
  case '>':

    if (peek(L) == '>') {
      L->pos++;
      L->cur.type = T_RSHIFT;
    } else if (peek(L) == '=') {
      L->pos++;
      L->cur.type = T_GE;
    } else
      L->cur.type = T_GT;

    break;
  case '&':
    L->cur.type = (peek(L) == '&') ? (L->pos++, T_LAND) : T_AMP;

    break;
  case '|':
    L->cur.type = (peek(L) == '|') ? (L->pos++, T_LOR) : T_PIPE;

    break;

  case '^':
    L->cur.type = T_CARET;
    break;

  default:
    L->cur.type = T_EOF;
    break;
  }
}

static void lex_init(lexer_t *L, const char *src) {

  L->src = src;
  L->pos = 0;
  lex_next(L);
}

#define CUR(L) ((L)->cur.type)
#define ADV(L) lex_next(L)

static cast_type_t identify_cast(const char *n) {
  if (!strcmp(n, "float"))
    return CAST_FLOAT;
  if (!strcmp(n, "double"))
    return CAST_DOUBLE;
  if (!strcmp(n, "int8_t"))
    return CAST_INT8;
  if (!strcmp(n, "uint8_t"))
    return CAST_UINT8;
  if (!strcmp(n, "int16_t"))
    return CAST_INT16;
  if (!strcmp(n, "uint16_t"))
    return CAST_UINT16;
  if (!strcmp(n, "int32_t"))
    return CAST_INT32;
  if (!strcmp(n, "uint32_t"))
    return CAST_UINT32;
  if (!strcmp(n, "int64_t"))
    return CAST_INT64;
  if (!strcmp(n, "uint64_t"))
    return CAST_UINT64;
  return CAST_NONE;
}
static bool is_type_name(const char *n) {
  return identify_cast(n) != CAST_NONE;
}

static func_id_t identify_func(const char *n) {
  if (!strcmp(n, "sin"))
    return FN_SIN;
  if (!strcmp(n, "cos"))
    return FN_COS;
  if (!strcmp(n, "tan"))
    return FN_TAN;
  if (!strcmp(n, "asin"))
    return FN_ASIN;
  if (!strcmp(n, "acos"))
    return FN_ACOS;
  if (!strcmp(n, "atan"))
    return FN_ATAN;
  if (!strcmp(n, "sqrt"))
    return FN_SQRT;
  if (!strcmp(n, "ln"))
    return FN_LN;
  if (!strcmp(n, "log10"))
    return FN_LOG10;
  if (!strcmp(n, "abs"))
    return FN_ABS;
  if (!strcmp(n, "floor"))
    return FN_FLOOR;
  if (!strcmp(n, "ceil"))
    return FN_CEIL;
  if (!strcmp(n, "unixtime"))
    return FN_UNIXTIME;
  return -1;
}

static expr_node_t *node_new(node_kind_t k) {
  expr_node_t *n = calloc(1, sizeof(expr_node_t));

  if (n)
    n->kind = k;

  return n;
}

static expr_node_t *parse_expr(lexer_t *L) {

  expr_node_t *cond = parse_lor(L);
  if(!cond) return NULL;

  if (CUR(L) == T_QUESTION) {
    ADV(L);

    expr_node_t *yes = parse_expr(L);
    if (!yes) {
      expr_free(cond);
      return NULL;
    }

    if (CUR(L) != T_COLON)
    {
      expr_free( yes );
      expr_free( cond );
      return NULL;
    }

    ADV(L);

    expr_node_t *no = parse_expr(L);
    if (!no) {
      expr_free(cond);
      expr_free(yes);
      return NULL;
    }

    expr_node_t *n = node_new(NODE_TERNARY);
    if (!n) {
      expr_free(cond);
      expr_free(yes);
      expr_free(no);
      return NULL;
    }

    n->ternary.cond = cond;
    n->ternary.yes = yes;
    n->ternary.no = no;

    return n;
  }

  return cond;
}

static expr_node_t *parse_lor(lexer_t *L) {
  expr_node_t *left = parse_land(L);
  if(!left) return NULL;

  while (1) {
    int op = -1;
    switch (CUR(L)) {
    case T_LOR:
      op = OP_LOR;
      break;
    }
    if (op < 0)
      break;
    ADV(L);
    expr_node_t *right = parse_land(L);
    if (!right) {
      expr_free(left);
      return NULL;
    }
    expr_node_t *n = node_new(NODE_BINARY);
    if (!n) {
      expr_free(left);
      expr_free(right);
      return NULL;
    }
    n->binary.op = op;
    n->binary.left = left;
    n->binary.right = right;
    left = n;
  }
  return left;
}

static expr_node_t *parse_land(lexer_t *L) {
  expr_node_t *left = parse_bor(L);
  if(!left) return NULL;

  while (1) {
    int op = -1;
    switch (CUR(L)) {
    case T_LAND:
      op = OP_LAND;
      break;
    }
    if (op < 0)
      break;
    ADV(L);
    expr_node_t *right = parse_bor(L);
    if(!right)
    {
      expr_free(left);
      return NULL;
    }
    expr_node_t *n = node_new(NODE_BINARY);
    if(!n) {
      expr_free(right);
      expr_free(left);
      return NULL;
    }
    n->binary.op = op;
    n->binary.left = left;
    n->binary.right = right;
    left = n;
  }
  return left;
}

static expr_node_t *parse_bor(lexer_t *L) {
  expr_node_t *left = parse_bxor(L);
  if(!left) return NULL;
  while (1) {
    int op = -1;
    switch (CUR(L)) {
    case T_PIPE:
      op = OP_OR;
      break;
    }
    if (op < 0)
      break;
    ADV(L);
    expr_node_t *right = parse_bxor(L);
    if(!right) {
      expr_free(left);
      return NULL;
    }
    expr_node_t *n = node_new(NODE_BINARY);
    if(!n) {
      expr_free(right);
      expr_free(left);
      return NULL;
    }
    n->binary.op = op;
    n->binary.left = left;
    n->binary.right = right;
    left = n;
  }
  return left;
}

static expr_node_t *parse_bxor(lexer_t *L) {
  expr_node_t *left = parse_band(L);
  if(!left) return NULL;

  while (1) {
    int op = -1;
    switch (CUR(L)) {
    case T_CARET:
      op = OP_XOR;
      break;
    }
    if (op < 0)
      break;
    ADV(L);
    expr_node_t *right = parse_band(L);
    if(!right) {
      expr_free(left);
      return NULL;
    }
    expr_node_t *n = node_new(NODE_BINARY);
    if (!n) {
      expr_free(right);
      expr_free(left);
      return NULL;
    }
    n->binary.op = op;
    n->binary.left = left;
    n->binary.right = right;
    left = n;
  }
  return left;
}

static expr_node_t *parse_band(lexer_t *L) {
  expr_node_t *left = parse_eq(L);
  if(!left) return NULL;

  while (1) {
    int op = -1;
    switch (CUR(L)) {
    case T_AMP:
      op = OP_AND;
      break;
    }
    if (op < 0)
      break;
    ADV(L);
    expr_node_t *right = parse_eq(L);
    if(!right) {
      expr_free(left);
      return NULL;
    }
    expr_node_t *n = node_new(NODE_BINARY);
    if(!n) {
      expr_free(right);
      expr_free(left);
      return NULL;
    }
    n->binary.op = op;
    n->binary.left = left;
    n->binary.right = right;
    left = n;
  }
  return left;
}

static expr_node_t *parse_eq(lexer_t *L) {
  expr_node_t *left = parse_rel(L);
  if(!left) return NULL;

  while (1) {
    int op = -1;
    switch (CUR(L)) {
    case T_EQ:
      op = OP_EQ;
      break;
    case T_NE:
      op = OP_NE;
      break;
    }
    if (op < 0)
      break;
    ADV(L);
    expr_node_t *right = parse_rel(L);
    if(!right) {
      expr_free(left);
      return NULL;
    }
    expr_node_t *n = node_new(NODE_BINARY);
    if (!n) {
      expr_free(right);
      expr_free(left);
      return NULL;
    }
    n->binary.op = op;
    n->binary.left = left;
    n->binary.right = right;
    left = n;
  }
  return left;
}

static expr_node_t *parse_rel(lexer_t *L) {
  expr_node_t *left = parse_shift(L);
  if(!left) return NULL;

  while (1) {
    int op = -1;
    switch (CUR(L)) {
    case T_LT:
      op = OP_LT;
      break;
    case T_GT:
      op = OP_GT;
      break;
    case T_LE:
      op = OP_LE;
      break;
    case T_GE:
      op = OP_GE;
      break;
    }
    if (op < 0)
      break;
    ADV(L);
    expr_node_t *right = parse_shift(L);
    if(!right) {
      expr_free(left);
      return NULL;
    }
    expr_node_t *n = node_new(NODE_BINARY);
    if (!n) {
      expr_free(right);
      expr_free(left);
      return NULL;
    }
    n->binary.op = op;
    n->binary.left = left;
    n->binary.right = right;
    left = n;
  }
  return left;
}

static expr_node_t *parse_shift(lexer_t *L) {
  expr_node_t *left = parse_add(L);
  if (!left) return NULL;

  while (1) {
    int op = -1;
    switch (CUR(L)) {
    case T_LSHIFT:
      op = OP_SHL;
      break;
    case T_RSHIFT:
      op = OP_SHR;
      break;
    }
    if (op < 0)
      break;
    ADV(L);
    expr_node_t *right = parse_add(L);
    if(!right) {
      expr_free(left);
      return NULL;
    }
    expr_node_t *n = node_new(NODE_BINARY);
    if (!n) {
      expr_free(right);
      expr_free(left);
      return NULL;
    }
    n->binary.op = op;
    n->binary.left = left;
    n->binary.right = right;
    left = n;
  }
  return left;
}

static expr_node_t *parse_add(lexer_t *L) {
  expr_node_t *left = parse_mul(L);
  if (!left) return NULL;

  while (1) {
    int op = -1;
    switch (CUR(L)) {
    case T_PLUS:
      op = OP_ADD;
      break;
    case T_MINUS:
      op = OP_SUB;
      break;
    }
    if (op < 0)
      break;
    ADV(L);
    expr_node_t *right = parse_mul(L);
    if (!right) {
      expr_free(left);
      return NULL;
    }
    expr_node_t *n = node_new(NODE_BINARY);
    if (!n) {
      expr_free(right);
      expr_free(left);
      return NULL;
    }
    n->binary.op = op;
    n->binary.left = left;
    n->binary.right = right;
    left = n;
  }
  return left;
}

static expr_node_t *parse_mul(lexer_t *L) {
  expr_node_t *left = parse_unary(L);
  if (!left) return NULL;

  while (1) {
    int op = -1;
    switch (CUR(L)) {
    case T_STAR:
      op = OP_MUL;
      break;
    case T_SLASH:
      op = OP_DIV;
      break;
    case T_PERCENT:
      op = OP_MOD;
      break;
    }
    if (op < 0)
      break;
    ADV(L);
    expr_node_t *right = parse_unary(L);
    if (!right) {
      expr_free(left);
      return NULL;
    }
    expr_node_t *n = node_new(NODE_BINARY);
    if (!n) {
      expr_free(right);
      expr_free(left);
      return NULL;
    }
    n->binary.op = op;
    n->binary.left = left;
    n->binary.right = right;
    left = n;
  }
  return left;
}

static expr_node_t *parse_unary(lexer_t *L) {

  if (CUR(L) == T_NOT) {
    ADV(L);
    expr_node_t *child = parse_unary(L);
    if(!child) return NULL;

    expr_node_t *n = node_new(NODE_UNARY);
    if(!n) {
      expr_free( child );
      return NULL;
    }
    n->unary.op = UNARY_NOT;
    n->unary.child = child;
    return n;
  }

  if (CUR(L) == T_TILDE) {
    ADV(L);
    expr_node_t *child = parse_unary(L);
    if(!child) return NULL;

    expr_node_t *n = node_new(NODE_UNARY);
    if(!n) {
      expr_free( child );
      return NULL;
    }
    n->unary.op = UNARY_BNOT;
    n->unary.child = child;
    return n;
  }

  if (CUR(L) == T_MINUS) {
    ADV(L);
    expr_node_t *child = parse_unary(L);
    if(!child) return NULL;

    expr_node_t *n = node_new(NODE_UNARY);
    if(!n) {
      expr_free( child );
      return NULL;
    }
    n->unary.op = UNARY_NEG;
    n->unary.child = child;
    return n;
  }

  if (CUR(L) == T_LPAREN) {
    lexer_t saved = *L;
    ADV(L);
    if (CUR(L) == T_IDENT && is_type_name(L->cur.ident)) {
      cast_type_t ct = identify_cast(L->cur.ident);
      ADV(L);
      if (CUR(L) == T_RPAREN) {
        ADV(L);
        expr_node_t *child = parse_unary(L);
        if(!child) return NULL;

        expr_node_t *n = node_new(NODE_CAST);
        if(!n) {
          expr_free(child);
          return NULL;
        }
        n->cast.ctype = ct;
        n->cast.child = child;
        return n;
      }
    }
    *L = saved;
  }

  return parse_primary(L);
}

static expr_node_t *parse_primary(lexer_t *L) {

  if (CUR(L) == T_NUM_INT) {
    expr_node_t *n = node_new(NODE_LIT_INT);
    if(!n) return NULL;

    n->lit_int = L->cur.ival;
    ADV(L);
    return n;
  }

  if (CUR(L) == T_NUM_FLOAT) {
    expr_node_t *n = node_new(NODE_LIT_FLOAT);
    if(!n) return NULL;

    n->lit_float = L->cur.fval;
    ADV(L);
    return n;
  }

  if (CUR(L) == T_IDENT) {
    /* HR-РЕГИСТР modbus */
    if (!strcmp(L->cur.ident, "HR")) {
      ADV(L);

      if (CUR(L) != T_LBRACK) {
        fprintf(stderr,
                "Внимание! Некорректное описание HR-регистра, ожидается [\n");
        return NULL;
      }

      ADV(L);
      if (CUR(L) == T_RBRACK) {
        fprintf(stderr,
                "Внимание! Некорректное описание HR-регистра, без номера\n");
        return NULL;
      }

      expr_node_t *idx = parse_expr(L);
      if(!idx) return NULL;

      if (CUR(L) != T_RBRACK) {
        fprintf(stderr, "Внимание! Некорректное описание HR-регистра, ожидается ]\n");
        expr_free(idx);
        return NULL;
      }
      ADV(L);

      if (idx->kind == NODE_LIT_INT) {
        int r = (int)idx->lit_int;
        expr_free(idx);
        if (r > 65535) {
          fprintf(stderr, "Внимание! Номер регистра больше 65535, HR[reg>65535]\n");
          return NULL;
        }

        expr_node_t *n = node_new(NODE_REG_HR);
        if (!n) return NULL;
        n->reg_idx = r;
        return n;
      }
      expr_free(idx);
      return NULL;
    } /* HR-REG */

    /* IR-РЕГИСТР modbus */
    if (!strcmp(L->cur.ident, "IR")) {
      ADV(L);

      if (CUR(L) != T_LBRACK) {
        fprintf(stderr, "Внимание! Некорректное описание IR-регистра, ожидается [\n");
        return NULL;
      }

      ADV(L);
      if (CUR(L) == T_RBRACK) {
        fprintf(stderr, "Внимание! Некорректное описание IR-регистра, без номера\n");
        return NULL;
      }

      expr_node_t *idx = parse_expr(L);
      if(!idx) return NULL;

      if (CUR(L) != T_RBRACK) {
        fprintf(stderr,
                "Внимание! Некорректное описание IR-регистра, ожидается ]\n");
        expr_free(idx);
        return NULL;
      }
      ADV(L);

      if (idx->kind == NODE_LIT_INT) {
        int r = (int)idx->lit_int;
        expr_free(idx);
        if (r > 65535) {
          fprintf(stderr,
                  "Внимание! Номер регистра больше 65535, IR[reg>65535]\n");
          return NULL;
        }
        expr_node_t *n = node_new(NODE_REG_IR);
        if (!n) return NULL;
        n->reg_idx = r;
        return n;
      }
      expr_free(idx);
      return NULL;
    } /* IR-REG */

    /* Байтовый конвертор в FLOAT */
    if (!strcmp(L->cur.ident, "FLOAT")) {
      ADV(L);

      if (CUR(L) != T_LPAREN)
        return NULL;

      ADV(L);

      expr_node_t *c = parse_expr(L);
      if(!c) return NULL;

      if (CUR(L) != T_RPAREN) {
        expr_free(c);
        return NULL;
      }

      ADV(L);

      expr_node_t *n = node_new(NODE_FLOAT_CONVERT);
      if (!n) {
        expr_free(c);
        return NULL;
      }
      n->byteconvert.child = c;

      return n;
    }

    /* Байтовый конвертор в DOUBLE */
    if (!strcmp(L->cur.ident, "DOUBLE")) {
      ADV(L);

      if (CUR(L) != T_LPAREN)
        return NULL;

      ADV(L);

      expr_node_t *c = parse_expr(L);
      if(!c) return NULL;

      if (CUR(L) != T_RPAREN) {
        expr_free(c);
        return NULL;
      }

      ADV(L);

      expr_node_t *n = node_new(NODE_DOUBLE_CONVERT);
      if(!n) {
        expr_free(c);
        return NULL;
      }
      n->byteconvert.child = c;

      return n;
    }

    func_id_t fn_id = identify_func(L->cur.ident);
    /* ФУНКЦИЯ */
    if ((int)fn_id >= 0) {
      ADV(L);

      if (CUR(L) != T_LPAREN)
        return NULL;

      ADV(L);

      expr_node_t *child = parse_expr(L);
      if(!child) return NULL;

      if (CUR(L) != T_RPAREN) {
        expr_free(child);
        return NULL;
      }

      ADV(L);

      expr_node_t *n = node_new(NODE_FUNC);
      if(!n) {
        expr_free(child);
        return NULL;
      }
      n->func.fn = fn_id;
      n->func.child = child;
      return n;
    }

    expr_node_t *n = node_new(NODE_VAR);
    if(!n) return NULL;

    snprintf( n->var_name, sizeof(n->var_name), "%s", L->cur.ident );

    ADV(L);
    return n;
  }

  if (CUR(L) == T_LPAREN) {
    ADV(L);
    expr_node_t *n = parse_expr(L);
    if(!n) return NULL;

    if (CUR(L) != T_RPAREN) {
      expr_free(n);
      return NULL;
    }
    ADV(L);
    return n;
  }

  return NULL;
}

expr_node_t *expr_parse(const char *str) {
  lexer_t L;

  lex_init(&L, str);

  return parse_expr(&L);
}

void expr_free(expr_node_t *n) {
  if (!n)
    return;

  switch (n->kind) {

  case NODE_UNARY:
    expr_free(n->unary.child);
    break;

  case NODE_BINARY:
    expr_free(n->binary.left);
    expr_free(n->binary.right);
    break;

  case NODE_TERNARY:
    expr_free(n->ternary.cond);
    expr_free(n->ternary.yes);
    expr_free(n->ternary.no);
    break;

  case NODE_CAST:
    expr_free(n->cast.child);
    break;

  case NODE_FLOAT_CONVERT:
    expr_free(n->byteconvert.child);
    break;

  case NODE_DOUBLE_CONVERT:
    expr_free(n->byteconvert.child);
    break;

  case NODE_FUNC:
    expr_free(n->func.child);
    break;

  default:
    break;
  }

  free(n);
}

/* Вычислитель */
static expr_val_t val_int(int64_t i) {
  return (expr_val_t){.is_float = false, .i = i};
}

static expr_val_t val_flt(double f) {
  return (expr_val_t){.is_float = true, .f = f};
}

/* конвертация double <--> int64 */
static int64_t to_int(expr_val_t v) { return v.is_float ? (int64_t)v.f : v.i; }

static double to_flt(expr_val_t v) { return v.is_float ? v.f : (double)v.i; }

void read_bitfield(const void *struct_ptr, const field_map_t *fm, expr_val_t *result) {

    const uint8_t *bytes = (const uint8_t *)struct_ptr;

    /* Определяем размер по позиции битов */
    int max_bit = fm->bit_pos + fm->bit_width;

    /* Выбираем правильный тип на основе максимального бита */
    if (max_bit <= 8) {
        uint8_t raw;
        memcpy(&raw, bytes + fm->offset, 1);
        uint8_t mask = (fm->bit_width == 8) ? 0xFF : ((1u << fm->bit_width) - 1);
        result->i = (raw >> fm->bit_pos) & mask;
        result->is_float = false;
    } else if (max_bit <= 16) {
        uint16_t raw;
        memcpy(&raw, bytes + fm->offset, 2);
        uint16_t mask = (fm->bit_width == 16) ? 0xFFFF : ((1u << fm->bit_width) - 1);
        result->i = (raw >> fm->bit_pos) & mask;
        result->is_float = false;
    } else if (max_bit <= 32) {
        uint32_t raw;
        memcpy(&raw, bytes + fm->offset, 4);
        uint32_t mask = (fm->bit_width == 32) ? 0xFFFFFFFFu : ((1u << fm->bit_width) - 1);
        result->i = (raw >> fm->bit_pos) & mask;
        result->is_float = false;
    } else if (max_bit <= 64) {
        uint64_t raw;
        memcpy(&raw, bytes + fm->offset, 8);
        uint64_t mask = (fm->bit_width == 64) ? 0xFFFFFFFFFFFFFFFFull : ((1ull << fm->bit_width) - 1);
        result->i = (raw >> fm->bit_pos) & mask;
        result->is_float = false;
    } else {
        fprintf( stderr, "Ошибка, битовое поле больше 64 бит\n" );
        result->i = 0;
        result->is_float = false;
    }
}

void write_bitfield(void *struct_ptr, const field_map_t *fm, expr_val_t val) {
    uint8_t *bytes = (uint8_t *)struct_ptr;
    int max_bit = fm->bit_pos + fm->bit_width;
    uint64_t value = (uint64_t)val.i;

    if (max_bit <= 8) {
        uint8_t raw;
        memcpy(&raw, bytes + fm->offset, 1);
        uint8_t mask = (fm->bit_width == 8) ? 0xFF : ((1u << fm->bit_width) - 1);
        uint8_t new_bits = (value & mask) << fm->bit_pos;
        raw = (raw & ~(mask << fm->bit_pos)) | new_bits;
        memcpy(bytes + fm->offset, &raw, 1);
    } else if (max_bit <= 16) {
        uint16_t raw;
        memcpy(&raw, bytes + fm->offset, 2);
        uint16_t mask = (fm->bit_width == 16) ? 0xFFFF : ((1u << fm->bit_width) - 1);
        uint16_t new_bits = (value & mask) << fm->bit_pos;
        raw = (raw & ~(mask << fm->bit_pos)) | new_bits;
        memcpy(bytes + fm->offset, &raw, 2);
    } else if (max_bit <= 32) {
        uint32_t raw;
        memcpy(&raw, bytes + fm->offset, 4);
        uint32_t mask = (fm->bit_width == 32) ? 0xFFFFFFFFu : ((1u << fm->bit_width) - 1);
        uint32_t new_bits = (value & mask) << fm->bit_pos;
        raw = (raw & ~(mask << fm->bit_pos)) | new_bits;
        memcpy(bytes + fm->offset, &raw, 4);
    } else if (max_bit <= 64) {
        uint64_t raw;
        memcpy(&raw, bytes + fm->offset, 8);
        uint64_t mask = (fm->bit_width == 64) ? 0xFFFFFFFFFFFFFFFFull : ((1ull << fm->bit_width) - 1);
        uint64_t new_bits = (value & mask) << fm->bit_pos;
        raw = (raw & ~(mask << fm->bit_pos)) | new_bits;
        memcpy(bytes + fm->offset, &raw, 8);
    }
}

/* чтение поля структуры/составного типа (AppStruct) */
static expr_val_t read_field(const void *struct_ptr, const field_map_t *fm) {
  const uint8_t *bytes = (const uint8_t *)struct_ptr;

  if (fm->type == FTYPE_FLOAT) {

    float f;
    memcpy(&f, bytes + fm->offset, sizeof(float));
    return val_flt(f);

  } else if (fm->type == FTYPE_DOUBLE) {

    double f;
    memcpy(&f, bytes + fm->offset, sizeof(double));
    return val_flt(f);

  } else if (fm->type == FTYPE_INT16) {

    int16_t v;
    memcpy(&v, bytes + fm->offset, sizeof(int16_t));
    return val_int(v);

  } else if (fm->type == FTYPE_UINT16) {

    uint16_t v;
    memcpy(&v, bytes + fm->offset, sizeof(uint16_t));
    return val_int(v);

  } else if (fm->type == FTYPE_INT32) {

    int32_t v;
    memcpy(&v, bytes + fm->offset, sizeof(int32_t));
    return val_int(v);

  } else if (fm->type == FTYPE_UINT32) {

    uint32_t v;
    memcpy(&v, bytes + fm->offset, sizeof(uint32_t));
    return val_int(v);

  } else if (fm->type == FTYPE_INT64) {

    int64_t v;
    memcpy(&v, bytes + fm->offset, sizeof(int64_t));
    return val_int(v);
  } else if (fm->type == FTYPE_UINT64) {

    uint64_t v;
    memcpy(&v, bytes + fm->offset, sizeof(uint64_t));
    return val_int((int64_t)v);

  } else if (fm->type == FTYPE_UINT8) {

    uint8_t v;
    memcpy(&v, bytes + fm->offset, sizeof(uint8_t));
    return val_int(v);

  } else if (fm->type == FTYPE_BITFIELD) {

    expr_val_t result;
    read_bitfield(struct_ptr, fm, &result);
    return result;
  }

  return val_int(0);
}

expr_val_t expr_eval(const expr_node_t *n,
                     const uint16_t *HR_regs, size_t HR_reg_count,
                     const uint16_t *IR_regs, size_t IR_reg_count,
                     const expr_var_t *vars, size_t var_count,
                     const void *struct_ptr,
                     const field_map_t *fmap, size_t fmap_size) {
  if (!n)
    return val_int(0);

  if(!HR_regs || !IR_regs || !vars || !struct_ptr || !fmap )
    return val_int(0);

  switch (n->kind) {

  case NODE_LIT_INT:
    return val_int(n->lit_int);

  case NODE_LIT_FLOAT:
    return val_flt(n->lit_float);

  case NODE_REG_HR:
    return val_int((n->reg_idx >= 0 && (size_t)n->reg_idx < HR_reg_count)
                       ? HR_regs[n->reg_idx]
                       : 0);

  case NODE_REG_IR:
    return val_int((n->reg_idx >= 0 && (size_t)n->reg_idx < IR_reg_count)
                       ? IR_regs[n->reg_idx]
                       : 0);

  case NODE_VAR: {
    /* 1. Вначале проверяем имя на соответствие внутренней переменной
       (у них преимущество) */
    for (size_t i = 0; i < var_count; i++) { /* здесь бы соптимизировать */
      if (strcmp(vars[i].name, n->var_name) == 0)
        return vars[i].val;
    }
    /* 2. Если не найдено, проверим - это имя поля в структуре? */
    if (struct_ptr && fmap) {
      for (size_t i = 0; i < fmap_size; i++) {
        if (strcmp(fmap[i].name, n->var_name) == 0) {
          return read_field(struct_ptr, &fmap[i]);
        }
      }
    }
    return val_int(0);
  }

  /* если это имя функции */
  case NODE_FUNC: {
    /* вычисляем аргумент у функции - рекурсивный спуск */
    expr_val_t c =
        expr_eval(n->func.child, HR_regs, HR_reg_count, IR_regs, IR_reg_count,
                  vars, var_count, struct_ptr, fmap, fmap_size);
    /* аргумент функции */
    double arg = to_flt(c);
    /* результат вычисления функции */
    double res = 0.0;

    switch (n->func.fn) {
    case FN_SIN:
      res = sin(arg);
      break;
    case FN_COS:
      res = cos(arg);
      break;
    case FN_TAN:
      res = tan(arg);
      break;
    case FN_ASIN:
      res = asin(arg);
      break;
    case FN_ACOS:
      res = acos(arg);
      break;
    case FN_ATAN:
      res = atan(arg);
      break;
    case FN_SQRT:
      res = sqrt(arg);
      break;
    case FN_LN:
      res = log(arg);
      break;
    case FN_LOG10:
      res = log10(arg);
      break;
    case FN_ABS:
      res = fabs(arg);
      break;
    case FN_FLOOR:
      res = floor(arg);
      break;
    case FN_CEIL:
      res = ceil(arg);
      break;
    case FN_UNIXTIME:
      return val_int(time(NULL) + to_int(c));
      break;
    }
    return val_flt(res);
  }

  /* ОПЕРАЦИЯ ОДНОГО АРГУМЕНТА */
  case NODE_UNARY: {
    /* вычисление аргумента унарной операции */
    expr_val_t c =
        expr_eval(n->unary.child, HR_regs, HR_reg_count, IR_regs, IR_reg_count,
                  vars, var_count, struct_ptr, fmap, fmap_size);
    switch (n->unary.op) {
    case UNARY_NOT:
      return val_int(!to_int(c));
    case UNARY_BNOT:
      return val_int(~to_int(c));
    case UNARY_NEG:
      return c.is_float ? val_flt(-c.f) : val_int(-c.i);
    }
    return c;
  }

  /* ОПЕРАЦИЯ ДВУХ АРГУМЕНТОВ */
  case NODE_BINARY: {
    /* вычисляем левый аргумент */
    expr_val_t L =
        expr_eval(n->binary.left, HR_regs, HR_reg_count, IR_regs, IR_reg_count,
                  vars, var_count, struct_ptr, fmap, fmap_size);
    /* вычисляем правый аргумент */
    expr_val_t R =
        expr_eval(n->binary.right, HR_regs, HR_reg_count, IR_regs, IR_reg_count,
                  vars, var_count, struct_ptr, fmap, fmap_size);
    /* если один из аргументов вещественный - то результат тоже */
    bool uf = L.is_float || R.is_float;
    switch (n->binary.op) {
    case OP_MUL:
      return uf ? val_flt(to_flt(L) * to_flt(R)) : val_int(L.i * R.i);
    case OP_DIV:
/*
      return uf ? val_flt(to_flt(L) / to_flt(R))
                : (R.i ? val_int(L.i / R.i) : val_int(0));
 */
      if (uf) {
        double divisor = to_flt(R);
        if (divisor == 0.0) {
/*        return val_flt(INFINITY); */
          fprintf( stderr, "Внимание, деление на 0.0\n" );
          return val_flt(0);
        }
        return val_flt(to_flt(L) / divisor);
      }
      else {
/*
        int64_t divisor = R.i;
        return (divisor != 0) ? val_int(L.i / divisor) : val_int(0);
 */
        int64_t divisor = R.i;
        if (divisor == 0) {
          fprintf( stderr, "Внимание, деление на 0\n" );
          return val_int(0);
        }
        if (divisor == -1 && L.i == INT64_MIN) return val_int(INT64_MAX);
        return val_int(L.i / divisor);
      }
      break;
    case OP_MOD:
      return (R.i != 0) ? val_int(L.i % R.i) : val_int(0);
    case OP_ADD:
      return uf ? val_flt(to_flt(L) + to_flt(R)) : val_int(L.i + R.i);
    case OP_SUB:
      return uf ? val_flt(to_flt(L) - to_flt(R)) : val_int(L.i - R.i);
    case OP_SHL:
      return val_int((int64_t)safe_shl(to_int(L), to_int(R)));
    case OP_SHR:
      return val_int((int64_t)safe_shr(to_int(L), to_int(R)));
    case OP_AND:
      return val_int(to_int(L) & to_int(R));
    case OP_OR:
      return val_int(to_int(L) | to_int(R));
    case OP_XOR:
      return val_int(to_int(L) ^ to_int(R));
    case OP_LT:
      return val_int(uf ? (to_flt(L) < to_flt(R)) : (L.i < R.i));
    case OP_GT:
      return val_int(uf ? (to_flt(L) > to_flt(R)) : (L.i > R.i));
    case OP_EQ:
      return val_int(uf ? (to_flt(L) == to_flt(R)) : (L.i == R.i));
    case OP_NE:
      return val_int(uf ? (to_flt(L) != to_flt(R)) : (L.i != R.i));
    case OP_LE:
      return val_int(uf ? (to_flt(L) <= to_flt(R)) : (L.i <= R.i));
    case OP_GE:
      return val_int(uf ? (to_flt(L) >= to_flt(R)) : (L.i >= R.i));
    case OP_LAND:
      return val_int(to_int(L) && to_int(R));
    case OP_LOR:
      return val_int(to_int(L) || to_int(R));
    }
    return val_int(0);
  }

  /* ТЕРНАРНАЯ ОПЕРАЦИЯ */
  case NODE_TERNARY:
    return to_int(expr_eval(n->ternary.cond, HR_regs, HR_reg_count, IR_regs,
                            IR_reg_count, vars, var_count, struct_ptr, fmap,
                            fmap_size))
               ? expr_eval(n->ternary.yes, HR_regs, HR_reg_count, IR_regs,
                           IR_reg_count, vars, var_count, struct_ptr, fmap,
                           fmap_size)
               : expr_eval(n->ternary.no, HR_regs, HR_reg_count, IR_regs,
                           IR_reg_count, vars, var_count, struct_ptr, fmap,
                           fmap_size);

  case NODE_CAST: {
    expr_val_t c =
        expr_eval(n->cast.child, HR_regs, HR_reg_count, IR_regs, IR_reg_count,
                  vars, var_count, struct_ptr, fmap, fmap_size);
    int64_t r = to_int(c);
    switch (n->cast.ctype) {
    case CAST_FLOAT:
      return val_flt((float)r);
    case CAST_DOUBLE:
      return val_flt((double)r);
    case CAST_INT8:
      return val_int((int8_t)r);
    case CAST_UINT8:
      return val_int((uint8_t)r);
    case CAST_INT16:
      return val_int((int16_t)r);
    case CAST_UINT16:
      return val_int((uint16_t)r);
    case CAST_INT32:
      return val_int((int32_t)r);
    case CAST_UINT32:
      return val_int((uint32_t)r);
    default:
      return c;
    }
  }

  case NODE_FLOAT_CONVERT: {
    expr_val_t c =
        expr_eval(n->byteconvert.child, HR_regs, HR_reg_count, IR_regs,
                  IR_reg_count, vars, var_count, struct_ptr, fmap, fmap_size);
    uint32_t bits = (uint32_t)to_int(c);
    float f;
    memcpy(&f, &bits, 4);
    return val_flt(f);
  }

  case NODE_DOUBLE_CONVERT: {
    expr_val_t c =
        expr_eval(n->byteconvert.child, HR_regs, HR_reg_count, IR_regs,
                  IR_reg_count, vars, var_count, struct_ptr, fmap, fmap_size);
    uint64_t bits = (uint64_t)to_int(c);
    double d;
    memcpy(&d, &bits, 8);
    return val_flt(d);
  }

  default:
    return val_int(0);
  }
}
