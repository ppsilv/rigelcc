// Code generator for the Motorola 68000 (vasm, Motorola syntax module).
//
// Model (same idea as the original x86-64 chibicc: a stack machine):
//   d0  = result of an expression (or the ADDRESS of an array/struct/function)
//   d1  = second operand, a0/a1 = scratch address registers
//   Temporaries go to the stack with move.l d0,-(sp) / move.l (sp)+,d1.
//   Only d0,d1,a0,a1 are used, so nothing has to be saved (they are the
//   scratch registers of the gcc m68k ABI).
//
// Calling convention (matches m68k-elf-gcc):
//   - all arguments on the stack, pushed right to left, 4-byte slots;
//     a char/short sits at the END of its slot (big-endian)
//   - caller pops the arguments
//   - result in d0
//   - struct return: buffer address in a0 (callee copies and returns it)
//
// Phase 1 limits: no floating point, no 64-bit integers (long long), no
// atomics, no thread-local variables.

#include "rigelcc.h"

static FILE *output_file;
static int depth;           // bytes pushed on the stack by the expression code
static Obj *current_fn;
static HashMap refs;        // symbols referenced by this file
static HashMap defs;        // symbols defined by this file

static void gen_expr(Node *node);
static void gen_stmt(Node *node);

__attribute__((format(printf, 1, 2)))
static void println(char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vfprintf(output_file, fmt, ap);
  va_end(ap);
  fprintf(output_file, "\n");
}

static int count(void) {
  static int i = 1;
  return i++;
}

// Round up `n` to the nearest multiple of `align`.
int align_to(int n, int align) {
  return (n + align - 1) / align * align;
}

static void ref_sym(char *name) {
  hashmap_put(&refs, name, (void *)1);
}

static void def_sym(char *name) {
  hashmap_put(&defs, name, (void *)1);
}

static void push(void) {
  println("  move.l d0,-(sp)");
  depth += 4;
}

static void pop(char *reg) {
  println("  move.l (sp)+,%s", reg);
  depth -= 4;
}

// 32-bit pattern of a constant (so that 0xffffffff prints as -1).
static long imm32(long v) {
  return (int32_t)v;
}

// Adjust the stack pointer by +n bytes (n >= 0).
static void sp_add(int n) {
  if (n == 0)
    return;
  if (n <= 8)
    println("  addq.l #%d,sp", n);
  else if (n <= 32767)
    println("  lea %d(sp),sp", n);
  else
    println("  adda.l #%d,sp", n);
}

// Adjust the stack pointer by -n bytes (n >= 0).
static void sp_sub(int n) {
  if (n == 0)
    return;
  if (n <= 8)
    println("  subq.l #%d,sp", n);
  else if (n <= 32767)
    println("  lea -%d(sp),sp", n);
  else
    println("  suba.l #%d,sp", n);
}

// d0 += n
static void add_const(long n) {
  n = imm32(n);
  if (n == 0)
    return;
  if (n > 0 && n <= 8)
    println("  addq.l #%ld,d0", n);
  else if (n < 0 && n >= -8)
    println("  subq.l #%ld,d0", -n);
  else
    println("  add.l #%ld,d0", n);
}

// Shift d0 by a constant. op is "lsl", "lsr" or "asr".
static void shift_const(char *op, int n) {
  if (n <= 0)
    return;
  if (n <= 8) {
    println("  %s.l #%d,d0", op, n);
  } else {
    println("  moveq #%d,d1", n);
    println("  %s.l d1,d0", op);
  }
}

static void check_type(Type *ty, Token *tok) {
  if (!ty)
    return;
  if (is_flonum(ty))
    error_tok(tok, "floating-point types are not supported yet (m68k phase 1)");
  if (is_integer(ty) && ty->size > 4)
    error_tok(tok, "64-bit integers (long long) are not supported yet (m68k phase 1)");
}

// Load a value of type `ty` from the operand `src` (an assembler operand
// such as "(a0)" or "-4(a6)") into d0, extending it to 32 bits.
static void load_mem(Type *ty, char *src) {
  if (ty->size == 1) {
    if (ty->is_unsigned) {
      println("  moveq #0,d0");
      println("  move.b %s,d0", src);
    } else {
      println("  move.b %s,d0", src);
      println("  ext.w d0");
      println("  ext.l d0");
    }
  } else if (ty->size == 2) {
    if (ty->is_unsigned) {
      println("  moveq #0,d0");
      println("  move.w %s,d0", src);
    } else {
      println("  move.w %s,d0", src);
      println("  ext.l d0");
    }
  } else {
    println("  move.l %s,d0", src);
  }
}

// Load a bitfield storage unit of `size` bytes from (a0) into d0, zero-extended.
static void load_unit(int size) {
  println("  moveq #0,d0");
  if (size == 1)
    println("  move.b (a0),d0");
  else if (size == 2)
    println("  move.w (a0),d0");
  else
    println("  move.l (a0),d0");
}

// Compute the absolute address of a given node into d0.
static void gen_addr(Node *node) {
  switch (node->kind) {
  case ND_VAR:
    // Variable-length array, which is always local.
    if (node->var->ty->kind == TY_VLA) {
      println("  move.l %d(a6),d0", node->var->offset);
      return;
    }

    // Local variable
    if (node->var->is_local) {
      println("  lea %d(a6),a0", node->var->offset);
      println("  move.l a0,d0");
      return;
    }

    if (node->var->is_tls)
      error_tok(node->tok, "thread-local variables are not supported");

    // Function or global variable: absolute address (the ELF relocation
    // is kept by the linker with -Wl,-q and fixed by the Rigel loader).
    ref_sym(node->var->name);
    println("  move.l #%s,d0", node->var->name);
    return;
  case ND_DEREF:
    gen_expr(node->lhs);
    return;
  case ND_COMMA:
    gen_expr(node->lhs);
    gen_addr(node->rhs);
    return;
  case ND_MEMBER:
    gen_addr(node->lhs);
    add_const(node->member->offset);
    return;
  case ND_FUNCALL:
    if (node->ret_buffer) {
      gen_expr(node);
      return;
    }
    break;
  case ND_ASSIGN:
  case ND_COND:
    if (node->ty->kind == TY_STRUCT || node->ty->kind == TY_UNION) {
      gen_expr(node);
      return;
    }
    break;
  case ND_VLA_PTR:
    println("  lea %d(a6),a0", node->var->offset);
    println("  move.l a0,d0");
    return;
  }

  error_tok(node->tok, "not an lvalue");
}

// Load a value from where d0 is pointing to.
static void load(Type *ty) {
  switch (ty->kind) {
  case TY_ARRAY:
  case TY_STRUCT:
  case TY_UNION:
  case TY_FUNC:
  case TY_VLA:
    // An array/struct is represented by its address; nothing to load.
    return;
  }

  println("  move.l d0,a0");
  load_mem(ty, "(a0)");
}

// Copy `size` bytes from (src) to (dst); both are address registers which
// are clobbered. `align` is the known alignment of both blocks.
static void emit_copy(char *src, char *dst, int size, int align) {
  int unit = 1;
  char sfx = 'b';
  if (align >= 2 && size % 2 == 0) {
    unit = 2;
    sfx = 'w';
    if (size % 4 == 0) {
      unit = 4;
      sfx = 'l';
    }
  }

  if (size <= 16 * unit) {
    for (int i = 0; i < size; i += unit)
      println("  move.%c %d(%s),%d(%s)", sfx, i, src, i, dst);
    return;
  }

  int c = count();
  println("  move.l #%d,d1", size / unit);
  println("__Lcp%d:", c);
  println("  move.%c (%s)+,(%s)+", sfx, src, dst);
  println("  subq.l #1,d1");
  println("  bne __Lcp%d", c);
}

// Store d0 to the address that the stack top is pointing to.
static void store(Type *ty) {
  pop("a0");

  switch (ty->kind) {
  case TY_STRUCT:
  case TY_UNION:
    println("  move.l d0,a1");
    emit_copy("a1", "a0", ty->size, ty->align);
    return;
  }

  if (ty->size == 1)
    println("  move.b d0,(a0)");
  else if (ty->size == 2)
    println("  move.w d0,(a0)");
  else
    println("  move.l d0,(a0)");
}

static void cmp_zero(Type *ty) {
  check_type(ty, NULL);
  println("  tst.l d0");
}

// Normalize d0 after a cast from `from` to `to` (both integer-like, <= 4 bytes).
static void cast(Type *from, Type *to) {
  if (to->kind == TY_VOID)
    return;

  if (to->kind == TY_BOOL) {
    cmp_zero(from);
    println("  sne d0");
    println("  and.l #1,d0");
    return;
  }

  if (to->size >= 4)
    return;

  // Nothing to do if the value already fits and has the right signedness.
  if (from->size == to->size && from->is_unsigned == to->is_unsigned &&
      from->kind != TY_PTR)
    return;
  if (from->size < to->size && (from->is_unsigned || !to->is_unsigned) &&
      from->kind != TY_PTR)
    return;

  if (to->size == 1) {
    if (to->is_unsigned) {
      println("  and.l #255,d0");
    } else {
      println("  ext.w d0");
      println("  ext.l d0");
    }
  } else {
    if (to->is_unsigned)
      println("  and.l #65535,d0");
    else
      println("  ext.l d0");
  }
}

// Push a struct (address in d0) by value: a 4-byte aligned copy.
static void push_struct(Type *ty) {
  int sz = align_to(ty->size, 4);
  sp_sub(sz);
  depth += sz;
  println("  move.l d0,a0");
  println("  move.l sp,a1");
  emit_copy("a0", "a1", ty->size, ty->align);
}

static int arg_slot(Type *ty) {
  return align_to(ty->size, 4);
}

static void push_args2(Node *args) {
  if (!args)
    return;
  push_args2(args->next);

  check_type(args->ty, args->tok);
  gen_expr(args);

  switch (args->ty->kind) {
  case TY_STRUCT:
  case TY_UNION:
    push_struct(args->ty);
    break;
  default:
    push();
  }
}

static void builtin_alloca(void) {
  // d0 = requested size. Round it up to 4 bytes.
  println("  add.l #3,d0");
  println("  and.l #-4,d0");

  // Shift the temporary area (from sp up to alloca_bottom) down by d0.
  int c = count();
  int off = current_fn->alloca_bottom->offset;
  println("  move.l d0,d1");
  println("  move.l sp,a1");
  println("  suba.l d1,sp");
  println("  move.l sp,a0");
  println("  move.l %d(a6),d0", off);
  println("  sub.l a1,d0");
  println("  beq __Lal%d", c);
  println("__Lac%d:", c);
  println("  move.b (a1)+,(a0)+");
  println("  subq.l #1,d0");
  println("  bne __Lac%d", c);
  println("__Lal%d:", c);

  // Move the alloca_bottom pointer; it is the result.
  println("  move.l %d(a6),d0", off);
  println("  sub.l d1,d0");
  println("  move.l d0,%d(a6)", off);
}

static bool is_pow2(long v) {
  return v > 0 && (v & (v - 1)) == 0;
}

static int log2l(long v) {
  int n = 0;
  while (v > 1) {
    v >>= 1;
    n++;
  }
  return n;
}

static void libcall2(char *fn) {
  // d0 = lhs. rhs is already on the stack. Both are passed as stack
  // arguments (lhs first), exactly like a C call fn(lhs, rhs).
  push();
  ref_sym(fn);
  println("  jsr %s", fn);
  println("  addq.l #8,sp");
  depth -= 8;
}

// Generate code for a given node.
static void gen_expr(Node *node) {
  check_type(node->ty, node->tok);

  switch (node->kind) {
  case ND_NULL_EXPR:
    return;
  case ND_NUM: {
    long v = imm32(node->val);
    if (v >= -128 && v <= 127)
      println("  moveq #%ld,d0", v);
    else
      println("  move.l #%ld,d0", v);
    return;
  }
  case ND_NEG:
    gen_expr(node->lhs);
    println("  neg.l d0");
    return;
  case ND_VAR:
    if (node->var->is_local && node->var->ty->kind != TY_VLA) {
      switch (node->var->ty->kind) {
      case TY_ARRAY:
      case TY_STRUCT:
      case TY_UNION:
      case TY_FUNC:
        break;
      default: {
        char operand[64];
        snprintf(operand, sizeof(operand), "%d(a6)", node->var->offset);
        load_mem(node->var->ty, operand);
        return;
      }
      }
    }
    gen_addr(node);
    load(node->ty);
    return;
  case ND_MEMBER: {
    gen_addr(node);
    Member *mem = node->member;
    if (mem->is_bitfield) {
      println("  move.l d0,a0");
      load_unit(mem->bf_size);
      shift_const("lsl", 32 - mem->bit_width - mem->bit_offset);
      if (mem->ty->is_unsigned)
        shift_const("lsr", 32 - mem->bit_width);
      else
        shift_const("asr", 32 - mem->bit_width);
      return;
    }
    load(node->ty);
    return;
  }
  case ND_DEREF:
    gen_expr(node->lhs);
    load(node->ty);
    return;
  case ND_ADDR:
    gen_addr(node->lhs);
    return;
  case ND_ASSIGN:
    gen_addr(node->lhs);
    push();
    gen_expr(node->rhs);

    if (node->lhs->kind == ND_MEMBER && node->lhs->member->is_bitfield) {
      Member *mem = node->lhs->member;
      long mask = (mem->bit_width >= 32) ? 0xffffffffL
                                         : ((1L << mem->bit_width) - 1);

      // Keep the assigned value (it is the result of the expression).
      push();

      // d1 = (value & mask) << bit_offset
      if (mem->bit_width < 32)
        println("  and.l #%ld,d0", imm32(mask));
      shift_const("lsl", mem->bit_offset);
      println("  move.l d0,d1");

      // d0 = (current unit & ~(mask << bit_offset)) | d1
      println("  move.l 4(sp),a0");
      load_unit(mem->bf_size);
      println("  and.l #%ld,d0", imm32(~(mask << mem->bit_offset)));
      println("  or.l d1,d0");
      println("  move.l 4(sp),a0");
      if (mem->bf_size == 1)
        println("  move.b d0,(a0)");
      else if (mem->bf_size == 2)
        println("  move.w d0,(a0)");
      else
        println("  move.l d0,(a0)");

      pop("d0");
      println("  addq.l #4,sp");
      depth -= 4;
      return;
    }

    store(node->ty);
    return;
  case ND_STMT_EXPR:
    for (Node *n = node->body; n; n = n->next)
      gen_stmt(n);
    return;
  case ND_COMMA:
    gen_expr(node->lhs);
    gen_expr(node->rhs);
    return;
  case ND_CAST:
    gen_expr(node->lhs);
    cast(node->lhs->ty, node->ty);
    return;
  case ND_MEMZERO: {
    int size = node->var->ty->size;
    int off = node->var->offset;
    int align = node->var->align;
    int unit = 1;
    char sfx = 'b';
    if (align >= 2 && size % 2 == 0) {
      unit = 2;
      sfx = 'w';
      if (size % 4 == 0) {
        unit = 4;
        sfx = 'l';
      }
    }
    if (size <= 16 * unit) {
      for (int i = 0; i < size; i += unit)
        println("  clr.%c %d(a6)", sfx, off + i);
    } else {
      int c = count();
      println("  lea %d(a6),a0", off);
      println("  move.l #%d,d1", size / unit);
      println("__Lmz%d:", c);
      println("  clr.%c (a0)+", sfx);
      println("  subq.l #1,d1");
      println("  bne __Lmz%d", c);
    }
    return;
  }
  case ND_COND: {
    int c = count();
    gen_expr(node->cond);
    cmp_zero(node->cond->ty);
    println("  beq __Lc_else_%d", c);
    gen_expr(node->then);
    println("  bra __Lc_end_%d", c);
    println("__Lc_else_%d:", c);
    gen_expr(node->els);
    println("__Lc_end_%d:", c);
    return;
  }
  case ND_NOT:
    gen_expr(node->lhs);
    cmp_zero(node->lhs->ty);
    println("  seq d0");
    println("  and.l #1,d0");
    return;
  case ND_BITNOT:
    gen_expr(node->lhs);
    println("  not.l d0");
    return;
  case ND_LOGAND: {
    int c = count();
    gen_expr(node->lhs);
    cmp_zero(node->lhs->ty);
    println("  beq __Lc_false_%d", c);
    gen_expr(node->rhs);
    cmp_zero(node->rhs->ty);
    println("  beq __Lc_false_%d", c);
    println("  moveq #1,d0");
    println("  bra __Lc_end_%d", c);
    println("__Lc_false_%d:", c);
    println("  moveq #0,d0");
    println("__Lc_end_%d:", c);
    return;
  }
  case ND_LOGOR: {
    int c = count();
    gen_expr(node->lhs);
    cmp_zero(node->lhs->ty);
    println("  bne __Lc_true_%d", c);
    gen_expr(node->rhs);
    cmp_zero(node->rhs->ty);
    println("  bne __Lc_true_%d", c);
    println("  moveq #0,d0");
    println("  bra __Lc_end_%d", c);
    println("__Lc_true_%d:", c);
    println("  moveq #1,d0");
    println("__Lc_end_%d:", c);
    return;
  }
  case ND_FUNCALL: {
    if (node->lhs->kind == ND_VAR && !strcmp(node->lhs->var->name, "alloca")) {
      gen_expr(node->args);
      builtin_alloca();
      return;
    }

    // Total size of the argument area.
    int stack = 0;
    for (Node *arg = node->args; arg; arg = arg->next)
      stack += arg_slot(arg->ty);

    int saved_depth = depth;
    push_args2(node->args);

    bool direct = node->lhs->kind == ND_VAR && !node->lhs->var->is_local &&
                  node->lhs->var->is_function;
    if (!direct) {
      gen_expr(node->lhs);
      println("  move.l d0,a1");
    }

    // The struct-return buffer address travels in a0.
    if (node->ret_buffer)
      println("  lea %d(a6),a0", node->ret_buffer->offset);

    if (direct) {
      ref_sym(node->lhs->var->name);
      println("  jsr %s", node->lhs->var->name);
    } else {
      println("  jsr (a1)");
    }
    sp_add(stack);
    depth = saved_depth;

    // The upper bits of d0 may contain garbage for small return types.
    switch (node->ty->kind) {
    case TY_BOOL:
      println("  and.l #255,d0");
      return;
    case TY_CHAR:
      if (node->ty->is_unsigned) {
        println("  and.l #255,d0");
      } else {
        println("  ext.w d0");
        println("  ext.l d0");
      }
      return;
    case TY_SHORT:
      if (node->ty->is_unsigned)
        println("  and.l #65535,d0");
      else
        println("  ext.l d0");
      return;
    }

    if (node->ret_buffer) {
      println("  lea %d(a6),a0", node->ret_buffer->offset);
      println("  move.l a0,d0");
    }
    return;
  }
  case ND_LABEL_VAL:
    println("  move.l #%s,d0", node->unique_label);
    return;
  case ND_CAS:
  case ND_EXCH:
    error_tok(node->tok, "atomic operations are not supported on the 68000");
  }

  // Binary operators. Operands are at most 32 bits here.
  bool uns = node->lhs->ty->is_unsigned || node->lhs->ty->base;

  // Fast path: the right operand is a small integer constant.
  if (node->rhs->kind == ND_NUM && is_integer(node->rhs->ty) &&
      node->rhs->ty->size <= 4) {
    long n = imm32(node->rhs->val);

    switch (node->kind) {
    case ND_ADD:
      gen_expr(node->lhs);
      add_const(n);
      return;
    case ND_SUB:
      gen_expr(node->lhs);
      add_const(-n);
      return;
    case ND_MUL:
      if (is_pow2(n)) {
        gen_expr(node->lhs);
        shift_const("lsl", log2l(n));
        return;
      }
      break;
    case ND_DIV:
      if (node->ty->is_unsigned && is_pow2(n)) {
        gen_expr(node->lhs);
        shift_const("lsr", log2l(n));
        return;
      }
      break;
    case ND_MOD:
      if (node->ty->is_unsigned && is_pow2(n)) {
        gen_expr(node->lhs);
        println("  and.l #%ld,d0", n - 1);
        return;
      }
      break;
    case ND_BITAND:
      gen_expr(node->lhs);
      println("  and.l #%ld,d0", n);
      return;
    case ND_BITOR:
      gen_expr(node->lhs);
      println("  or.l #%ld,d0", n);
      return;
    case ND_BITXOR:
      gen_expr(node->lhs);
      println("  eor.l #%ld,d0", n);
      return;
    case ND_SHL:
      gen_expr(node->lhs);
      shift_const("lsl", n & 31);
      return;
    case ND_SHR:
      gen_expr(node->lhs);
      shift_const(node->lhs->ty->is_unsigned ? "lsr" : "asr", n & 31);
      return;
    case ND_EQ:
    case ND_NE:
    case ND_LT:
    case ND_LE:
      gen_expr(node->lhs);
      println("  cmp.l #%ld,d0", n);
      goto set_cc;
    }
  }

  gen_expr(node->rhs);
  push();

  switch (node->kind) {
  case ND_MUL:
    gen_expr(node->lhs);
    libcall2("__mulsi3");
    return;
  case ND_DIV:
    gen_expr(node->lhs);
    libcall2(node->ty->is_unsigned ? "__udivsi3" : "__divsi3");
    return;
  case ND_MOD:
    gen_expr(node->lhs);
    libcall2(node->ty->is_unsigned ? "__umodsi3" : "__modsi3");
    return;
  }

  gen_expr(node->lhs);
  pop("d1");

  switch (node->kind) {
  case ND_ADD:
    println("  add.l d1,d0");
    return;
  case ND_SUB:
    println("  sub.l d1,d0");
    return;
  case ND_BITAND:
    println("  and.l d1,d0");
    return;
  case ND_BITOR:
    println("  or.l d1,d0");
    return;
  case ND_BITXOR:
    println("  eor.l d1,d0");
    return;
  case ND_EQ:
  case ND_NE:
  case ND_LT:
  case ND_LE:
    println("  cmp.l d1,d0");
    goto set_cc;
  case ND_SHL:
    println("  lsl.l d1,d0");
    return;
  case ND_SHR:
    if (node->lhs->ty->is_unsigned)
      println("  lsr.l d1,d0");
    else
      println("  asr.l d1,d0");
    return;
  }

  error_tok(node->tok, "invalid expression");

set_cc:
  if (node->kind == ND_EQ)
    println("  seq d0");
  else if (node->kind == ND_NE)
    println("  sne d0");
  else if (node->kind == ND_LT)
    println(uns ? "  scs d0" : "  slt d0");
  else
    println(uns ? "  sls d0" : "  sle d0");
  println("  and.l #1,d0");
}

static void gen_stmt(Node *node) {
  switch (node->kind) {
  case ND_IF: {
    int c = count();
    gen_expr(node->cond);
    cmp_zero(node->cond->ty);
    println("  beq __Lc_else_%d", c);
    gen_stmt(node->then);
    println("  bra __Lc_end_%d", c);
    println("__Lc_else_%d:", c);
    if (node->els)
      gen_stmt(node->els);
    println("__Lc_end_%d:", c);
    return;
  }
  case ND_FOR: {
    int c = count();
    if (node->init)
      gen_stmt(node->init);
    println("__Lc_begin_%d:", c);
    if (node->cond) {
      gen_expr(node->cond);
      cmp_zero(node->cond->ty);
      println("  beq %s", node->brk_label);
    }
    gen_stmt(node->then);
    println("%s:", node->cont_label);
    if (node->inc)
      gen_expr(node->inc);
    println("  bra __Lc_begin_%d", c);
    println("%s:", node->brk_label);
    return;
  }
  case ND_DO: {
    int c = count();
    println("__Lc_begin_%d:", c);
    gen_stmt(node->then);
    println("%s:", node->cont_label);
    gen_expr(node->cond);
    cmp_zero(node->cond->ty);
    println("  bne __Lc_begin_%d", c);
    println("%s:", node->brk_label);
    return;
  }
  case ND_SWITCH:
    gen_expr(node->cond);

    for (Node *n = node->case_next; n; n = n->case_next) {
      if (n->begin == n->end) {
        println("  cmp.l #%ld,d0", imm32(n->begin));
        println("  beq %s", n->label);
        continue;
      }

      // [GNU] Case ranges
      println("  move.l d0,d1");
      println("  sub.l #%ld,d1", imm32(n->begin));
      println("  cmp.l #%ld,d1", imm32(n->end - n->begin));
      println("  bls %s", n->label);
    }

    if (node->default_case)
      println("  bra %s", node->default_case->label);

    println("  bra %s", node->brk_label);
    gen_stmt(node->then);
    println("%s:", node->brk_label);
    return;
  case ND_CASE:
    println("%s:", node->label);
    gen_stmt(node->lhs);
    return;
  case ND_BLOCK:
    for (Node *n = node->body; n; n = n->next)
      gen_stmt(n);
    return;
  case ND_GOTO:
    println("  bra %s", node->unique_label);
    return;
  case ND_GOTO_EXPR:
    gen_expr(node->lhs);
    println("  move.l d0,a0");
    println("  jmp (a0)");
    return;
  case ND_LABEL:
    println("%s:", node->unique_label);
    gen_stmt(node->lhs);
    return;
  case ND_RETURN:
    if (node->lhs) {
      gen_expr(node->lhs);
      Type *ty = node->lhs->ty;

      switch (ty->kind) {
      case TY_STRUCT:
      case TY_UNION:
        // Copy the struct to the caller's buffer (address saved from a0).
        println("  move.l d0,a1");
        println("  move.l %d(a6),a0", current_fn->ret_ptr->offset);
        emit_copy("a1", "a0", ty->size, ty->align);
        println("  move.l %d(a6),d0", current_fn->ret_ptr->offset);
        break;
      }
    }

    println("  bra __Lret_%s", current_fn->name);
    return;
  case ND_EXPR_STMT:
    gen_expr(node->lhs);
    return;
  case ND_ASM:
    println("  %s", node->asm_str);
    return;
  }

  error_tok(node->tok, "invalid statement");
}

// Assign offsets to parameters (positive, from a6) and locals (negative).
static void assign_lvar_offsets(Obj *prog) {
  for (Obj *fn = prog; fn; fn = fn->next) {
    if (!fn->is_function)
      continue;

    // Arguments start at 8(a6): saved a6 at 0(a6), return address at 4(a6).
    int top = 8;
    int bottom = 0;

    for (Obj *var = fn->params; var; var = var->next) {
      int size = var->ty->size;
      int slot = align_to(size, 4);

      // char/short sit at the end of their 4-byte slot (big-endian).
      if (var->ty->kind != TY_STRUCT && var->ty->kind != TY_UNION) {
        if (size == 1)
          var->offset = top + 3;
        else if (size == 2)
          var->offset = top + 2;
        else
          var->offset = top;
      } else {
        var->offset = top;
      }
      top += slot;
    }

    for (Obj *var = fn->locals; var; var = var->next) {
      if (var->offset)
        continue;

      bottom += var->ty->size;
      bottom = align_to(bottom, MAX(var->align, 2));
      var->offset = -bottom;
    }

    fn->stack_size = align_to(bottom, 4);
    if (fn->stack_size > 32000)
      error("%s: stack frame too large for the 68000 (%d bytes)", fn->name,
            fn->stack_size);
  }
}

static void emit_bytes(unsigned char *data, int from, int to) {
  int n = 0;
  for (int i = from; i < to; i++) {
    if (n == 0)
      fprintf(output_file, "  dc.b %d", data[i]);
    else
      fprintf(output_file, ",%d", data[i]);
    if (++n == 16) {
      fprintf(output_file, "\n");
      n = 0;
    }
  }
  if (n)
    fprintf(output_file, "\n");
}

static void emit_data(Obj *prog) {
  for (Obj *var = prog; var; var = var->next) {
    if (var->is_function || !var->is_definition)
      continue;

    if (var->is_tls)
      error("%s: thread-local variables are not supported", var->name);
    check_type(var->ty, NULL);

    def_sym(var->name);
    if (!var->is_static)
      println("  xdef %s", var->name);

    // Data with an initializer
    if (var->init_data) {
      println("  section .data,data");
      if (var->align >= 2)
        println("  even");
      println("%s:", var->name);

      Relocation *rel = var->rel;
      int pos = 0;
      while (pos < var->ty->size) {
        if (rel && rel->offset == pos) {
          ref_sym(*rel->label);
          println("  dc.l %s%+ld", *rel->label, rel->addend);
          rel = rel->next;
          pos += 4;
        } else {
          int end = pos;
          while (end < var->ty->size && !(rel && rel->offset == end))
            end++;
          emit_bytes((unsigned char *)var->init_data, pos, end);
          pos = end;
        }
      }
      continue;
    }

    // Zero-initialized data
    println("  section .bss,bss");
    if (var->align >= 2)
      println("  even");
    println("%s:", var->name);
    println("  ds.b %d", var->ty->size);
  }
}

static void emit_text(Obj *prog) {
  for (Obj *fn = prog; fn; fn = fn->next) {
    if (!fn->is_function || !fn->is_definition)
      continue;

    // No code is emitted for "static inline" functions
    // if no one is referencing them.
    if (!fn->is_live)
      continue;

    def_sym(fn->name);
    if (!fn->is_static)
      println("  xdef %s", fn->name);

    println("  section .text,code");
    println("  even");
    println("%s:", fn->name);
    current_fn = fn;

    // Prologue
    if (fn->stack_size > 0)
      println("  link a6,#-%d", fn->stack_size);
    else
      println("  link a6,#0");
    println("  move.l sp,%d(a6)", fn->alloca_bottom->offset);

    // A struct is returned through the hidden pointer received in a0.
    if (fn->ret_ptr)
      println("  move.l a0,%d(a6)", fn->ret_ptr->offset);

    // Emit code
    gen_stmt(fn->body);
    assert(depth == 0);

    // [https://www.sigbus.info/n1570#5.1.2.2.3p1] Reaching the end of
    // main is equivalent to returning 0.
    if (strcmp(fn->name, "main") == 0)
      println("  moveq #0,d0");

    // Epilogue
    println("__Lret_%s:", fn->name);
    println("  unlk a6");
    println("  rts");
  }
}

void codegen(Obj *prog, FILE *out) {
  // The body is generated first into a memory buffer, because the
  // `xref` lines (symbols that this file only references) are only
  // known once everything has been generated.
  char *buf;
  size_t buflen;
  output_file = open_memstream(&buf, &buflen);

  assign_lvar_offsets(prog);
  emit_data(prog);
  emit_text(prog);
  fclose(output_file);

  output_file = out;
  println("; generated by chibicc68k (vasm, Motorola syntax)");
  for (int i = 0; i < refs.capacity; i++) {
    HashEntry *ent = &refs.buckets[i];
    if (ent->key && ent->key != (char *)-1 &&
        !hashmap_get2(&defs, ent->key, ent->keylen))
      println("  xref %.*s", ent->keylen, ent->key);
  }
  fwrite(buf, buflen, 1, out);
  free(buf);
}
