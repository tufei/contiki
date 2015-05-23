/*
 * Contiki Epiphany Port project
 *
 * Copyright (c) 2015, Yufei Yuan (http://github.com/tufei)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \addtogroup Epiphany eCore Contiki Port
 *
 * @{
 */

/**
 * \file   cpu/ecore/mtarch.c
 * \brief  Implementation of multithreading in eCore. To be done.
 * \author Yufei Yuan <http://github.com/tufei>
 * \date   2015-05-23
 */

#include <stdio.h>
#include <stdint.h>
#include "sys/mt.h"

static uint32_t *sp_tmp;
static struct mtarch_thread *running;

/*--------------------------------------------------------------------------*/
void mtarch_init(void)
{
}

/*--------------------------------------------------------------------------*/
static void
mtarch_wrapper(void)
{
  /* call thread function with argument */
  ((void (*)(void *))running->function)((void *)running->data);
}

/*--------------------------------------------------------------------------*/
void
mtarch_start(struct mtarch_thread *t, void (*function)(void *), void *data)
{
  int i;

  for(i = 0; i < MTARCH_STACKSIZE; ++i) {
    t->stack[i] = i;
  }

  t->sp = &t->stack[MTARCH_STACKSIZE - 1];

  *t->sp = (uint32_t)mt_exit;
  --t->sp;

  *t->sp = (uint32_t)mtarch_wrapper;
  --t->sp;

  /* space for registers */
  t->sp -= 4;

  /* store function and argument (used in mtarch_wrapper) */
  t->data = data;
  t->function = function;
}

/*--------------------------------------------------------------------------*/
static void
sw(void)
{
  /* global interrupt disable */
  __asm__("gid");

  __asm__("sub      sp, sp, #256");
  __asm__("strd     r62, [sp, #1]");
  __asm__("strd     r60, [sp, #2]");
  __asm__("strd     r58, [sp, #3]");
  __asm__("strd     r56, [sp, #4]");
  __asm__("strd     r54, [sp, #5]");
  __asm__("strd     r52, [sp, #6]");
  __asm__("strd     r50, [sp, #7]");
  __asm__("strd     r48, [sp, #8]");
  __asm__("strd     r46, [sp, #9]");
  __asm__("strd     r44, [sp, #10]");
  __asm__("strd     r42, [sp, #11]");
  __asm__("strd     r40, [sp, #12]");
  __asm__("strd     r38, [sp, #13]");
  __asm__("strd     r36, [sp, #14]");
  __asm__("strd     r34, [sp, #15]");
  __asm__("strd     r32, [sp, #16]");
  __asm__("strd     r30, [sp, #17]");
  __asm__("strd     r28, [sp, #18]");
  __asm__("strd     r26, [sp, #19]");
  __asm__("strd     r24, [sp, #20]");
  __asm__("strd     r22, [sp, #21]");
  __asm__("strd     r20, [sp, #22]");
  __asm__("strd     r18, [sp, #23]");
  __asm__("strd     r16, [sp, #24]");
  __asm__("strd     r14, [sp, #25]");
  __asm__("strd     r12, [sp, #26]");
  __asm__("strd     r10, [sp, #27]");
  __asm__("strd     r8, [sp, #28]");
  __asm__("strd     r6, [sp, #29]");
  __asm__("strd     r4, [sp, #30]");
  __asm__("strd     r2, [sp, #31]");
  __asm__("strd     r0, [sp, #32]");

  sp_tmp = running->sp;
  __asm__("str      sp, %0" : "=m" (running->sp));
  __asm__("ldr      sp, %0" : : "m" (sp_tmp));

  __asm__("ldrd     r62, [sp, #1]");
  __asm__("ldrd     r60, [sp, #2]");
  __asm__("ldrd     r58, [sp, #3]");
  __asm__("ldrd     r56, [sp, #4]");
  __asm__("ldrd     r54, [sp, #5]");
  __asm__("ldrd     r52, [sp, #6]");
  __asm__("ldrd     r50, [sp, #7]");
  __asm__("ldrd     r48, [sp, #8]");
  __asm__("ldrd     r46, [sp, #9]");
  __asm__("ldrd     r44, [sp, #10]");
  __asm__("ldrd     r42, [sp, #11]");
  __asm__("ldrd     r40, [sp, #12]");
  __asm__("ldrd     r38, [sp, #13]");
  __asm__("ldrd     r36, [sp, #14]");
  __asm__("ldrd     r34, [sp, #15]");
  __asm__("ldrd     r32, [sp, #16]");
  __asm__("ldrd     r30, [sp, #17]");
  __asm__("ldrd     r28, [sp, #18]");
  __asm__("ldrd     r26, [sp, #19]");
  __asm__("ldrd     r24, [sp, #20]");
  __asm__("ldrd     r22, [sp, #21]");
  __asm__("ldrd     r20, [sp, #22]");
  __asm__("ldrd     r18, [sp, #23]");
  __asm__("ldrd     r16, [sp, #24]");
  __asm__("ldrd     r14, [sp, #25]");
  __asm__("ldrd     r12, [sp, #26]");
  __asm__("ldrd     r10, [sp, #27]");
  __asm__("ldrd     r8, [sp, #28]");
  __asm__("ldrd     r6, [sp, #29]");
  __asm__("ldrd     r4, [sp, #30]");
  __asm__("ldrd     r2, [sp, #31]");
  __asm__("ldrd     r0, [sp, #32]");
  __asm__("add      sp, sp, #256");

  /* global interrupt enable */
  __asm__("gie");
}

/*--------------------------------------------------------------------------*/
void
mtarch_exec(struct mtarch_thread *t)
{
  running = t;
  sw();
  running = NULL;
}

/*--------------------------------------------------------------------------*/
void
mtarch_remove(void)
{
}

/*--------------------------------------------------------------------------*/
void
mtarch_yield(void)
{
  sw();
}

/*--------------------------------------------------------------------------*/
void
mtarch_pstop(void)
{
}

/*--------------------------------------------------------------------------*/
void
mtarch_pstart(void)
{
}

/*--------------------------------------------------------------------------*/
void
mtarch_stop(struct mtarch_thread *thread)
{
}

/*--------------------------------------------------------------------------*/
int
mtarch_stack_usage(struct mt_thread *t)
{
  int i;

  for(i = 0; i < MTARCH_STACKSIZE; ++i) {
    if(t->thread.stack[i] != (uint32_t)i) {
      return MTARCH_STACKSIZE - i;
    }
  }

  return MTARCH_STACKSIZE;
}

/*--------------------------------------------------------------------------*/

/** @} */

