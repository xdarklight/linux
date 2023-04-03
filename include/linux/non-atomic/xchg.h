/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_NON_ATOMIC_XCHG_H
#define _LINUX_NON_ATOMIC_XCHG_H

/**
 * __xchg - set variable pointed by @ptr to @val, return old value
 * @ptr: pointer to affected variable
 * @val: value to be written
 *
 * This is non-atomic variant of xchg.
 */
#define __xchg(ptr, val) ({		\
	__auto_type __ptr = ptr;	\
	__auto_type __t = *__ptr;	\
	*__ptr = (val);			\
	__t;				\
})

#endif
