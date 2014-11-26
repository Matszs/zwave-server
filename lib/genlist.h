//
//  Created by Praveen Murali Nair on 09/07/2013.
//  Copyright (c) 2013 Praveen M Nair. All rights reserved.
//
// 	Redistribution and use in source and binary forms, with or without
//      modification, are permitted provided that the following conditions are met:
//      * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//      * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//      * Neither the name of the <organization> nor the
//      names of its contributors may be used to endorse or promote products
//      derived from this software without specific prior written permission.
//
//      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//      ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//      WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//      DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//      DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//      (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//      ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//      SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#ifndef _GENLIST_H_
#define _GENLIST_H_

#include <stdio.h>
#include <stdlib.h>

typedef struct _node
{
    struct _node *next;
    struct _node *prev;
}list_node, list_head;

#define INIT_LIST_HEAD(head) { &head, &head }
#define LIST_HEAD(head) list_head head = INIT_LIST_HEAD(head);

#define list_foreach(node, list) for(node = list->next; node != list; node = node->next)

static inline int list_empty(list_head *list)
{
	if ( list->next == list ) return 1;
	else			  return 0;
}
static inline void _list_add(list_node *node, list_node *prev, list_node *next)
{
    prev->next = node;
    next->prev = node;
    node->next = next;
    node->prev = prev;
}

static inline void list_add(list_node *node, list_node *new_node)
{
    _list_add(new_node, node->prev, node);
}

static inline void list_add_head(list_head *list, list_node *node)
{
    _list_add(node, list, list->next);
}

static inline void list_add_tail(list_head *list, list_node *node)
{
    _list_add(node, list->prev, list);
}

static inline list_node* list_pop_front(list_head *list)
{
	list_node *node = list->next;
	list->next = node->next;
	node->next->prev = list;
	return node;
}

static inline list_node* list_front( list_head *list )
{
	if ( list->next == list )
		return NULL;
	else
		return list->next;
}

static inline void list_remove(list_head *list, list_node *node)
{
    list_node *temp = NULL;
    list_foreach( temp, list ) {
        if ( temp == node ) {
            temp->next->prev = temp->prev;
            temp->prev->next = temp->next;
            break;
        }
    }
}

#endif /*_GENLIST_H_*/
