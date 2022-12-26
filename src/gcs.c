/*
Copyright (c) 2022, Caleb Heydon
All rights reserved.
*/

#include "gcs.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

int gcs_destroy_node(gcs_node *node)
{
    if (!node)
        return -1;

    free(node);
    return 0;
}

int gcs_create_point(gcs_node **node, double x, double y)
{
    if (!node)
        return -1;

    *node = malloc(sizeof(**node) + sizeof(double) * 2 + sizeof(bool) * 2);
    if (!*node)
        return -1;
    memset(*node, 0, sizeof(**node) + sizeof(double) * 2 + sizeof(bool) * 2);

    (*node)->values = (uint8_t *)*node + sizeof(**node);
    (*node)->fixed = (uint8_t *)*node + sizeof(**node) + sizeof(double) * 2;

    (*node)->values[0] = x;
    (*node)->values[1] = y;

    return 0;
}
