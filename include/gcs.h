/*
Copyright (c) 2022, Caleb Heydon
All rights reserved.
*/

#pragma once

#include <stdbool.h>

#define GCS_NODE_TYPE_POINT 0

typedef struct
{
    int type;
    double *values;
    bool *fixed;
} gcs_node;

#define GCS_CONSTRAINT_TYPE_DISTANCE 0
#define GCS_CONSTRAINT_TYPE_DISTANCE_X 1
#define GCS_CONSTRAINT_TYPE_DISTANCE_Y 2

typedef struct
{
    int type;
    double value;
} gcs_constraint;

int gcs_destroy_node(gcs_node *node);
int gcs_create_point(gcs_node **node, double x, double y);
