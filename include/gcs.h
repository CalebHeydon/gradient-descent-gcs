/*
Copyright (c) 2022, Caleb Heydon
All rights reserved.
*/

#pragma once

#include <stdbool.h>
#include <stddef.h>

#define GCS_NODE_TYPE_POINT 0

typedef struct gcs_node
{
    int type;
    double *values;
    bool *fixed;
    int dof;
} gcs_node_t;

int gcs_destroy_node(gcs_node_t *node);
int gcs_create_point(gcs_node_t **node, double x, double y);

#define GCS_CONSTRAINT_TYPE_DISTANCE 0
#define GCS_CONSTRAINT_TYPE_DISTANCE_X 1
#define GCS_CONSTRAINT_TYPE_DISTANCE_Y 2

typedef struct gcs_constraint
{
    int type;
    double value;
    gcs_node_t *nodes[2];
    struct gcs_constraint *next;
} gcs_constraint_t;

typedef struct gcs_graph_node
{
    gcs_node_t *node;
    struct gcs_graph_node *next;
} gcs_graph_node_t;

typedef struct gcs_graph
{
    gcs_graph_node_t *nodes;
    gcs_constraint_t *edges;
} gcs_graph_t;

bool gcs_graph_contains_node(gcs_graph_t *graph, gcs_node_t *node);
int gcs_graph_add_node(gcs_graph_t *graph, gcs_node_t *node);
int gcs_graph_delete_node(gcs_graph_t *graph, gcs_node_t *node);
int gcs_graph_add_constraint(gcs_graph_t *graph, int type, double value, gcs_node_t *node1, gcs_node_t *node2, gcs_constraint_t **constraint);
int gcs_graph_delete_constraint(gcs_graph_t *graph, gcs_constraint_t *constraint);
int gcs_graph_get_parameters(gcs_graph_t *graph, double ***parameters, size_t *num_parameters);

int gcs_dof_analysis(gcs_graph_t *graph);

#define GCS_EPSILON 0.0001

double gcs_error(gcs_graph_t *graph);
int gcs_compute_gradient(gcs_graph_t *graph, double **parameters, size_t num_parameters, double *gradient);
