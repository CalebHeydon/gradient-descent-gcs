/*
BSD 2-Clause License

Copyright (c) 2023, Caleb Heydon
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>

#define GCS_NODE_TYPE_POINT 0
#define GCS_NODE_TYPE_LINE 1
#define GCS_NODE_TYPE_CIRCLE 2

typedef struct gcs_node
{
    int type;
    double *values;
    bool *fixed;
    int dof;
} gcs_node_t;

int gcs_node_destroy(gcs_node_t *node);
int gcs_node_create_point(gcs_node_t **node, double x, double y);
int gcs_node_create_line(gcs_node_t **node, double theta, double distance);
int gcs_node_create_circle(gcs_node_t **node, double x, double y, double r);

#define GCS_CONSTRAINT_TYPE_DISTANCE 0
#define GCS_CONSTRAINT_TYPE_DISTANCE_X 1
#define GCS_CONSTRAINT_TYPE_DISTANCE_Y 2
#define GCS_CONSTRAINT_TYPE_ANGLE 3
#define GCS_CONSTRAINT_TYPE_ON_EDGE 4
#define GCS_CONSTRAINT_TYPE_TANGENT 5

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
int gcs_graph_destroy(gcs_graph_t *graph);

#define GCS_PARAMETER_TYPE_DISTANCE 0
#define GCS_PARAMETER_TYPE_ANGLE 1

typedef struct gcs_parameter
{
    int type;
    double *value;
} gcs_parameter_t;

int gcs_graph_get_parameters(gcs_graph_t *graph, gcs_parameter_t **parameters, size_t *num_parameters);

int gcs_dof_analysis(gcs_graph_t *graph);

#define GCS_EPSILON 0.00001
#define GCS_PI 3.14159265358979323846

double gcs_normalize_angle(double angle);

double gcs_distance_two_points(gcs_node_t *point1, gcs_node_t *point2);
double gcs_distance_line_point(gcs_node_t *line, gcs_node_t *point);
double gcs_angle_two_lines(gcs_node_t *line1, gcs_node_t *line2);
double gcs_error(gcs_graph_t *graph);
int gcs_gradient(gcs_graph_t *graph, gcs_parameter_t *parameters, size_t num_parameters, double *gradient);
int gcs_solve(gcs_graph_t *graph, double rate, int max_iterations);

#ifdef __cplusplus
}
#endif
