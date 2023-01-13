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

#include <gcs.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    gcs_graph_t graph = {0};

    gcs_node_t *origin;
    gcs_node_create_point(&origin, 0, 0);
    origin->fixed[0] = true;
    origin->fixed[1] = true;
    gcs_graph_add_node(&graph, origin);

    gcs_node_t *x_axis;
    gcs_node_create_line(&x_axis, 0, 0);
    x_axis->fixed[0] = true;
    x_axis->fixed[1] = true;
    gcs_graph_add_node(&graph, x_axis);

    gcs_node_t *y_axis;
    gcs_node_create_line(&y_axis, GCS_PI / 2, 0);
    y_axis->fixed[1] = true;
    gcs_graph_add_node(&graph, y_axis);

    gcs_node_t *circle;
    gcs_node_create_circle(&circle, 1.0, 1.0, 0.5);
    circle->fixed[2] = true;
    gcs_graph_add_node(&graph, circle);

    gcs_node_t *line;
    gcs_node_create_line(&line, 0, 0);
    gcs_graph_add_node(&graph, line);

    gcs_constraint_t *angle_constraint;
    gcs_graph_add_constraint(&graph, GCS_CONSTRAINT_TYPE_ANGLE, GCS_PI / 2.0, x_axis, y_axis, &angle_constraint);

    gcs_constraint_t *distance_constraint;
    gcs_graph_add_constraint(&graph, GCS_CONSTRAINT_TYPE_DISTANCE, 1.0, origin, circle, &distance_constraint);

    gcs_constraint_t *coincident_constraint;
    gcs_graph_add_constraint(&graph, GCS_CONSTRAINT_TYPE_DISTANCE, 0, y_axis, circle, &coincident_constraint);

    gcs_constraint_t *tangent_constraint;
    gcs_graph_add_constraint(&graph, GCS_CONSTRAINT_TYPE_TANGENT, 0, circle, line, &tangent_constraint);

    gcs_constraint_t *colinear_constraint;
    gcs_graph_add_constraint(&graph, GCS_CONSTRAINT_TYPE_DISTANCE, 0, line, origin, &colinear_constraint);

    int total_dof = gcs_dof_analysis(&graph);
    printf("circle dof: %d\n", circle->dof);
    printf("line dof: %d\n", line->dof);
    printf("system dof: %d\n", total_dof);

    int iterations = gcs_solve(&graph, 0.01, 10000);
    printf("iterations: %d\n", iterations);
    printf("circle: %f, %f, %f\n", circle->values[0], circle->values[1], circle->values[2]);
    printf("line: %f, %f\n", line->values[0], line->values[1]);

    gcs_graph_destroy(&graph);
    gcs_node_destroy(line);
    gcs_node_destroy(circle);
    gcs_node_destroy(y_axis);
    gcs_node_destroy(x_axis);
    gcs_node_destroy(origin);

    return 0;
}
