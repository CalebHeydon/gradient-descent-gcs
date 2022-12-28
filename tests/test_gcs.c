/*
Copyright (c) 2022, Caleb Heydon
All rights reserved.
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
    y_axis->fixed[0] = true;
    y_axis->fixed[1] = true;
    gcs_graph_add_node(&graph, y_axis);

    gcs_node_t *point;
    gcs_node_create_point(&point, 1.0, 1.0);
    gcs_graph_add_node(&graph, point);

    gcs_constraint_t *distance_constraint;
    gcs_graph_add_constraint(&graph, GCS_CONSTRAINT_TYPE_DISTANCE, 1.0, origin, point, &distance_constraint);

    int total_dof = gcs_dof_analysis(&graph);
    printf("point dof: %d\n", point->dof);
    printf("total dof: %d\n", total_dof);

    int iterations = gcs_solve(&graph, 0.01, 1000);
    printf("iterations: %d\n", iterations);
    printf("point: %f, %f\n", point->values[0], point->values[1]);

    gcs_graph_destroy(&graph);
    gcs_node_destroy(point);
    gcs_node_destroy(y_axis);
    gcs_node_destroy(x_axis);
    gcs_node_destroy(origin);

    return 0;
}
