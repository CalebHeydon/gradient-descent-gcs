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

    gcs_node_t *point1;
    gcs_create_point(&point1, 0, 0);
    point1->fixed[0] = true;
    point1->fixed[1] = true;
    gcs_graph_add_node(&graph, point1);

    gcs_node_t *point2;
    gcs_create_point(&point2, 1.0, 1.0);
    gcs_graph_add_node(&graph, point2);

    gcs_constraint_t *distance_constraint;
    gcs_graph_add_constraint(&graph, GCS_CONSTRAINT_TYPE_DISTANCE, 1.0, point1, point2, &distance_constraint);

    int total_dof = gcs_dof_analysis(&graph);
    printf("point1 dof: %d\n", point1->dof);
    printf("point2 dof: %d\n", point2->dof);
    printf("total dof: %d\n", total_dof);

    double **parameters;
    size_t num_parameters;
    gcs_graph_get_parameters(&graph, &parameters, &num_parameters);

    double *gradient = malloc(sizeof(double) * num_parameters);
    if (!gradient)
        return -1;
    gcs_compute_gradient(&graph, parameters, num_parameters, gradient);

    printf("gradient: %f, %f\n", gradient[0], gradient[1]);

    return 0;
}
