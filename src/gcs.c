/*
Copyright (c) 2022, Caleb Heydon
All rights reserved.
*/

#include "gcs.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

int gcs_node_destroy(gcs_node_t *node)
{
    if (!node)
        return -1;

    free(node);
    return 0;
}

int gcs_node_create_point(gcs_node_t **node, double x, double y)
{
    if (!node)
        return -1;

    *node = malloc(sizeof(**node) + sizeof(double) * 2 + sizeof(bool) * 2);
    if (!*node)
        return -1;
    memset(*node, 0, sizeof(**node) + sizeof(double) * 2 + sizeof(bool) * 2);

    (*node)->values = (void *)((uint8_t *)*node + sizeof(**node));
    (*node)->fixed = (void *)((uint8_t *)*node + sizeof(**node) + sizeof(double) * 2);

    (*node)->values[0] = x;
    (*node)->values[1] = y;

    (*node)->dof = 2;

    return 0;
}

int gcs_node_create_line(gcs_node_t **node, double theta, double distance)
{
    if (!node)
        return -1;

    *node = malloc(sizeof(**node) + sizeof(double) * 2 + sizeof(bool) * 2);
    if (!*node)
        return -1;
    memset(*node, 0, sizeof(**node) + sizeof(double) * 2 + sizeof(bool) * 2);

    (*node)->values = (void *)((uint8_t *)*node + sizeof(**node));
    (*node)->fixed = (void *)((uint8_t *)*node + sizeof(**node) + sizeof(double) * 2);

    (*node)->values[0] = theta;
    (*node)->values[1] = distance;

    (*node)->dof = 2;

    return 0;
}

bool gcs_graph_contains_node(gcs_graph_t *graph, gcs_node_t *node)
{
    if (!graph || !graph->nodes || !node)
        return false;

    for (gcs_graph_node_t *graph_node = graph->nodes; graph_node; graph_node = graph_node->next)
        if (graph_node->node == node)
            return true;

    return false;
}

int gcs_graph_add_node(gcs_graph_t *graph, gcs_node_t *node)
{
    if (!graph || !node || gcs_graph_contains_node(graph, node))
        return -1;

    gcs_graph_node_t *graph_node = malloc(sizeof(*graph_node));
    if (!graph_node)
        return -1;
    memset(graph_node, 0, sizeof(*graph_node));
    graph_node->node = node;

    if (graph->nodes)
        graph_node->next = graph->nodes;
    graph->nodes = graph_node;

    return 0;
}

int gcs_graph_delete_node(gcs_graph_t *graph, gcs_node_t *node)
{
    if (!graph || !node || !gcs_graph_contains_node(graph, node))
        return -1;

    gcs_graph_node_t **next_node = &graph->nodes;
    for (gcs_graph_node_t *graph_node = graph->nodes; graph_node; graph_node = graph_node->next)
    {
        if (graph_node->node == node)
        {
            *next_node = graph_node->next;
            free(graph_node);
            break;
        }

        next_node = &graph_node->next;
    }

    gcs_constraint_t **next_constraint = &graph->edges;
    for (gcs_constraint_t *constraint = graph->edges; constraint; constraint = constraint->next)
    {
        if (constraint->nodes[0] == node || constraint->nodes[1] == node)
        {
            *next_constraint = constraint->next;
            free(constraint);
            break;
        }

        next_constraint = &constraint->next;
    }

    return 0;
}

int gcs_graph_add_constraint(gcs_graph_t *graph, int type, double value, gcs_node_t *node1, gcs_node_t *node2, gcs_constraint_t **constraint)
{
    if (!graph || !node1 || !node2 || !constraint)
        return -1;

    *constraint = malloc(sizeof(**constraint));
    if (!*constraint)
        return -1;
    memset(*constraint, 0, sizeof(**constraint));

    (*constraint)->type = type;
    (*constraint)->value = value;
    (*constraint)->nodes[0] = node1;
    (*constraint)->nodes[1] = node2;
    (*constraint)->next = graph->edges;

    graph->edges = *constraint;
    return 0;
}

int gcs_graph_delete_constraint(gcs_graph_t *graph, gcs_constraint_t *constraint)
{
    if (!graph || !constraint)
        return -1;

    gcs_constraint_t **next_constraint = &graph->edges;
    for (gcs_constraint_t *graph_constraint = graph->edges; graph_constraint; graph_constraint = graph_constraint->next)
    {
        if (graph_constraint == constraint)
        {
            *next_constraint = graph_constraint->next;
            free(graph_constraint);
            return 0;
        }

        next_constraint = &graph_constraint->next;
    }

    return -1;
}

int gcs_graph_destroy(gcs_graph_t *graph)
{
    gcs_graph_node_t *next_node;
    for (gcs_graph_node_t *graph_node = graph->nodes; graph_node; graph_node = next_node)
    {
        next_node = graph_node->next;

        if (gcs_graph_delete_node(graph, graph_node->node))
            return -1;
    }

    return 0;
}

int gcs_graph_get_parameters(gcs_graph_t *graph, gcs_parameter_t **parameters, size_t *num_parameters)
{
    if (!graph || !parameters || !num_parameters)
        return -1;

    *num_parameters = 0;
    for (gcs_graph_node_t *graph_node = graph->nodes; graph_node; graph_node = graph_node->next)
    {
        gcs_node_t *node = graph_node->node;
        int len = 0;
        switch (node->type)
        {
        case GCS_NODE_TYPE_POINT:
        case GCS_NODE_TYPE_LINE:
            len = 2;
            break;
        default:
            return -1;
        }

        for (int i = 0; i < len; i++)
            *num_parameters += !node->fixed[i];
    }

    *parameters = malloc(sizeof(**parameters) * *num_parameters);
    if (!*parameters)
        return -1;

    int i = 0;
    for (gcs_graph_node_t *graph_node = graph->nodes; graph_node; graph_node = graph_node->next)
    {
        gcs_node_t *node = graph_node->node;
        int len = 0;
        switch (node->type)
        {
        case GCS_NODE_TYPE_POINT:
        case GCS_NODE_TYPE_LINE:
            len = 2;
            break;
        default:
            return -1;
        }

        for (int j = 0; j < len; j++)
            if (!node->fixed[j])
            {
                (*parameters)[i].type = node->type == GCS_NODE_TYPE_LINE && j == 0 ? GCS_PARAMETER_TYPE_ANGLE : GCS_PARAMETER_TYPE_DISTANCE;
                (*parameters)[i].value = node->values + j;
                i++;
            }
    }

    return 0;
}

int gcs_dof_analysis(gcs_graph_t *graph)
{
    if (!graph)
        return -1;

    int original_dof = 0;
    int constraint_dof = 0;

    for (gcs_graph_node_t *graph_node = graph->nodes; graph_node; graph_node = graph_node->next)
    {
        gcs_node_t *node = graph_node->node;

        switch (node->type)
        {
        case GCS_NODE_TYPE_POINT:
        case GCS_NODE_TYPE_LINE:
        default:
            node->dof = 2;
            break;
        }

        int num_values = node->dof;
        for (int i = 0; i < num_values; i++)
            node->dof -= node->fixed[i];

        original_dof += node->dof;

        for (gcs_constraint_t *constraint = graph->edges; constraint; constraint = constraint->next)
            if (constraint->nodes[0] == node || constraint->nodes[1] == node)
                switch (constraint->type)
                {
                case GCS_CONSTRAINT_TYPE_DISTANCE:
                case GCS_CONSTRAINT_TYPE_DISTANCE_X:
                case GCS_CONSTRAINT_TYPE_DISTANCE_Y:
                    node->dof -= 1;
                    constraint_dof += constraint->nodes[0] == node;
                    break;
                default:
                    return -1;
                }
    }

    return original_dof - constraint_dof;
}

double gcs_distance_two_points(gcs_node_t *point1, gcs_node_t *point2)
{
    if (!point1 || !point2)
        return -1.0;

    return sqrt(pow(point2->values[0] - point1->values[0], 2.0) + pow(point2->values[1] - point1->values[1], 2.0));
}

double gcs_distance_line_point(gcs_node_t *line, gcs_node_t *point)
{
    if (!line || !point)
        return -1.0;

    if (line->values[0] == GCS_PI / 2 || line->values[0] == (3 * GCS_PI) / 2)
        return fabs(line->values[1] - point->values[0]);

    double a = tan(line->values[0]);
    double b = -1.0;
    double c = line->values[1] / cos(line->values[0]);

    double x = point->values[0];
    double y = point->values[1];

    return fabs(a * x + b * y + c) / sqrt(pow(a, 2.0) + pow(b, 2.0));
}

double gcs_error(gcs_graph_t *graph)
{
    if (!graph)
        return -1.0;

    double error = 0;

    for (gcs_constraint_t *constraint = graph->edges; constraint; constraint = constraint->next)
        switch (constraint->type)
        {
        case GCS_CONSTRAINT_TYPE_DISTANCE:
            if (constraint->nodes[0]->type == GCS_NODE_TYPE_POINT && constraint->nodes[1]->type == GCS_NODE_TYPE_POINT)
            {
                double distance = gcs_distance_two_points(constraint->nodes[0], constraint->nodes[1]);
                error += fabs(pow(distance - constraint->value, 2.0));
            }
            else if ((constraint->nodes[0]->type == GCS_NODE_TYPE_LINE && constraint->nodes[1]->type == GCS_NODE_TYPE_POINT) || (constraint->nodes[0]->type == GCS_NODE_TYPE_POINT && constraint->nodes[1]->type == GCS_NODE_TYPE_LINE))
            {
                gcs_node_t *line = constraint->nodes[0]->type == GCS_NODE_TYPE_LINE ? constraint->nodes[0] : constraint->nodes[1];
                gcs_node_t *point = constraint->nodes[1]->type == GCS_NODE_TYPE_POINT ? constraint->nodes[1] : constraint->nodes[0];
                double distance = gcs_distance_line_point(line, point);
                error += fabs(pow(distance - constraint->value, 2.0));
            }
            else
                return -1.0;
            break;
        case GCS_CONSTRAINT_TYPE_DISTANCE_X:
            if (constraint->nodes[0]->type == GCS_NODE_TYPE_POINT && constraint->nodes[1]->type == GCS_NODE_TYPE_POINT)
            {
                double distance = gcs_distance_two_points(constraint->nodes[0], constraint->nodes[1]);
                error += fabs(pow(distance - constraint->value, 2.0));
            }
            else
                return -1.0;
            break;
        case GCS_CONSTRAINT_TYPE_DISTANCE_Y:
            if (constraint->nodes[0]->type == GCS_NODE_TYPE_POINT && constraint->nodes[1]->type == GCS_NODE_TYPE_POINT)
            {
                double distance = gcs_distance_two_points(constraint->nodes[0], constraint->nodes[1]);
                error += fabs(pow(distance - constraint->value, 2.0));
            }
            else
                return -1.0;
            break;
        default:
            return -1.0;
        }

    return error;
}

double gcs_normalize_angle(double angle)
{
    while (angle > 2 * GCS_PI)
        angle -= 2 * GCS_PI;
    while (angle < 0)
        angle += 2 * GCS_PI;

    return angle;
}

int gcs_gradient(gcs_graph_t *graph, gcs_parameter_t *parameters, size_t num_parameters, double *gradient)
{
    if (!graph || !parameters || !gradient)
        return -1;

    for (int i = 0; i < num_parameters; i++)
        if (parameters[i].type == GCS_PARAMETER_TYPE_ANGLE)
            *(parameters[i].value) = gcs_normalize_angle(*(parameters[i].value));

    double *backup = malloc(sizeof(*backup) * num_parameters);
    if (!backup)
        return -1;
    for (int i = 0; i < num_parameters; i++)
        backup[i] = *(parameters[i].value);

    double error = gcs_error(graph);
    if (error < 0)
        return -1;

    for (int i = 0; i < num_parameters; i++)
    {
        *(parameters[i].value) += GCS_EPSILON;
        if (parameters[i].type == GCS_PARAMETER_TYPE_ANGLE)
            *(parameters[i].value) = gcs_normalize_angle(*(parameters[i].value));
        double epsilon_error = gcs_error(graph);
        if (epsilon_error < 0)
            return -1;
        *(parameters[i].value) = backup[i];

        gradient[i] = (epsilon_error - error) / GCS_EPSILON;
    }

    free(backup);

    return 0;
}

int gcs_solve(gcs_graph_t *graph, double rate, int max_iterations)
{
    if (!graph)
        return -1;

    gcs_parameter_t *parameters;
    size_t num_parameters;
    if (gcs_graph_get_parameters(graph, &parameters, &num_parameters))
        return -1;

    double *gradient = malloc(sizeof(*gradient) * num_parameters);
    if (!gradient)
        return -1;

    int i;
    for (i = 0; i < max_iterations; i++)
    {
        if (gcs_gradient(graph, parameters, num_parameters, gradient))
            return -1;

        double sum = 0;
        for (int j = 0; j < num_parameters; j++)
            sum += pow(gradient[j], 2.0);
        double magnitude = sqrt(sum);
        if (magnitude < GCS_EPSILON)
            break;

        for (int j = 0; j < num_parameters; j++)
            *(parameters[j].value) -= rate * gradient[j];
        if (parameters[i].type == GCS_PARAMETER_TYPE_ANGLE)
            *(parameters[i].value) = gcs_normalize_angle(*(parameters[i].value));
    }

    free(gradient);
    free(parameters);

    return i;
}
