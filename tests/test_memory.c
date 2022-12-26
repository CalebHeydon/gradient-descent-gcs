/*
Copyright (c) 2022, Caleb Heydon
All rights reserved.
*/

#include <gcs.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    printf("testing gcs_create_point...");
    gcs_node *node;
    if (gcs_create_point(&node, 2, 3))
    {
        printf("failed\n");
        return -1;
    }
    node->fixed[0] = false;
    node->fixed[1] = true;
    if (node->values[0] != 2 || node->values[1] != 3 || node->fixed[0] || !node->fixed[1])
    {
        printf("failed\n");
        return -1;
    }
    printf("ok\n");

    printf("testing gcs_destroy_node...");
    if (gcs_destroy_node(node))
    {
        printf("failed\n");
        return -1;
    }
    printf("ok\n");

    return 0;
}
