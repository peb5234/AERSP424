#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include "sim.h"

int main(int argc, char **argv)
{
    // DEFAULT SIMULATION PARAMETER
    double max_time = 20;
    double dt = 0.01;

    // Loop through all user-provided arguments and process the options
    char c;
    while ((c = getopt(argc, argv, "t:d:h")) != -1)
    {
        switch (c)
        {
        case 't':
            printf("Max Time has option %s\n", optarg);
            max_time = atof(optarg);
            break;
        case 'd':
            printf("Delta Time has option %s\n", optarg);
            dt = atof(optarg);
            break;
        case 'h':
            printf("main [options] \n");
            printf("-h    Prints this help message\n");
            printf("-t    Sets max sim time as a double\n");
            printf("-d    Sets dt in sim as a double\n");
            return -1;
        }
    }

    // optind is for the extra arguments which are not parsed
    for (; optind < argc; optind++)
    {
        printf("extra arguments : %s\n", argv[optind]);
    }
    // Perform the simulation
    simulate(dt, max_time);
}
