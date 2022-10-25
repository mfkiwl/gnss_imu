#include <vector>
#include <iostream>
#include <algorithm> 
#include <ublox_msgs/RxmRAWX.h>
#include <ublox_msgs/NavSAT.h>
#include <ublox_msgs/NavPVT.h>

#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

typedef struct gps_raw {
    int svId;                   // satellite ID
    int elev;                   // elevation
    int azim;                   // azimuth
    int cno;                    // carrier to noise ratio
    double prMes;    // pseudorange measurement
    int prStdev;     // pseudorange std
} gps_raw_t;