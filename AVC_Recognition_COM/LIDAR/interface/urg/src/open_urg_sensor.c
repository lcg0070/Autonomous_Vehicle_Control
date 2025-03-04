/*!
  \~japanese
  \brief URG との接続
  \~english
  \brief Connects to URG
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "open_urg_sensor.h"
#include "urg_utils.h"

#include <stdio.h>


int open_urg_sensor(urg_t *urg)
{

    urg_connection_type_t connection_type = URG_ETHERNET;
    long baudrate_or_port = 10940;
    const char *device = "192.168.0.10";

    if (urg_open(urg, connection_type, device, baudrate_or_port) < 0) {
        printf("urg_open: %s, %ld: %s\n",
            device, baudrate_or_port, urg_error(urg));
        return -1;
    }

    return 0;
}
