
#include "ros/ros.h"
#include "ros/time.h"
#include <ros/console.h>

#include "std_msgs/String.h"

#include <sstream>

#include "../Communication/CommMsg.hpp"
#include "../Communication/Communication.hpp"
#include "../Communication/SerialBoost.hpp"

#include "MotorEPOS.hpp"
#include "LinAct.hpp"
#include "WirelessVIV.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <vatroslav/CanMsg.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <cstring>	// for memecpy
#include <algorithm>
#include <iostream>
#include <cassert>
#include <list>



