#!/usr/bin/env python
import depth_map
import sys
import numpy as np
import cw4
import coastal_surveyor
if __name__ == '__main__':
    file = sys.argv[1]
    model = cw4.cw4
    if(file == "cw4"):
        model = cw4.cw4
    elif (file == "coastal_surveyor"):
        model = coastal_surveyor.coastal_surveyor

    print(model['max_rpm'])
    print(model['max_power'])
    print(model['idle_rpm'])
    print(model['prop_ratio'])
    print(model['prop_pitch'])
    print(model['max_rpm_change_rate'])
    print(model['max_speed'])
    print(model['mass'])
    print(model['max_rudder_angle'])
    print(model['rudder_distance'])
    print(model['rudder_coefficient'])
