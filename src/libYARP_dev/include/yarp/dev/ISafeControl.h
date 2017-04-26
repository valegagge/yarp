/*
 * Copyright (C) 2017 ICub Facility - Istituto italiano di Tecnologia
 * Author: Alessandro Scalzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef YARP_DEV_ISAFECONTROL_H
#define YARP_DEV_ISAFECONTROL_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/sig/Vector.h>

namespace yarp
{
    namespace dev
    {

/**
 * @ingroup dev_iface_other
 *
 * A generic safe control interface
 */
class YARP_dev_API ISafeControl
{
public:
    /**
     * Check next configuration
     * @param p joint configuration
     * @return true/false.
     */
    virtual bool checkPosition(double *p) = 0;

    /**
     * Check velocity at present configuration
     * @param v joint velocity
     * @return true/false.
     */
    virtual bool checkVelocity(double *v) = 0;
};

    }
}

#endif // YARP_DEV_ISAFECONTROL_H
