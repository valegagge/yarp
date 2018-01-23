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
    virtual bool checkPosition(int part, double *p) = 0;
    virtual bool checkPosition(int part, int njoints, int *joints, double *p) = 0;

    /**
     * Check velocity at present configuration
     * @param v joint velocity
     * @return true/false.
     */
    virtual bool checkVelocity(int part, double *v) = 0;
    virtual bool checkVelocity(int part, int njoints, int *joints, double *v) = 0;

    virtual void map(int k, int wBase, int wTop, int base, int top);
    virtual int bind(std::string part) = 0;
};

    }
}

#endif // YARP_DEV_ISAFECONTROL_H
