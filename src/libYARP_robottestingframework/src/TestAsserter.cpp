/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/robottestingframework/TestAsserter.h>
#include <yarp/sig/Vector.h>
#include <robottestingframework/TestAssert.h>
#include <cmath>


class yarp::robottestingframework::TestAsserter::Private
{
};

yarp::robottestingframework::TestAsserter::TestAsserter() :
        mPriv(new Private)
{
}

yarp::robottestingframework::TestAsserter::~TestAsserter()
{
    delete mPriv;
}

bool yarp::robottestingframework::TestAsserter::isApproxEqual(const double *left,
                                            const double *right,
                                            const double *thresholds,
                                            int lenght)
{
    return isApproxEqual(left, right, thresholds, thresholds, lenght);
}

bool yarp::robottestingframework::TestAsserter::isApproxEqual(const double *left,
                                            const double *right,
                                            const double *l_thresholds,
                                            const double *h_thresholds,
                                            int lenght)
{
    bool reached = true;
    for(int j = 0; j < lenght; j++)
    {
        if (left[j]<(right[j]-fabs(l_thresholds[j])) || left[j]>(right[j]+fabs(h_thresholds[j]))) {
            reached=false;
        }
    }
    return reached;
}


bool yarp::robottestingframework::TestAsserter::isApproxEqual(const yarp::sig::Vector &left,
                                            const yarp::sig::Vector &right,
                                            const yarp::sig::Vector &thresholds)
{
    if (left.size() != right.size() && right.size() != thresholds.size()) {
        ROBOTTESTINGFRAMEWORK_ASSERT_ERROR("yarp::robottestingframework::TestAsserter::isApproxEqual : vectors must have same size!");
        return false;
    }
    return isApproxEqual(left.data(), right.data(), thresholds.data(), left.size());
}

bool yarp::robottestingframework::TestAsserter::isApproxEqual(double left,
                                            double right,
                                            double l_th,
                                            double h_th)
{

    if (left >= right - fabs(l_th) && left <= right + fabs(h_th)) {
        return true;
    } else {
        return false;
    }
}

