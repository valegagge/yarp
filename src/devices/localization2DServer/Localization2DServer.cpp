/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/math/Math.h>
#include "Localization2DServer.h"

#include <cmath>

/*! \file Localization2DServer.cpp */

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace std;

#define DEFAULT_THREAD_PERIOD 0.01

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//------------------------------------------------------------------------------------------------------------------------------

Localization2DServer::Localization2DServer() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_ros_node == nullptr;
    m_current_status = yarp::dev::Nav2D::LocalizationStatusEnum::localization_status_not_yet_localized;
    m_period = DEFAULT_THREAD_PERIOD;
    m_stats_time_last = yarp::os::Time::now();
    iLoc = 0;
    m_getdata_using_periodic_thread = true;
}

bool Localization2DServer::attachAll(const PolyDriverList &device2attach)
{
    if (device2attach.size() != 1)
    {
        yError("Localization2DServer: cannot attach more than one device");
        return false;
    }

    yarp::dev::PolyDriver * Idevice2attach = device2attach[0]->poly;

    if (Idevice2attach->isValid())
    {
        Idevice2attach->view(iLoc);
    }

    if (nullptr == iLoc)
    {
        yError("Localization2DServer: subdevice passed to attach method is invalid");
        return false;
    }

    //initialize m_current_position and m_current_status, if available
    bool ret = true;
    yarp::dev::Nav2D::LocalizationStatusEnum status;
    Map2DLocation loc;
    ret &= iLoc->getLocalizationStatus(status);
    ret &= iLoc->getCurrentPosition(loc);
    if (ret)
    {
        m_current_status = status;
        m_current_position = loc;
    }
    else
    {
        yWarning() << "Localization data not yet available during server initialization";
    }

    PeriodicThread::setPeriod(m_period);
    return PeriodicThread::start();
}

bool Localization2DServer::detachAll()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    iLoc = nullptr;
    return true;
}

bool Localization2DServer::open(Searchable& config)
{
    Property params;
    params.fromString(config.toString().c_str());
    yDebug() << "Localization2DServer configuration: \n" << config.toString().c_str();

    if (config.check("GENERAL") == false)
    {
        yWarning() << "Missing GENERAL group, assuming default options";
    }

    Bottle& general_group = config.findGroup("GENERAL");
    if (!general_group.check("period"))
    {
        yInfo() << "Localization2DServer: missing 'period' parameter. Using default value: " << DEFAULT_THREAD_PERIOD;
        m_period = DEFAULT_THREAD_PERIOD;
    }
    else
    {
        m_period = general_group.find("period").asFloat64();
        yInfo() << "Localization2DServer: period requested: " << m_period;
    }

    if (!general_group.check("retrieve_position_periodically"))
    {
        yInfo() << "Localization2DServer: missing 'retrieve_position_periodically' parameter. Using default value: true. Period:" << m_period ;
        m_getdata_using_periodic_thread = true;
    }
    else
    {
        m_getdata_using_periodic_thread = general_group.find("retrieve_position_periodically").asBool();
        if (m_getdata_using_periodic_thread)
            { yInfo() << "Localization2DServer: retrieve_position_periodically requested, Period:" << m_period; }
        else
            { yInfo() << "Localization2DServer: retrieve_position_periodically NOT requested. Localization data obtained asynchronously."; }
    }


    string local_name = "/localizationServer";
    if (!general_group.check("name"))
    {
        yInfo() << "Localization2DServer: missing 'name' parameter. Using default value: /localizationServer";
    }
    else
    {
        local_name = general_group.find("name").asString();
        if (local_name.c_str()[0] != '/') { yError() << "Missing '/' in name parameter" ;  return false; }
        yInfo() << "Localization2DServer: using local name:" << local_name;
    }

    m_rpcPortName = local_name + "/rpc";
    m_2DLocationPortName = local_name + "/streaming:o";
    m_odometryPortName = local_name + "/odometry:o";

    if (config.check("subdevice"))
    {
        Property       p;
        PolyDriverList driverlist;
        p.fromString(config.toString(), false);
        p.put("device", config.find("subdevice").asString());

        if (!pLoc.open(p) || !pLoc.isValid())
        {
            yError() << "Localization2DServer: failed to open subdevice.. check params";
            return false;
        }

        driverlist.push(&pLoc, "1");
        if (!attachAll(driverlist))
        {
            yError() << "Localization2DServer: failed to open subdevice.. check params";
            return false;
        }
    }
    else
    {
        yInfo() << "Localization2DServer: waiting for device to attach";
    }
    m_stats_time_last = yarp::os::Time::now();

    if (!initialize_YARP(config))
    {
        yError() << "Localization2DServer: Error initializing YARP ports";
        return false;
    }

    return true;
}

bool Localization2DServer::initialize_YARP(yarp::os::Searchable &params)
{
    if (!m_2DLocationPort.open(m_2DLocationPortName.c_str()))
    {
        yError("Localization2DServer: failed to open port %s", m_2DLocationPortName.c_str());
        return false;
    }

    if (!m_odometryPort.open(m_odometryPortName.c_str()))
    {
        yError("Localization2DServer: failed to open port %s", m_odometryPortName.c_str());
        return false;
    }

    if (!m_rpcPort.open(m_rpcPortName.c_str()))
    {
        yError("Localization2DServer: failed to open port %s", m_rpcPortName.c_str());
        return false;
    }
    m_rpcPort.setReader(*this);

    return true;
}

bool Localization2DServer::close()
{
    yTrace("Localization2DServer::Close");
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }

    detachAll();

    m_2DLocationPort.interrupt();
    m_2DLocationPort.close();
    m_odometryPort.interrupt();
    m_odometryPort.close();
    m_rpcPort.interrupt();
    m_rpcPort.close();

    yDebug() << "Localization2DServer execution terminated";
    return true;
}

bool Localization2DServer::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle command;
    yarp::os::Bottle reply;
    bool ok = command.read(connection);
    if (!ok) return false;

    reply.clear();

    if (command.get(0).isVocab())
    {
        if (command.get(0).asVocab() == VOCAB_INAVIGATION && command.get(1).isVocab())
        {
            int request = command.get(1).asVocab();
            if (request == VOCAB_NAV_GET_CURRENT_POS)
            {
                if (m_getdata_using_periodic_thread)
                {
                    //m_current_position is obtained by run()
                    reply.addVocab(VOCAB_OK);
                    reply.addString(m_current_position.map_id);
                    reply.addFloat64(m_current_position.x);
                    reply.addFloat64(m_current_position.y);
                    reply.addFloat64(m_current_position.theta);
                }
                else
                {
                    //m_current_position is obtained by getCurrentPosition()
                    iLoc->getCurrentPosition(m_current_position);
                    reply.addVocab(VOCAB_OK);
                    reply.addString(m_current_position.map_id);
                    reply.addFloat64(m_current_position.x);
                    reply.addFloat64(m_current_position.y);
                    reply.addFloat64(m_current_position.theta);
                }
            }
            else if (request == VOCAB_NAV_SET_INITIAL_POS)
            {
                Map2DLocation init_loc;
                init_loc.map_id = command.get(2).asString();
                init_loc.x = command.get(3).asFloat64();
                init_loc.y = command.get(4).asFloat64();
                init_loc.theta = command.get(5).asFloat64();
                iLoc->setInitialPose(init_loc);
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_GET_CURRENT_POSCOV)
            {
                Map2DLocation init_loc;
                yarp::sig::Matrix cov(3, 3);
                iLoc->getCurrentPosition(init_loc, cov);
                reply.addVocab(VOCAB_OK);
                reply.addString(m_current_position.map_id);
                reply.addFloat64(m_current_position.x);
                reply.addFloat64(m_current_position.y);
                reply.addFloat64(m_current_position.theta);
                yarp::os::Bottle& mc = reply.addList();
                for (size_t i = 0; i < 3; i++) { for (size_t j = 0; j < 3; j++) { mc.addFloat64(cov[i][j]); } }
            }
            else if (request == VOCAB_NAV_SET_INITIAL_POSCOV)
            {
                Map2DLocation init_loc;
                yarp::sig::Matrix cov(3,3);
                init_loc.map_id = command.get(2).asString();
                init_loc.x = command.get(3).asFloat64();
                init_loc.y = command.get(4).asFloat64();
                init_loc.theta = command.get(5).asFloat64();
                Bottle* mc = command.get(6).asList();
                if (mc!=nullptr && mc->size() == 9)
                {
                    for (size_t i = 0; i < 3; i++) { for (size_t j = 0; j < 3; j++) { cov[i][j] = mc->get(i * 3 + j).asFloat64(); } }
                    bool ret = iLoc->setInitialPose(init_loc, cov);
                    if (ret) { reply.addVocab(VOCAB_OK); }
                    else     { reply.addVocab(VOCAB_ERR); }
                }
                else
                {
                    reply.addVocab(VOCAB_ERR);
                }
            }
            else if (request == VOCAB_NAV_LOCALIZATION_START)
            {
                iLoc->startLocalizationService();
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_LOCALIZATION_STOP)
            {
                iLoc->stopLocalizationService();
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_GET_LOCALIZER_STATUS)
            {
                if (m_getdata_using_periodic_thread)
                {
                    //m_current_status is obtained by run()
                    reply.addVocab(VOCAB_OK);
                    reply.addVocab(m_current_status);
                }
                else
                {
                    //m_current_status is obtained by getLocalizationStatus()
                    iLoc->getLocalizationStatus(m_current_status);
                    reply.addVocab(VOCAB_OK);
                    reply.addVocab(m_current_status);
                }
            }
            else if (request == VOCAB_NAV_GET_LOCALIZER_POSES)
            {
                std::vector<Map2DLocation> poses;
                iLoc->getEstimatedPoses(poses);
                reply.addVocab(VOCAB_OK);
                reply.addInt32(poses.size());
                for (size_t i=0; i<poses.size(); i++)
                {
                    Bottle& b = reply.addList();
                    b.addString(poses[i].map_id);
                    b.addFloat64(poses[i].x);
                    b.addFloat64(poses[i].y);
                    b.addFloat64(poses[i].theta);
                }
            }
            else
            {
                reply.addVocab(VOCAB_ERR);
            }
        }
        else
        {
            yError() << "Invalid vocab received";
            reply.addVocab(VOCAB_ERR);
        }
    }
    else if (command.get(0).isString() && command.get(0).asString() == "help")
    {
        reply.addVocab(Vocab::encode("many"));
        reply.addString("Available commands are:");
        reply.addString("getLoc");
        reply.addString("initLoc <map_name> <x> <y> <angle in degrees>");
    }
    else if (command.get(0).isString() && command.get(0).asString() == "getLoc")
    {
        Map2DLocation curr_loc;
        iLoc->getCurrentPosition(curr_loc);
        std::string s = std::string("Current Location is: ") + curr_loc.toString();
        reply.addString(s);
    }
    else if (command.get(0).isString() && command.get(0).asString() == "initLoc")
    {
        Map2DLocation init_loc;
        init_loc.map_id = command.get(1).asString();
        init_loc.x = command.get(2).asFloat64();
        init_loc.y = command.get(3).asFloat64();
        init_loc.theta = command.get(4).asFloat64();
        iLoc->setInitialPose(init_loc);
        std::string s = std::string("Localization initialized to: ") + init_loc.toString();
        reply.addString(s);
    }
    else
    {
        yError() << "Invalid command type";
        reply.addVocab(VOCAB_ERR);
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender != nullptr)
    {
        reply.write(*returnToSender);
    }

    return true;
}

void Localization2DServer::run()
{
    double m_stats_time_curr = yarp::os::Time::now();
    if (m_stats_time_curr - m_stats_time_last > 5.0)
    {
        yInfo() << "Localization2DServer running";
        m_stats_time_last = yarp::os::Time::now();
    }

    if (m_getdata_using_periodic_thread)
    {
        bool ret = iLoc->getLocalizationStatus(m_current_status);
        if (ret == false)
        {
            yError() << "Localization2DServer: getLocalizationStatus() failed";
        }

        if (m_current_status == LocalizationStatusEnum::localization_status_localized_ok)
        {
            bool ret2 = iLoc->getCurrentPosition(m_current_position);
            if (ret2 == false)
            {
                yError() << "Localization2DServer: getCurrentPosition() failed";
            }
        }
        else
        {
            yWarning("The system is not properly localized!");
        }
    }

    if (m_ros_node && m_odometry_publisher.asPort().getOutputCount() > 0)
    {
        yarp::rosmsg::nav_msgs::Odometry& odom = m_odometry_publisher.prepare();
        odom.clear();
        odom.header.frame_id = m_fixed_frame;
        odom.header.seq = m_loc_stamp.getCount();
        odom.header.stamp = m_loc_stamp.getTime();
        odom.child_frame_id = m_robot_frame;

        odom.pose.pose.position.x = vecpos[0];
        odom.pose.pose.position.y = vecpos[1];
        odom.pose.pose.position.z = vecpos[2];
        vecrpy[0] = vecrpy[0] * M_PI / 180.0;
        vecrpy[1] = vecrpy[1] * M_PI / 180.0;
        vecrpy[2] = vecrpy[2] * M_PI / 180.0;
        yarp::sig::Matrix matrix = yarp::math::rpy2dcm(vecrpy);
        yarp::math::Quaternion q; q.fromRotationMatrix(matrix);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        //odom.pose.covariance = 0;

        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.x = 0;
        odom.twist.twist.angular.y = 0;
        odom.twist.twist.angular.z = 0;
        //odom.twist.covariance = 0;

        m_odometry_publisher.write();
    }

    if (m_odometryPort.getOutputCount() > 0)
    {
        yarp::dev::OdometryData& odom = m_odometryPort.prepare();
        odom = m_current_odometry;

        //update the timestamp
        m_odom_stamp.update();
        m_odometryPort.setEnvelope(m_odom_stamp);

        //send data to port
        m_odometryPort.write();
    }

    if (m_2DLocationPort.getOutputCount() > 0)
    {
        Nav2D::Map2DLocation& loc = m_2DLocationPort.prepare();
        if (m_current_status == LocalizationStatusEnum::localization_status_localized_ok)
        {
            loc = m_current_position;
        }
        else
        {
            Map2DLocation temp_loc;
            temp_loc.x = std::nan("");
            temp_loc.y = std::nan("");
            temp_loc.theta = std::nan("");
            loc = temp_loc;
        }

        //update the timestamp
        m_loc_stamp.update();
        m_2DLocationPort.setEnvelope(m_loc_stamp);

        //send data to port
        m_2DLocationPort.write();
    }
}
