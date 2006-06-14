// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/////////////////////////////////////////////////////////////////////////
///                                                                   ///
///       YARP - Yet Another Robotic Platform (c) 2001-2004           ///
///                                                                   ///
///                    #Add our name(s) here#                         ///
///                                                                   ///
///     "Licensed under the Academic Free License Version 1.0"        ///
///                                                                   ///
/// The complete license description is contained in the              ///
/// licence.template file included in this distribution in            ///
/// $YARP_ROOT/conf. Please refer to this file for complete           ///
/// information about the licensing of YARP                           ///
///                                                                   ///
/// DISCLAIMERS: LICENSOR WARRANTS THAT THE COPYRIGHT IN AND TO THE   ///
/// SOFTWARE IS OWNED BY THE LICENSOR OR THAT THE SOFTWARE IS         ///
/// DISTRIBUTED BY LICENSOR UNDER A VALID CURRENT LICENSE. EXCEPT AS  ///
/// EXPRESSLY STATED IN THE IMMEDIATELY PRECEDING SENTENCE, THE       ///
/// SOFTWARE IS PROVIDED BY THE LICENSOR, CONTRIBUTORS AND COPYRIGHT  ///
/// OWNERS "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, ///
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   ///
/// FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO      ///
/// EVENT SHALL THE LICENSOR, CONTRIBUTORS OR COPYRIGHT OWNERS BE     ///
/// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   ///
/// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN ///
/// CONNECTION WITH THE SOFTWARE.                                     ///
///                                                                   ///
/////////////////////////////////////////////////////////////////////////

///
///
///       YARP - Yet Another Robotic Platform (c) 2001-2003 
///
///                    #emmebi#
///
///     "Licensed under the Academic Free License Version 1.0"
///

///
/// $Id: DragonflyDeviceDriver.h,v 1.10 2006-06-14 08:39:44 eshuy Exp $
///
///

#ifndef __DragonflyDeviceDriverh__
#define __DragonflyDeviceDriverh__

//=============================================================================
// YARP Includes
//=============================================================================

// May 06, readapted for YARP2 by nat

#include <yarp/os/Semaphore.h>
#include <yarp/dev/FrameGrabberInterfaces.h>

namespace yarp {
    namespace dev {
        class DragonflyOpenParameters;
        class DragonflyDeviceDriver;
    }
}

/**
 * \file DragonflyDeviceDriver.h device driver for managing the 
 * Dragonfly IEEE-1394 Camera (WINNT version)
 */

/**
 * Structure for defining the open() parameters of the camera.
 */
class yarp::dev::DragonflyOpenParameters
{
public:
	/**
	 * Constructor. Add here the parameters for the open().
	 */
	DragonflyOpenParameters()
	{
		// parameters initialization
		_unit_number = 0;
		_size_x = 0;
		_size_y = 0;
		_video_type = 0;
		_offset_y = 0;
		_offset_x = 0;
		_alfa = 0;
	}

	// Parameters
	unsigned int _unit_number;
	unsigned int _size_x;
	unsigned int _size_y;
	unsigned int _video_type;
	int _offset_y;				/** NOT USED */
	int _offset_x;				/** NOT USED */
	float _alfa;				/** NOT USED */
};

class yarp::dev::DragonflyDeviceDriver : 
    public IFrameGrabber, public IFrameGrabberRgb, public IFrameGrabberImage, public IFrameGrabberControls, public DeviceDriver
{
private:
	DragonflyDeviceDriver(const DragonflyDeviceDriver&);
	void operator=(const DragonflyDeviceDriver&);

public:
	/**
	 * Constructor.
	 */
	DragonflyDeviceDriver();

	/**
	 * Destructor.
	 */
	virtual ~DragonflyDeviceDriver();

    // temp: here for debug purposes only
    void recColorFSBilinear(const unsigned char *src, unsigned char *out);
    void recColorFSNN(const unsigned char *src, unsigned char *out);
    void recColorHSBilinear(const unsigned char *src, unsigned char *out);


    /**
	 * Open the device driver.
     * @param par parameters for the device driver
	 * @return returns true on success, false on failure.
	 */
    bool open(const DragonflyOpenParameters& par);

	/**
	 * Closes the device driver.
	 * @return returns true/false on success/failure.
	 */
	virtual bool close(void);

    /**
     * Implements FrameGrabber basic interface.
     * @param buffer the pointer to the array to store the last frame.
     * @return returns true/false on success/failure.
     */
    virtual bool getRawBuffer(unsigned char *buffer);

    /**
     * Implements the Frame grabber basic interface.
     * @return the size of the raw buffer (for the Dragonfly
     * camera this is 1x640x480).
     */
    virtual int getRawBufferSize();

    /**
     * Implements FrameGrabber basic interface.
     */
    virtual int height() const;
    
    /**
     * Implements FrameGrabber basic interface.
     */
    virtual int width() const;

    /** 
     * FrameGrabber bgr interface, returns the last acquired frame as
     * a buffer of bgr triplets. A demosaicking method is applied to 
     * reconstuct the color from the Bayer pattern of the sensor.
     * @param buffer pointer to the array that will contain the last frame.
     * @return true/false upon success/failure
     */
    virtual bool getRgbBuffer(unsigned char *buffer);

    /** 
     * FrameGrabber image interface, returns the last acquired frame as
     * an rgb image. A demosaicking method is applied to 
     * reconstuct the color from the Bayer pattern of the sensor.
     * @param image that will store the last frame.
     * @return true/false upon success/failure
     */
    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);

    /**
     * Set image brightness.
     */
    virtual bool setBrightness(double v);

    /**
     * Set shutter time.
     */
    virtual bool setShutter(double v);

    /**
     * Set gain.
     */
    virtual bool setGain(double v);

    /**
     * Get shutter time.
     */
    virtual double getShutter() const;

    /**
     * Get gain.
     */
    virtual double getGain() const;

    /**
     * Get image brightness.
     */
    virtual double getBrightness() const;

protected:
	void *system_resources;
};


#endif
