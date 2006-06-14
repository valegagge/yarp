// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _YARP2_RANDOM_
#define _YARP2_RANDOM_


/**
 * \file Random.h Contains methods and classes for generating random
 * numbers with various (simple) distributions.
 */


namespace yarp {
    namespace os {
        class Random;
    }
}

/**
 * A collection of basic random number generation algorithms.
 */
class yarp::os::Random {
public:
	/**
	 * Generates a random number in the range 0 to 1.
	 * @return a random floating point number in the range 0 to 1.
	 */
    static double uniform();

	/**
	 * A normal random number generator.
	 * @param m is the mean of the gaussian distribution.
	 * @param s is the standard deviation of the distribution.
	 * @return a random number with normal distribution.
	 */
    static double normal(double m, double s);

	/**
	 * Sets the seed of the random number generator.
	 * @param seed is the new seed of the random number generator.
	 */
	static void seed(int seed);

	/**
	 * @return a random number normally distributed with 0 mean and 
	 * unitary standard deviation.
	 */
	static double normal() { 
        return normal(0.0, 1.0); 
    }

	/**
	 * A random number in a specified range.
	 * @param min is the minimum of the range.
	 * @param max is the maximum of the range.
	 * @return a random integer in the specified range.
	 */
	static int uniform(int min, int max);
};


#endif 

