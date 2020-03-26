#include "Timer.h"

#include <ctime>
#include <pthread.h>
#include <iostream>
#include <cstring>
#include <errno.h>


/**
 * @brief Constructs an instance of Timer. The constructed Timer will contain the time
 * when it was created.
 */
Timer::Timer() {

	getTime();

	ticking = false;

}


/**
 * @brief Constructor used to specify the time contained within the Timer instance. Usually,
 * this contructor will be used to create a time interval rather than a time point. The
 * milliseconds and seconds parameters will be added together to form the time stored in the
 * Timer instance created.
 * @param milliseconds Initial milliseconds (positive or negative) with up to nanosecond
 * precision.
 * @param seconds Initial seconds (positive or negative)
 */
Timer::Timer(double milliseconds, int seconds) {

	ticking = false;

	_time.tv_sec = seconds;
	_time.tv_nsec = static_cast<long>(milliseconds * 1000000.0);
	adjustForRollover();

}


/**
 * @brief Copy constructor, the new Timer instance is only a copy of the time stored
 * and nothing else.
 * @param other Instance of Timer from which to create this Timer
 */
Timer::Timer(const Timer& other) {

	_time = other._time;

	ticking = false;

}


/**
 * @brief Destructor. Makes sure to stop any threads created for function calling.
 */
Timer::~Timer() {

	stopTicking();

}


/**
 * @brief Begins calling user_function at a specified interval in a separate thread with the
 * specified priority. The user_function is a standard function and does not take any
 * parameters. It is repeatedly called until stopTicking() is called or the provided function
 * returns false. See the full description of this class for more details and examples of usage.
 * @param user_function A function pointer to a user defined function to call at regular intervals.
 * The function must return a boolean and take no parameters.
 * @param timestep_ms Initial time interval between calls to user_function in milliseconds
 * @param priority Scheduling priority for the thread which calls user_function. This can be from
 * 0 to 99 where 0 is the lowest priority and 99 is the highest.
 * @return True if the ticking has been successfully started
 */
bool Timer::startTicking(bool (*user_function)(), double timestep_ms, int priority) {
	int ret = 0;

	if (ticking)
		return true;

	this->ticking_timestep_ms = timestep_ms;

	NormalFunctionThreadArguments args = {user_function, this};

	// Allocate and copy the argument structure to the private member 'function_arguments' (the space will be deleted by runNormalFunction).
	void *function_arguments = new unsigned char[sizeof(NormalFunctionThreadArguments)];
	memcpy(function_arguments, &args, sizeof(NormalFunctionThreadArguments));

	// create the thread attribute and initialize it
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

	// set the subthread priority and policy
	if (priority > 0)
		pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	else
		pthread_attr_setschedpolicy(&attr, SCHED_OTHER);

	struct sched_param param;
	param.sched_priority = priority;
	pthread_attr_setschedparam(&attr, &param);

	// Need to set this true before we create the thread because the new thread might be able to get past the while loop before
	// this thread can set ticking to true.
	ticking = true;

	// Start up a new thread executed which executes the runNormalFunction() function with the provided arguments.
	if ((ret = pthread_create(&thread_handle, &attr, &Timer::runNormalFunction, function_arguments)) != 0) {
		ticking = false;
		std::cout << "TIMER: could not create the thread for the timer: " << strerror(ret) << " ERROR" << std::endl;
		return false;
	}

	pthread_attr_destroy(&attr);

	return true;
}


/**
 * @brief This is the function that startTicking passes to the pthread_create() function which the
 * new thread will execute. It defines how and when the user defined function will get called for
 * the case of a standard function with no parameters.
 */
void *Timer::runNormalFunction(void *args) {

	NormalFunctionThreadArguments *my_args = static_cast<NormalFunctionThreadArguments*>(args);
	bool (*user_function)() = my_args->user_function;
	Timer *timer = my_args->timer;

	delete[] static_cast<unsigned char*>(args);

	// Store the current time.
	timer->getTime();

	// Keep looping until boolean ticking variable is set to false, then stop.
	while (timer->ticking && (*user_function)()) {
		timer->increment(timer->getTickingTimestep()); // increment the time to the time interval

		if (!timer->waitUntilPassed()) // wait until the time interval has passed
			break;
	}

	// If the ticking variable is still true, then an error occured (i.e. stopTicking() was not called) and ticking should be stopped
	if (timer->ticking) {
		std::cout << "TIMER: an error caused the timer to stop ticking. Likely the user function returned false. ERROR" << std::endl;
		// Since stopTicking() was not called, then pthread_join() is not expected to be called. Therefore, we detach this thread so
		// that resouces will be cleaned up when this thread terminates. We also set the ticking variable to false so that it is safe
		// to call startTicking() or stopTicking() (which will happen when the program closes).
		// TODO: Perhaps this class should have an errorOccured callback function that users can register and then gets called here.
		// This way, the thread which created this thread can handle the error and start the timer again.
		pthread_detach(pthread_self()); // An error occured so no one will be calling pthread_join() for this thread
		timer->ticking = false;
	}

	pthread_exit(nullptr);

}


/**
 * @brief Begins calling user_function at a specified interval in a separate thread with the
 * specified priority. The user_function is a standard function and takes a double parameter
 * which will contain the elapsed time in milliseconds. It is repeatedly called until
 * stopTicking() is called or the provided function returns false. See the full description
 * of this class for more details and examples of usage.
 * @param user_function A function pointer to a user defined function to call at regular intervals.
 * The function must return a boolean and take a double parameter which will
 * contain the elapsed time from the time startTicking() was called in milliseconds.
 * @param timestep_ms Initial time interval between calls to user_function in milliseconds
 * @param priority Scheduling priority for the thread which calls user_function. This can be from
 * 0 to 99 where 0 is the lowest priority and 99 is the highest.
 * @return True if the ticking has been successfully started
 */
bool Timer::startTicking(bool (*user_function)(double), double timestep_ms, int priority) {
	int ret = 0;

	if (ticking)
		return true;

	this->ticking_timestep_ms = timestep_ms;

	NormalFunctionThreadArgumentsWithElapsedTime args = {user_function, this};
	
	// Allocate and copy the argument structure to the private member 'function_arguments' (the space will be deleted by runNormalFunction).
	void *function_arguments = new unsigned char[sizeof(NormalFunctionThreadArgumentsWithElapsedTime)];
	memcpy(function_arguments, &args, sizeof(NormalFunctionThreadArgumentsWithElapsedTime));
	
	// create the thread attribute and initialize it
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

	// set the subthread priority and policy
	if (priority > 0)
		pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	else
		pthread_attr_setschedpolicy(&attr, SCHED_OTHER);

	struct sched_param param;
	param.sched_priority = priority;
	pthread_attr_setschedparam(&attr, &param);

	// Need to set this true before we create the thread because the new thread might be able to get past the while loop before
	// this thread can set ticking to true.
	ticking = true;

	// Start up a new thread executed which executes the runNormalFunction() function with the provided arguments.
	if ((ret = pthread_create(&thread_handle, &attr, &Timer::runNormalFunctionWithElapsedTime, function_arguments)) != 0) {
		ticking = false;
		std::cout << "TIMER: could not create the thread for the timer: " << strerror(ret) << " ERROR" << std::endl;
		return false;
	}

	pthread_attr_destroy(&attr);

	return true;
}


/**
 * @brief This is the function that startTicking passes to the pthread_create() function which the
 * new thread will execute. It defines how and when the user defined function will get called for
 * the case of a standard function with the elapsed time parameter.
 */
void *Timer::runNormalFunctionWithElapsedTime(void *args) {

	Timer start_time;

	NormalFunctionThreadArgumentsWithElapsedTime *my_args = static_cast<NormalFunctionThreadArgumentsWithElapsedTime*>(args);
	bool (*user_function)(double) = my_args->user_function;
	Timer *timer = my_args->timer;

	
	delete[] static_cast<unsigned char*>(args);
	
	// Store the current time.
	start_time.getTime(); 
	timer->getTime(); 

	// Keep looping until boolean ticking variable is set to false, then stop.
	while (timer->ticking && (*user_function)(start_time.elapsedTime().milliseconds())) {
		timer->increment(timer->getTickingTimestep()); // increment the time to the time interval

		if (!timer->waitUntilPassed()) // wait until the time interval has passed
			break;
	}

	// If the ticking variable is still true, then an error occured (i.e. stopTicking() was not called) and ticking should be stopped
	if (timer->ticking) {
		std::cout << "TIMER: an error caused the timer to stop ticking. Likely the user function returned false. ERROR" << std::endl;
		// Since stopTicking() was not called, then pthread_join() is not expected to be called. Therefore, we detach this thread so
		// that resouces will be cleaned up when this thread terminates. We also set the ticking variable to false so that it is safe
		// to call startTicking() or stopTicking() (which will happen when the program closes).
		// TODO: Perhaps this class should have an errorOccured callback function that users can register and then gets called here.
		// This way, the thread which created this thread can handle the error and start the timer again.
		pthread_detach(pthread_self()); // An error occured so no one will be calling pthread_join() for this thread
		timer->ticking = false;
	}

	pthread_exit(nullptr);

}


/**
 * @brief Stops repeatedly calling the user defined function provided to startTicking() and cleans
 * up resources.
 * @return True if successful
 */
bool Timer::stopTicking() {
	int ret = 0;

	if (!ticking)
		return true;

	ticking = false;

	// FIXME: For some reason this function sometimes returns an error claiming that the thread being canceled does not exist.
	// This has been confirmed to happen when the pthread_cancel function is called after the thread calls pthread_exit().
	if ((ret = pthread_cancel(thread_handle)) != 0) {
		std::cout << "TIMER: couldn't cancel the thread: " << strerror(ret) << " ERROR" << std::endl;
		return false;
	}

	if ((ret = pthread_join(thread_handle, nullptr)) != 0) {
		std::cout << "TIMER: there was an error stopping the thread for the timer: " << strerror(ret) << " ERROR" << std::endl;
		return false;
	}

	return true;
}


/**
 * @brief Returns true if the function provided to startTicking() is still being called repeatedly at the desired
 * time interval.
 * @return
 */
bool Timer::isTicking() const {

	return ticking;

}


/**
 * @brief Returns the time interval currently in use for calling the user defined function.
 * If isTicking() returns false, then the value returned by this function is undefined.
 * @return
 */
double Timer::getTickingTimestep() const {

	return this->ticking_timestep_ms;

}


/**
 * @brief Sets the time interval in use for calling the user defined function. This
 * function only works when isTicking() returns true. This is because the interval
 * will be overwritten when the ticking is started by a call to startTicking()
 * @param timestep_ms New time interval between calls to the user defined function in milliseconds
 */
void Timer::setTickingTimestep(double timestep_ms) {

	this->ticking_timestep_ms = timestep_ms;

}


/**
 * @brief Updates this instance of Timer to contain the current time. Note this instance of Timer is
 * now a timepoint if it wasn't already.
 * @return True if successful
 */
bool Timer::getTime() {

	if (clock_gettime(CLOCK_MONOTONIC, &_time) < 0) {
		std::cout << "TIMER: an error occured when getting the current time: " << strerror(errno) << " ERROR" << std::endl;
		
		_time.tv_sec = 0;
		_time.tv_nsec = 0;
		
		return false;
	}

	return true;
}


/**
 * @brief Sets the stored time to the sum of milliseconds and seconds (respecting units of course). This function
 * is usually used to make this instance of Timer represent a time interval.
 * @param milliseconds Milliseconds (positive or negative) with up to nanosecond precision.
 * @param seconds Seconds (positive or negative)
 */
void Timer::setTime(double milliseconds, int seconds) {

	_time.tv_sec = seconds;
	_time.tv_nsec = static_cast<long>(milliseconds * 1000000.0);

	adjustForRollover();

}


/**
 * @brief Adds the given number of milliseconds to the time stored in the Timer.
 * @param milliseconds Milliseconds (positive or negative) with up to nanosecond precision.
 */
void Timer::increment(double milliseconds) {

	_time.tv_nsec += milliseconds * 1000000;
	adjustForRollover();

}


/**
 * @brief Subtracts the given number of milliseconds from the time stored in the Timer.
 * @param milliseconds Milliseconds (positive or negative) with up to nanosecond precision.
 */
void Timer::decrement(double milliseconds) {

	_time.tv_nsec -= milliseconds * 1000000;
	adjustForRollover();

}


/**
 * @brief Returns a Timer that stores the elapsed time since that which is stored in this
 * instance of timer. This instance of Timer is not affected.
 * @return
 */
Timer Timer::elapsedTime() const {

	Timer current_time;

	return current_time - *this;

}


/**
 * @brief Updates the stored time in this instance of Timer to the current time and returns
 * a Timer containing the elapsed time. This is the same as elapsedTime() except it also updates
 * this Timer to the current time.
 * @return
 */
Timer Timer::updateTime() {

	Timer new_time; // Get the time
	Timer elapsed_time = new_time - *this; // Find the elapsed time
	*this = new_time; // Set this Timer to the new time

	return elapsed_time;

}


/**
 * @brief Returns the stored time in milliseconds with nanosecond precision.
 * @return
 */
double Timer::milliseconds() const {

	return (_time.tv_sec * 1000.0) + (_time.tv_nsec / 1000000.0);

}


/**
 * @brief Returns the stored time in seconds with nanosecond precision.
 * @return
 */
double Timer::seconds() const {

	return _time.tv_sec + _time.tv_nsec / 1000000000.0;

}


/**
 * @brief Blocks/deschedules the calling thread until the time point stored in this Timer instance has passed. If the time
 * has already passed, then it returns immediately. If this timer instance represents a time interval, it will
 * be treated as a time point, in which case, the behavior is undefined.
 * @return True if successful, false if an error occurred.
 */
bool Timer::waitUntilPassed() {

	int ret = 0;

	if ((ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &_time, nullptr))) {
		std::cout << "TIMER: an error occured while waiting for time to pass: " << strerror(ret) << " ERROR" << std::endl;
		return false;
	}
	
	return true;
}


/**
 * @brief Blocks/deschedules the calling thread until the specified milliseconds have passed. This is done
 * by incrementing this instance of Timer and calling waitUntilPassed(), therefore, this Timer
 * will be modified to represent the time point at which you want to stop blocking.
 * @param milliseconds Milliseconds (positive only) with up to nanosecond precision.
 * @return True if successful, false if an error occurred.
 */
bool Timer::waitUntilPassed(double milliseconds) {

	increment(milliseconds);

	return waitUntilPassed();

}


/**
 * @brief A static function which returns the system monotonic clock in milliseconds. See full class description for more info.
 * @return
 */
long Timer::getSysTime() {

	timespec tmp_time;

	if (clock_gettime(CLOCK_MONOTONIC, &tmp_time) < 0) {
		std::cout << "TIMER: an error occured when getting the current system time: " << strerror(errno) << " ERROR" << std::endl;
		return -1;
	}

	return (tmp_time.tv_sec * 1000L) + (tmp_time.tv_nsec / 1000000L);

}


/**
 * @brief A static function which returns the system monotonic clock in milliseconds with nanosecond precision. See full class
 * description for more info.
 * @return
 */
double Timer::getSysTimeHighPrecision() {

	timespec tmp_time;

	if (clock_gettime(CLOCK_MONOTONIC, &tmp_time) < 0) {
		std::cout << "TIMER: an error occured when getting the current system time: " << strerror(errno) << " ERROR" << std::endl;
		return -1;
	}

	return (tmp_time.tv_sec * 1000.0) + (tmp_time.tv_nsec / 1000000.0);

}


/**
 * @brief A static function which returns the system monotonic clock in nanoseconds. See full class description for more info.
 * @return
 */
long long Timer::getSysTimeNanos() {

	timespec tmp_time;

	if (clock_gettime(CLOCK_MONOTONIC, &tmp_time) < 0) {
		std::cout << "TIMER: an error occured when getting the current system time: " << strerror(errno) << " ERROR" << std::endl;
		return -1;
	}

	return (tmp_time.tv_sec * 1000000000L) + tmp_time.tv_nsec;

}


/**
 * @brief A static function which blocks/deschedules the calling thread for the specified number of milliseconds.
 * @param milliseconds Milliseconds (positive only).
 * @return
 */
bool Timer::wait(long milliseconds) {

	int ret = 0;

	timespec increment_time;
	if (clock_gettime(CLOCK_MONOTONIC, &increment_time) < 0) {
		std::cout << "TIMER: an error occurred when getting the current time: " + std::string(strerror(errno)) + " ERROR" << std::endl;
		return false;
	}

	milliseconds += increment_time.tv_nsec / 1000000L; // add current nano seconds to requested time

	increment_time.tv_sec += milliseconds / 1000L; // add roll-over to seconds
	milliseconds -= (milliseconds / 1000L) * 1000L; // subtract roll-over seconds from milliseconds

	increment_time.tv_nsec = milliseconds * 1000000L; // add back remaining nanoseconds

	if ((ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &increment_time, nullptr))) {
		std::cout << "TIMER: an error occurred while waiting for time to pass: " + std::string(strerror(ret)) + " ERROR" << std::endl;
		return false;
	}

	return true;

}


/**
 * @brief This function will modify the internal representation of the time to an equivalent pair of seconds and
 * nanoseconds which both have the same sign and where nanoseconds only represents fractions of a second (i.e.
 * nanoseconds is less than 1000000000. It is used whenever the internal representation of time is changed.
 */
void Timer::adjustForRollover() {

	if (_time.tv_nsec >= 1000000000 || _time.tv_nsec <= -1000000000) {
		long rolloverSeconds = _time.tv_nsec / 1000000000;
		_time.tv_sec += rolloverSeconds;
		_time.tv_nsec -= rolloverSeconds * 1000000000;
	}


	if (_time.tv_sec > 0 && _time.tv_nsec < 0) {
		_time.tv_sec -= 1;
		_time.tv_nsec += 1000000000;
	}
	else if (_time.tv_sec < 0 && _time.tv_nsec > 0) {
		_time.tv_sec += 1;
		_time.tv_nsec -= 1000000000;
	}

}


/**
 * @brief Increments the stored time by the specified number of milliseconds.
 * @param milliseconds Milliseconds (positive or negative) with up to nanosecond precision.
 * @return
 */
Timer &Timer::operator+=(double milliseconds) {

	increment(milliseconds);

	return *this;

}


/**
 * @brief Decrements the stored time by the specified number of milliseconds.
 * @param milliseconds Milliseconds (positive or negative) with up to nanosecond precision.
 * @return
 */
Timer &Timer::operator-=(double milliseconds) {

	decrement(milliseconds);

	return *this;

}


/**
 * @brief Increments the stored time by the time stored in another instance of Timer.
 * @param timer Another instance of Timer
 * @return
 */
Timer &Timer::operator+=(const Timer &timer) {

	_time.tv_sec += timer._time.tv_sec;
	_time.tv_nsec += timer._time.tv_nsec;
	adjustForRollover();

	return *this;

}


/**
 * @brief Decrements the stored time by the time stored in another instance of Timer.
 * @param timer Another instance of Timer
 * @return
 */
Timer &Timer::operator-=(const Timer &timer) {

	_time.tv_sec -= timer._time.tv_sec;
	_time.tv_nsec -= timer._time.tv_nsec;
	adjustForRollover();

	return *this;

}


/**
 * @brief Computes the sum of the stored time and the specified number of milliseconds.
 * @param milliseconds Milliseconds (positive or negative) with up to nanosecond precision.
 * @return A Timer representing the sum.
 */
Timer Timer::operator+(double milliseconds) {

	Timer tmp = *this;
	
	tmp += milliseconds;

	return tmp;

}


/**
 * @brief Computes the difference between the stored time and the specified number of milliseconds.
 * @param milliseconds Milliseconds (positive or negative) with up to nanosecond precision.
 * @return A Timer representing the difference.
 */
Timer Timer::operator-(double milliseconds) {

	Timer tmp = *this;
	
	tmp -= milliseconds;

	return tmp;

}


/**
 * @brief Computes the sum of the stored time and the time stored in another Timer.
 * @param timer Another instance of Timer
 * @return A Timer representing the sum.
 */
Timer Timer::operator+(const Timer &timer) {

	Timer new_timer = *this;
	new_timer += timer;

	return new_timer;

}


/**
 * @brief Computes the difference between the stored time and the time stored in another Timer.
 * @param timer Another instance of Timer
 * @return A Timer representing the difference.
 */
Timer Timer::operator-(const Timer &timer) {

	Timer new_timer = *this;
	new_timer -= timer;

	return new_timer;

}


/**
 * @brief Copy assignment operator, the assigned Timer instance only takes on the stored
 * time of the other Timer and nothing else.
 * @param other Another instance of Timer
 * @return
 */
Timer& Timer::operator=(const Timer& other) {

	_time = other._time;

	return *this;

}


/**
 * @brief Sets the stored time to the specified time in milliseconds.
 * @param milliseconds Milliseconds (positive or negative) with up to nanosecond precision.
 * @return
 */
Timer Timer::operator=(double milliseconds) {

	_time.tv_sec = static_cast<long>(milliseconds / 1000.0);
	milliseconds -= _time.tv_sec * 1000.0; // handles both positive and negative values for milliseconds input
	_time.tv_nsec = static_cast<long>(milliseconds * 1000000.0);

	return *this;

}


/**
 * @brief Prints the stored time in seconds and nanoseconds to the provided output stream.
 * @return
 */
std::ostream &operator<<(std::ostream &stream, const Timer &timer) {

	stream << "(s: " << timer._time.tv_sec << ", ns: " << timer._time.tv_nsec << ")";

	return stream;
}
