#ifndef TIMER_H
#define TIMER_H

#include <ctime>
#include <pthread.h>
#include <iostream>
#include <cstring>

/**
 * @brief The Timer class is a versatile class used for timing with nanosecond precision and can play one of several
 * different roles which require precise timing. It only works with Linux.
 *
 * The Timer class can play 3 major roles which require precise timing:
 *
 * <b>Role 1)</b> It can be used to call a provided function at regular and modifiable time intervals.\n
 * <b>Role 2)</b> It can be used for measuring the length of time between time instants such as timing a section of code.\n
 * <b>Role 3)</b> It can be used to block (or deschedule) the current thread for a specified amount of time.
 *
 * If an instance of Timer is being used for Role 1, then <b>DO NOT</b> use it for either of the other two roles. If you
 * do, you <b>WILL</b> ruin the timing of the calls to the provided function. If you need to time a section of code,
 * for example, create a new instance of Timer to do the timing.
 *
 * The Timer class also provides 4 static functions which do not require an instance of the class to use:
 *
 * 1) getSysTime() used for getting the current system time in milliseconds with millisecond precision. It is useful for
 * getting a unique system timestamp, etc.\n
 * 2) getSysTimeHighPrecision() used for getting the current system time in milliseconds with nanosecond precision (returned
 * as a double).\n
 * 3) getSysTimeNanos() used for getting the current system time in nanoseconds.\n
 * 4) wait() used for blocking the current thread for a specified time interval. This is a static version of Role 3 above.
 *
 * This class references the system monotonic clock. The system monotonic clock is a nanosecond precision clock that is always
 * increasing and represents time elapsed since a certain point (usually the computer boot-time). Type <i>man clock_gettime</i>
 * in a terminal for more information on the system monotonic clock.
 *
 * Since this class uses the pthread library, you must link the pthread library when compiling with the -pthread linker flag.
 * If you are using an older GCC compiler, then you must also link the real-time library with -lrt.
 *
 * <h3>Role 1</h3>
 *
 * In this form of usage, the class will create a thread which repeatedly calls the provided function at the given time
 * interval. The interval can be changed while the thread is running. The scheduling priority of the thread itself is configurable
 * in case the function is important for real-time computation. The scheduling priorities use the Linux Real-Time priorities.
 * This means you can give your Timer a priority between 0 and 99 where 0 is the lowest and 99 is the highest. The scheduling
 * policy for any priority other than 0 is SCHED_FIFO. While a higher priority will help your function execute at more precise
 * intervals by preempting other processes and threads on your computer, there is still some overhead before the operating system
 * gets around to scheduling the thread calling your function. Type <i>man sched</i> in a terminal for documentation on Linux scheduling
 * policies. To ensure that you have operating system permission to change the thread priority (normal users don't have the permission by
 * default), you need to add the line:
 *
 * \verbatim
	@USERNAME	-	rtprio	99 \endverbatim
 *
 * to the /etc/security/limits.conf file (you need root access in order to do this).  Once the line has been added, reboot the machine
 * and you should be good to go. For more information on this, type 'man ulimit' or 'man limits.conf' into a terminal an look at the
 * documentation about rtprio.
 *
 * The user defined function declaration can take on two forms:
 *
 * 1) bool <i>functionName</i>()\n
 * 2) bool <i>functionName</i>(double)
 *
 * where <i>functionName</i> is any valid function name you choose. For form 2, the double parameter will contain the elapsed
 * time in milliseconds (with nanosecond precision) from the moment the startTicking() function was called. If you need to access
 * the current system time within your function, then use the static getSysTime() function mentioned above. The boolean return value
 * is used by Timer to recognize an error in your user defined function and automatically stop the timer. If you encounter an error
 * in your function, then return false and the Timer will safely stop calling your function. You can use the isTicking() function
 * to recognize if an error has stopped the Timer. Make sure to return true at the end of your function to indicate to the Timer
 * class that your function finshed properly. The user defined function can be a stand-alone function or a member function of a class.
 * Below are some examples of how to use the Timer class for this role:
 *
 * 1) Calling a fuction, that is NOT a member of a class, at a constant time interval:
 *
 * \verbatim
   bool myfunction(double elapsed_time) { // print out the elapsed time since the function started getting called
	  cout << "Hello World " << elapsed_time << endl;
	  return true;
   }

   // somewhere else in your code...
   Timer my_timer;
   my_timer.startTicking(&myfunction, 1000); // call myfunction() every 1000 milliseconds
   // do other things here...
   my_timer.stopTicking(); // stop calling myfunction()
   \endverbatim
 *
 * 2) Calling a function, that IS a member of a class, at a constant time interval:
 *
 * \verbatim
   class MyClass {
	  bool mymember(double elapsed_time) { // print out the elapsed time since the function started getting called
		 cout << "Hello World " << elapsed_time << endl;
		 return true;
	  };
   };

   // somewhere else in your code...
   MyClass instance_of_myclass;
   Timer my_timer;
   my_timer.startTicking(instance_of_myclass, &MyClass::mymember, 1000); // call mymember() every 1000 milliseconds
   // do other things here...
   my_timer.stopTicking(); // stop calling myfunction()
   \endverbatim
 *
 * The following are the only functions safe to use with a Timer instance that is performing this role (even though the
 * last three in the list are safe to use, there really isn't much of a reason to use them):
 *
 * - startTicking()\n
 * - stopTicking()\n
 * - isTicking()\n
 * - getTickingTimestep()\n
 * - setTickingTimestep()\n
 * - elapsedTime()\n
 * - milliseconds()\n
 * - seconds()\n
 *
 *
 * <h3>Roles 2 and 3</h3>
 *
 * In this usage, the class basically becomes a stopwatch and a structure to store time. Time can be added/subtracted
 * to/from the time stored in the class using the increment() and decrement() functions or any of the overloaded
 * mathematical operators. Note that all of these modifying functions use time in milliseconds or the time contained in
 * another instance of Timer. There are also functions to find elapsed time or wait until time has passed. Also note that
 * a Timer instance can either contain a time point, or a time interval which can be positive or negative. You must recognize
 * the difference based on the context of use. The following illustrates this difference:
 *
 * \verbatim
   Timer t1;                     // t1 contains a time point which corresponds to the time when it was created
   Timer t2 = t1.elapsedTime();  // t2 contains a time interval

   Timer t3 = t1 + t2;           // t3 contains a time point which is t2 after t1
   Timer t4 = t1 - t3;           // t4 contains a time interval which corresponds to the negative interval of t2
   Timer t5 = t2 + t4;           // t5 contains a 0 time interval since t2 and t4 are intervals of equal length but opposite direction

   t1 += 500;                    // t1 is still a time point, but it has been moved 500 milliseconds later
   t1 += t2;                     // t1 is still a time point, but it has been moved t2 time later
   t2.getTime();                 // t2 is now a time point corresponding to the time when this line was executed
   t1 += t3;                     // DON'T ADD TIMEPOINTS - while this will work, t1 is now a timepoint with an undefined value
   \endverbatim
 *
 * Here are some additional examples of using Timer with Roles 2 and 3:
 *
 * 1) Timing a section of code:
 *
 * \verbatim
   Timer my_timer;
   // do code here
   cout << my_timer.elapsedTime().milliseconds() << endl;  // report milliseconds elapsed
   // or...
   cout << my_timer.elapsedTime().seconds() << endl; // if you prefer the seconds elapsed to be reported instead
   \endverbatim
 *
 *
 * 2) Performing accurate frequency timing in a control loop of some sort.  The following code will run at a
 *    given frequency regardless of how long the code inside the loop takes to execute. This is similar to the
 *    built in functionality in Role 1, except it happens in the current thread rather than creating a dedicated
 *    thread:
 *
 * \verbatim
   Timer my_timer;
   while (true) {
	  my_timer += 10; // millisecond increment time, makes the loop run at 100 Hz
	  // do stuff
	  my_timer.waitUntilPassed(); // waits until the time has been reached, if your code in the loop
								  // takes more than 10 ms, then it won't wait at all
   }
   \endverbatim
 *
 * If you are tempted to create your own version of this class tailored to your specific needs, <b>STOP IT!</b> I
 * promise you that this class will cover 99.999% of your needs in robotics. If you really are that stubborn, try
 * to add the functionality to this class <b>without modifying the current functionality</b> instead of creating a
 * completely separate class that is almost exactly the same except for your one seemingly nonnegotiable addition. I
 * combined this class with several others which the authors thought needed to be their own classes but were perfectly
 * capable of coexisiting in this class.
 *
 */
// TODO: This class has a few minor bugs to fix. They all kind of revolve around situtations that can cause calls
// to clock_nanosleep with a negative time. This includes giving the ticking functions a negative value for the
// timestep, and a subtle bug where if someone is doing the "control loop" format, the timer instance (timepoint)
// is created outside the loop (i.e. persists between loop iterations), and the increment is accidentally negative
// (e.g. it is determined from a function or something). In this scenario, the timepoint created will continually
// decrement until it goes negative. These things may either be fixed by clarifying the documentation or by returning
// and printing an error (i.e. force the user to recognize that something is wrong and don't start the timer.) Also
// I have made a class that is dedicated to timing sections of code. Maybe this should be mentioned in the documentation?
class Timer {

	private:
		timespec _time;

		pthread_t thread_handle;
		bool ticking;
		double ticking_timestep_ms;

		void adjustForRollover();

		// These structures store data that is passed to the timing thread when an ordinary function is
		// provided as a user defined function.
		struct NormalFunctionThreadArgumentsWithElapsedTime {
			bool (*user_function)(double elapsed_ms);
			Timer* timer;
		};
		struct NormalFunctionThreadArguments {
			bool (*user_function)();
			Timer* timer;
		};

		// These structures store data that is passed to the timing thread when executing a user defined
		// function that is a member of a class at regular intervals.
		template<typename T>
		struct ClassSubmemberThreadArgumentsWithElapsedTime {
			T* object;
			bool (T::*user_function)(double elapsed_ms);
			Timer* timer;
		};
		template<typename T>
		struct ClassSubmemberThreadArguments {
			T* object;
			bool (T::*user_function)();
			Timer* timer;
		};

		static void* runNormalFunctionWithElapsedTime(void* args);
		static void* runNormalFunction(void* args);
		template<typename T>
		static void* runClassSubmemberWithElapsedTime(void* args);
		template<typename T>
		static void* runClassSubmember(void* args);

	public:
		Timer();
		Timer(double milliseconds, int seconds = 0);
		Timer(const Timer& other);
		~Timer();

		bool startTicking(bool (*user_function)(), double timestep_ms, int priority = 0);
		bool startTicking(bool (*user_function)(double), double timestep_ms, int priority = 0);
		template<typename T>
		bool startTicking(T& object, bool (T::*user_function)(), double timestep_ms, int priority = 0);
		template<typename T>
		bool startTicking(T& object, bool (T::*user_function)(double), double timestep_ms, int priority = 0);

		bool stopTicking();
		bool isTicking() const;
		double getTickingTimestep() const;
		void setTickingTimestep(double timestep_ms);

		bool getTime();
		void setTime(double milliseconds, int seconds = 0);
		void increment(double milliseconds);
		void decrement(double milliseconds);
		Timer elapsedTime() const;
		Timer updateTime();

		double milliseconds() const;
		double seconds() const;

		bool waitUntilPassed();
		bool waitUntilPassed(double milliseconds);

		static long getSysTime();
		static double getSysTimeHighPrecision();
		static long long getSysTimeNanos();
		static bool wait(long milliseconds);

		Timer& operator+=(double milliseconds);
		Timer& operator-=(double milliseconds);
		Timer& operator+=(const Timer& timer);
		Timer& operator-=(const Timer& timer);
		Timer operator+(double milliseconds);
		Timer operator-(double milliseconds);
		Timer operator+(const Timer& timer);
		Timer operator-(const Timer& timer);

		Timer& operator=(const Timer& other_timer);
		Timer operator=(double milliseconds);

		friend std::ostream& operator<<(std::ostream& stream, const Timer& time);

};


/**
 * @brief Begins calling user_function at a specified interval in a separate thread with the
 * specified priority. The user_function is a class member function and does not take any
 * parameters. It is repeatedly called until stopTicking() is called or the provided function
 * returns false. See the full description of this class for more details and examples of usage.
 * @param object An instance of the class which contains user_function as a member function and
 * for which you want to call user_function at regular intervals.
 * @param user_function A function pointer to a user defined class member function to call at
 * regular intervals. The function must return a boolean and take no parameters.
 * @param timestep_ms Initial time interval between calls to user_function in milliseconds
 * @param priority Scheduling priority for the thread which calls user_function. This can be from
 * 0 to 99 where 0 is the lowest priority and 99 is the highest.
 * @return True if the ticking has been successfully started
 */
template<typename T>
bool Timer::startTicking(T& object, bool (T::*user_function)(), double timestep_ms, int priority) {
	int ret = 0;

	if (ticking)
		return true;

	this->ticking_timestep_ms = timestep_ms;

	// Create a new instance of the thread argument structure and populate it with information.
	ClassSubmemberThreadArguments<T> args = {&object, user_function, this};

	// Allocate and copy the argument structure to the private member 'function_arguments' (the space will be deleted by runClassSubmember).
	void* function_arguments = new unsigned char[sizeof(ClassSubmemberThreadArguments<T>)];
	memcpy(function_arguments, &args, sizeof(ClassSubmemberThreadArguments<T>));

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

	// Start up a new thread executed which executes the runClassSubmember() function with the provided arguments.
	if ((ret = pthread_create(&thread_handle, &attr, &Timer::runClassSubmember<T>, function_arguments)) != 0) {
		ticking = false;
		std::cout << "TIMER: there was an error starting the thread for the timer: " << strerror(ret) << " ERROR" << std::endl;
		return false;
	}

	pthread_attr_destroy(&attr);

	return true;

}


/**
 * @brief This is the function that startTicking passes to the pthread_create() function which the
 * new thread will execute. It defines how and when the user defined function will get called for
 * the case of a class member function with no parameters.
 */
template<typename T>
void* Timer::runClassSubmember(void* args) {

	ClassSubmemberThreadArguments<T>* my_args = static_cast<ClassSubmemberThreadArguments<T>*>(args);
	T* object = my_args->object;
	bool (T::*user_function)() = my_args->user_function;
	Timer* timer = my_args->timer;

	delete[] static_cast<unsigned char*>(args); // delete the place in memory where the function arguments were stored

	// Store the current time.
	timer->getTime();

	// Keep looping until boolean ticking variable is set to false, then stop.
	while (timer->ticking && (object->*user_function)() ) {
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
 * specified priority. The user_function is a class member function and takes a double parameter
 * which will contain the elapsed time in milliseconds. It is repeatedly called until stopTicking()
 * is called or the provided function returns false. See the full description of this class for
 * more details and examples of usage.
 * @param object An instance of the class which contains user_function as a member function and
 * for which you want to call user_function at regular intervals.
 * @param user_function A function pointer to a user defined class member function to call at
 * regular intervals. The function must return a boolean and take a double parameter which will
 * contain the elapsed time from the time startTicking() was called in milliseconds.
 * @param timestep_ms Initial time interval between calls to user_function in milliseconds
 * @param priority Scheduling priority for the thread which calls user_function. This can be from
 * 0 to 99 where 0 is the lowest priority and 99 is the highest.
 * @return True if the ticking has been successfully started
 */
template<typename T>
bool Timer::startTicking(T &object, bool (T::*user_function)(double), double timestep_ms, int priority) {
	int ret = 0;

	if (ticking)
		return true;

	this->ticking_timestep_ms = timestep_ms;

	// Create a new instance of the thread argument structure and populate it with information.
	ClassSubmemberThreadArgumentsWithElapsedTime<T> args = {&object, user_function, this};

	// Allocate and copy the argument structure to the private member 'function_arguments' (the space will be deleted by runClassSubmember).
	void *function_arguments = new unsigned char[sizeof(ClassSubmemberThreadArgumentsWithElapsedTime<T>)];
	memcpy(function_arguments, &args, sizeof(ClassSubmemberThreadArgumentsWithElapsedTime<T>));

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

	// Start up a new thread executed which executes the runClassSubmember() function with the provided arguments.
	if ((ret = pthread_create(&thread_handle, &attr, &Timer::runClassSubmemberWithElapsedTime<T>, function_arguments)) != 0) {
		ticking = false;
		std::cout << "TIMER: there was an error starting the thread for the timer: " << strerror(ret) << " ERROR" << std::endl;
		return false;
	}

	pthread_attr_destroy(&attr);

	return true;
}


/**
 * @brief This is the function that startTicking passes to the pthread_create() function which the
 * new thread will execute. It defines how and when the user defined function will get called for
 * the case of a class member function with the elapsed time parameter.
 */
template<typename T>
void* Timer::runClassSubmemberWithElapsedTime(void* args) {
	
	ClassSubmemberThreadArgumentsWithElapsedTime<T> *my_args = static_cast<ClassSubmemberThreadArgumentsWithElapsedTime<T>*>(args);
	T *object = my_args->object;
	bool (T::*user_function)(double) = my_args->user_function;
	Timer *timer = my_args->timer;

	delete[] static_cast<unsigned char*>(args); // delete the place in memory where the function arguments were stored

	// Store the current time.
	Timer start_time;
	timer->getTime();
		
	// Keep looping until boolean ticking variable is set to false, then stop.
	while (timer->ticking && (object->*user_function)(start_time.elapsedTime().milliseconds()) ) {
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

#endif
