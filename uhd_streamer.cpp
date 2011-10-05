//
// Copyright 2011 Horizon Analog, Inc.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdlib.h>

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/single_usrp.hpp>

#include <boost/thread.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/interprocess/sync/file_lock.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>

#include <iostream>
#include <fstream>
#include <complex>
#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <cstdio>

#include <stdexcept>
#include <signal.h>
#include <float.h>
#include <errno.h>
#include <arpa/inet.h>

namespace
{
	namespace 		po = boost::program_options;
	const double	unspecified = -FLT_MAX;
	
	enum CommandResults
	{
		kCommandSuccess = 0,
		kCommandArgError = 1,
		kCommandUsrpNotFoundError = 2,
		kCommandFileError = 3,
		kCommandFifoError = 4,
		kCommandInterrupted = 5,
		kCommandUsrpConfigError = 6,
		kCommandUsrpReceiveError = 7,
		kCommandRetryLimitExceededError = 8,
		kCommandRestartServer = 9,
	};
	
	volatile bool gotSIGINT = false;
	volatile bool gotSIGHUP = false;
	
	void sig_handler(int sig)
	{
		if (sig == SIGINT)
			gotSIGINT = true;
		else if (sig == SIGHUP)
			gotSIGHUP = true;
	}
	
	void install_sig_handler(int signum, void (*new_handler)(int))
	{
		struct sigaction new_action;
		memset (&new_action, 0, sizeof (new_action));
		
		new_action.sa_handler = new_handler;
		sigemptyset (&new_action.sa_mask);
		new_action.sa_flags = 0;
		
		if (sigaction (signum, &new_action, 0) < 0)
		{
			std::cerr << boost::format ("Error %s from sigaction (install new)") % strerror(errno) << std::endl;
			throw std::runtime_error ("sigaction");
		}
	}
	
	void EnableRealtimeScheduling ()
	{
		assert(uhd::set_thread_priority_safe ());
	}
	
	class TimeOfDay
	{
		public:
			struct timeval _timeval;
		
		public:
			TimeOfDay ()
			{
				gettimeofday(&_timeval, NULL);
			}
	};
	
	double ElapsedTimeInSeconds (TimeOfDay startingTime, TimeOfDay endingTime = TimeOfDay())
	{
		return (double)(endingTime._timeval.tv_sec - startingTime._timeval.tv_sec) 
			 + (double)(endingTime._timeval.tv_usec - startingTime._timeval.tv_usec)*1e-6;
	}

	template <typename RangeType>
	double MiddleOfRange (RangeType range)
	{
		return (range.start() + range.stop())/2.0;		// @@@ mod range.step()
	}

	void printStat (std::ostream &output, const char *s, const char *name, size_t count)
	{
		if (count > 0)
			output << boost::format("%s: %u %s") % s % count % name << std::endl;
	}
	
	struct EventCounts
	{
		volatile uint64_t		_EVENT_CODE_BURST_ACK;
		volatile uint64_t		_EVENT_CODE_UNDERFLOW;
		volatile uint64_t		_EVENT_CODE_SEQ_ERROR;
		volatile uint64_t		_EVENT_CODE_TIME_ERROR;
		volatile uint64_t		_EVENT_CODE_UNDERFLOW_IN_PACKET;
		volatile uint64_t		_EVENT_CODE_SEQ_ERROR_IN_BURST;
		
		EventCounts ()
		{
			reset ();
		}

		void reset ()
		{
			_EVENT_CODE_BURST_ACK = 0;
			_EVENT_CODE_UNDERFLOW = 0;
			_EVENT_CODE_SEQ_ERROR = 0;
			_EVENT_CODE_TIME_ERROR = 0;
			_EVENT_CODE_UNDERFLOW_IN_PACKET = 0;
			_EVENT_CODE_SEQ_ERROR_IN_BURST = 0;
		}
		
		bool anyProblems () const { return _EVENT_CODE_UNDERFLOW != 0
										||  _EVENT_CODE_SEQ_ERROR != 0
										||  _EVENT_CODE_TIME_ERROR != 0
										||  _EVENT_CODE_UNDERFLOW_IN_PACKET != 0
										||  _EVENT_CODE_SEQ_ERROR_IN_BURST != 0; }
		
		void print (std::ostream &output, const char *s)
		{
			printStat (output, s, "events with EVENT_CODE_BURST_ACK", 			_EVENT_CODE_BURST_ACK);
			printStat (output, s, "events with EVENT_CODE_UNDERFLOW", 			_EVENT_CODE_UNDERFLOW);
			printStat (output, s, "events with EVENT_CODE_SEQ_ERROR", 			_EVENT_CODE_SEQ_ERROR);
			printStat (output, s, "events with EVENT_CODE_TIME_ERROR", 			_EVENT_CODE_TIME_ERROR);
			printStat (output, s, "events with EVENT_CODE_UNDERFLOW_IN_PACKET",	_EVENT_CODE_UNDERFLOW_IN_PACKET);
			printStat (output, s, "events with EVENT_CODE_SEQ_ERROR_IN_BURST",	_EVENT_CODE_SEQ_ERROR_IN_BURST);
		}

		void tally (const uhd::async_metadata_t &metadata)
		{
			_EVENT_CODE_BURST_ACK 			+= (metadata.event_code & uhd::async_metadata_t::EVENT_CODE_BURST_ACK) != 0;
			_EVENT_CODE_UNDERFLOW 			+= (metadata.event_code & uhd::async_metadata_t::EVENT_CODE_UNDERFLOW) != 0;
			_EVENT_CODE_SEQ_ERROR 			+= (metadata.event_code & uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR) != 0;
			_EVENT_CODE_TIME_ERROR			+= (metadata.event_code & uhd::async_metadata_t::EVENT_CODE_TIME_ERROR) != 0;
			_EVENT_CODE_UNDERFLOW_IN_PACKET	+= (metadata.event_code & uhd::async_metadata_t::EVENT_CODE_UNDERFLOW_IN_PACKET) != 0;
			_EVENT_CODE_SEQ_ERROR_IN_BURST	+= (metadata.event_code & uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR_IN_BURST) != 0;
		}
		
		void operator += (const EventCounts &otherCounts)
		{
			_EVENT_CODE_BURST_ACK += otherCounts._EVENT_CODE_BURST_ACK;
			_EVENT_CODE_UNDERFLOW += otherCounts._EVENT_CODE_UNDERFLOW;
			_EVENT_CODE_SEQ_ERROR += otherCounts._EVENT_CODE_SEQ_ERROR;
			_EVENT_CODE_TIME_ERROR += otherCounts._EVENT_CODE_TIME_ERROR;
			_EVENT_CODE_UNDERFLOW_IN_PACKET += otherCounts._EVENT_CODE_UNDERFLOW_IN_PACKET;
			_EVENT_CODE_SEQ_ERROR_IN_BURST += otherCounts._EVENT_CODE_SEQ_ERROR_IN_BURST;
		}
	};
	
	class EventMonitor
	{
	  private:
	  	uhd::usrp::single_usrp::sptr	 	_usrp;
	  	uhd::device::sptr					_device;
		boost::thread						_thread;
		volatile bool						_stop;
		uhd::time_spec_t 					_startingTime;
		EventCounts							_eventCounts;

	  public:
		typedef boost::shared_ptr<EventMonitor> sptr;

		EventCounts eventCounts () { return _eventCounts; }
		const EventCounts eventCounts () const { return _eventCounts; }
		
	  	EventMonitor (uhd::usrp::single_usrp::sptr usrp)
	  	: _usrp (usrp)
		, _device(usrp->get_device())
	  	, _thread()
	  	, _stop(false)
	  	, _startingTime()
	  	, _eventCounts()
	  	{}
	  	
	  	~EventMonitor ()
	  	{
	  		Stop ();
	  	}
	  	
		void Start ()
		{
			Stop ();
			_stop = false;
			
			_thread = boost::thread(boost::bind(&EventMonitor::Monitor, this));
		}
		
		void Reset (uhd::time_spec_t startingTime)
		{
			_startingTime = startingTime;
			_eventCounts.reset ();
		}
		
		void Stop ()
		{
			_stop = true;
			_thread.join();
		}
	
	  private:
		void Monitor ()
		{
			EnableRealtimeScheduling ();
			
			while (!_stop)
			{
				uhd::async_metadata_t 		metadata;
    			if (_device->recv_async_msg(metadata))
    			{
    				if (!metadata.has_time_spec || metadata.time_spec >= _startingTime)
    					_eventCounts.tally (metadata);
    			}
			}
		}
			
	};
	
	template <typename T>
	class MonitoredValue
	{
		private:
			bool		_changed;
			T			_value;
		
		public:
			MonitoredValue ()
			 :	_changed(true)
			 ,	_value ()
			{}

			bool changed () const	{ return _changed; }
			
			operator T () { _changed = false; return _value; }
			T value () { _changed = false; return _value; }
			
			void operator = (T value)
			{
				if (value != _value)
				{
					_changed = true;
					_value = value;
				}
			}
	};
	
	class ManagedUsrp
	{
		public:
			typedef boost::shared_ptr<ManagedUsrp>	sptr;
			
			bool							_verbose;
			uhd::usrp::single_usrp::sptr	usrp;
			
			std::string						_deviceArgs;
			MonitoredValue<std::string>		clockSource;
			bool							usingExternalClock;
			MonitoredValue<double>			txFrequency;
			MonitoredValue<double>			txOffset;
			MonitoredValue<double>			txGain;
			MonitoredValue<double>			txBandwidth;
			MonitoredValue<double>			txSampleRate;
			MonitoredValue<std::string>		txAntenna;
	  		EventMonitor::sptr				txMonitor;
			
			MonitoredValue<double>			rxFrequency;
			MonitoredValue<double>			rxOffset;
			MonitoredValue<double>			rxRefFrequency;
			MonitoredValue<double>			rxGain;
			MonitoredValue<double>			rxBandwidth;
			MonitoredValue<double>			rxSampleRate;
			MonitoredValue<std::string>		rxAntenna;
						
			
			ManagedUsrp (bool verbose, uhd::usrp::single_usrp::sptr theUsrp, std::string device)
			 :	_verbose(verbose)
			 ,	usrp (theUsrp)
			 ,	_deviceArgs (device)
			 ,	usingExternalClock(false)
			{}
			
			void ConfigureClock ()
			{
				if (clockSource.changed())
				{
					if (_verbose)
						std::cout << boost::format ("\"%s\".clockSource = \"%s\"\n") % _deviceArgs % clockSource.value();
					
					usingExternalClock = clockSource.value() == "EXT";
					if (clockSource.value() == "INT")
						usrp->set_clock_config(uhd::clock_config_t::internal());
					else if (usingExternalClock)
						usrp->set_clock_config(uhd::clock_config_t::external());
					else
						throw std::runtime_error("unknown clock type: " + clockSource.value());

				}
			}
			
			bool ConfigureTx ()
			{
				if (!txMonitor)
				{
					txMonitor = EventMonitor::sptr(new EventMonitor(usrp));
					txMonitor->Start ();
				}
				
				bool	frequencyChanged = txFrequency.changed() || txOffset.changed();
				if (frequencyChanged)
				{
					if (_verbose)
					{
						std::cout << boost::format ("\"%s\".txFrequency = \"%16.12e\"\n") % _deviceArgs % txFrequency.value();
						std::cout << boost::format ("\"%s\".txOffset = \"%16.12e\"\n") % _deviceArgs % txOffset.value();
					}
					
					uhd::tune_result_t tuneResult = usrp->set_tx_freq(uhd::tune_request_t(txFrequency.value(), txOffset.value()));
					
					double actualFrequency = round(usrp->get_tx_freq());
    				if (actualFrequency != round(txFrequency))
    				{
						std::cerr << boost::format ("\"%s\".txFrequency = %16.12e unsupported, replaced with %16.12e\n") % _deviceArgs % txFrequency % actualFrequency;
						txFrequency = 0;
						return false;
					}
				}
				if (txGain.changed())
				{
					double gain = txGain.value();
					if (gain == unspecified)
						gain = MiddleOfRange(usrp->get_tx_gain_range());
					if (_verbose)
						std::cout << boost::format ("\"%s\".txGain = %g\n") % _deviceArgs % gain;
					
					usrp->set_tx_gain(gain);
					if (fabs(usrp->get_tx_gain() - gain) > 0.05)
					{
						std::cerr << boost::format ("\"%s\".txGain = %g unsupported, replaced with %g\n") % _deviceArgs % gain % usrp->get_tx_gain();
						txGain = 0;
						return false;
					}
				}

				if (txBandwidth.changed())
				{
					double bandwidth = txBandwidth.value();
					if (bandwidth != unspecified)
					{
						if (_verbose)
							std::cout << boost::format ("\"%s\".txBandwidth = %g\n") % _deviceArgs % bandwidth;
						
						usrp->set_tx_bandwidth(bandwidth);
					}
				}

				if (txSampleRate.changed())
				{
					if (_verbose)
						std::cout << boost::format ("\"%s\".txSampleRate = %26.14g\n") % _deviceArgs % txSampleRate.value();
   					
   					usrp->set_tx_rate(txSampleRate);
   					
   					double		actualSampleRate = usrp->get_tx_rate();
					if (fabs(actualSampleRate - txSampleRate) > 1e-6)
					{
						std::cerr << boost::format ("\"%s\".txSampleRate = %26.14g unsupported, replaced with %26.14g, differs by %26.14g\n") 
														% _deviceArgs % txSampleRate % actualSampleRate % (actualSampleRate - txSampleRate);
						txSampleRate = 0;
						return false;
					}
				}
    				
    			if (frequencyChanged)
    			{
					TimeOfDay 			startTime;
				
					while (!usrp->get_tx_sensor("lo_locked").to_bool())
					{
						if (ElapsedTimeInSeconds (startTime) > 1.0)
						{
							std::cerr << boost::format ("\"%s\".txFrequency = \"%16.12e\". Local oscillator failed to lock.\n") % _deviceArgs % txFrequency.value();
    						txFrequency = 0;
							return false;
						}
					}
				}
				return true;
			}
			
			bool ConfigureRx ()
			{
				bool	frequencyChanged = rxFrequency.changed() || rxOffset.changed();
				if (frequencyChanged)
				{
  					if (_verbose)
					{
						std::cout << boost::format ("\"%s\".rxFrequency = \"%16.12e\"\n") % _deviceArgs % rxFrequency.value();
						std::cout << boost::format ("\"%s\".rxOffset = \"%16.12e\"\n") % _deviceArgs % rxOffset.value();
					}
 					
					uhd::tune_result_t tuneResult = usrp->set_rx_freq(uhd::tune_request_t(rxFrequency.value(), rxOffset.value()));

  					if (_verbose)
						std::cerr << boost::format ("\"%s\".tuneResult = %s\n") % _deviceArgs % tuneResult.to_pp_string();

    				double actualFrequency = round(usrp->get_rx_freq());
    				if (fabs(actualFrequency - round(rxFrequency)) > 15)
    				{
						std::cerr << boost::format ("\"%s\".rxFrequency = %16.12e unsupported, replaced with %16.12e\n") % _deviceArgs % rxFrequency % actualFrequency;
    					rxFrequency = 0;
    					return false;
    				}
    			}
    			
				if (rxGain.changed())
				{
					double gain = rxGain.value();
					if (gain == unspecified)
						gain = MiddleOfRange(usrp->get_rx_gain_range());
					if (_verbose)
						std::cout << boost::format ("\"%s\".rxGain = %g\n") % _deviceArgs % gain;
   					
					usrp->set_rx_gain(gain);
					if (fabs(usrp->get_rx_gain() - gain) > 0.05)
					{
						std::cerr << boost::format ("\"%s\".rxGain = %g unsupported, replaced with %g\n") % _deviceArgs % gain % usrp->get_rx_gain();
						rxGain = 0;
						return false;
					}
				}

				if (rxBandwidth.changed())
				{
					double bandwidth = rxBandwidth.value();
					if (bandwidth != unspecified)
					{
						if (_verbose)
							std::cout << boost::format ("\"%s\".rxBandwidth = %g\n") % _deviceArgs % bandwidth;
						
						usrp->set_rx_bandwidth(bandwidth);
					}
				}

				if (rxSampleRate.changed())
				{
					if (_verbose)
						std::cout << boost::format ("\"%s\".rxSampleRate = %26.14g\n") % _deviceArgs % rxSampleRate.value();
   					
					if (rxSampleRate == 100e6)
    					usrp->set_rx_rate(100e6/4);	// Must be using custom N210 FPGA
					else
					{
    					usrp->set_rx_rate(rxSampleRate);
    					
    					double		actualSampleRate = usrp->get_rx_rate();
						if (fabs(actualSampleRate - rxSampleRate) > 1e-6)
						{
							std::cerr << boost::format ("\"%s\".rxSampleRate = %26.14g unsupported, replaced with %26.14g, differs by %26.14g\n") 
														% _deviceArgs % rxSampleRate % actualSampleRate % (actualSampleRate - rxSampleRate);
    						rxSampleRate = 0;
    						return false;
    					}
    				}
				}
				
    			if (frequencyChanged)
    			{
					TimeOfDay 			startTime;
				
					while (!usrp->get_rx_sensor("lo_locked").to_bool())
					{
						if (ElapsedTimeInSeconds (startTime) > 1.0)
						{
							std::cerr << boost::format ("\"%s\".rxFrequency = \"%16.12e\". Local oscillator failed to lock.\n") % _deviceArgs % rxFrequency.value();
    						rxFrequency = 0;
							return false;
						}
					}
				}
				return true;
			}
	};
	
	class UsrpCache
	{
		private:
			typedef std::map<std::string, ManagedUsrp::sptr> 	DeviceMap;
			
			bool				_verbose;
			DeviceMap			_devices;
			bool				_usePPS;
			bool				_synchronized;
			
		public:
			UsrpCache (bool verbose)
			 :	_verbose (verbose)
			 ,	_devices()
			 ,	_usePPS (false)
			 ,	_synchronized (false)
			{}
			
			ManagedUsrp::sptr GetUsrp (std::string deviceName, bool fullRate, std::string clockSource, std::ostream &output)
			{
				DeviceMap::iterator			existingDevice = _devices.find(deviceName);
				ManagedUsrp::sptr			managedUsrp;
				
				if (existingDevice != _devices.end())
				{
					bool	previousFullRate = existingDevice->second->rxSampleRate == 100e6;
					if (fullRate != previousFullRate)
					{
						_devices.erase(existingDevice);
						existingDevice = _devices.end();
					}
				}
				
				if (existingDevice == _devices.end())
				{
					// UHD_SWAP_RX_DSPS is used in uhd/lib/usrp/usrp2/usrp2_impl.cpp
					int result = fullRate ? putenv("UHD_SWAP_RX_DSPS=1") : putenv("UHD_SWAP_RX_DSPS=0");
					if (result != 0)
						std::cerr << boost::format ("putenv return error %s\n") % strerror(errno);
					
					uhd::usrp::single_usrp::sptr usrp;
					try
					{
						usrp = uhd::usrp::single_usrp::make(deviceName);
						if (usrp)
						{
							if (_verbose)
								std::cout << boost::format ("Adding \"%s\", fullRate %d\n") % deviceName % fullRate;
							
							managedUsrp = ManagedUsrp::sptr(new ManagedUsrp(_verbose, usrp, deviceName));
							_devices[deviceName] = managedUsrp;
							_synchronized = false;
						}
					}
					catch (std::runtime_error e)
					{
						output << e.what() << std::endl;
					}
				}
				else
				{
					if (_verbose)
						std::cout << boost::format ("Reusing \"%s\", fullRate %d\n") % deviceName % fullRate;
					
					managedUsrp = existingDevice->second;
				}
				managedUsrp->clockSource = clockSource;
				if (managedUsrp->clockSource.changed())
				{
					managedUsrp->ConfigureClock ();
					_synchronized = false;
				}
				return managedUsrp;
			}
			
			void SynchronizeClocks (bool usePPS)
			{
				if (!_synchronized || usePPS != _usePPS)
				{
					if (_verbose)
					{
						if (usePPS)
							std::cout << "Synchronizing clocks...\n";
						else
							std::cout << "Resetting clocks...\n";
					}
					
					_synchronized = true;
					_usePPS = usePPS;
					if (_usePPS)
						ZeroAllTimesAtNextPPS ();
					else
						ZeroAllTimesNow ();

					if (_verbose)
						std::cout << "...Done.\n";
				}
			}

		private:
			void ZeroAllTimesAtNextPPS ()
			{
				DeviceMap::iterator it;

				for (it = _devices.begin(); it != _devices.end(); ++it)
				{
					if (it->second->usingExternalClock)
						it->second->usrp->set_time_now(uhd::time_spec_t(10.0));
					else
						it->second->usrp->set_time_now(uhd::time_spec_t(0.0));
				}
				while (true)
				{
					for (it = _devices.begin(); it != _devices.end(); ++it)
					{
						if (it->second->usingExternalClock)
							it->second->usrp->set_time_next_pps(uhd::time_spec_t(0.0));
					}
					
					// if the PPS happens inbetween calls to set_time_next_pps, the USRPs will be one second apart.
					
					uhd::time_spec_t		lowest;
					uhd::time_spec_t		highest;
					
					while (true)
					{
						bool	allHaveReset = true;
						lowest = 10.0;
						highest = 0.0;
						for (it = _devices.begin(); it != _devices.end(); ++it)
						{
							if (it->second->usingExternalClock)
							{
								uhd::time_spec_t		time = it->second->usrp->get_time_now();
								if (time >= 10.0)
								{
									allHaveReset = false;
									break;
								}
								if (time < lowest)
									lowest = time;
								if (time > highest)
									highest = time;
							}
						}
						if (allHaveReset)
							break;
						
						// Wait for the times to get reset
						boost::this_thread::sleep(boost::posix_time::milliseconds(5));
					}
					if (highest - lowest < 0.1)
						break;
				}
			}
			
			void ZeroAllTimesNow ()
			{
				DeviceMap::iterator it;

				while (true)
				{
					for (it = _devices.begin(); it != _devices.end(); ++it)
						it->second->usrp->set_time_now(uhd::time_spec_t(0.0));
						
					uhd::time_spec_t		lowest = 10.0;
					uhd::time_spec_t		highest = 0.0;

					for (it = _devices.begin(); it != _devices.end(); ++it)
					{
						uhd::time_spec_t		time = it->second->usrp->get_time_now();
						if (time < lowest)
							lowest = time;
						if (time > highest)
							highest = time;
					}
					if (highest - lowest < 0.1)
						break;
				}
			}
	};
	
	class Transmitter
	{
	  private:
	  	ManagedUsrp::sptr					_usrp;
		size_t								_numSamplesInBuffer;
		uint32_t*							_buffer;	// Cheaper than std::vector<std::complex<int16_t>>
		boost::thread						_thread;
		volatile bool						_stopTransmitting;
	  	
	  public:
		typedef boost::shared_ptr<Transmitter> sptr;

		EventCounts eventCounts () const { return _usrp->txMonitor->eventCounts(); }
		
	  	Transmitter (ManagedUsrp::sptr managedUsrp)
	  	: _usrp (managedUsrp)
	  	, _numSamplesInBuffer(0)
	  	, _buffer(NULL)
	  	, _thread()
	  	, _stopTransmitting(false)
	  	{}
	  	
	  	~Transmitter ()
	  	{
	  		Stop(NULL);
	  		free(_buffer);
	  	}
	  	
		bool LoadFile (std::string filePath, std::ostream &errorOutput)
		{
			FILE	       *file = fopen(filePath.data(), "rb");
			if (file == NULL)
			{
				errorOutput << boost::format ("Error %s opening %s") % strerror(errno) % filePath.data() << std::endl;
				return false;
			}
			
			if (fseek (file, 0, SEEK_END) < 0)
			{
				errorOutput << boost::format ("Error %s seeing %s") % strerror(errno) % filePath.data() << std::endl;
				fclose(file);
				return false;
			}
			
			long		numBytes = ftell (file);
			if (numBytes < 0)
			{
				errorOutput << boost::format ("Error %s ftell %s") % strerror(errno) % filePath.data() << std::endl;
				fclose(file);
				return false;
			}
			rewind (file);
			
			size_t		numSamples = numBytes/sizeof(uint32_t);
			
			free(_buffer);
			_numSamplesInBuffer = numSamples;
			_buffer = (uint32_t*) calloc(numSamples, sizeof(uint32_t));
			
			size_t		readSoFar = 0;
			while (readSoFar < numSamples)
			{
				size_t readThisTime = fread(&_buffer[readSoFar], sizeof(_buffer[0]), numSamples-readSoFar, file);
				if (readThisTime == 0)
				{
					int error = ferror(file);
					if (error == EINTR)
						continue;
						
					fclose(file);
					return false;
				}
				readSoFar -= readThisTime;
			}
			fclose(file);
	
			return true;
		}
		
		void Start (uhd::time_spec_t startingTime, uhd::time_spec_t txDuration, double timeoutInSeconds)
		{
			_stopTransmitting = false;
			_usrp->txMonitor->Reset(startingTime);
			_thread = boost::thread(boost::bind(&Transmitter::Transmit, this, startingTime, txDuration, timeoutInSeconds));
		}
		
		void Wait (std::ostream *output)
		{
			_thread.join();
			if (output && _usrp->txMonitor->eventCounts().anyProblems())
			{
				_usrp->txMonitor->eventCounts().print(*output, "Transmit");
			}
		}

		void Stop (std::ostream *output)
		{
			_stopTransmitting = true;
			Wait(output);
		}
	
		void Transmit(uhd::time_spec_t startingTime, uhd::time_spec_t txDuration, double timeoutInSeconds)
		{
			
			bool	continuous = txDuration == std::numeric_limits<double>::max();
			size_t	numSamples =  continuous ? std::numeric_limits<size_t>::max() 
											 : (size_t) txDuration.get_tick_count(_usrp->txSampleRate);
			
			EnableRealtimeScheduling ();
			
			uhd::tx_metadata_t	metadata;
			
			metadata.start_of_burst = true;
			metadata.end_of_burst = numSamples == 0;
			metadata.has_time_spec  = true;
			metadata.time_spec = startingTime;
			
			while (!metadata.end_of_burst)
			{
				assert(numSamples > 0);
				
				if (_stopTransmitting || gotSIGINT || gotSIGHUP)
					numSamples = 1;	// Zero makes more sense, but UHD sometimes sends 1 anyway, triggering the WARNING below.
				
				size_t numToSend = _numSamplesInBuffer;
				if (numToSend >= numSamples)
				{
					numToSend = numSamples;
					metadata.end_of_burst = true;
				}
				if (false)
					std::cerr << boost::format ("Tx %d of %d at time %12.9g through %12.9g")
											% numToSend 
											% _numSamplesInBuffer 
											% metadata.time_spec.get_real_secs() 
											% (metadata.time_spec + uhd::time_spec_t(0, numToSend, _usrp->txSampleRate)).get_real_secs() << std::endl;

				size_t numSent = _usrp->usrp->get_device()->send(_buffer, numToSend, metadata, 
																	uhd::io_type_t::COMPLEX_INT16, 
																	uhd::device::SEND_MODE_FULL_BUFF,
																	timeoutInSeconds + 0.1);
				if (numSent != numToSend)
				{
					std::cerr << boost::format ("WARNING: %d samples sent out of %d requested.\n") % numSent % numToSend;
					if (numSent == 0)
						break;		// Probably timed out.
				}
				if (!continuous)
					numSamples = numSamples - numSent;
				
				metadata.start_of_burst = false;
				metadata.has_time_spec  = false;
				timeoutInSeconds = 0.0;
			}
		}
	};
	
	class Receiver
	{
	  public:
		enum ReceiveResult
		{
			kReceiveNotFinished,
			kReceiveSuccessful,
			kReceiveDataMissing,
			kReceiveFailed
		};
		
	  private:
	  	ManagedUsrp::sptr					_usrp;
	  	long								_delayMultiple;
		size_t								_numSamplesInBuffer;
		size_t								_numSamplesReceived;
		uint32_t*							_buffer;	// Cheaper than std::vector<std::complex<int16_t>>
		boost::thread						_thread;
		ReceiveResult						_receiveResult;
		
	  public:
		typedef boost::shared_ptr<Receiver> sptr;
	
		Receiver(ManagedUsrp::sptr managedUsrp, double sampleTime, long delayMultiple)
		: _usrp(managedUsrp)
		, _delayMultiple(delayMultiple)
		, _numSamplesInBuffer(sampleTime * managedUsrp->rxSampleRate)
		, _numSamplesReceived(0)
		, _buffer((uint32_t*) calloc(_numSamplesInBuffer, sizeof(uint32_t)))
	  	, _thread()
	  	, _receiveResult(kReceiveNotFinished)
		{}
		
		~Receiver ()
		{
			free(_buffer);
		}
		
		size_t NumSamplesStillNeeded () const { return _numSamplesInBuffer - _numSamplesReceived; }
		size_t NumSamplesToReceiveThisTime () const
		{
 			size_t	numSamplesToReceive = NumSamplesStillNeeded();
 			
			if (_usrp->rxSampleRate == 100e6)
			{
				size_t	maxSamples = pow(2.0, 18);	// The size of the N210's SRAM in samples.
			
				if (_delayMultiple != 0)
				{
					// Read an integral number of delay multiples so each block starts at the same
					// phase relationship between the sample clock and the local oscillator.
					maxSamples = (maxSamples / _delayMultiple) * _delayMultiple;
				}
				if (numSamplesToReceive > maxSamples)
					numSamplesToReceive = maxSamples;
			}
			return numSamplesToReceive;
		}
		size_t NumSamplesAlreadyReceived () const { return _numSamplesReceived; }
		uhd::time_spec_t NumSecondsAlreadyReceived () const { return uhd::time_spec_t(0, _numSamplesReceived, _usrp->rxSampleRate); }
		uhd::time_spec_t NumSecondsToReceiveThisTime () const { return uhd::time_spec_t(0, NumSamplesToReceiveThisTime(), _usrp->rxSampleRate); }
		
		void Start (uhd::time_spec_t startingTime, std::ostream *errorOutput, double timeoutInSeconds)
		{
    		// Must issue streamCommand before starting to transmit. Otherwise, transmit flow control
    		// makes it very difficult to get the necessary responses.	@@@ May not be true with latest UHD code.
    		//
 			size_t	numSamplesToReceive = NumSamplesToReceiveThisTime();
   		
			if (false)
				std::cerr << boost::format ("Rx %d at %d through %d at time %12.9g through %12.9g")
											% numSamplesToReceive 
											% _numSamplesReceived 
											% (_numSamplesReceived + numSamplesToReceive)
											% startingTime.get_real_secs() 
											% (startingTime + uhd::time_spec_t(0, numSamplesToReceive, _usrp->rxSampleRate)).get_real_secs() << std::endl;
											
			uhd::stream_cmd_t streamCommand(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
			streamCommand.num_samps = numSamplesToReceive;
			streamCommand.stream_now = false;
			streamCommand.time_spec = startingTime;
			_usrp->usrp->issue_stream_cmd(streamCommand);

	  		_receiveResult = kReceiveNotFinished;
			_thread = boost::thread(boost::bind(&Receiver::Receive, this, numSamplesToReceive, errorOutput, timeoutInSeconds));
		}
		
		ReceiveResult Wait ()
		{
			_thread.join();
			return _receiveResult;
		}

		void Receive (size_t numSamplesToReceive, std::ostream *errorOutput, double timeoutInSeconds)
		{
			EnableRealtimeScheduling ();
			
			uhd::rx_metadata_t metadata;
				
			size_t numReceived = _usrp->usrp->get_device()->recv(_buffer + _numSamplesReceived, numSamplesToReceive, metadata,
																 uhd::io_type_t::COMPLEX_INT16, uhd::device::RECV_MODE_FULL_BUFF,
																 timeoutInSeconds + 0.1);
			_numSamplesReceived = _numSamplesReceived + numReceived;

			switch (metadata.error_code)
			{
			  case uhd::rx_metadata_t::ERROR_CODE_NONE:
				break;
			  case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
				_receiveResult = kReceiveDataMissing;
				std::cerr << boost::format("Rx got overrun") << std::endl;
				*errorOutput << boost::format("Rx got overrun") << std::endl;
				break;
			  case uhd::rx_metadata_t::ERROR_CODE_TIMEOUT:
				_receiveResult = kReceiveDataMissing;
				std::cerr << boost::format("Rx timeout") << std::endl;
				*errorOutput << boost::format("Rx timeout") << std::endl;
				break;
			  case uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND:
				_receiveResult = kReceiveDataMissing;
				std::cerr << boost::format("Rx late command") << std::endl;
				*errorOutput << boost::format("Rx late command") << std::endl;
				break;
			  default:
				_receiveResult = kReceiveFailed;
				std::cerr << boost::format("Rx got error code 0x%x")  % metadata.error_code << std::endl;
				*errorOutput << boost::format("Rx got error code 0x%x")  % metadata.error_code << std::endl;
				break;
			}
			assert(!metadata.more_fragments);
			
			if (numReceived == numSamplesToReceive)
			{
				if (_receiveResult == kReceiveNotFinished)
					_receiveResult = kReceiveSuccessful;
			}
			else
			{
				std::cerr << boost::format("Rx received %d samples out of %d requested")  % numReceived % numSamplesToReceive << std::endl;
				*errorOutput << boost::format("Rx received %d samples out of %d requested")  % numReceived % numSamplesToReceive << std::endl;
				if (_receiveResult == kReceiveNotFinished)
					_receiveResult = kReceiveDataMissing;	// Do it over.
			}
		}
		
		bool writeSamples (FILE *file)
		{
			size_t numWritten = 0;
			while (numWritten < _numSamplesInBuffer)
			{
				size_t	numRemaining = _numSamplesInBuffer - numWritten;
				size_t	numWrittenThisTime = fwrite(&_buffer[numWritten], sizeof(_buffer[0]), numRemaining, file);
				if (numWrittenThisTime == 0)
				{
					int error = ferror(file);
					if (error == EINTR)
						continue;
					return false;
				}
				numWritten += numWrittenThisTime;
			}
			return true;
		}
	};

	// Command line options
	struct DeviceOptions
	{
		std::string			device;
		double				centerFrequency;
		double				loOffset;
		double				refFrequency;
		double				sampleRate;
		double				gain;
		double				bandwidth;
		std::string			clockSource;
		std::string			antenna;
		std::string			filePath;

		bool Validate (std::string name, std::ostream &output)
		{
			bool			optionsAreValid = true;
			
			if (device.length() > 0)
			{
				if (centerFrequency == unspecified)
				{
					output << "Missing " << name << "Frequency" << std::endl;
					optionsAreValid = false;
				}
			}
			else
			{
				if (centerFrequency != unspecified)
					output << "Warning:  " << name << "Frequency ignored" << std::endl;
				if (gain != unspecified)
					output << "Warning:  " << name << "Gain ignored" << std::endl;
				if (bandwidth != unspecified)
					output << "Warning:  " << name << "Bandwidth ignored" << std::endl;
				if (antenna.length() > 0)
					output << "Warning:  " << name << "Antenna ignored" << std::endl;
				if (filePath.length() > 0)
					output << "Warning:  " << name << "File ignored" << std::endl;
			}
			return optionsAreValid;
		}
	};
	
	struct Options
	{
		DeviceOptions				t1;
		DeviceOptions				t2;
		DeviceOptions				rx;
		
		double						rxSeconds;
		double						rxDelay;
		double						delay;
		long						delayMultiple;
		double						clientWaitTime;
		bool						synchronize;
		bool 						verbose;
		bool						server;
		bool						client;
		bool						help;
		
		Options (int argc, char **argv, std::ostream &output)
		{
			/*
				--t1Device <device> --t1Antenna a --t1Gain g --t1Frequency f --t1File path --t1SampleRate r --t1Clock c
				--t2Device <device> --t2Antenna a --t2Gain g --t2Frequency f --t2File path --t2SampleRate r --t2Clock c
				--rxDevice <device> --rxAntenna a --rxGain g --rxFrequency f --rxFile path --rxSampleRate r --rxClock c
				--rxSeconds n  --verbose tf	--synchronize tf
			*/
			po::options_description 	description("Allowed options");
			description.add_options()
				("help", "help message")
				
				("server", "Become server process.")
				("client", "Route request to server process.")
	
				("t1Device", po::value<std::string>(&t1.device)->default_value(""), "Transmitter 1 uhd device address args")
				("t1Frequency", po::value<double>(&t1.centerFrequency)->default_value(unspecified), "Rf center frequency in Hz for transmitter 1")
				("t1Offset", po::value<double>(&t1.loOffset)->default_value(0.0), "Local oscillator offset for transmitter 1")
				("t1SampleRate", po::value<double>(&t1.sampleRate)->default_value(100e6/4), "Sample rate for transmitter 1")
				("t1Gain", po::value<double>(&t1.gain)->default_value(unspecified), "Transmitter 1 gain (defaults to middel of its range)")
				("t1Bandwidth", po::value<double>(&t1.bandwidth)->default_value(unspecified), "Transmitter 1 bandwidth")
				("t1Clock", po::value<std::string>(&t1.clockSource)->default_value("INT"), "Source of transmitter 1's reference clock and PPS (INT, EXT)")
				("t1Antenna", po::value<std::string>(&t1.antenna)->default_value(""), "Antenna to use for transmitter 1")
				("t1File", po::value<std::string>(&t1.filePath)->default_value(""), "File with data to transmit for transmitter 1")
	
				("t2Device", po::value<std::string>(&t2.device)->default_value(""), "Transmitter 2 uhd device address args")
				("t2Frequency", po::value<double>(&t2.centerFrequency)->default_value(unspecified), "Rf center frequency in Hz for transmitter 2")
				("t2Offset", po::value<double>(&t2.loOffset)->default_value(0.0), "Local oscillator offset for transmitter 2")
				("t2SampleRate", po::value<double>(&t2.sampleRate)->default_value(100e6/4), "Sample rate for transmitter 2")
				("t2Gain", po::value<double>(&t2.gain)->default_value(unspecified), "Transmitter 2 gain (defaults to middel of its range)")
				("t2Bandwidth", po::value<double>(&t2.bandwidth)->default_value(unspecified), "Transmitter 2 bandwidth")
				("t2Clock", po::value<std::string>(&t2.clockSource)->default_value("INT"), "Source of transmitter 2's reference clock and PPS (INT, EXT)")
				("t2Antenna", po::value<std::string>(&t2.antenna)->default_value(""), "Antenna to use for transmitter 2")
				("t2File", po::value<std::string>(&t2.filePath)->default_value(""), "File with data to transmit for transmitter 2")
	
				("rxDevice", po::value<std::string>(&rx.device)->default_value(""), "Receiver uhd device address args")
				("rxFrequency", po::value<double>(&rx.centerFrequency)->default_value(unspecified), "Rf center frequency in Hz for receiver")
				("rxOffset", po::value<double>(&rx.loOffset)->default_value(0.0), "Local oscillator offset for receiver")
				("rxRefFrequency", po::value<double>(&rx.refFrequency)->default_value(10e6), "Modified DBSRX2 only: Rf reference frequency in Hz for receiver")
				("rxSampleRate", po::value<double>(&rx.sampleRate)->default_value(100e6/4), "Sample rate for receiver")
				("rxGain", po::value<double>(&rx.gain)->default_value(unspecified), "Receiver gain (defaults to middel of its range)")
				("rxBandwidth", po::value<double>(&rx.bandwidth)->default_value(unspecified), "Receivers bandwidth")
				("rxClock", po::value<std::string>(&rx.clockSource)->default_value("INT"), "Source of receiver's reference clock and PPS (INT, EXT)")
				("rxAntenna", po::value<std::string>(&rx.antenna)->default_value(""), "Antenna to use for receiver")
				("rxFile", po::value<std::string>(&rx.filePath)->default_value(""), "File to store received data")
	
				("rxSeconds", po::value<double>(&rxSeconds)->default_value(0.25), "number of seconds to receive")
				("synchronize", po::value<bool>(&synchronize)->default_value(false), "Synchronize transmit and receive times")
				("verbose", po::value<bool>(&verbose)->default_value(false), "Verbose output")
				("delay", po::value<double>(&delay)->default_value(0.02), "Receive, transmit delay for synchronization in seconds.")
				("delayMultiple", po::value<long>(&delayMultiple)->default_value(0), "Required delay multiple in sample clock ticks to keep samples in phase across receives.")
				("rxDelay", po::value<double>(&rxDelay)->default_value(0.0), "Extra receive delay for synchronization in seconds.")
				("waitTime", po::value<double>(&clientWaitTime)->default_value(unspecified), "(Internal) Time client spent waiting for access in seconds.")
				;
				
			po::variables_map vm;
			po::store(po::parse_command_line(argc, argv, description), vm);
			po::notify(vm);
		
			server = vm.count("server") > 0;
			client = vm.count("client") > 0;
			help = vm.count("help") > 0;
			if (help)
				output << boost::format("UHD Streamer %s") % description << std::endl;
		}
		
		bool Validate (std::ostream &output)
		{
			if (help)
				return false;
			
			bool	optionsAreValid = true;
			
			if (server && client)
			{
				output << "Can't be both --client and --server." << std::endl;
				optionsAreValid = false;
			}
			
			optionsAreValid &= t1.Validate("t1", output);
			optionsAreValid &= t2.Validate("t2", output);
			optionsAreValid &= rx.Validate("rx", output);
			if (!server && t1.device.length() == 0 && t2.device.length() ==0  && rx.device.length() == 0)
			{
				output << "Need to specify at least one of --t1Device, --t2Device, --rxDevice." << std::endl;
				optionsAreValid = false;
			}
			
			return optionsAreValid;
		}
	};	
		
	bool WriteSamples (std::string filePath, Receiver::sptr receiver, std::ostream &errorOutput)
	{
		FILE	       *file = fopen(filePath.data(), "wb");
		if (file == NULL)
		{
			errorOutput << boost::format ("Error %s opening %s") % strerror(errno)  % filePath.data() << std::endl;
			return false;
		}
		if (!receiver->writeSamples(file))
		{
			errorOutput << boost::format ("Error %s writing %s") % strerror(errno)  % filePath.data() << std::endl;
			return false;
		}
		fclose(file);
		
		return true;
	}

	CommandResults TransmitTransmitReceive (UsrpCache &usrpCache, Options options, std::ostream &output)
	{
		ManagedUsrp::sptr	t1;
		ManagedUsrp::sptr	t2;
		ManagedUsrp::sptr	rx;

		Transmitter::sptr 	transmitter1;
		Transmitter::sptr 	transmitter2;
		Receiver::sptr 		receiver;
		
		if (options.t1.device.length() > 0)
		{
			t1 = usrpCache.GetUsrp (options.t1.device, false, options.t1.clockSource, output);
			if (!t1)
				return kCommandUsrpNotFoundError;
			
			t1->txFrequency = options.t1.centerFrequency;
			t1->txOffset = options.t1.loOffset;
			t1->txAntenna = options.t1.antenna;
			t1->txGain = options.t1.gain;
			t1->txBandwidth = options.t1.bandwidth;
			t1->txSampleRate = options.t1.sampleRate;

			if (!t1->ConfigureTx())
				return kCommandUsrpConfigError;
			
			transmitter1 = Transmitter::sptr(new Transmitter(t1));
			if (!transmitter1->LoadFile (options.t1.filePath, output))
				return kCommandFileError;
			
			if (options.verbose)
			{
				output << boost::format ("Transmitter 1:  %s\n %s") % options.t1.device.data() % t1->usrp->get_pp_string().data() << std::endl;
				output << boost::format("Transmitting file %s at %g MHz") % options.t1.filePath % (options.t1.centerFrequency/1e6) << std::endl;
			}
		}
		
		if (options.t2.device.length() > 0)
		{
			t2 = usrpCache.GetUsrp (options.t2.device, false, options.t2.clockSource, output);
			if (!t2)
				return kCommandUsrpNotFoundError;
			
			t2->txFrequency = options.t2.centerFrequency;
			t2->txOffset = options.t2.loOffset;
			t2->txAntenna = options.t2.antenna;
			t2->txGain = options.t2.gain;
			t2->txBandwidth = options.t2.bandwidth;
			t2->txSampleRate = options.t2.sampleRate;

			if (!t2->ConfigureTx())
				return kCommandUsrpConfigError;
			
			transmitter2 = Transmitter::sptr(new Transmitter(t2));
			if (!transmitter2->LoadFile (options.t2.filePath, output))
				return kCommandFileError;
			
			if (options.verbose)
			{
				output << boost::format ("Transmitter 2:  %s\n %s") % options.t2.device.data() % t2->usrp->get_pp_string().data() << std::endl;
				output << boost::format("Transmitting file %s at %g MHz") % options.t2.filePath % (options.t2.centerFrequency/1e6) << std::endl;
			}
		}
		
		if (options.rx.device.length() > 0)
		{
			rx = usrpCache.GetUsrp (options.rx.device, options.rx.sampleRate == 100e6, options.rx.clockSource, output);
			if (!rx)
				return kCommandUsrpNotFoundError;
			
			rx->rxFrequency = options.rx.centerFrequency;
			rx->rxOffset = options.rx.loOffset;
			rx->rxRefFrequency = options.rx.refFrequency;
			rx->rxAntenna = options.rx.antenna;
			rx->rxGain = options.rx.gain;
			rx->rxBandwidth = options.rx.bandwidth;
			rx->rxSampleRate = options.rx.sampleRate;

			if (!rx->ConfigureRx())
				return kCommandUsrpConfigError;
			
			receiver = Receiver::sptr(new Receiver(rx, options.rxSeconds, options.delayMultiple));

			if (options.verbose)
			{
				output << boost::format ("Receiver:  %s\n %s") % options.rx.device.data() % rx->usrp->get_pp_string().data() << std::endl;
				output << boost::format("Receiving file %s at %g MHz") % options.rx.filePath % (options.rx.centerFrequency/1e6) << std::endl;
			}
		}
		assert (t1 || t2 || rx);
		
		int numSynchronizing = (t1 && t1->usingExternalClock)
							 + (t2 && t2->usingExternalClock)
							 + (rx && rx->usingExternalClock);
		
		bool synchronize = options.synchronize && (numSynchronizing > 1);
		
		usrpCache.SynchronizeClocks (synchronize);
		
		int numTries = 0;
		while (true)
		{
			if (gotSIGINT || gotSIGHUP)
				return kCommandInterrupted;
				
			uhd::time_spec_t 	t1Time;
			uhd::time_spec_t 	t2Time;
			uhd::time_spec_t 	rxTime;
			uhd::time_spec_t	baseDelay(options.delay);
			
			if (synchronize)
			{
				uhd::time_spec_t 	commonTime;
				
				if (t1 && t1->usingExternalClock)
					commonTime = t1->usrp->get_time_now();
				else if (t2 && t2->usingExternalClock)
					commonTime = t2->usrp->get_time_now();
				else if (rx && rx->usingExternalClock)
					commonTime = rx->usrp->get_time_now();
				else
					assert(false);
				
				if (options.delayMultiple != 0)
				{
					// Make the starting time an integral multiple of delayMultiple by adding to baseDelay.
					// This preserves the phase relationship between the local oscillators and sample clocks.
					//
					//@@@ Except it doesn't. There must be something else affecting the relationship.
					// 
					// Careful:  get_tick_count only returns the ticks for the fractional seconds.
					//
					long	startingTimeInTicks = boost::math::lround((commonTime + baseDelay).get_real_secs() * 100e6);
					long	numMultiples = (startingTimeInTicks + options.delayMultiple - 1) / options.delayMultiple;
					long	desiredStartingTimeInTicks = numMultiples * options.delayMultiple;
					long	extraDelayInTicks = desiredStartingTimeInTicks - startingTimeInTicks;
					
					baseDelay = baseDelay + uhd::time_spec_t (0, extraDelayInTicks, 100e6);
				}
				commonTime = commonTime + baseDelay;
				
				if (t1)
				{
					if (t1->usingExternalClock)
						t1Time = commonTime;
					else
						t1Time = t1->usrp->get_time_now() + baseDelay;
				}
				if (t2)
				{
					if (t2->usingExternalClock)
						t2Time = commonTime;
					else
						t2Time = t2->usrp->get_time_now() + baseDelay;
				}
				if (rx)
				{
					if (rx->usingExternalClock)
						rxTime = commonTime;
					else
						rxTime = rx->usrp->get_time_now() + baseDelay;
				}
			}
			else
			{
				if (t1)
					t1Time = t1->usrp->get_time_now() + baseDelay;
				if (t2)
					t2Time = t2->usrp->get_time_now() + baseDelay;
				if (rx)
					rxTime = rx->usrp->get_time_now() + baseDelay;
			}

			// Must start receiver before starting transmitters. If doing full duplex on one USRP2,
			// transmit flow control will cause timeouts when setting up the receive.
			// The results should be okay because actual start times are in the future.
			// @@@ May not be true with latest UHD code.
			//
			
			if (receiver)
			{
				uhd::time_spec_t	rxExtraDelay = uhd::time_spec_t(options.rxDelay) + receiver->NumSecondsAlreadyReceived();
				uhd::time_spec_t	rxDuration = receiver->NumSecondsToReceiveThisTime();
				uhd::time_spec_t 	txDuration = rxExtraDelay + rxDuration + uhd::time_spec_t(synchronize ? 0.0 : 2*options.delay);
		
				rxTime = rxTime + uhd::time_spec_t(rxExtraDelay);
				
				if (false && options.delayMultiple != 0 && transmitter1)
				{
					long	txTimeInTicks = boost::math::lround(t1Time.get_real_secs() * 100e6);
					long	rxTimeInTicks = boost::math::lround(rxTime.get_real_secs() * 100e6);
					std::cout << boost::format ("t1Time = %16d, rxTime = %16d\n") % txTimeInTicks % rxTimeInTicks;
					//std::cout << boost::format ("t1 rem = %16d, rx rem = %16d\n") 
					//							% (txTimeInTicks % options.delayMultiple)
					//							% (rxTimeInTicks % options.delayMultiple);
				}
				if (false)
				{
					std::cerr << boost::format ("rx %12.9g through %12.9g ") % rxTime.get_real_secs() % (rxTime + rxDuration).get_real_secs() << std::endl;
					std::cerr << boost::format ("t1 %12.9g through %12.9g ") % t1Time.get_real_secs() % (t1Time + txDuration).get_real_secs() << std::endl;
				}
				
				receiver->Start(rxTime, &output, (baseDelay + rxExtraDelay).get_real_secs());
				if (transmitter1)
					transmitter1->Start(t1Time, txDuration, baseDelay.get_real_secs());
				if (transmitter2)
					transmitter2->Start(t2Time, txDuration, baseDelay.get_real_secs());
	
				Receiver::ReceiveResult		result = receiver->Wait();
				if (transmitter1)
					transmitter1->Stop(&output);
				if (transmitter2)
					transmitter2->Stop(&output);

				switch (result)
				{
					case Receiver::kReceiveNotFinished:
						assert(result != Receiver::kReceiveNotFinished);	// Always fires.
					case Receiver::kReceiveSuccessful:
						if ((transmitter1 && transmitter1->eventCounts().anyProblems())
						 || (transmitter2 && transmitter2->eventCounts().anyProblems()))
						{
							if (transmitter1 && transmitter1->eventCounts().anyProblems())
								transmitter1->eventCounts().print (std::cerr, "T1");
							if (transmitter2 && transmitter2->eventCounts().anyProblems())
								transmitter2->eventCounts().print (std::cerr, "T2");
							
							output << boost::format ("Repeating due to transmitter problems.") << std::endl;
							std::cerr << boost::format ("Repeating due to transmitter problems.") << std::endl;
						}
						else
						{
							if (receiver->NumSamplesStillNeeded() == 0)
							{
								if (WriteSamples (options.rx.filePath, receiver, output))
									return kCommandSuccess;
								else
									return kCommandFileError;
							}
							numTries = 0;
						}
						break;
					case Receiver::kReceiveDataMissing:
						output << boost::format ("Repeating due to receiver problems.") << std::endl;
						std::cerr << boost::format ("Repeating due to receiver problems.") << std::endl;
						break;
					case Receiver::kReceiveFailed:	
						return kCommandUsrpReceiveError;
				}
				++numTries;
				if (numTries > 15)
				{
					output << boost::format ("Giving up after %d attempts.") % numTries << std::endl;
					std::cerr << boost::format ("Giving up after %d attempts.") % numTries << std::endl;
					return kCommandRetryLimitExceededError;
				}
			}
			else
			{
				if (transmitter1)
					transmitter1->Start(t1Time, std::numeric_limits<double>::max(), options.delay);
				if (transmitter2)
					transmitter2->Start(t2Time, std::numeric_limits<double>::max(), options.delay);
				
				if (transmitter1)
					transmitter1->Wait(&output);
				if (transmitter2)
					transmitter2->Wait(&output);
				
				return kCommandSuccess;
			}
		}
	}

	const char*		serverRequestFifoName = "/tmp/uhd_streamer.request";
	const char*		serverResponseFifoName = "/tmp/uhd_streamer.response";
	const char*		serverStdoutFifoName = "/tmp/uhd_streamer.stdout";
	
	CommandResults BecomeServer (Options options)
	{
		gotSIGHUP = false;
		
		unlink(serverRequestFifoName);
		unlink(serverResponseFifoName);
		unlink(serverStdoutFifoName);
		
		if (mkfifo(serverRequestFifoName, 0600) != 0)
		{
			std::cerr << boost::format ("mkfifo(%s) failed: %s") % serverRequestFifoName % strerror(errno) << std::endl;
			return kCommandFifoError;
		}
		if (mkfifo(serverResponseFifoName, 0600) != 0)
		{
			std::cerr << boost::format ("mkfifo(%s) failed: %s") % serverResponseFifoName % strerror(errno) << std::endl;
			return kCommandFifoError;
		}
		if (mkfifo(serverStdoutFifoName, 0600) != 0)
		{
			std::cerr << boost::format ("mkfifo(%s) failed: %s") % serverStdoutFifoName % strerror(errno) << std::endl;
			return kCommandFifoError;
		}
		
		UsrpCache		usrpCache (options.verbose);
		
		while (!gotSIGINT && !gotSIGHUP)
		{
			try
			{
				if (options.verbose)
					std::cout << "Waiting ..." << std::endl;
					
				TimeOfDay 			startTime;
				
				std::ifstream	requestFifo (serverRequestFifoName, std::ifstream::in);
				if (!requestFifo.good())
					continue;
				
				TimeOfDay 			haveClientTime;

				std::vector<std::string>	args;

				const bool debugCommunication = options.verbose && false;
				
				if (debugCommunication)
					std::cout << "Client args:";
					
				while (requestFifo.good())
				{
					char			lineBuffer[1024];
					
					requestFifo.getline(lineBuffer, 1024, '\n');
					args.push_back (std::string(lineBuffer));
					if (debugCommunication)
						std::cout << " " << lineBuffer;
				}
				if (debugCommunication)
					std::cout << std::endl;
					
				std::ofstream	output (serverStdoutFifoName);
				std::ofstream	responseFifo (serverResponseFifoName);

				TimeOfDay 			haveArgsTime;

				char	*argv[args.size()];
				for (unsigned int a = 0; a < args.size(); ++a)
					argv[a] = const_cast<char*>(args[a].c_str());
				
				Options		clientOptions (args.size(), argv, output);
				
				CommandResults	result;
				if (clientOptions.Validate(output))
					result = TransmitTransmitReceive (usrpCache, clientOptions, output);
				else
					result = kCommandArgError;
				
				responseFifo << (int) result;
				if (debugCommunication)
					std::cout << "Result: " << (int) result << std::endl;
				
				if (options.verbose)
				{
					TimeOfDay 			clientFinishedTime;

					std::cout << boost::format("Server wait time %10.6f seconds.") % ElapsedTimeInSeconds (startTime, haveClientTime) << std::endl;
					std::cout << boost::format("Client wait time %10.6f seconds.") % clientOptions.clientWaitTime << std::endl;
					std::cout << boost::format("      Setup time %10.6f seconds.") % ElapsedTimeInSeconds (haveClientTime, haveArgsTime) << std::endl;
					std::cout << boost::format(" Processing time %10.6f seconds.") % ElapsedTimeInSeconds (haveArgsTime, clientFinishedTime) << std::endl;
					std::cout << boost::format("    Receive time %10.6f seconds.") % clientOptions.rxSeconds << std::endl;
				}
				
				if (result == kCommandRetryLimitExceededError)
					break;
			}
			catch (std::runtime_error e)
			{
				std::cerr << e.what() << std::endl;
			}
		}
		unlink(serverRequestFifoName);
		unlink(serverResponseFifoName);
		unlink(serverStdoutFifoName);

		if (gotSIGINT)
			return kCommandSuccess;
		else
			return kCommandRestartServer;
	}
	
	CommandResults AskServerTo (int argc, char *argv[])
	{
		TimeOfDay 			startTime;
		
		boost::interprocess::file_lock radioAccess ("/var/lock/uhd_streamer");	// Must already exist.
		radioAccess.lock();			// Do this BEFORE EnableRealtimeScheduling.
	
		TimeOfDay 			radioAccessTime;
		
		double waitTimeInSeconds = ElapsedTimeInSeconds (startTime, radioAccessTime);

		const bool debugCommunication = false;
		
		if (debugCommunication)
			std::cout << "Client args:";
			
		while (true)
		{
			std::ofstream	requestFifo (serverRequestFifoName);
	
			for (int a = 0; a < argc; ++a)
			{
				requestFifo << argv[a] << std::endl;
				if (debugCommunication)
					std::cout << argv[a] << std::endl;
			}
			requestFifo << "--waitTime" << std::endl << waitTimeInSeconds << std::endl;
			if (debugCommunication)
				std::cout << "--waitTime" << std::endl << waitTimeInSeconds << std::endl;
			
			if (requestFifo.good())
				break;
		}

		std::ifstream	stdoutFifo (serverStdoutFifoName);
		std::ifstream	responseFifo (serverResponseFifoName);
		while (true)
		{
			if (gotSIGINT || gotSIGHUP)
				return kCommandInterrupted;
			
			char	c = (char) stdoutFifo.get();
			if (!stdoutFifo.good())
				break;
			std::cout << c;
		}
		
		int				result;
 		responseFifo >> result;

		return (CommandResults) result;
	}
	
} // private namespace

int UHD_SAFE_MAIN(int argc, char *argv[])
{
	install_sig_handler(SIGINT, sig_handler);
	install_sig_handler(SIGHUP, sig_handler);
	install_sig_handler(SIGPIPE, sig_handler);
		
	Options		options (argc, argv, std::cout);

	if (!options.Validate(std::cout))
		return kCommandArgError;
	
	CommandResults	result;

	if (options.server)
	{
		EnableRealtimeScheduling ();

		while (true)
		{
			result = BecomeServer (options);
			if (result != kCommandRestartServer)
				break;
		}
	}
	else if (options.client)
	{
		result = AskServerTo (argc, argv);
	}
	else
	{
		boost::interprocess::file_lock radioAccess ("/var/lock/uhd_streamer");	// Must already exist.
		radioAccess.lock();			// Do this BEFORE EnableRealtimeScheduling.
	
		EnableRealtimeScheduling ();
		{
			UsrpCache		usrpCache (options.verbose);
				
			result = TransmitTransmitReceive (usrpCache, options, std::cout);
		}
	}
	if (options.verbose)
		std::cout << boost::format ("Finished with result %d") % result << std::endl;
	return result;
}
