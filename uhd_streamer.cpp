//
// Copyright 2011-2012 Horizon Analog, Inc.
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

#define _CRT_SECURE_NO_DEPRECATE		// Microsoft's xxx_s functions aren't portable. Shut off warnings.

#include <stdlib.h>

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/version.hpp>
#include <uhd/device.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/usrp/dboard_id.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>

#include <boost/algorithm/string.hpp> //for split
#include <boost/thread.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/asio.hpp>

#include <iostream>
#include <fstream>
#include <complex>
#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>
#include <cstdio>

#include <stdexcept>
#include <signal.h>
#include <float.h>
#include <errno.h>

#define	CLIENT_SERVER	1

namespace DeviceLister
{
	using namespace uhd;
	
	std::string prop_names_to_pp_string(const std::vector<std::string> &prop_names){
		std::stringstream ss; size_t count = 0;
		ss << "{";
		BOOST_FOREACH(const std::string &prop_name, prop_names){
			ss << ((count++)? ", " : "") << "'" << prop_name << "'";
		}
		ss << "}";
		return ss.str();
	}
	
	std::string path_name_and_value (property_tree::sptr tree, const fs_path &path)
	{
		std::stringstream ss;
		ss <<  boost::format("'%s', '%s'") % path % (tree->access<std::string>(path).get());
		return ss.str();
	}
	
	std::string path_name_and_vector (property_tree::sptr tree, const fs_path &path)
	{
		std::stringstream ss;
		ss << boost::format("'%s', %s") % path% prop_names_to_pp_string(tree->access<std::vector<std::string> >(path).get());
		return ss.str();
	}
	
	std::string path_name_and_list (property_tree::sptr tree, const fs_path &path)
	{
		std::stringstream ss;
		ss << boost::format("'%s', %s") % path % prop_names_to_pp_string(tree->list(path));
		return ss.str();
	}
	
	std::string path_name_and_gains (property_tree::sptr tree, const fs_path &path)
	{
		std::stringstream ss;
		std::vector<std::string> gain_names = tree->list(path / "gains");
		ss << boost::format("'%s', {") % (path / "gains");
		std::string		separator = "";
		BOOST_FOREACH(const std::string &name, gain_names){
			fs_path	gain_path = path / "gains" / name / "range";
			meta_range_t gain_range = tree->access<meta_range_t>(gain_path).get();
			ss << boost::format("%s{'%s', [%.1f %.1f %.1f]}") % separator % name % gain_range.start() % gain_range.step() % gain_range.stop();
			separator = ", ";
		}
		ss << "}" << std::endl;
		return ss.str();
	}
	
	std::string path_name_and_frequency_range (property_tree::sptr tree, const fs_path &path)
	{
		std::stringstream ss;
		fs_path	range_path = path / "range";
		meta_range_t freq_range = tree->access<meta_range_t>(range_path).get();
		ss << boost::format("'%s', [%g %g]") % range_path % freq_range.start() % freq_range.stop() << std::endl;
		return ss.str();
	}
	
	std::string get_dsp_pp_string(property_tree::sptr tree, const fs_path &path){
		std::stringstream ss;
		ss << path_name_and_frequency_range (tree, path / "freq");
		return ss.str();
	}
	
	std::string get_subdev_pp_string(const std::string &type, property_tree::sptr tree, const fs_path &path){
		std::stringstream ss;
	
		ss << path_name_and_value(tree, (path / "name")) << std::endl;
		ss << path_name_and_vector(tree, (path / "antenna/options")) << std::endl;
		ss << path_name_and_list(tree, (path / "sensors")) << std::endl;
		ss << path_name_and_frequency_range (tree, path / "freq");
		ss << path_name_and_frequency_range (tree, path / "bandwidth");
		ss << path_name_and_gains (tree, path);
	
		return ss.str();
	}
	
	std::string get_codec_pp_string(property_tree::sptr tree, const fs_path &path){
		std::stringstream ss;
	
		ss << path_name_and_value(tree, (path / "name")) << std::endl;
		ss << path_name_and_gains (tree, path);

		return ss.str();
	}
	
	std::string get_dboard_pp_string(const std::string &prefix, property_tree::sptr tree, const fs_path &path){
		std::stringstream ss;

		usrp::dboard_eeprom_t db_eeprom = tree->access<usrp::dboard_eeprom_t>(path / (prefix + "_eeprom")).get();
		if (db_eeprom.id != usrp::dboard_id_t::none()) {
			ss << boost::format("'%s/id', '%s'") % (path / (prefix + "_eeprom")) % db_eeprom.id.to_pp_string() << std::endl;
			
			if (not db_eeprom.serial.empty()) 
				ss << boost::format("'%s/serial', '%s'") % (path / (prefix + "_eeprom")) % db_eeprom.serial << std::endl;
			else
				ss << boost::format("'%s/serial', ''") % (path / (prefix + "_eeprom")) << std::endl;
			
			if (prefix == "tx"){
				usrp::dboard_eeprom_t gdb_eeprom = tree->access<usrp::dboard_eeprom_t>(path / "gdb_eeprom").get();
				if (gdb_eeprom.id != usrp::dboard_id_t::none()) ss << boost::format("'%s/id', '%s'") % (path / "gdb_eeprom") % gdb_eeprom.id.to_pp_string() << std::endl;
				if (not gdb_eeprom.serial.empty()) ss << boost::format("'%s/serial', '%s'") % (path / "gdb_eeprom") % gdb_eeprom.serial << std::endl;
			}
			
			BOOST_FOREACH(const std::string &name, tree->list(path / (prefix + "_frontends"))){
				ss << get_subdev_pp_string(prefix, tree, path / (prefix + "_frontends") / name);
			}
			ss << get_codec_pp_string(tree, path.branch_path().branch_path() / (prefix + "_codecs") / path.leaf());
		}
		return ss.str();
	}
	
	std::string get_mboard_pp_string(property_tree::sptr tree, const fs_path &path){
		std::stringstream ss;
		ss << path_name_and_value(tree, (path / "name")) << std::endl;

		usrp::mboard_eeprom_t mb_eeprom = tree->access<usrp::mboard_eeprom_t>(path / "eeprom").get();
		BOOST_FOREACH(const std::string &key, mb_eeprom.keys()){
			if (not mb_eeprom[key].empty()) ss << boost::format("'%s/%s', '%s'") % path % key % mb_eeprom[key] << std::endl;
		}
		ss << path_name_and_vector(tree, (path / "time_source/options")) << std::endl;
		ss << path_name_and_vector(tree, (path / "clock_source/options")) << std::endl;
		ss << path_name_and_list(tree, (path / "sensors")) << std::endl;

		BOOST_FOREACH(const std::string &name, tree->list(path / "rx_dsps")){
			ss << get_dsp_pp_string(tree, path / "rx_dsps" / name);
		}
		BOOST_FOREACH(const std::string &name, tree->list(path / "dboards")){
			ss << get_dboard_pp_string("rx", tree, path / "dboards" / name);
		}
		BOOST_FOREACH(const std::string &name, tree->list(path / "tx_dsps")){
			ss << get_dsp_pp_string(tree, path / "tx_dsps" / name);
		}
		BOOST_FOREACH(const std::string &name, tree->list(path / "dboards")){
			ss << get_dboard_pp_string("tx", tree, path / "dboards" / name);
		}
		return ss.str();
	}
	
	
	std::string get_device_pp_string(property_tree::sptr tree){
		std::stringstream ss;
		ss << "{    ..." << std::endl;
		BOOST_FOREACH(const std::string &name, tree->list("/mboards")){
			ss << get_mboard_pp_string(tree, "/mboards/" + name);
		}
		ss << "};    ..." << std::endl;
		return ss.str();
	}
	
	void print_tree(std::ostream &output, const uhd::fs_path &path, uhd::property_tree::sptr tree){
		output << path << std::endl;
		BOOST_FOREACH(const std::string &name, tree->list(path)){
			print_tree(output, path / name, tree);
		}
	}
	
	int ListDevices (std::ostream &output, boost::program_options::variables_map vm)
	{
		uhd::device_addrs_t device_addrs = uhd::device::find(std::string(""));
	
		if (device_addrs.size() == 0)
		{
			std::cerr << "No UHD Devices Found" << std::endl;
			return ~0;
		}
	
		output << "{" << std::endl;
		for (size_t i = 0; i < device_addrs.size(); i++)
		{
			output << "%{" << std::endl;	// In stand-alone mode device::make produces output. Make it a comment.
			
			device::sptr dev = device::make(device_addrs[i]);
			property_tree::sptr tree = dev->get_tree();

			output << "%}" << std::endl;

			if (vm.count("list"))
			{
				output << get_device_pp_string(tree);
			}
			
			if (vm.count("string"))
			{
				output << device_addrs[i].to_pp_string() << std::endl << std::endl;
				output << tree->access<std::string>(vm["string"].as<std::string>()).get() << std::endl;
			}
		
			if (vm.count("tree") != 0)
			{
				print_tree(output, "/", tree);
			}
		}
		output << "}" << std::endl;
		return 0;
	}
	
} // namespace DeviceLister

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
	
	void sig_handler(int sig)
	{
		if (sig == SIGINT)
			gotSIGINT = true;
	}
	
	void install_sig_handler(int signum, void (*new_handler)(int))
	{
#ifdef _WIN32
		signal(signum, new_handler);
#else
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
#endif
	}
	
	void EnableRealtimeScheduling ()
	{
		assert(uhd::set_thread_priority_safe ());
	}
	
	class TimeOfDay
	{
		public:
			boost::posix_time::ptime _ptime;
		
		public:
			TimeOfDay ()
			{
				_ptime = boost::posix_time::microsec_clock::universal_time();
			}
	};
	
	double ElapsedTimeInSeconds (TimeOfDay startingTime, TimeOfDay endingTime = TimeOfDay())
	{
		boost::posix_time::time_period		period (startingTime._ptime, endingTime._ptime);
		boost::posix_time::time_duration 	duration = period.length();
		
		return (double) duration.ticks() / (double) boost::posix_time::time_duration::ticks_per_second();
	}

	template <typename RangeType>
	double MiddleOfRange (RangeType range)
	{
		return (range.start() + range.stop())/2.0;		// @@@ mod range.step()
	}

	void printStat (std::ostream &output, const char *s, const char *name, uint64_t count)
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
	  	uhd::usrp::multi_usrp::sptr	 		_usrp;
	  	uhd::device::sptr					_device;
		boost::thread						_thread;
		volatile bool						_stop;
		uhd::time_spec_t 					_startingTime;
		EventCounts							_eventCounts;

	  public:
		typedef boost::shared_ptr<EventMonitor> sptr;

		EventCounts eventCounts () { return _eventCounts; }
		const EventCounts eventCounts () const { return _eventCounts; }
		
	  	EventMonitor (uhd::usrp::multi_usrp::sptr usrp)
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
			uhd::usrp::multi_usrp::sptr		usrp;
			bool							_closeWhenUnused;
			bool							inUseThisTime;
			
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
			
			bool							rxNeedToSetSubdevices;
			MonitoredValue<double>			rxFrequency;
			MonitoredValue<double>			rxOffset;
			MonitoredValue<double>			rxRefFrequency;
			MonitoredValue<double>			rxGain;
			MonitoredValue<double>			rxBandwidth;
			MonitoredValue<double>			rxSampleRate;
			MonitoredValue<std::string>		rxAntenna;
						
			ManagedUsrp (bool verbose, uhd::usrp::multi_usrp::sptr theUsrp, std::string device, bool closeWhenUnused)
			 :	_verbose(verbose)
			 ,	usrp (theUsrp)
			 ,	_deviceArgs (device)
			 ,	_closeWhenUnused(closeWhenUnused)
			 ,	inUseThisTime(true)
			 ,	usingExternalClock(false)
			 ,	rxNeedToSetSubdevices(true)
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
			
			bool ConfigureTx (bool requireLock)
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
					
					double actualFrequency = boost::math::round(usrp->get_tx_freq());
    				if (actualFrequency != boost::math::round(txFrequency.value()))
    				{
						std::cerr << boost::format ("\"%s\".txFrequency = %16.12e unsupported, replaced with %16.12e\n") % _deviceArgs % txFrequency.value() % actualFrequency;
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
														% _deviceArgs % txSampleRate.value() % actualSampleRate % (actualSampleRate - txSampleRate);
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
    						if (requireLock)
    							return false;
    						else
    							break;
						}
					}
				}
				return true;
			}
			
			bool ConfigureRx (bool requireLock)
			{
				if (rxNeedToSetSubdevices)
				{
					assert(usrp->get_rx_num_channels() == 2);
					usrp->set_rx_subdev_spec(uhd::usrp::subdev_spec_t("A:0 A:0"));		// Two channels so we can get at the 2nd one for 100 Msps.
					rxNeedToSetSubdevices = false;
				}

				bool	frequencyChanged = rxFrequency.changed() || rxOffset.changed();
				if (frequencyChanged)
				{
  					if (_verbose)
					{
						std::cout << boost::format ("\"%s\".rxFrequency = \"%16.12e\"\n") % _deviceArgs % rxFrequency.value();
						std::cout << boost::format ("\"%s\".rxOffset = \"%16.12e\"\n") % _deviceArgs % rxOffset.value();
					}
 					
 					uhd::tune_request_t		tune_request(rxFrequency.value(), rxOffset.value());
 					
 					for (size_t channel = 0; channel < 2; ++channel)
 					{
						uhd::tune_result_t tuneResult = usrp->set_rx_freq(tune_request, channel);
	
						if (_verbose)
							std::cerr << boost::format ("\"%s\".tuneResult(%d) = %s\n") % _deviceArgs % channel % tuneResult.to_pp_string();
	
						double actualFrequency = boost::math::round(usrp->get_rx_freq());
						if (fabs(actualFrequency - boost::math::round(rxFrequency.value())) > 15)
						{
							std::cerr << boost::format ("\"%s\".rxFrequency = %16.12e unsupported, replaced with %16.12e\n") % _deviceArgs % rxFrequency.value() % actualFrequency;
							rxFrequency = 0;
							return false;
						}
 					}
    			}
    			
				if (rxGain.changed())
				{
					double gain = rxGain.value();
					if (gain == unspecified)
						gain = MiddleOfRange(usrp->get_rx_gain_range(0));
					if (_verbose)
						std::cout << boost::format ("\"%s\".rxGain = %g\n") % _deviceArgs % gain;
   					
 					for (size_t channel = 0; channel < 2; ++channel)
 					{
						usrp->set_rx_gain(gain, channel);
						if (fabs(usrp->get_rx_gain(channel) - gain) > 0.05)
						{
							std::cerr << boost::format ("\"%s\".rxGain = %g unsupported, replaced with %g\n") % _deviceArgs % gain % usrp->get_rx_gain();
							rxGain = 0;
							return false;
						}
 					}
				}

				if (rxBandwidth.changed())
				{
					double bandwidth = rxBandwidth.value();
					if (bandwidth != unspecified)
					{
						if (_verbose)
							std::cout << boost::format ("\"%s\".rxBandwidth = %g\n") % _deviceArgs % bandwidth;
						
	 					for (size_t channel = 0; channel < 2; ++channel)
							usrp->set_rx_bandwidth(bandwidth, channel);
					}
				}

				if (rxSampleRate.changed())
				{
					if (_verbose)
						std::cout << boost::format ("\"%s\".rxSampleRate = %26.14g\n") % _deviceArgs % rxSampleRate.value();
   					
					if (rxSampleRate == 100e6)
					{
    					usrp->set_rx_rate(100e6/4, 1);	// Must be using custom N210 FPGA
    					usrp->set_rx_rate(100e6/4, 0);
    				}
					else
					{
    					usrp->set_rx_rate(rxSampleRate, 0);
    					usrp->set_rx_rate(rxSampleRate, 1);		// Not actually used.
    					
    					double		actualSampleRate = usrp->get_rx_rate(0);
						if (fabs(actualSampleRate - rxSampleRate) > 1e-6)
						{
							std::cerr << boost::format ("\"%s\".rxSampleRate = %26.14g unsupported, replaced with %26.14g, differs by %26.14g\n") 
														% _deviceArgs % rxSampleRate.value() % actualSampleRate % (actualSampleRate - rxSampleRate);
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
    						if (requireLock)
    							return false;
    						else
    							break;
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
			
			void clear ()
			{
				_devices.clear();
				_usePPS = false;
				_synchronized = false;
			}
			
			ManagedUsrp::sptr GetUsrp (std::string deviceName, bool closeWhenUnused, std::string clockSource, std::ostream &output)
			{
				DeviceMap::iterator			existingDevice = _devices.find(deviceName);
				ManagedUsrp::sptr			managedUsrp;
				
				if (existingDevice == _devices.end())
				{
					uhd::usrp::multi_usrp::sptr usrp;
					try
					{
						usrp = uhd::usrp::multi_usrp::make(deviceName);
						if (usrp)
						{
							if (_verbose)
								std::cout << boost::format ("Adding \"%s\"\n") % deviceName;
							
							managedUsrp = ManagedUsrp::sptr(new ManagedUsrp(_verbose, usrp, deviceName, closeWhenUnused));
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
						std::cout << boost::format ("Reusing \"%s\"\n") % deviceName;
					
					managedUsrp = existingDevice->second;
					managedUsrp->inUseThisTime = true;
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

			void StartNewRun ()
			{
				DeviceMap::iterator	i = _devices.end();
				for (i = _devices.begin(); i != _devices.end(); ++i)
					i->second->inUseThisTime = false;
			}
			
			void CloseUnusedDevicesIfRequired ()
			{
				while (true)
				{
					DeviceMap::iterator	usrpToDelete = _devices.end();
					for (usrpToDelete = _devices.begin(); usrpToDelete != _devices.end(); ++usrpToDelete)
					{
						if (usrpToDelete->second->_closeWhenUnused && !usrpToDelete->second->inUseThisTime)
							break;
					}
					if (usrpToDelete == _devices.end())
						break;
					
					_devices.erase(usrpToDelete);
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
	  	
		bool LoadFile (std::string filePath, double minimumSeconds, std::ostream &errorOutput)
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
			
			long		numBytesInFile = ftell (file);
			if (numBytesInFile < 0)
			{
				errorOutput << boost::format ("Error %s ftell %s") % strerror(errno) % filePath.data() << std::endl;
				fclose(file);
				return false;
			}
			size_t		numSamplesInFile = numBytesInFile/sizeof(uint32_t);
			
			size_t		minimumSamples;
			
			if (minimumSeconds == unspecified)
				minimumSamples = 1024*1024*1;
			else
				minimumSamples = (size_t) (minimumSeconds * _usrp->txSampleRate);
			
			size_t		numSamplesToBuffer;
			if (numSamplesInFile >= minimumSamples)
				numSamplesToBuffer = numSamplesInFile;
			else
				numSamplesToBuffer = ((minimumSamples/numSamplesInFile)+1) * numSamplesInFile;
				
			free(_buffer);
			_numSamplesInBuffer = numSamplesToBuffer;
			_buffer = (uint32_t*) calloc(numSamplesToBuffer, sizeof(uint32_t));
			
			size_t		numSamplesBuffered = 0;
			
			while (numSamplesBuffered < numSamplesToBuffer)
			{
				rewind (file);
				
				size_t		readSoFar = 0;
				while (readSoFar < numSamplesInFile)
				{
					size_t readThisTime = fread(&_buffer[numSamplesBuffered+readSoFar], sizeof(_buffer[0]), numSamplesInFile-readSoFar, file);
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
				numSamplesBuffered = numSamplesBuffered + numSamplesInFile;
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
			
		    uhd::stream_args_t stream_args("sc16");
		    uhd::tx_streamer::sptr tx_stream = _usrp->usrp->get_tx_stream(stream_args);

			uhd::tx_metadata_t	metadata;
			
			metadata.start_of_burst = true;
			metadata.end_of_burst = numSamples == 0;
			metadata.has_time_spec  = true;
			metadata.time_spec = startingTime;
			
			while (!metadata.end_of_burst)
			{
				assert(numSamples > 0);
				
				if (_stopTransmitting || gotSIGINT)
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

				size_t numSent = tx_stream->send(_buffer, numToSend, metadata, timeoutInSeconds + 0.5);
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
	  	bool								_continuous;
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
		, _continuous(sampleTime == unspecified)
		, _numSamplesInBuffer(_continuous ? 1024*1024*32 : static_cast<size_t>(sampleTime * managedUsrp->rxSampleRate))
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
				size_t	maxSamples = 1 << 18;	// The size of the N210's SRAM in samples.
			
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
 			size_t	numSamplesToReceive = NumSamplesToReceiveThisTime();
   		
			if (false)
				std::cerr << boost::format ("Rx %d at %d through %d at time %12.9g through %12.9g")
											% numSamplesToReceive 
											% _numSamplesReceived 
											% (_numSamplesReceived + numSamplesToReceive)
											% startingTime.get_real_secs() 
											% (startingTime + uhd::time_spec_t(0, numSamplesToReceive, _usrp->rxSampleRate)).get_real_secs() << std::endl;
											
	  		_receiveResult = kReceiveNotFinished;
			_thread = boost::thread(boost::bind(&Receiver::Receive, this, startingTime, numSamplesToReceive, errorOutput, timeoutInSeconds));
		}
		
		ReceiveResult Wait ()
		{
			_thread.join();
			return _receiveResult;
		}

		void Receive (uhd::time_spec_t startingTime, size_t numSamplesToReceive, std::ostream *errorOutput, double timeoutInSeconds)
		{
			EnableRealtimeScheduling ();
			
			uhd::rx_metadata_t metadata;
			
		    uhd::stream_args_t stream_args("sc16","sc16");
		    
		    size_t channel = _usrp->rxSampleRate == 100e6 ? 1 : 0;	// Tells custom N210 FPGA to use the undecimated path or not.
		    stream_args.channels.push_back(channel);
		    	
			uhd::rx_streamer::sptr rx_stream = _usrp->usrp->get_rx_stream(stream_args);

			uhd::stream_cmd_t streamCommand(_continuous ? uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS 
														: uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
			if (!_continuous)
				streamCommand.num_samps = numSamplesToReceive;
			streamCommand.stream_now = false;
			streamCommand.time_spec = startingTime;
			
			_usrp->usrp->issue_stream_cmd(streamCommand, channel);

			while (!gotSIGINT)
			{
				size_t numReceived = rx_stream->recv(_buffer + _numSamplesReceived, numSamplesToReceive, metadata,
																	 timeoutInSeconds + 0.5);
				if (!_continuous)
					_numSamplesReceived = _numSamplesReceived + numReceived;
	
				switch (metadata.error_code)
				{
				  case uhd::rx_metadata_t::ERROR_CODE_NONE:
					break;
				  case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
					_receiveResult = kReceiveDataMissing;
					std::cerr << boost::format("Rx ERROR_CODE_OVERFLOW") << std::endl;
					*errorOutput << boost::format("Rx ERROR_CODE_OVERFLOW") << std::endl;
					break;
				  case uhd::rx_metadata_t::ERROR_CODE_TIMEOUT:
					_receiveResult = kReceiveFailed;
					std::cerr << boost::format("Rx ERROR_CODE_TIMEOUT") << std::endl;
					*errorOutput << boost::format("Rx ERROR_CODE_TIMEOUT") << std::endl;
					break;
				  case uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND:
					_receiveResult = kReceiveFailed;
					std::cerr << boost::format("Rx ERROR_CODE_LATE_COMMAND") << std::endl;
					*errorOutput << boost::format("Rx ERROR_CODE_LATE_COMMAND") << std::endl;
					break;
				  case uhd::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN:
					_receiveResult = kReceiveFailed;
					std::cerr << boost::format("Rx ERROR_CODE_BROKEN_CHAIN") << std::endl;
					*errorOutput << boost::format("Rx ERROR_CODE_BROKEN_CHAIN") << std::endl;
					break;
				  case uhd::rx_metadata_t::ERROR_CODE_ALIGNMENT:
					_receiveResult = kReceiveFailed;
					std::cerr << boost::format("Rx ERROR_CODE_ALIGNMENT") << std::endl;
					*errorOutput << boost::format("Rx ERROR_CODE_ALIGNMENT") << std::endl;
					break;
				  case uhd::rx_metadata_t::ERROR_CODE_BAD_PACKET:
					_receiveResult = kReceiveFailed;
					std::cerr << boost::format("Rx ERROR_CODE_BAD_PACKET") << std::endl;
					*errorOutput << boost::format("Rx ERROR_CODE_BAD_PACKET") << std::endl;
					break;
				  default:
					_receiveResult = kReceiveFailed;
					std::cerr << boost::format("Rx got unknown error code 0x%x")  % metadata.error_code << std::endl;
					*errorOutput << boost::format("Rx got unknown error code 0x%x")  % metadata.error_code << std::endl;
					break;
				}
				assert(!metadata.more_fragments);
				
				if (!_continuous || _receiveResult != kReceiveNotFinished)
				{
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
					break;
				}
			}
			if (_continuous)
				_usrp->usrp->issue_stream_cmd(uhd::stream_cmd_t(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS));
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
		bool				closeWhenUnused;

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
		po::variables_map 			vm;

		bool						listDevices;
		
		DeviceOptions				t1;
		DeviceOptions				t2;
		DeviceOptions				rx;
		
		double						rxSeconds;
		double						rxDelay;
		double						delay;
		double						ppsOffset;
		long						delayMultiple;
		double						clientWaitTime;
		bool						synchronize;
		bool						requireLock;
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
				
				("version", "print the version string and exit")
				("list", "list devices")
				("tree", "print a complete property tree of each device")
       			("string", po::value<std::string>(), "query a string value from the properties tree of each device")

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
				("t1Close", po::value<bool>(&t1.closeWhenUnused)->default_value(false), "Close transmitter 1 device when not in use.")
	
				("t2Device", po::value<std::string>(&t2.device)->default_value(""), "Transmitter 2 uhd device address args")
				("t2Frequency", po::value<double>(&t2.centerFrequency)->default_value(unspecified), "Rf center frequency in Hz for transmitter 2")
				("t2Offset", po::value<double>(&t2.loOffset)->default_value(0.0), "Local oscillator offset for transmitter 2")
				("t2SampleRate", po::value<double>(&t2.sampleRate)->default_value(100e6/4), "Sample rate for transmitter 2")
				("t2Gain", po::value<double>(&t2.gain)->default_value(unspecified), "Transmitter 2 gain (defaults to middel of its range)")
				("t2Bandwidth", po::value<double>(&t2.bandwidth)->default_value(unspecified), "Transmitter 2 bandwidth")
				("t2Clock", po::value<std::string>(&t2.clockSource)->default_value("INT"), "Source of transmitter 2's reference clock and PPS (INT, EXT)")
				("t2Antenna", po::value<std::string>(&t2.antenna)->default_value(""), "Antenna to use for transmitter 2")
				("t2File", po::value<std::string>(&t2.filePath)->default_value(""), "File with data to transmit for transmitter 2")
				("t2Close", po::value<bool>(&t2.closeWhenUnused)->default_value(false), "Close transmitter 2 device when not in use.")
	
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
				("rxClose", po::value<bool>(&rx.closeWhenUnused)->default_value(false), "Close receiver device when not in use.")
	
				("rxSeconds", po::value<double>(&rxSeconds)->default_value(unspecified), "number of seconds to receive")
				("synchronize", po::value<bool>(&synchronize)->default_value(false), "Synchronize transmit and receive times")
				("verbose", po::value<bool>(&verbose)->default_value(false), "Verbose output")
				("delay", po::value<double>(&delay)->default_value(0.07), "Receive, transmit delay for synchronization in seconds.")
				("ppsOffset", po::value<double>(&ppsOffset)->default_value(unspecified), "Align transmission relative to next PPS.")
				("delayMultiple", po::value<long>(&delayMultiple)->default_value(0), "Required delay multiple in sample clock ticks to keep samples in phase across receives.")
				("rxDelay", po::value<double>(&rxDelay)->default_value(0.0), "Extra receive delay for synchronization in seconds.")
				("waitTime", po::value<double>(&clientWaitTime)->default_value(unspecified), "(Internal) Time client spent waiting for access in seconds.")

				("require-lock", po::value<bool>(&requireLock)->default_value(true), "Require local oscillator lock.")
				;
				
			po::store(po::parse_command_line(argc, argv, description), vm);
			po::notify(vm);
		
			listDevices = vm.count("list") || vm.count("string")  || vm.count("tree");
			
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
			
#if CLIENT_SERVER
			if (server && client)
			{
				output << "Choose only one of --server, --client." << std::endl;
				optionsAreValid = false;
			}
#else
			if (server || client)
			{
				output << "Neither --server or --client are support on this platform." << std::endl;
				optionsAreValid = false;
			}
#endif
			
			if (server + vm.count("list") + vm.count("string") + vm.count("tree") + vm.count("version") > 1)
			{
				output << "Choose only one of --server, --list, --tree, or --string." << std::endl;
				optionsAreValid = false;
			}
			
			if (!listDevices && !vm.count("version"))
			{
				optionsAreValid &= t1.Validate("t1", output);
				optionsAreValid &= t2.Validate("t2", output);
				optionsAreValid &= rx.Validate("rx", output);
				if (!server && t1.device.length() == 0 && t2.device.length() ==0  && rx.device.length() == 0)
				{
					output << "Need to specify at least one of --t1Device, --t2Device, --rxDevice." << std::endl;
					optionsAreValid = false;
				}
			}
	
			return optionsAreValid;
		}
	};	

	bool WriteSamples (std::string filePath, Receiver::sptr receiver, std::ostream &errorOutput)
	{
		if (filePath.empty())
			return true;
			
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
		CommandResults result = kCommandSuccess;
			
		ManagedUsrp::sptr	t1;
		ManagedUsrp::sptr	t2;
		ManagedUsrp::sptr	rx;

		Transmitter::sptr 	transmitter1;
		Transmitter::sptr 	transmitter2;
		Receiver::sptr 		receiver;
		
		double minimumTxSeconds = options.rxSeconds;
		if (minimumTxSeconds != unspecified && options.rxDelay != unspecified)
			minimumTxSeconds = minimumTxSeconds + options.rxDelay;
		
		usrpCache.StartNewRun ();
		if (options.t1.device.length() > 0)
		{
			t1 = usrpCache.GetUsrp (options.t1.device, options.t1.closeWhenUnused, options.t1.clockSource, output);
			if (!t1)
				return kCommandUsrpNotFoundError;
			
			t1->txFrequency = options.t1.centerFrequency;
			t1->txOffset = options.t1.loOffset;
			t1->txAntenna = options.t1.antenna;
			t1->txGain = options.t1.gain;
			t1->txBandwidth = options.t1.bandwidth;
			t1->txSampleRate = options.t1.sampleRate;

			if (!t1->ConfigureTx(options.requireLock))
				return kCommandUsrpConfigError;
			
			transmitter1 = Transmitter::sptr(new Transmitter(t1));
			if (!transmitter1->LoadFile (options.t1.filePath, minimumTxSeconds, output))
				return kCommandFileError;
			
			if (options.verbose)
			{
				output << boost::format ("Transmitter 1:  %s\n %s") % options.t1.device.data() % t1->usrp->get_pp_string().data() << std::endl;
				output << boost::format("Transmitting file %s at %g MHz") % options.t1.filePath % (options.t1.centerFrequency/1e6) << std::endl;
			}
		}
		
		if (options.t2.device.length() > 0)
		{
			t2 = usrpCache.GetUsrp (options.t2.device, options.t2.closeWhenUnused, options.t2.clockSource, output);
			if (!t2)
				return kCommandUsrpNotFoundError;
			
			t2->txFrequency = options.t2.centerFrequency;
			t2->txOffset = options.t2.loOffset;
			t2->txAntenna = options.t2.antenna;
			t2->txGain = options.t2.gain;
			t2->txBandwidth = options.t2.bandwidth;
			t2->txSampleRate = options.t2.sampleRate;

			if (!t2->ConfigureTx(options.requireLock))
				return kCommandUsrpConfigError;
			
			transmitter2 = Transmitter::sptr(new Transmitter(t2));
			if (!transmitter2->LoadFile (options.t2.filePath, minimumTxSeconds, output))
				return kCommandFileError;
			
			if (options.verbose)
			{
				output << boost::format ("Transmitter 2:  %s\n %s") % options.t2.device.data() % t2->usrp->get_pp_string().data() << std::endl;
				output << boost::format("Transmitting file %s at %g MHz") % options.t2.filePath % (options.t2.centerFrequency/1e6) << std::endl;
			}
		}
		
		if (options.rx.device.length() > 0)
		{
			rx = usrpCache.GetUsrp (options.rx.device, options.rx.closeWhenUnused, options.rx.clockSource, output);
			if (!rx)
				return kCommandUsrpNotFoundError;
			
			rx->rxFrequency = options.rx.centerFrequency;
			rx->rxOffset = options.rx.loOffset;
			rx->rxRefFrequency = options.rx.refFrequency;
			rx->rxAntenna = options.rx.antenna;
			rx->rxGain = options.rx.gain;
			rx->rxBandwidth = options.rx.bandwidth;
			rx->rxSampleRate = options.rx.sampleRate;

			if (!rx->ConfigureRx(options.requireLock))
				return kCommandUsrpConfigError;
			
			receiver = Receiver::sptr(new Receiver(rx, options.rxSeconds, options.delayMultiple));

			if (options.verbose)
			{
				output << boost::format ("Receiver:  %s\n %s") % options.rx.device.data() % rx->usrp->get_pp_string().data() << std::endl;
				output << boost::format("Receiving file %s at %g MHz") % options.rx.filePath % (options.rx.centerFrequency/1e6) << std::endl;
			}
		}
		assert (t1 || t2 || rx);
		
		usrpCache.CloseUnusedDevicesIfRequired ();

		int numSynchronizing = (t1 && t1->usingExternalClock)
							 + (t2 && t2->usingExternalClock)
							 + (rx && rx->usingExternalClock);
		
		bool synchronize = options.synchronize && (numSynchronizing > 0);
		
		if (options.verbose)
			output << boost::format ("Synchronize:  %d, numSynchronizing: %d, options.synchronize: %d\n") % synchronize % numSynchronizing % options.synchronize << std::endl;
			
		usrpCache.SynchronizeClocks (synchronize);
		
		int numTries = 0;
		while (true)
		{
			if (gotSIGINT)
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

				if (options.ppsOffset != unspecified)
				{
					uhd::time_spec_t	timeAdjust = -commonTime.get_frac_secs() + 1.0 + options.ppsOffset;
					commonTime = commonTime + timeAdjust;
					baseDelay = baseDelay + timeAdjust;		// Used for timeout calculation.
					
					if (options.verbose)
					{
						output << boost::format ("timeAdjust:  %g\n") % timeAdjust.get_real_secs() << std::endl;
						output << boost::format ("commonTime:  %g\n") % commonTime.get_real_secs() << std::endl;
					}
				}
				
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

			if (options.rxSeconds != unspecified)
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
								if (gotSIGINT)
									return kCommandInterrupted;
									
								if (!WriteSamples (options.rx.filePath, receiver, output))
									return kCommandFileError;

								return kCommandSuccess;
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
				if (receiver)
					receiver->Start(rxTime, &output, (uhd::time_spec_t(options.delay) + uhd::time_spec_t(options.rxDelay)).get_real_secs());
				if (transmitter1)
					transmitter1->Start(t1Time, std::numeric_limits<double>::max(), options.delay);
				if (transmitter2)
					transmitter2->Start(t2Time, std::numeric_limits<double>::max(), options.delay);
				
				if (receiver)
					receiver->Wait();
				if (transmitter1)
					transmitter1->Wait(&output);
				if (transmitter2)
					transmitter2->Wait(&output);
				
				return kCommandSuccess;
			}
		}
	}

#if CLIENT_SERVER
	const unsigned short	kServerTcpPort		= 	4242;
	const std::string		kEndString("<end>");
	
	CommandResults BecomeServer (Options options)
	{
	    boost::asio::io_service 		io_service;
		
    	boost::asio::ip::tcp::endpoint 		endpoint(boost::asio::ip::tcp::v4(), kServerTcpPort);
	    boost::asio::ip::tcp::acceptor 		acceptor(io_service, endpoint);

		UsrpCache		usrpCache (options.verbose);
		
		while (!gotSIGINT)
		{
			try
			{
				if (options.verbose)
					std::cout << "Waiting ..." << std::endl;
					
				TimeOfDay 			startTime;
				
			  	boost::asio::ip::tcp::iostream 	clientStream;
			 	boost::system::error_code 	error;
			 	
			 	// Accept one client at a time to serialize access to the radios.
			 	//
			  	acceptor.accept(*clientStream.rdbuf(), error);
			  	if (error)
			  		continue;
				
				TimeOfDay 			haveClientTime;

				std::vector<std::string>	args;

				const bool debugCommunication = options.verbose && false;
				
				if (debugCommunication)
					std::cout << "Client args:" << std::endl;
					
				while (clientStream.good())
				{
					std::string		argString;
					
					std::getline(clientStream, argString);
					if (debugCommunication)
						std::cout << " " << argString << std::endl;

					if (argString == std::string(kEndString))
						break;
						
					args.push_back (argString);
				}
				
				TimeOfDay 			haveArgsTime;
				
				std::vector<char*>	argv(args.size());
				for (unsigned int a = 0; a < args.size(); ++a)
					argv[a] = const_cast<char*>(args[a].c_str());
				
				Options		clientOptions (argv.size(), &argv[0], clientStream);
				
				CommandResults	result;
				if (clientOptions.Validate(clientStream))
					if (clientOptions.vm.count("version"))
					{
						clientStream << uhd::get_version_string() << std::endl;
						result = kCommandSuccess;
					}
					else if (clientOptions.listDevices)
					{
						usrpCache.clear();
						result = (CommandResults) DeviceLister::ListDevices (clientStream, clientOptions.vm);
					}
					else
					{
						result = TransmitTransmitReceive (usrpCache, clientOptions, clientStream);
					}
				else
					result = kCommandArgError;
				
				clientStream << boost::format("\n%s\n%d\n") % kEndString % (int) result;
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
				
				if (result == kCommandRetryLimitExceededError || result == kCommandUsrpReceiveError)
					break; // Restart the server
			}
			catch (std::runtime_error e)
			{
				std::cerr << e.what() << std::endl;
			}
		}

		if (gotSIGINT)
			return kCommandSuccess;
		else
			return kCommandRestartServer;
	}
	
	CommandResults AskServerTo (int argc, char *argv[])
	{
		TimeOfDay 						startTime;
		
		boost::asio::ip::tcp::endpoint	endpoint(boost::asio::ip::address_v4::loopback(), kServerTcpPort);
	    boost::asio::ip::tcp::iostream	serverStream(endpoint);
		
		double waitTimeInSeconds = ElapsedTimeInSeconds (startTime);

		const bool debugCommunication = false;
		
		if (debugCommunication)
			std::cout << "Client args:";
			
		for (int a = 0; a < argc; ++a)
		{
			serverStream << argv[a] << std::endl;
			if (debugCommunication)
				std::cout << argv[a] << std::endl;
		}
		serverStream << "--waitTime" << std::endl << waitTimeInSeconds << std::endl;
		serverStream << kEndString << std::endl;

		if (debugCommunication)
			std::cout << "--waitTime" << std::endl << waitTimeInSeconds << std::endl;
		
		std::string		outputString;
		
		while (serverStream.good())
		{
			if (gotSIGINT)
				return kCommandInterrupted;
			
			std::getline(serverStream, outputString);
			
			if (outputString == std::string(kEndString))
				break;
		
			std::cout << outputString << std::endl;
		}
		
		std::getline(serverStream, outputString);
		
		int				result;
		std::istringstream (outputString) >> result;
		
		return (CommandResults) result;
	}
#endif

} // private namespace

int UHD_SAFE_MAIN(int argc, char *argv[])
{
#ifndef _WIN32
	install_sig_handler(SIGPIPE, sig_handler);
#endif

	Options		options (argc, argv, std::cout);

	if (!options.Validate(std::cout))
		return kCommandArgError;
	
	CommandResults	result;

#if CLIENT_SERVER
	if (options.server)
	{
		install_sig_handler(SIGINT, sig_handler);
			
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
#endif

	{
		if (options.vm.count("version"))
		{
			std::cout << uhd::get_version_string() << std::endl;
			result = kCommandSuccess;
		}
		else
		{
			if (options.listDevices)
				result = (CommandResults) DeviceLister::ListDevices (std::cout, options.vm);
			else
			{
				EnableRealtimeScheduling ();
				{
					UsrpCache		usrpCache (options.verbose);
						
					result = TransmitTransmitReceive (usrpCache, options, std::cout);
				}
			}
		}
	}
	if (options.verbose)
		std::cout << boost::format ("Finished with result %d") % (int) result << std::endl;
	return result;
}
