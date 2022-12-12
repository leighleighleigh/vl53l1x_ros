/*
 * STM VL53L1X ToF rangefinder driver for ROS
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under BSD 3-Clause License (available at https://opensource.org/licenses/BSD-3-Clause).
 *
 * Documentation used:
 * VL53L1X datasheet - https://www.st.com/resource/en/datasheet/vl53l1x.pdf
 * VL53L1X API user manual - https://www.st.com/content/ccc/resource/technical/document/user_manual/group0/98/0d/38/38/5d/84/49/1f/DM00474730/files/DM00474730.pdf/jcr:content/translations/en.DM00474730.pdf
 *
 */

#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <vl53l1x/MeasurementData.h>

#include "vl53l1_api.h"
#include "i2c.h"
#include <wiringPi.h>

#define xSTR(x) #x
#define STR(x) xSTR(x)

#define CHECK_STATUS(func) { \
	VL53L1_Error status = func; \
	if (status != VL53L1_ERROR_NONE) { \
		ROS_WARN("VL53L1X: Error %d on %s", status, STR(func)); \
	} \
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vl53l1x");
	ros::NodeHandle nh, nh_priv("~");

	sensor_msgs::Range range;
	vl53l1x::MeasurementData data;
	
	range.radiation_type = sensor_msgs::Range::INFRARED;
	ros::Publisher setup_done_pub = nh_priv.advertise<std_msgs::Bool>("setup_done", 50, true);
	ros::Publisher range_pub = nh_priv.advertise<sensor_msgs::Range>("range", 50);
	ros::Publisher data_pub = nh_priv.advertise<vl53l1x::MeasurementData>("data", 50);
	ros::Rate xshut_toggle_rate(2.0);

	// Read parameters
	int mode, i2c_bus, i2c_address, i2c_address_default, xshut_gpio;
	double poll_rate, timing_budget, offset;
	bool ignore_range_status, wait_for_setup_signal, found_device_on_bus;
	std::vector<int> pass_statuses { VL53L1_RANGESTATUS_RANGE_VALID,
	                                 VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL,
	                                 VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE };

	nh_priv.param("mode", mode, 3);
	nh_priv.param("i2c_bus", i2c_bus, 1);
	// The address we try to connect to first - if not found, we will use XSHUT to change it.
	nh_priv.param("i2c_address", i2c_address, 0x40);
	// i2c_address_default is the address we will look for when toggling the XSHUT pin. 
	// The device address will then be changed to i2c_address, if it differs.
	nh_priv.param("i2c_address_default", i2c_address_default, 0x29);
	// wiringPi uses it's own GPIO numbering system
	// you may find a useful reference table here
	// https://pinout.xyz/pinout/wiringpi
	nh_priv.param("xshut_gpio", xshut_gpio, 27);
	// To handle multiple devices, all competing for the I2C bus during setup,
	// we can 'chain' them via the ~setup topic boolean. Set this to 'True' for every
	// sensor after the first one, and remap the topics appropriately.
	nh_priv.param("wait_for_setup", wait_for_setup_signal, true);
	nh_priv.param("poll_rate", poll_rate, 100.0);
	nh_priv.param("ignore_range_status", ignore_range_status, false);
	nh_priv.param("timing_budget", timing_budget, 0.1);
	nh_priv.param("offset", offset, 0.0);
	nh_priv.param<std::string>("frame_id", range.header.frame_id, "");
	nh_priv.param("field_of_view", range.field_of_view, 0.471239f); // 27 deg, source: datasheet
	nh_priv.param("min_range", range.min_range, 0.0f);
	nh_priv.param("max_range", range.max_range, 4.0f);

	nh_priv.getParam("pass_statuses", pass_statuses);

	if (timing_budget < 0.02 || timing_budget > 1) {
		ROS_FATAL("Error: timing_budget should be within 0.02 and 1 s (%g is set)", timing_budget);
		ros::shutdown();
	}

	// The minimum inter-measurement period must be longer than the timing budget + 4 ms (*)
	double inter_measurement_period = timing_budget + 0.004;

	
	// Our device variables	
	VL53L1_Dev_t dev;
	VL53L1_Error dev_error;

	// If xshut_gpio is not -1, we will setup a GPIO pin to control it
	if (xshut_gpio != -1)
	{
		// Initialise GPIO
		wiringPiSetup();
		pinMode(xshut_gpio, OUTPUT);

		digitalWrite(xshut_gpio, LOW); // Put device to sleep, XSHUT low
		xshut_toggle_rate.sleep(); // Wait for sleep

		// Wait for a message on the ~setup topic, before continuing.
		// A message will be published on the setup_done topic, when we are done
		// This allows sensors to be chained together
		if (wait_for_setup_signal)
		{
			ROS_INFO("Waiting for message on ~setup");
			ros::topic::waitForMessage<std_msgs::Bool>("setup", nh_priv);
		}

		digitalWrite(xshut_gpio, HIGH); // Wake device up
		xshut_toggle_rate.sleep(); // Wait for wakeup

		// Try to find device at the specified default address
		if (i2c_address_default != i2c_address)
		{
			found_device_on_bus = i2c_setup(i2c_bus, i2c_address_default);

			// If unsuccessful, abort
			if (!found_device_on_bus)
			{
				ROS_FATAL("Failed to find I2C device at default address 0x%02x", i2c_address_default);
				ros::shutdown();
			}else{
				ROS_INFO("Found I2C device at default address 0x%02x, remapping to 0x%02x\n", i2c_address_default, i2c_address);
				VL53L1_software_reset(&dev);
				VL53L1_WaitDeviceBooted(&dev);
				VL53L1_SetDeviceAddress(&dev,i2c_address<<1);
				xshut_toggle_rate.sleep(); // Wait for wakeup
			}
		}
	}
	
	// Setup I2C bus, first trying the target address in case the sensor has been pre-configured
	found_device_on_bus = i2c_setup(i2c_bus, i2c_address);

	if (!found_device_on_bus)
	{
		ROS_WARN("Failed to find I2C device at target address 0x%02x", i2c_address);
		ros::shutdown();
	}

	// Setup is complete - in the sense that we have finished competing for the default I2C address
	std_msgs::Bool donemsg;
	donemsg.data = true;
	setup_done_pub.publish(donemsg);

	// Init sensor
	VL53L1_DataInit(&dev);
	VL53L1_StaticInit(&dev);
	VL53L1_SetPresetMode(&dev, VL53L1_PRESETMODE_AUTONOMOUS);

	// Print device info
	VL53L1_DeviceInfo_t device_info;
	CHECK_STATUS(VL53L1_GetDeviceInfo(&dev, &device_info));
	ROS_INFO("VL53L1X: Device name: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.Name);
	ROS_INFO("VL53L1X: Device type: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.Type);
	ROS_INFO("VL53L1X: Product ID: %." STR(VL53L1_DEVINFO_STRLEN) "s", device_info.ProductId);
	ROS_INFO("VL53L1X: Type: %u Version: %u.%u", device_info.ProductType,
	          device_info.ProductRevisionMajor, device_info.ProductRevisionMinor);

	// Setup sensor
	CHECK_STATUS(VL53L1_SetDistanceMode(&dev, mode));
	CHECK_STATUS(VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, round(timing_budget * 1e6)));

	double min_signal;
	if (nh_priv.getParam("min_signal", min_signal)) {
		CHECK_STATUS(VL53L1_SetLimitCheckValue(&dev, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, min_signal * 65536));
	}

	double max_sigma;
	if (nh_priv.getParam("max_sigma", max_sigma)) {
		CHECK_STATUS(VL53L1_SetLimitCheckValue(&dev, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, max_sigma * 1000 * 65536));
	}

	// Start sensor
	for (int i = 0; i < 100; i++) {
		CHECK_STATUS(VL53L1_SetInterMeasurementPeriodMilliSeconds(&dev, round(inter_measurement_period * 1e3)));
		dev_error = VL53L1_StartMeasurement(&dev);
		if (dev_error == VL53L1_ERROR_INVALID_PARAMS) {
			inter_measurement_period += 0.001; // Increase inter_measurement_period to satisfy condition (*)
		} else break;
	}

	// Check for errors after start
	if (dev_error != VL53L1_ERROR_NONE) {
		ROS_FATAL("VL53L1X: Can't start measurement: error %d", dev_error);
		ros::shutdown();
	}

	ROS_INFO("VL53L1X: ranging");

	VL53L1_RangingMeasurementData_t measurement_data;

	// Main loop
	ros::Rate r(poll_rate);
	while (ros::ok()) {
		r.sleep();
		range.header.stamp = ros::Time::now();

		// Check the data is ready
		uint8_t data_ready = 0;
		VL53L1_GetMeasurementDataReady(&dev, &data_ready);
		if (!data_ready) {
			continue;
		}

		// Read measurement
		VL53L1_GetRangingMeasurementData(&dev, &measurement_data);
		VL53L1_ClearInterruptAndStartMeasurement(&dev);

		// Publish measurement data
		data.header.stamp = range.header.stamp;
		data.signal = measurement_data.SignalRateRtnMegaCps / 65536.0;
		data.ambient = measurement_data.AmbientRateRtnMegaCps / 65536.0;
		data.effective_spad = measurement_data.EffectiveSpadRtnCount / 256;
		data.sigma = measurement_data.SigmaMilliMeter / 65536.0 / 1000.0;
		data.status = measurement_data.RangeStatus;
		data_pub.publish(data);

		// Check measurement for validness
		if (!ignore_range_status &&
		    std::find(pass_statuses.begin(), pass_statuses.end(), measurement_data.RangeStatus) == pass_statuses.end()) {
			char range_status[VL53L1_MAX_STRING_LENGTH];
			VL53L1_get_range_status_string(measurement_data.RangeStatus, range_status);
			ROS_DEBUG("Range measurement status is not valid: %s", range_status);
			ros::spinOnce();
			continue;
		}

		// Publish measurement
		range.range = measurement_data.RangeMilliMeter / 1000.0 + offset;
		range_pub.publish(range);

		ros::spinOnce();
	}

	// Release
	ROS_INFO("VL53L1X: stop ranging");
	VL53L1_StopMeasurement(&dev);
	i2c_release();
}
