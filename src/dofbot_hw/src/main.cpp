#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

class ArmDriver : public rclcpp::Node
{
public:
	ArmDriver()
		: Node("arm_driver")
	{
		rcl_interfaces::msg::ParameterDescriptor startupDescriptor;
		startupDescriptor.read_only = true;
		const auto serialPort = declare_parameter<std::string>("port", "/dev/ttyUSB0", startupDescriptor);
		timeoutMs = declare_parameter<int>("timeout_ms", 200, startupDescriptor);
		const auto startupDelayMs = declare_parameter<int>("startup_delay_ms", 2000, startupDescriptor);
		const auto continuous = declare_parameter<bool>("continuous", true, startupDescriptor);
		const auto pollIntervalMs = declare_parameter<int>("poll_interval_ms", 500, startupDescriptor);
		jointZeros = declare_parameter<std::vector<double>>(
			"joint_zero_degrees", {90.0, 90.0, 90.0, 90.0, 90.0}, startupDescriptor);
		jointDirections = declare_parameter<std::vector<std::int64_t>>(
			"joint_directions", {1, 1, 1, 1, 1}, startupDescriptor);
		gripperOpenDegrees = declare_parameter<double>("gripper_open_degrees", 0.82, startupDescriptor);
		gripperClosedDegrees = declare_parameter<double>("gripper_closed_degrees", 172.12, startupDescriptor);

		if (serialPort.empty() || timeoutMs < 1 || timeoutMs > 5000 ||
			startupDelayMs < 0 || startupDelayMs > 5000)
		{
			throw std::invalid_argument(
				"port must not be empty; timeout_ms must be 1..5000; "
				"startup_delay_ms must be 0..5000");
		}
		if (pollIntervalMs < 20 || pollIntervalMs > 10000 ||
			jointZeros.size() != 5 || jointDirections.size() != 5 ||
			!std::all_of(jointZeros.begin(), jointZeros.end(), [](double value) { return std::isfinite(value); }) ||
			!std::all_of(jointDirections.begin(), jointDirections.end(),
				[](std::int64_t value) { return value == 1 || value == -1; }))
		{
			throw std::invalid_argument(
				"poll_interval_ms must be 20..10000; joint_zero_degrees needs five finite values; "
				"joint_directions needs five values of 1 or -1");
		}
		if (!std::isfinite(gripperOpenDegrees) || !std::isfinite(gripperClosedDegrees) ||
			gripperOpenDegrees < 0.0 || gripperClosedDegrees > 180.0 ||
			gripperOpenDegrees >= gripperClosedDegrees)
		{
			throw std::invalid_argument(
				"gripper endpoints must be finite and satisfy 0 <= open < closed <= 180 degrees");
		}

		serialFd = open(serialPort.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
		if (serialFd < 0)
		{
			throw_system_error("Failed to open " + serialPort);
		}

		try
		{
			configure_serial();
			RCLCPP_INFO(get_logger(), "Opened %s at 115200 baud", serialPort.c_str());
			rclcpp::sleep_for(std::chrono::milliseconds(startupDelayMs));
			if (!rclcpp::ok())
			{
				throw std::runtime_error("Startup interrupted");
			}
			if (continuous)
			{
				feedbackPublisher = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
				feedbackTimer = create_wall_timer(
					std::chrono::milliseconds(pollIntervalMs), [this]() { publish_positions(); });
				RCLCPP_INFO(get_logger(), "Publishing measured joint positions every %lld ms with candidate joint mapping",
					static_cast<long long>(pollIntervalMs));
			}
		}
		catch (...)
		{
			close_serial();
			throw;
		}
	}

	~ArmDriver() override
	{
		close_serial();
	}

	bool read_positions()
	{
		bool successful = true;
		for (int servoId = 1; servoId <= 6 && rclcpp::ok(); ++servoId)
		{
			try
			{
				const auto rawPosition = request_servo_position(servoId);
				const auto angleDegrees = servo_angle_degrees(servoId, rawPosition);

				RCLCPP_INFO(
					get_logger(), "Servo %d: raw position %u, angle %.2f degrees",
					servoId, static_cast<unsigned>(rawPosition), angleDegrees);
			}
			catch (const std::exception &error)
			{
				RCLCPP_ERROR(get_logger(), "Servo %d: %s", servoId, error.what());
				successful = false;
			}
		}
		return successful && rclcpp::ok();
	}

private:
	using serial_deadline = std::chrono::steady_clock::time_point;

	static double servo_angle_degrees(int servoId, std::uint16_t rawPosition)
	{
		const double minimum = servoId == 5 ? 380.0 : 900.0;
		const double maximum = servoId == 5 ? 3700.0 : 3100.0;
		const double angleDegrees = (rawPosition - minimum) *
			(servoId == 5 ? 270.0 : 180.0) / (maximum - minimum);
		return servoId >= 2 && servoId <= 4 ? 180.0 - angleDegrees : angleDegrees;
	}

	void publish_positions()
	{
		for (int servoId = 1; servoId <= 6 && rclcpp::ok(); ++servoId)
		{
			try
			{
				const auto rawPosition = request_servo_position(servoId);
				const auto angleDegrees = servo_angle_degrees(servoId, rawPosition);
				sensor_msgs::msg::JointState measurement;
				measurement.header.stamp = now();
				if (servoId == 6)
				{
					const auto openFraction = std::clamp(
						(gripperClosedDegrees - angleDegrees) / (gripperClosedDegrees - gripperOpenDegrees),
						0.0, 1.0);
					measurement.name = {"grip_joint"};
					measurement.position = {-1.54 * openFraction};
				}
				else
				{
					measurement.name = {"arm_joint" + std::to_string(servoId)};
					measurement.position = {
						jointDirections[servoId - 1] * (angleDegrees - jointZeros[servoId - 1]) *
						std::numbers::pi / 180.0};
				}
				feedbackPublisher->publish(measurement);
			}
			catch (const std::exception &error)
			{
				RCLCPP_ERROR(get_logger(), "Servo %d feedback skipped: %s", servoId, error.what());
			}
		}
	}

	[[noreturn]] void throw_system_error(const std::string &operation)
	{
		throw std::runtime_error(operation + ": " + std::strerror(errno));
	}

	void configure_serial()
	{
		if (tcgetattr(serialFd, &originalSettings) != 0)
		{
			throw_system_error("Failed to get serial settings");
		}
		settingsSaved = true;

		termios settings = originalSettings;
		cfmakeraw(&settings);
		settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
		settings.c_cflag |= CS8 | CREAD | CLOCAL;
		settings.c_cc[VMIN] = 0;
		settings.c_cc[VTIME] = 0;

		if (cfsetispeed(&settings, B115200) != 0 ||
			cfsetospeed(&settings, B115200) != 0)
		{
			throw_system_error("Failed to set baud rate");
		}
		if (tcsetattr(serialFd, TCSANOW, &settings) != 0)
		{
			throw_system_error("Failed to configure serial port");
		}
	}

	void close_serial() noexcept
	{
		if (serialFd >= 0)
		{
			if (settingsSaved)
			{
				tcsetattr(serialFd, TCSANOW, &originalSettings);
			}
			close(serialFd);
			serialFd = -1;
		}
	}

	void wait_for_serial(short events, serial_deadline deadline)
	{
		while (rclcpp::ok())
		{
			const auto remainingMs = std::chrono::duration_cast<std::chrono::milliseconds>(
										 deadline - std::chrono::steady_clock::now())
										 .count();
			if (remainingMs <= 0)
			{
				throw std::runtime_error("Serial operation timed out without valid feedback");
			}

			pollfd descriptor{serialFd, events, 0};
			const int ready = poll(&descriptor, 1, static_cast<int>(std::min(remainingMs, 50L)));
			if (ready < 0)
			{
				if (errno == EINTR)
				{
					continue;
				}
				throw_system_error("Serial polling failed");
			}
			if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
			{
				throw std::runtime_error("Serial connection failed or disconnected");
			}
			if (descriptor.revents & events)
			{
				return;
			}
		}
		throw std::runtime_error("Serial operation interrupted");
	}

	std::uint16_t request_servo_position(int servoId)
	{
		if (tcflush(serialFd, TCIFLUSH) != 0)
		{
			throw_system_error("Failed to clear serial input");
		}
		std::array<std::uint8_t, 8> response{};
		std::size_t receivedBytes = 0;
		try
		{
			send_position_request(servoId);
			read_position_reply(servoId, response, receivedBytes);
		}
		catch (const std::exception &error)
		{
			if (!rclcpp::ok())
			{
				throw;
			}
			RCLCPP_WARN(get_logger(), "Servo %d refresh reply unavailable: %s", servoId, error.what());
		}
		rclcpp::sleep_for(std::chrono::milliseconds(1));
		for (int attempt = 1; attempt <= 2; ++attempt)
		{
			try
			{
				send_position_request(servoId);
				const auto rawPosition = read_position_reply(servoId, response, receivedBytes);
				const unsigned minimum = servoId == 5 ? 380 : 900;
				const unsigned maximum = servoId == 5 ? 3700 : 3100;
				if (rawPosition < minimum || rawPosition > maximum)
				{
					throw std::runtime_error("Position reply is outside the expected servo range");
				}
				return rawPosition;
			}
			catch (const std::exception &error)
			{
				if (attempt == 2 || !rclcpp::ok())
				{
					throw;
				}
				RCLCPP_WARN(get_logger(), "Servo %d: retrying position request after %s", servoId, error.what());
			}
		}
		throw std::runtime_error("Position request failed");
	}

	void send_position_request(int servoId)
	{
		const auto command = static_cast<std::uint8_t>(0x30 + servoId);
		const std::array<std::uint8_t, 5> packet{
			0xFF, 0xFC, 0x03, command, static_cast<std::uint8_t>(0x03 + command)};
		write_packet(packet);
	}

	void write_packet(std::span<const std::uint8_t> packet)
	{
		const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
		std::size_t sentBytes = 0;
		while (sentBytes < packet.size())
		{
			wait_for_serial(POLLOUT, deadline);
			const auto count = write(serialFd, packet.data() + sentBytes, packet.size() - sentBytes);
			if (count < 0)
			{
				if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
				{
					continue;
				}
				throw_system_error("Failed to send serial packet");
			}
			if (count == 0)
			{
				throw std::runtime_error("Serial write made no progress");
			}
			sentBytes += static_cast<std::size_t>(count);
		}
	}

	std::uint16_t read_position_reply(
		int servoId, std::array<std::uint8_t, 8> &response, std::size_t &receivedBytes)
	{
		const auto command = static_cast<std::uint8_t>(0x30 + servoId);
		const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
		while (true)
		{
			wait_for_serial(POLLIN, deadline);
			const auto count = read(serialFd, response.data() + receivedBytes, 1);
			if (count < 0)
			{
				if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
				{
					continue;
				}
				throw_system_error("Failed to read position reply");
			}
			if (count == 0)
			{
				throw std::runtime_error("Serial connection returned no data");
			}
			++receivedBytes;
			if (receivedBytes < response.size())
			{
				continue;
			}

			unsigned checksum = 0;
			for (std::size_t byteIndex = 2; byteIndex < 7; ++byteIndex)
			{
				checksum += response[byteIndex];
			}
			const auto rawPosition = static_cast<std::uint16_t>(
				(static_cast<unsigned>(response[4]) << 8) | response[5]);
			if (response[0] == 0xFF && response[1] == 0xFB &&
				response[2] == 0x06 && response[3] == 0x0A &&
				response[6] == command && response[7] == static_cast<std::uint8_t>(checksum))
			{
				receivedBytes = 0;
				return rawPosition;
			}

			for (std::size_t byteIndex = 0; byteIndex < response.size() - 1; ++byteIndex)
			{
				response[byteIndex] = response[byteIndex + 1];
			}
			receivedBytes = response.size() - 1;
		}
	}

	int serialFd = -1;
	int timeoutMs = 200;
	termios originalSettings{};
	bool settingsSaved = false;
	std::vector<double> jointZeros;
	std::vector<std::int64_t> jointDirections;
	double gripperOpenDegrees;
	double gripperClosedDegrees;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedbackPublisher;
	rclcpp::TimerBase::SharedPtr feedbackTimer;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	int exitCode = 1;
	try
	{
		ArmDriver armDriver;
		if (armDriver.get_parameter("continuous").as_bool())
		{
			rclcpp::spin(armDriver.get_node_base_interface());
			exitCode = 0;
		}
		else
		{
			exitCode = armDriver.read_positions() ? 0 : 1;
		}
	}
	catch (const std::exception &error)
	{
		RCLCPP_ERROR(rclcpp::get_logger("arm_driver"), "%s", error.what());
	}
	rclcpp::shutdown();
	return exitCode;
}
