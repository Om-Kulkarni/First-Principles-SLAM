#include "euroc_driver/data_publisher.hpp"
#include <filesystem>
#include <algorithm>

namespace euroc_driver
{

	DataPublisher::DataPublisher()
			: Node("euroc_data_publisher"), is_playing_(false), start_timestamp_(0)
	{

		try
		{
			RCLCPP_INFO(this->get_logger(), "Initializing parameters...");
			initializeParameters();

			RCLCPP_INFO(this->get_logger(), "Initializing publishers...");
			initializePublishers();

			RCLCPP_INFO(this->get_logger(), "Initializing readers...");
			if (initializeReaders())
			{
				RCLCPP_INFO(this->get_logger(), "EuRoC data publisher initialized successfully");
				
				is_playing_ = true;
				start_time_ = std::chrono::steady_clock::now();
				
				// Calculate start timestamp (min of all streams)
				// initializeReaders() already primed the streams so we look at them
				// Actually playbackLoop does step 1: Find earliest start time.
				
				RCLCPP_INFO(this->get_logger(), "Starting playback thread...");
				playback_thread_ = std::thread(&DataPublisher::playbackLoop, this);
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Failed to initialize CSV readers");
			}
		}
		catch (const std::exception &e)
		{
			RCLCPP_ERROR(this->get_logger(), "Exception in constructor: %s", e.what());
			throw;
		}
	}

	DataPublisher::~DataPublisher()
	{
		is_playing_ = false;
		if (playback_thread_.joinable())
		{
			playback_thread_.join();
		}
	}

	void DataPublisher::initializeParameters()
	{
		// Declare parameters with default values
		this->declare_parameter("dataset_path", "/workspace/data/MH_01_easy/mav0");
		this->declare_parameter("playback_rate", 1.0);
		this->declare_parameter("loop_playback", false);
		this->declare_parameter("publish_images", true);
		
		// Sensor-specific rates from individual config files
		this->declare_parameter("imu.rate_hz", 200.0);
		this->declare_parameter("cam0.rate_hz", 20.0);
		this->declare_parameter("cam1.rate_hz", 20.0);

		// Get parameter values
		dataset_path_ = this->get_parameter("dataset_path").as_string();
		playback_rate_ = this->get_parameter("playback_rate").as_double();
		loop_playback_ = this->get_parameter("loop_playback").as_bool();
		publish_images_ = this->get_parameter("publish_images").as_bool();
		
		// Get sensor rates from individual config files
		imu_rate_hz_ = this->get_parameter("imu.rate_hz").as_double();
		camera_rate_hz_ = this->get_parameter("cam0.rate_hz").as_double();  // Use cam0 rate for both cameras

		RCLCPP_INFO(this->get_logger(), "Dataset path: %s", dataset_path_.c_str());
		RCLCPP_INFO(this->get_logger(), "Playback rate: %.2f", playback_rate_);
		RCLCPP_INFO(this->get_logger(), "Loop playback: %s", loop_playback_ ? "true" : "false");
		RCLCPP_INFO(this->get_logger(), "Publish images: %s", publish_images_ ? "true" : "false");
		RCLCPP_INFO(this->get_logger(), "IMU rate: %.1f Hz", imu_rate_hz_);
		RCLCPP_INFO(this->get_logger(), "Camera rate: %.1f Hz", camera_rate_hz_);
	}

	void DataPublisher::initializePublishers()
	{
		// Create publishers
		imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 100);
		pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 100);
		velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 100);

		initializeClock();

		// Initialize image transport publishers directly using raw node pointer
		if (publish_images_)
		{
			image_publisher_cam0_ = image_transport::create_publisher(this, "cam0/image_raw");
			image_publisher_cam1_ = image_transport::create_publisher(this, "cam1/image_raw");
			RCLCPP_INFO(this->get_logger(), "Image transport publishers initialized");
		}
	}

	void DataPublisher::initializeClock()
	{
		clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
	}

	bool DataPublisher::initializeReaders()
	{
		// Initialize CSV readers
		std::string imu_path = dataset_path_ + "/imu0/data.csv";
		std::string cam0_path = dataset_path_ + "/cam0/data.csv";
		std::string cam1_path = dataset_path_ + "/cam1/data.csv";
		std::string pose_path = dataset_path_ + "/state_groundtruth_estimate0/data.csv";

		// Check if files exist
		if (!std::filesystem::exists(imu_path))
		{
			RCLCPP_ERROR(this->get_logger(), "IMU data file not found: %s", imu_path.c_str());
			return false;
		}

		if (!std::filesystem::exists(pose_path))
		{
			RCLCPP_ERROR(this->get_logger(), "Pose data file not found: %s", pose_path.c_str());
			return false;
		}

		// Initialize readers
		imu_reader_ = std::make_unique<CSVReader>(imu_path, true);
		pose_reader_ = std::make_unique<CSVReader>(pose_path, true);

		if (publish_images_)
		{
			if (std::filesystem::exists(cam0_path))
			{
				cam0_reader_ = std::make_unique<CSVReader>(cam0_path, true);
			}
			if (std::filesystem::exists(cam1_path))
			{
				cam1_reader_ = std::make_unique<CSVReader>(cam1_path, true);
			}
		}

		// Initialize parsers
		imu_parser_ = std::make_unique<IMUParser>();
		pose_parser_ = std::make_unique<PoseParser>();

		if (publish_images_)
		{
			if (cam0_reader_)
			{
				cam0_parser_ = std::make_unique<CameraParser>(dataset_path_ + "/cam0/data");
			}
			if (cam1_reader_)
			{
				cam1_parser_ = std::make_unique<CameraParser>(dataset_path_ + "/cam1/data");
			}
		}

		// Verify readers are open
		if (!imu_reader_->isOpen() || !pose_reader_->isOpen())
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to open required CSV files");
			return false;
		}

		// Activate streams and prime them
		state_imu_.active = true;
		advanceStream(state_imu_, *imu_reader_, *imu_parser_);

		state_pose_.active = true;
		advanceStream(state_pose_, *pose_reader_, *pose_parser_);

		if (cam0_reader_)
		{
			state_cam0_.active = true;
			advanceStream(state_cam0_, *cam0_reader_, *cam0_parser_);
		}

		if (cam1_reader_)
		{
			state_cam1_.active = true;
			advanceStream(state_cam1_, *cam1_reader_, *cam1_parser_);
		}

		return true;
	}

	void DataPublisher::advanceStream(StreamState& state, CSVReader& reader, DataParser& parser)
	{
		if (!state.active) return;
		
		if (reader.readNextRow(state.raw_row))
		{
			if (parser.parseRow(state.raw_row))
			{
				state.timestamp_ns = parser.getTimestamp();
				state.has_data = true;
			}
			else
			{
				// Sort of a skip, try again recursively or just mark no data effectively
				// Ideally we loop till valid data or EOF, but for now let's just recurse once or mark bad
				// A simple loop here is safer:
				while (reader.readNextRow(state.raw_row)) {
					if (parser.parseRow(state.raw_row)) {
						state.timestamp_ns = parser.getTimestamp();
						state.has_data = true;
						return;
					}
				}
				state.has_data = false;
			}
		}
		else
		{
			state.has_data = false;
		}
	}

	void DataPublisher::playbackLoop()
	{
		// 1. Find the earliest start time to baseline
		start_timestamp_ = state_imu_.timestamp_ns; // Default to IMU
		
		// 2. Main loop
		while (rclcpp::ok() && is_playing_)
		{
			// Find stream with minimum timestamp
			StreamState* best_stream = nullptr;
			uint64_t best_time = std::numeric_limits<uint64_t>::max();
			std::string best_name = "";

			// Check IMU
			if (state_imu_.has_data && state_imu_.timestamp_ns < best_time)
			{
				best_time = state_imu_.timestamp_ns;
				best_stream = &state_imu_;
				best_name = "IMU";
			}
			
			// Check Cam0
			if (state_cam0_.has_data && state_cam0_.timestamp_ns < best_time)
			{
				best_time = state_cam0_.timestamp_ns;
				best_stream = &state_cam0_;
				best_name = "Cam0";
			}

			// Check Cam1
			if (state_cam1_.has_data && state_cam1_.timestamp_ns < best_time)
			{
				best_time = state_cam1_.timestamp_ns;
				best_stream = &state_cam1_;
				best_name = "Cam1";
			}

			// Check Pose
			if (state_pose_.has_data && state_pose_.timestamp_ns < best_time)
			{
				best_time = state_pose_.timestamp_ns;
				best_stream = &state_pose_;
				best_name = "Pose";
			}

			if (!best_stream)
			{
				// No more data in any stream
				if (loop_playback_)
				{
					resetReaders();
					continue;
				}
				else
				{
					RCLCPP_INFO(this->get_logger(), "End of dataset reached.");
					break;
				}
			}

			// Publish Clock
			rosgraph_msgs::msg::Clock clock_msg;
			clock_msg.clock = convertTimestamp(best_time);
			clock_publisher_->publish(clock_msg);

			// Realtime Factor throttle
			// Calculate Sim Time elapsed
			double sim_elapsed = (best_time - start_timestamp_) * 1e-9;
			// Calculate Real Time elapsed
			auto now = std::chrono::steady_clock::now();
			double real_elapsed = std::chrono::duration<double>(now - start_time_).count();
			
			double target_real_time = sim_elapsed / playback_rate_;
			
			if (real_elapsed < target_real_time)
			{
				std::this_thread::sleep_for(std::chrono::duration<double>(target_real_time - real_elapsed));
			}

			// Publish Data
			if (best_name == "IMU")
			{
				const auto &imu_data = imu_parser_->getData();
				auto imu_msg = sensor_msgs::msg::Imu();
				imu_msg.header.stamp = convertTimestamp(best_time);
				imu_msg.header.frame_id = "imu0";

				imu_msg.angular_velocity.x = imu_data.angular_velocity_x;
				imu_msg.angular_velocity.y = imu_data.angular_velocity_y;
				imu_msg.angular_velocity.z = imu_data.angular_velocity_z;

				imu_msg.linear_acceleration.x = imu_data.linear_acceleration_x;
				imu_msg.linear_acceleration.y = imu_data.linear_acceleration_y;
				imu_msg.linear_acceleration.z = imu_data.linear_acceleration_z;

				for (int i = 0; i < 9; ++i) {
					imu_msg.angular_velocity_covariance[i] = -1.0;
					imu_msg.linear_acceleration_covariance[i] = -1.0;
					imu_msg.orientation_covariance[i] = -1.0;
				}
				imu_publisher_->publish(imu_msg);
				advanceStream(state_imu_, *imu_reader_, *imu_parser_);
			}
			else if (best_name == "Cam0")
			{
				const auto &cam_data = cam0_parser_->getData();
				publishImage(cam_data.filename, best_time, image_publisher_cam0_, "cam0");
				advanceStream(state_cam0_, *cam0_reader_, *cam0_parser_);

			}
			else if (best_name == "Cam1")
			{
				const auto &cam_data = cam1_parser_->getData();
				publishImage(cam_data.filename, best_time, image_publisher_cam1_, "cam1");
				advanceStream(state_cam1_, *cam1_reader_, *cam1_parser_);
			}
			else if (best_name == "Pose")
			{
				const auto &pose_data = pose_parser_->getData();
				
				auto pose_msg = geometry_msgs::msg::PoseStamped();
				pose_msg.header.stamp = convertTimestamp(best_time);
				pose_msg.header.frame_id = "world";

				pose_msg.pose.position.x = pose_data.position_x;
				pose_msg.pose.position.y = pose_data.position_y;
				pose_msg.pose.position.z = pose_data.position_z;

				pose_msg.pose.orientation.w = pose_data.quaternion_w;
				pose_msg.pose.orientation.x = pose_data.quaternion_x;
				pose_msg.pose.orientation.y = pose_data.quaternion_y;
				pose_msg.pose.orientation.z = pose_data.quaternion_z;

				pose_publisher_->publish(pose_msg);

				auto velocity_msg = geometry_msgs::msg::TwistStamped();
				velocity_msg.header.stamp = convertTimestamp(best_time);
				velocity_msg.header.frame_id = "world";

				velocity_msg.twist.linear.x = pose_data.velocity_x;
				velocity_msg.twist.linear.y = pose_data.velocity_y;
				velocity_msg.twist.linear.z = pose_data.velocity_z;

				velocity_publisher_->publish(velocity_msg);
				
				advanceStream(state_pose_, *pose_reader_, *pose_parser_);
			}
		}
	}

	rclcpp::Time DataPublisher::convertTimestamp(uint64_t timestamp_ns)
	{
		return rclcpp::Time(timestamp_ns, RCL_ROS_TIME);
	}

	void DataPublisher::publishImage(const std::string &raw_path, uint64_t timestamp_ns,
																	 image_transport::Publisher &publisher, const std::string &frame_id)
	{
		try
		{
			std::string image_path = raw_path;

			// Remove carriage returns (\r) and newlines (\n) from the string
			image_path.erase(std::remove(image_path.begin(), image_path.end(), '\r'), image_path.end());
			image_path.erase(std::remove(image_path.begin(), image_path.end(), '\n'), image_path.end());

			if (!std::filesystem::exists(image_path))
			{
				RCLCPP_ERROR(this->get_logger(),
										 "File not found on disk: [%s] (Original was: [%s])",
										 image_path.c_str(), raw_path.c_str());
				return;
			}

			cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

			if (image.empty())
			{
				RCLCPP_WARN(this->get_logger(), "OpenCV loaded empty image: %s", image_path.c_str());
				return;
			}

			std_msgs::msg::Header header;
			header.stamp = convertTimestamp(timestamp_ns);
			header.frame_id = frame_id;

			sensor_msgs::msg::Image::SharedPtr img_msg =
					cv_bridge::CvImage(header, "mono8", image).toImageMsg();

			publisher.publish(*img_msg);
		}
		catch (const std::exception &e)
		{
			RCLCPP_ERROR(this->get_logger(), "Error publishing image %s: %s", raw_path.c_str(), e.what());
		}
	}

	void DataPublisher::resetReaders()
	{
		if (imu_reader_)
			imu_reader_->reset(true);
		if (pose_reader_)
			pose_reader_->reset(true);
		if (cam0_reader_)
			cam0_reader_->reset(true);
		if (cam1_reader_)
			cam1_reader_->reset(true);

		// Re-prime
		advanceStream(state_imu_, *imu_reader_, *imu_parser_);
		advanceStream(state_pose_, *pose_reader_, *pose_parser_);
		if(state_cam0_.active) advanceStream(state_cam0_, *cam0_reader_, *cam0_parser_);
		if(state_cam1_.active) advanceStream(state_cam1_, *cam1_reader_, *cam1_parser_);

		// Reset time reference
		start_timestamp_ = state_imu_.timestamp_ns; // Or closest
		start_time_ = std::chrono::steady_clock::now();
	}

} // namespace euroc_driver
