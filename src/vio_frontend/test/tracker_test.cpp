#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "vio_frontend/feature_tracker.hpp"

class TrackerTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Setup config with EuroC parameters (same as original check)
    config_.max_features = 100;
    config_.min_dist = 20;
    config_.K = (cv::Mat_<double>(3, 3) << 458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0);
    config_.D = (cv::Mat_<double>(4, 1) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);

    tracker_ = std::make_unique<vio_frontend::FeatureTracker>(config_);
  }

  // Helper to generate synthetic images with features
  cv::Mat generate_image(const std::vector<cv::Point2f>& points) {
    cv::Mat img = cv::Mat::zeros(480, 752, CV_8UC1);
    for (const auto& p : points) {
      cv::circle(img, p, 3, cv::Scalar(255), -1);
    }
    return img;
  }

  vio_frontend::TrackerConfig config_;
  std::unique_ptr<vio_frontend::FeatureTracker> tracker_;
};

TEST_F(TrackerTest, DetectsFeatures) {
  // Create image with 3 distinct features
  std::vector<cv::Point2f> points = {
    {100, 100},
    {300, 200},
    {500, 300}
  };
  cv::Mat img = generate_image(points);

  auto tracks = tracker_->track_features(img, 0);

  // Should find all 3 features
  // Note: FeatureTracker max_features is 100, min_dist 20.
  // Our points are far apart so all should be kept.
  EXPECT_EQ(tracks.size(), 3u);
}

TEST_F(TrackerTest, TracksMovingFeatures) {
  // Frame 1: Features at initial positions
  std::vector<cv::Point2f> points1 = {
    {100, 100},
    {300, 200},
    {500, 300}
  };
  cv::Mat img1 = generate_image(points1);

  auto tracks1 = tracker_->track_features(img1, 0);
  EXPECT_EQ(tracks1.size(), 3u);

  // Frame 2: Features moved by (10, 10)
  std::vector<cv::Point2f> points2;
  for (const auto& p : points1) {
    points2.emplace_back(p.x + 10, p.y + 10);
  }
  cv::Mat img2 = generate_image(points2);

  // dt = 0.1s
  auto tracks2 = tracker_->track_features(img2, 100000000); // 100ms in ns
  
  EXPECT_EQ(tracks2.size(), 3u);
  
  // Check if tracked positions are correct relative to first frame
  // We compare against tracks1 + delta, because goodFeaturesToTrack might not 
  // detect features exactly at the center of our synthetic circles.
  // By checking relative motion, we verify the tracking logic specifically.
  for (const auto& feature : tracks2) {
    bool found_match = false;
    for (const auto& prev : tracks1) {
      if (prev.id == feature.id) {
        found_match = true;
        
        // Expected position is previous position + (10, 10)
        float expected_u = prev.u + 10.0;
        float expected_v = prev.v + 10.0;
        
        float dx = feature.u - expected_u;
        float dy = feature.v - expected_v;
        
        // LK tracking should be very accurate for pure translation
        EXPECT_NEAR(dx, 0.0, 1.0) << "Feature " << feature.id << " u error too high";
        EXPECT_NEAR(dy, 0.0, 1.0) << "Feature " << feature.id << " v error too high";
        break;
      }
    }
    EXPECT_TRUE(found_match) << "Tracked feature id " << feature.id << " not found in previous frame";
  }
}

TEST_F(TrackerTest, CalculatesVelocity) {
  // Single feature moving horizontally
  cv::Mat img1 = generate_image({{100, 100}});
  tracker_->track_features(img1, 0);

  // Move 10 pixels right in 0.1s => 100 px/s
  cv::Mat img2 = generate_image({{110, 100}});
  auto tracks = tracker_->track_features(img2, 100000000); // 100ms

  ASSERT_EQ(tracks.size(), 1u);
  auto f = tracks[0];

  // Velocity is in normalized coordinates, not pixels!
  // We just check it's non-zero and in right direction
  std::cout << "Horizontal Velocity: " << f.velocity_x << " Vertical Velocity: " << f.velocity_y << std::endl;
  EXPECT_GT(f.velocity_x, 0.0);
  EXPECT_NEAR(f.velocity_y, 0.0, 0.1); // Should be roughly 0 vertical velocity
}
