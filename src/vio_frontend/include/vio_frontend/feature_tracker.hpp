#pragma once

#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include "vio_frontend/msg/feature.hpp"

namespace vio_frontend {

/**
 * @brief Configuration for the Feature Tracker
 */
struct TrackerConfig {
    int max_features = 150;           // Target number of features to maintain
    int min_dist = 30;                // Minimum distance between features
    double quality_level = 0.01;      // Corner quality level for detection
    bool equalize = true;             // Equalize histogram before processing
    
    // Camera Parameters
    cv::Mat K;                        // Intrinsics (3x3)
    cv::Mat D;                        // Distortion Coefficients
    
    // KLT Parameters
    cv::Size win_size = cv::Size(21, 21);
    int max_level = 3;
    
    // RANSAC Parameters
    bool use_ransac = true;
    double f_threshold = 3.0; // Fundamental matrix threshold
};

/**
 * @brief KLT Feature Tracker implementation
 */
class FeatureTracker {
public:
    explicit FeatureTracker(const TrackerConfig& config);

    /**
     * @brief Process a new image frame
     * @param img_curr Current grayscale image
     * @param timestamp Timestamp of the current frame
     * @return Vector of Feature messages
     */
    std::vector<vio_frontend::msg::Feature> track_features(const cv::Mat& img_curr, uint64_t timestamp_ns);

    /**
     * @brief Get current feature tracks
     */
    const std::map<int, cv::Point2f>& get_tracks() const { return tracks_; }

    /**
     * @brief Debug visualization
     */
    cv::Mat draw_tracks() const;

private:
    TrackerConfig config_;
    bool first_frame_ = true;
    
    cv::Mat prev_img_, curr_img_;
    std::vector<cv::Point2f> prev_pts_, curr_pts_;
    std::vector<int> track_ids_;
    std::vector<int> track_cnt_; // Track age/count for stats
    
    int n_id_; // Global feature ID counter
    
    // Output map: ID -> Point
    std::map<int, cv::Point2f> tracks_;

    /**
     * @brief Detect new features to replenish lost tracks
     */
    void detect_new_features();

    /**
     * @brief Filter out points using RANSAC (Fundamental Matrix)
     */
    void reject_outliers();
    
    /**
     * @brief Set the mask to avoid detecting features too close to existing ones
     */
    void set_mask();

    /**
     * @brief Remove elements from vector based on status mask
     */
    template <typename T>
    void reduce_vector(std::vector<T>& v, const std::vector<uchar>& status) {
        int j = 0;
        for (size_t i = 0; i < v.size(); i++) {
            if (status[i]) {
                v[j] = v[i];
                j++;
            }
        }
        v.resize(j);
    }

    cv::Mat mask_;
};

} // namespace vio_frontend
