#include "vio_frontend/feature_tracker.hpp"
#include <rclcpp/rclcpp.hpp>

namespace vio_frontend {

FeatureTracker::FeatureTracker(const TrackerConfig& config) 
    : config_(config), first_frame_(true), n_id_(0) {
}

std::vector<vio_frontend::msg::Feature> FeatureTracker::track_features(const cv::Mat& img_in, uint64_t /*timestamp_ns*/) {
    std::vector<vio_frontend::msg::Feature> features;
 
    cv::Mat img;
    // Ensure grayscale
    if (img_in.channels() == 1) {
        img = img_in;
    } else {
        cv::cvtColor(img_in, img, cv::COLOR_BGR2GRAY);
    }

    // Histogram equalization
    if (config_.equalize) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, curr_img_);
    } else {
        curr_img_ = img;
    }

    // Tracking
    if (!first_frame_ && curr_pts_.size() > 0) {
        std::vector<uchar> status;
        std::vector<float> err;
        
        cv::calcOpticalFlowPyrLK(prev_img_, curr_img_, prev_pts_, curr_pts_, 
                                 status, err, config_.win_size, config_.max_level);

        // Filter out bad points (status=0) and OOB points
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i] && (curr_pts_[i].x < 0 || curr_pts_[i].y < 0 || 
                              curr_pts_[i].x >= curr_img_.cols || curr_pts_[i].y >= curr_img_.rows)) {
                status[i] = 0;
            }
        }
        
        reduce_vector(prev_pts_, status);
        reduce_vector(curr_pts_, status);
        reduce_vector(track_ids_, status);
        reduce_vector(track_cnt_, status);

        // Outlier rejection (RANSAC)
        if (config_.use_ransac) {
            reject_outliers();
        }
    } else {
        curr_pts_.clear();
    }

    // Detect new features if needed
    if (static_cast<int>(curr_pts_.size()) < config_.max_features) {
        detect_new_features();
    }

    // Prepare for next frame
    first_frame_ = false;
    
    // Calculate Undistorted Points and Velocity
    if (!curr_pts_.empty()) {
        std::vector<cv::Point2f> curr_norm_pts;
        if (!config_.K.empty() && !config_.D.empty()) {
             cv::undistortPoints(curr_pts_, curr_norm_pts, config_.K, config_.D);
        } else {
             curr_norm_pts = curr_pts_; // Fallback if no intrinsics
        }

        for (size_t i = 0; i < curr_pts_.size(); i++) {
            track_cnt_[i]++;
            
            vio_frontend::msg::Feature feature_msg;
            feature_msg.id = track_ids_[i];
            feature_msg.u = curr_pts_[i].x;
            feature_msg.v = curr_pts_[i].y;
            
            if (i < curr_norm_pts.size()) {
                feature_msg.u_norm = curr_norm_pts[i].x;
                feature_msg.v_norm = curr_norm_pts[i].y;
            }

            if (i < prev_pts_.size()) {
                 feature_msg.velocity_x = (curr_pts_[i].x - prev_pts_[i].x);
                 feature_msg.velocity_y = (curr_pts_[i].y - prev_pts_[i].y);
            }
            
            features.push_back(feature_msg);
            
            tracks_[track_ids_[i]] = curr_pts_[i];
        }
    }

    prev_img_ = curr_img_.clone();
    prev_pts_ = curr_pts_;

    return features;
}

void FeatureTracker::detect_new_features() {
    set_mask();

    int n_max_new = config_.max_features - static_cast<int>(curr_pts_.size());
    if (n_max_new > 0) {
        std::vector<cv::Point2f> n_pts;
        cv::goodFeaturesToTrack(curr_img_, n_pts, n_max_new, config_.quality_level, config_.min_dist, mask_);

        for (auto &p : n_pts) {
            curr_pts_.push_back(p);
            track_ids_.push_back(n_id_++);
            track_cnt_.push_back(1);
        }
    }
}

void FeatureTracker::reject_outliers() {
    if (prev_pts_.size() >= 8) {
        std::vector<uchar> status;
        cv::findFundamentalMat(prev_pts_, curr_pts_, cv::FM_RANSAC, config_.f_threshold, 0.99, status);
        
        reduce_vector(prev_pts_, status);
        reduce_vector(curr_pts_, status);
        reduce_vector(track_ids_, status);
        reduce_vector(track_cnt_, status);
    }
}

void FeatureTracker::set_mask() {
    mask_ = cv::Mat(curr_img_.size(), CV_8UC1, cv::Scalar(255));

    // Mask current feature positions to distribute features evenly
    for (auto &pt : curr_pts_) {
        cv::circle(mask_, pt, config_.min_dist, 0, -1);
    }
}

cv::Mat FeatureTracker::draw_tracks() const {
    cv::Mat img_out;
    cv::cvtColor(curr_img_, img_out, cv::COLOR_GRAY2BGR);
    
    for (size_t i = 0; i < curr_pts_.size(); i++) {
        double len = std::min(1.0, 1.0 * track_cnt_[i] / 20);
        cv::circle(img_out, curr_pts_[i], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    return img_out;
}

} // namespace vio_frontend
