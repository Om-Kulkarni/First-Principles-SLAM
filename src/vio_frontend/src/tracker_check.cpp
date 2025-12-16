#include "vio_frontend/feature_tracker.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: ./tracker_check <img1> <img2>" << std::endl;
        return 1;
    }

    std::cout << "Loading images..." << std::endl;
    cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

    if (img1.empty() || img2.empty()) {
        std::cerr << "Failed to load images" << std::endl;
        return 1;
    }

    vio_frontend::TrackerConfig config;
    config.max_features = 100;
    config.min_dist = 20;

    // Load EuroC parameters for test
    config.K = (cv::Mat_<double>(3, 3) << 458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0);
    config.D = (cv::Mat_<double>(4, 1) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);

    vio_frontend::FeatureTracker tracker(config);

    // Frame 1
    auto tracks1 = tracker.track_features(img1, 0);
    std::cout << "Frame 1: Found " << tracks1.size() << " features" << std::endl;
    
    // Frame 2
    auto tracks2 = tracker.track_features(img2, 100000000); // dt = 0.1s = 100ms = 100e6 ns
    std::cout << "Frame 2: Tracked " << tracks2.size() << " features" << std::endl;

    if (!tracks2.empty()) {
        auto f = tracks2[0];
        std::cout << "Feature 0: u=" << f.u << ", v=" << f.v 
                  << " | u_norm=" << f.u_norm << ", v_norm=" << f.v_norm 
                  << " | vel=(" << f.velocity_x << "," << f.velocity_y << ")" << std::endl;
    }

    // Draw
    cv::Mat debug = tracker.draw_tracks();
    cv::imwrite("tracker_debug.png", debug);
    std::cout << "Saved tracker_debug.png" << std::endl;

    return 0;
}
