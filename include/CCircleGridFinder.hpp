#ifndef __CCIRCLEGRIDFINDER_H__
#define __CCIRCLEGRIDFINDER_H__

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <numeric>
#include <algorithm>


class CircleGridFinder{
    public:
        CircleGridFinder(bool _isAsymmetricGrid);
        void hierarchicalClustering(const std::vector<cv::Point2f> &points, const cv::Size &patternSz, std::vector<cv::Point2f> &patternPoints);
        void findGrid(const std::vector<cv::Point2f> &points, cv::Size _patternSize, std::vector<cv::Point2f>& centers);
        void findCorners(const std::vector<cv::Point2f> &hull2f, std::vector<cv::Point2f> &corners);
        void findOutsideCorners(const std::vector<cv::Point2f> &corners, std::vector<cv::Point2f> &outsideCorners);
        void getSortedCorners(const std::vector<cv::Point2f> &hull2f, const std::vector<cv::Point2f> &patternPoints, const std::vector<cv::Point2f> &corners, 
                                const std::vector<cv::Point2f> &outsideCorners, std::vector<cv::Point2f> &sortedCorners);
        void rectifyPatternPoints(const std::vector<cv::Point2f> &patternPoints, const std::vector<cv::Point2f> &sortedCorners, 
                                    std::vector<cv::Point2f> &rectifiedPatternPoints);
        void parsePatternPoints(const std::vector<cv::Point2f> &patternPoints, const std::vector<cv::Point2f> &rectifiedPatternPoints, 
                                std::vector<cv::Point2f> &centers);


        bool isAsymmetricGrid;
        cv::Size patternSize;
        float squareSize, maxRectifiedDistance;

};
#endif