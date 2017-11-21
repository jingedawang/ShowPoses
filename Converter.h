#ifndef CONVERTER_H
#define CONVERTER_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>
#include <pangolin/pangolin.h>

class Converter
{
public:
//
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static pangolin::OpenGlMatrix GetCurrentOpenGLCameraMatrix(const cv::Mat &m);
};

#endif // CONVERTER_H
