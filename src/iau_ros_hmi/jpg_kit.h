
#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>

using namespace std;
cv::Mat stream2jpg(const char* buf, size_t len);


bool Base64Encode(const string& input, string* output);


bool Base64Decode(const string& input, string* output);

