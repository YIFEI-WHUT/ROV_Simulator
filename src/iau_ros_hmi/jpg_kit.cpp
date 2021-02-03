#include"jpg_kit.h"
#include <vector>
using namespace std;
using namespace cv;
using namespace boost::archive::iterators;


Mat stream2jpg(const char* buf, size_t len) {
	Mat dst;
	vector <unsigned char> img_data;

    std::string str(buf, len);
    std::string out_str;
    Base64Encode(str, &out_str);

	img_data.assign(out_str.begin(), out_str.end());
	dst = imdecode(img_data, CV_LOAD_IMAGE_COLOR);
	cv::namedWindow("dst", 0);//0:自适应大小 1:原始大小
	cv::imshow("dst", dst);
    cvWaitKey(0);
	return dst;
}




bool Base64Encode(const string& input, string* output)
{
    typedef base64_from_binary<transform_width<string::const_iterator, 6, 8>> Base64EncodeIterator;
    stringstream result;
    try {
        copy(Base64EncodeIterator(input.begin()), Base64EncodeIterator(input.end()), ostream_iterator<char>(result));
    }
    catch (...) {
        return false;
    }
    size_t equal_count = (3 - input.length() % 3) % 3;
    for (size_t i = 0; i < equal_count; i++)
    {
        result.put('=');
    }
    *output = result.str();
    return output->empty() == false;
}

bool Base64Decode(const string& input, string* output)
{
    typedef transform_width<binary_from_base64<string::const_iterator>, 8, 6> Base64DecodeIterator;
    stringstream result;
    try {
        copy(Base64DecodeIterator(input.begin()), Base64DecodeIterator(input.end()), ostream_iterator<char>(result));
    }
    catch (...) {
        return false;
    }
    *output = result.str();
    return output->empty() == false;
}

