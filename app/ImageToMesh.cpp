// image to mesh
// at first we just use gray value
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "Geometry/TriangleMesh/TriangleMesh.h"
using namespace dragon;
int main(int argc, char* argv[])
{
    if(argc != 2)
    {
        std::cout << "Usage: ImageToMesh [image]"<<std::endl;
        return 0;
    }
    cv::Mat rgb_ = cv::imread(argv[1]);
    CvSize size;
    double scale = 0.5;
    size.height = rgb_.rows * scale;
    size.width = rgb_.cols * scale;
    cv::Mat rgb = cv::Mat(size, rgb_.depth(), rgb_.channels());
    cv::resize(rgb_, rgb, size);
    cv::Mat gray;
    cv::cvtColor(rgb, gray, CV_RGB2GRAY);
    cv::imwrite("gray_cow.jpg", gray);
    int row = rgb.rows;
    int col = rgb.cols;
    geometry::TriangleMesh mesh;
    for(int i = 0; i != row; ++i)
    for(int j = 0; j != col; ++j)
    {
        mesh.points.push_back(geometry::Point3(i+0.5, j + 0.5, (gray.at<unsigned char>(i, j) + 0.0) * 0.5));
        auto color = rgb.at<cv::Vec3b>(i*col + j);
        // std::cout<<color(2)+0.0<<" "<<color(1)+0.0<<" "<<color(0)+0.0<<std::endl;
        mesh.colors.push_back(geometry::Point3(color(2)/255.0, color(1)/255.0, color(0)/255.0));
    }
    for(int i = 0; i != row; ++i)
    for(int j = 0; j != col; ++j)
    {
        if(i < row - 1 && j < col - 1)
        {
            int v1 = i * col + j;
            int v2 = i * col + j + 1;
            int v3 = (i + 1) * col + j;
            mesh.triangles.push_back(geometry::Point3ui(v3, v2, v1));
        }
        if(i < row - 1 && j > 0 )
        {
            int v1 = i * col + j;
            int v2 = (i + 1) * col + j;
            int v3 = (i + 1) * col + j - 1;
            mesh.triangles.push_back(geometry::Point3ui(v3, v2, v1));            
        }
    }
    mesh.WriteToPLY("from_image.ply");
    return 0;
}