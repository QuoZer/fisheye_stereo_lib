#include "models.hpp"

ScaramuzzaModel::ScaramuzzaModel()
{
    setModelName("Sc");
}

cv::Point2d ScaramuzzaModel::projectWorldToPixel(cv::Mat worldPoint)
{
    // unpack the coords
    double X = worldPoint.at<float>(0);
    double Y = worldPoint.at<float>(1);
    double Z = worldPoint.at<float>(2);
    // angle of attack
    double phi = atan2(Z, sqrt(X * X + Y * Y));
    double rho = 0;
    double error = 1;

    // bellow we iteratively try to find rho such that the angles of distorted point would match with the one from input
    int iter = 0;
    do
    {
        double R = scara_polynom[0];      // R = f(rho)
        for (int i = 1; i < scara_polynom.size(); i++)
        {   // assemble the polynomial
            R += scara_polynom[i] * pow(rho, i + 1);
        }

        error = atan2(R, rho) - phi;
        rho = rho + 200 * error;
        iter++;
        //std::cout << "It " << iter << " Point: " << worldPoint << " | Rho: " << rho << " f(Rho): " << 424242 << " Error: " << error << std::endl;
    } while (std::abs(error) > 0.005 && iter < 100);
    this->errorsum += error;
    
    lambda = sqrt(X * X + Y * Y) / rho;
    double u = X / lambda;
    double v = Y / lambda;

    cv::Point fypixel(stretchMatrix * cv::Vec2d(u, -v) + centerOffset);        
    //toCorner(fypixel, oldSize);
    return fypixel;
}

cv::Mat ScaramuzzaModel::projectPixelToWorld(cv::Point pixel)
{
    //cv::Vec2d undistPixel = cv::Vec2d(pixel.x, pixel.y); 
    cv::Mat cameraCoords(1, 3, CV_32F, float(0));
    double rho = norm(pixel);      

    cameraCoords.at<float>(0) = lambda * pixel.x;
    cameraCoords.at<float>(1) = lambda * pixel.y;
    cameraCoords.at<float>(2) = lambda * (scara_polynom[0] + scara_polynom[1] * pow(rho, 2) + scara_polynom[2] * pow(rho, 3) + scara_polynom[3] * pow(rho, 4));
    
    return cameraCoords;
}

void ScaramuzzaModel::setIntrinsics(std::initializer_list<double> coeffs, double lambda,
    cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    this->scara_polynom.assign(coeffs.begin(), coeffs.end());       // treats both values as pointers 
    this->centerOffset = centerOffset;
    this->stretchMatrix = stretchMatrix;
    this->lambda = lambda;
}

void ScaramuzzaModel::setIntrinsics(cv::Vec4d coeffs, double lambda,
    cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    this->scara_polynom.assign(coeffs.val, coeffs.val + 4);
    this->centerOffset = centerOffset;
    this->stretchMatrix = stretchMatrix;
    this->lambda = lambda;
}
