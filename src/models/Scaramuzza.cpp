#include "models.h"


cv::Point2d ScaramuzzaModel::projectWorldToPixel(cv::Mat worldPoint)
{
    double X = worldPoint.at<float>(0);
    double Y = worldPoint.at<float>(1);
    double Z = worldPoint.at<float>(2);
    double phi = atan2(Z, sqrt(X * X + Y * Y));
    double rho = 0;
    double error = 1;

    int iter = 0;
    do
    {
        double R = scara_polynom[0];      // R = f(rho)
        for (int i = 1; i < scara_polynom.size(); i++)
        {
            R += scara_polynom[i] * pow(rho, i + 1);
            //std::cout << "R: " << R << std::endl;
        }

        error = atan2(R, rho) - phi;
        iter++;
        rho = rho + 200 * error;
        //std::cout << "It " << iter << " Point: " << worldPoint << " | Rho: " << rho << " f(Rho): " << 424242 << " Error: " << error << std::endl;
    } while (std::abs(error) > 0.005 && iter < 100);
    this->errorsum += error;

    lambda = sqrt(X * X + Y * Y) / rho;
    double u = X / lambda;
    double v = Y / lambda;

    cv::Point fypixel(stretchMatrix * cv::Vec2d(u, v) + centerOffset);        // technically could do toCorner's job, but I'll keep it simple for now
    //toCorner(fypixel, oldSize);
    return fypixel;
}

cv::Mat ScaramuzzaModel::projectPixelToWorld(cv::Point pixel)
{
    cv::Vec2d undistPixel = cv::Vec2d(pixel.x, pixel.y); // stretchMatrix * (cv::Vec2d(pixel.x, pixel.y) - centerOffset);      // TODO: delete
    //std::cout << stretchMatrix << " | " << cv::Vec2d(pixel.x, pixel.y) - centerOffset << " | " << lambda * undistPixel[0] << std::endl;
    cv::Mat cameraCoords(1, 3, CV_32F, float(0));
    double rho = norm(pixel);      // sqrt ( x^2 + y^2 )
    cameraCoords.at<float>(0) = lambda * undistPixel[0];
    cameraCoords.at<float>(1) = lambda * undistPixel[1];
    cameraCoords.at<float>(2) = lambda * (scara_polynom[0] + scara_polynom[1] * pow(rho, 2) + scara_polynom[2] * pow(rho, 3) + scara_polynom[3] * pow(rho, 4));
    //cameraCoords *= lambda;
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
