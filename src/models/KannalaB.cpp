#include "models.hpp"


KBModel::KBModel()
{
    setModelName("KB");
}

KBModel::KBModel(std::initializer_list<double> coeffs,
    cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    setIntrinsics(coeffs, centerOffset, stretchMatrix);
}

void KBModel::setIntrinsics(std::initializer_list<double> coeffs,
    cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    this->kb_polynom.assign(coeffs.begin(), coeffs.end());       // treats both values as pointers 
    this->centerOffset = centerOffset;
    this->stretchMatrix = stretchMatrix;
}

void KBModel::setIntrinsics(cv::Vec4d coeffs,
    cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    this->kb_polynom.assign(coeffs.val, coeffs.val + 4);
    this->centerOffset = centerOffset;
    this->stretchMatrix = stretchMatrix;
}

cv::Point2d KBModel::projectWorldToPixel(cv::Mat worldPoint)
{

    double theta = acos(worldPoint.at<float>(2) / cv::norm(worldPoint, cv::NormTypes::NORM_L2));
    double phi = atan2(worldPoint.at<float>(1), worldPoint.at<float>(0));
    double poly = theta + kb_polynom[0] * pow(theta, 3) + kb_polynom[1] * pow(theta, 5) +
        kb_polynom[2] * pow(theta, 7) + kb_polynom[3] * pow(theta, 9);

    cv::Vec2d imgPixel = poly * cv::Point2d(cos(phi), -sin(phi));
    cv::Point fypixel(stretchMatrix * imgPixel + centerOffset);     

    return fypixel;
}

void KBModel::backprojectSymmetric(cv::Point pxl, double& theta, double& phi)
{
    double tol = 1e-10;
    double p_u_norm = norm(pxl);

    if (p_u_norm < 1e-10)
    {
        phi = 0.0;
    }
    else
    {
        phi = atan2(pxl.y, pxl.x);
    }

    int npow = 9;
    if (kb_polynom[3] == 0.0)
    {
        npow -= 2;
    }
    if (kb_polynom[2] == 0.0)
    {
        npow -= 2;
    }
    if (kb_polynom[1] == 0.0)
    {
        npow -= 2;
    }
    if (kb_polynom[0] == 0.0)
    {
        npow -= 2;
    }

    cv::Mat coeffs(npow + 1, 1, CV_64F, double(0));
    coeffs.at<double>(0) = -p_u_norm;
    coeffs.at<double>(1) = 1.0;

    if (npow >= 3)
    {
        coeffs.at<double>(3) = kb_polynom[0];
    }
    if (npow >= 5)
    {
        coeffs.at<double>(5) = kb_polynom[1];
    }
    if (npow >= 7)
    {
        coeffs.at<double>(7) = kb_polynom[2];
    }
    if (npow >= 9)
    {
        coeffs.at<double>(9) = kb_polynom[3];
    }

    if (npow == 1)
    {
        theta = p_u_norm;
    }
    else
    {
        // Get eigenvalues of companion matrix corresponding to polynomial.
        // Eigenvalues correspond to roots of polynomial.
        cv::Mat A(npow, npow, CV_64F, double(0));
        cv::setIdentity(A(cv::Rect(1, 0, npow - 1, npow - 1)));
        A.col(npow - 1) = -coeffs(cv::Rect(0, 0, npow, 1)) / coeffs.at<double>(npow);

        cv::PCA pt_pca(A, cv::Mat(), cv::PCA::DATA_AS_ROW, 0);
        cv::Mat eigval = pt_pca.eigenvalues;

        std::vector<double> thetas;
        for (int i = 0; i < eigval.rows; ++i)
        {
            //if (fabs(eigval.at<double>(i).imag()) > tol)
            //{
            //    continue;
            //}

            //double t = eigval(i).real();
            double t = 0;       // FIXME: this and everiyhing above should be rewritten
            if (t < -tol)
            {
                continue;
            }
            else if (t < 0.0)
            {
                t = 0.0;
            }

            thetas.push_back(t);
        }

        if (thetas.empty())
        {
            theta = p_u_norm;
        }
        else
        {
            theta = *std::min_element(thetas.begin(), thetas.end());
        }
    }
}

cv::Mat KBModel::projectPixelToWorld(cv::Point pixel)
{
    //int u0 = 540; // size/2
    //int v0 = 540; // size/2
    //double invK11 = 1.0 / mu;
    //double invK13 = -u0 / mu;
    //double invK22 = 1.0 / mv;
    //double invK23 = -v0 / mv;

    double theta, phi;

    backprojectSymmetric(pixel, theta, phi);

    double data[3] = { sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta) };

    return cv::Mat(1, 3, CV_64F, data);
}