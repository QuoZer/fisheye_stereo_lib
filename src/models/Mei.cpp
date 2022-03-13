#include "models.h"

MeiModel::MeiModel()
{
    k2 = 0;
    k3 = 0;
    k4 = 0;
    k5 = 0;
    mu = 0;
    mv = 0;
}

MeiModel::MeiModel(double k2, double k3, double k4, double k5, double mu, double mv)
{
    setIntrinsics(k2, k3, k4, k5, mu, mv);
}

void MeiModel::setIntrinsics(double k2, double k3, double k4, double k5, double mu, double mv)
{
    this->k2 = k2;
    this->k3 = k3;
    this->k4 = k4;
    this->k5 = k5;
    this->mu = mu;
    this->mv = mv;
}

cv::Point2d MeiModel::projectWorldToPixel(cv::Mat worldPoint)
{
	double theta = acos(worldPoint.at<float>(2) / cv::norm(worldPoint));
	double phi = atan2(worldPoint.at<float>(1), worldPoint.at<float>(0));

	// k1 =1
	double poly = theta + k2 * pow(theta, 3) + k3 * pow(theta, 5) + k4 * pow(theta, 7) + k5 * pow(theta, 9);
	// cos(acos()) ????
    cv::Point imgPixel = poly * cv::Point2d(cos(phi), sin(phi));
    toCorner( imgPixel, oldSize);
    return imgPixel;
}

void MeiModel::backprojectSymmetric(cv::Point pxl, double& theta, double& phi)
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
    if (k5 == 0.0)
    {
        npow -= 2;
    }
    if (k4 == 0.0)
    {
        npow -= 2;
    }
    if (k3 == 0.0)
    {
        npow -= 2;
    }
    if (k2 == 0.0)
    {
        npow -= 2;
    }

    cv::Mat coeffs(npow + 1, 1, CV_64F, double(0));
    coeffs.at<double>(0) = -p_u_norm;
    coeffs.at<double>(1) = 1.0;

    if (npow >= 3)
    {
        coeffs.at<double>(3) = k2;
    }
    if (npow >= 5)
    {
        coeffs.at<double>(5) = k3;
    }
    if (npow >= 7)
    {
        coeffs.at<double>(7) = k4;
    }
    if (npow >= 9)
    {
        coeffs.at<double>(9) = k5;
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

cv::Mat MeiModel::projectPixelToWorld(cv::Point pixel)
{
	//int u0 = 540; // size/2
	//int v0 = 540; // size/2
	//double invK11 = 1.0 / mu;
	//double invK13 = -u0 / mu;
	//double invK22 = 1.0 / mv;
	//double invK23 = -v0 / mv;

	double theta, phi;

    backprojectSymmetric(pixel, theta, phi);
	
	double data[3] = { sin(theta) * cos(phi), sin(theta)* sin(phi), cos(theta)};
	
	return cv::Mat(1, 3, CV_64F, data);
}