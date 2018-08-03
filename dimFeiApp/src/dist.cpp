#include <iostream>
#include "stdafx.h"
#include "dist.h"


Dist2D::Dist2D(intensity *d, const int w, const int h) {
    width = w;
    height = h;
    dist = new intensity[width*height];

    for (int i=0; i<(width*height-1); ++i)
        dist[i] = d[i];
}
Dist2D::~Dist2D() {
    delete [] dist;
}
int Dist2D::test_image() {
    const unsigned char threshold = 10;
    const unsigned char saturation_value = 255;
    const float fraction = 0.950F;
    const int nums = 5;

    int pn, cn, cs;

    std::cout << "Image (" << width << ", " << height << "):" << std::endl;

    for (int y = 0; y < height; y += 10) {
        for (int x = 0; x < width; x += 10) {
            intensity pixel = dist[y*width + x];

            if (pixel < threshold)
                std::cout << " .";
            else if (pixel == saturation_value)
                std::cout << " O";
            else
                std::cout << " o";
        }

        std::cout << std::endl;
    }

    pn = (int)(fraction*width*height);

    cn = cs = 0;
    for (int i=0; i<height; ++i)
        for (int j=0; j<width; ++j) {
            if (dist[i*width+j] < threshold)
                ++cn;
            if (dist[i*width+j] == saturation_value)
                ++cs;
        }

    if (cn > pn)
        return ERROR_WEAK_IMAGE;
    else if (cs >= nums)
        return ERROR_SATURATED_IMAGE;
    else
        return 0;
}
void Dist2D::calc_default_parameters(gaussian_parameters &prms_out) {
    float min = std::numeric_limits<float>::infinity();
    float max = 0;
    for (int i=0; i<width*height; ++i) {
        if(dist[i] < min)
            min = dist[i];
        if(dist[i] > max)
            max = dist[i];
    }

    prms_out.amplitude = max - min;
    prms_out.cent_ver = ((float)height-1)/2;
    prms_out.cent_hor = ((float)width-1)/2;
    prms_out.sigma_ver = sqrt((float)height-1);
    prms_out.sigma_hor = sqrt((float)width-1);
    prms_out.angle = 0.0;
    prms_out.offset = min;
}
int Dist2D::fit_gaussian(gaussian_parameters prms_in) {
    const int delta_max_min = 10;
    const int num_iters = 50;
    const float tol_chi2 = 1e-6F;
    const float lambda0 = 0.001F;
    const float lambda_change_factor = 10;

    int i, j, k, r;
    float chi2, chi2_new;
    gaussian_parameters prms_temp, prms_temp_new;

    float alpha[num_prms*num_prms], beta[num_prms];
    float alpha_new[num_prms*num_prms], beta_new[num_prms];
    float deltaP[num_prms];

    float lambda = lambda0;

    prms_temp = prms_in;
    chi2 = calc_chi2(prms_temp, beta, alpha);

    for (k=0; k<num_iters; ++k) {
        // Add lambda to alpha's diagonal
        for (i=0; i<num_prms; ++i)
            for (j=0; j<num_prms; ++j)
                if (i == j)
                    alpha_new[i*num_prms+j] = alpha[i*num_prms+j] * (1 + lambda);
                else
                    alpha_new[i*num_prms+j] = alpha[i*num_prms+j];

        r = solve_lin_sys(num_prms, alpha_new, beta, deltaP);
        if(r)
            return ERROR_PARAMETER_VARIATION;

        prms_temp_new.amplitude = prms_temp.amplitude + deltaP[0];
        prms_temp_new.cent_ver = prms_temp.cent_ver + deltaP[1];
        prms_temp_new.cent_hor = prms_temp.cent_hor + deltaP[2];
        prms_temp_new.sigma_ver = prms_temp.sigma_ver + deltaP[3];
        prms_temp_new.sigma_hor = prms_temp.sigma_hor + deltaP[4];
        prms_temp_new.angle = prms_temp.angle + deltaP[5];
        prms_temp_new.offset = prms_temp.offset + deltaP[6];

        chi2_new = calc_chi2(prms_temp_new, beta_new, alpha_new);

        if (std::abs(chi2_new-chi2) < tol_chi2)
            break;

        if (chi2_new < chi2) {
            lambda /= lambda_change_factor;
            prms_temp = prms_temp_new;
            chi2 = chi2_new;
            for (i=0; i<num_prms; ++i) {
                beta[i] = beta_new[i];
                for (j=0; j<num_prms; ++j)
                    alpha[i*num_prms+j] = alpha_new[i*num_prms+j];
            }
        }
        else {
            lambda *= lambda_change_factor;
        }
    }
    if(k == num_iters)
        return ERROR_CHI2_TOL_NOT_ACHIEVED;

    r = test_parameters(prms_temp);
    if(r)
        return ERROR_GAUSSIAN_FIT;

    correct_negative_sizes(prms_temp);
    swap_inverted_sizes(prms_temp);
    reduce_angle(prms_temp);

    prms = prms_temp;

    return SUCCESS;
}
void Dist2D::correct_negative_sizes(gaussian_parameters& prms)
{
    if (prms.sigma_hor < 0)
        prms.sigma_hor = -prms.sigma_hor;
    if (prms.sigma_ver < 0)
        prms.sigma_ver = -prms.sigma_ver;
}
void Dist2D::swap_inverted_sizes(gaussian_parameters& prms)
{
    if (prms.sigma_hor < prms.sigma_ver) {
        float sigma_temp = prms.sigma_hor;
        prms.sigma_hor = prms.sigma_ver;
        prms.sigma_ver = sigma_temp;
        prms.angle += (float)M_PI/2;
    }
}
void Dist2D::reduce_angle(gaussian_parameters& prms)
{
    prms.angle = fmod(prms.angle, (float)M_PI);

    if(prms.angle > M_PI/2)
        prms.angle -= (float)M_PI;
    else if(prms.angle < -M_PI/2)
        prms.angle += (float)M_PI;
}
int Dist2D::test_number(float x) {
    if (x != x)
        return ERROR_UNDETERMINED_VALUE; // NaN
    else if (abs(x) == std::numeric_limits<float>::infinity())
        return ERROR_INFINITE_VALUE;

    return SUCCESS;
}
int Dist2D::test_parameters(gaussian_parameters prms_in) {
    int r[num_prms];

    r[0] = test_number(prms_in.amplitude);
    r[1] = test_number(prms_in.cent_ver);
    r[2] = test_number(prms_in.cent_hor);
    r[3] = test_number(prms_in.sigma_ver);
    r[4] = test_number(prms_in.sigma_hor);
    r[5] = test_number(prms_in.angle);
    r[6] = test_number(prms_in.offset);

    for (int i=0; i<num_prms; ++i)
        if (r[i])
            return r[i];

    return SUCCESS;
}
float Dist2D::calc_chi2(gaussian_parameters prms_in, float *beta, float *alpha) {
    int i, j, k, m;
    float x, y, x2, y2, gaussian, gaussian1, dist_temp, delta, chi2, norm;
    float d_gaussian[num_prms];

    float a = prms_in.amplitude;
    float mu_ver = prms_in.cent_ver;
    float mu_hor = prms_in.cent_hor;
    float s_ver = prms_in.sigma_ver;
    float s_hor = prms_in.sigma_hor;
    float ang = prms_in.angle;
    float o = prms_in.offset;

    float cos_ang = cos(ang);
    float sin_ang = sin(ang);
    float s_ver2 = s_ver * s_ver;
    float s_hor2 = s_hor * s_hor;

    chi2 = 0.0F;
    norm = 0.0F;
    for (i=0; i<num_prms; ++i) {
        beta[i] = 0.0F;
        for (j=0; j<num_prms; ++j)
            alpha[i*num_prms+j] = 0.0F;
    }

    d_gaussian[6] = 1.0; // dG/do

    for (i=0; i<height; ++i)
        for (j=0; j<width; ++j) {
            // Calculate coordinates in rotated system
            x = (i - mu_ver)*cos_ang + (j - mu_hor)*sin_ang;
            y = -(i - mu_ver)*sin_ang + (j - mu_hor)*cos_ang;
            x2 = x*x;
            y2 = y*y;

            // Calculate distribution value
            gaussian = a*((float)exp(-0.5*(x2/s_ver2 + y2/s_hor2))) + o;
            gaussian1 = gaussian - o;

            // Calculate derivatives
            d_gaussian[0] = gaussian1/a; // dG/da
            d_gaussian[1] = gaussian1*(x*cos_ang/s_ver2 - y*sin_ang/s_hor2); // dG/dmu_ver
            d_gaussian[2] = gaussian1*(x*sin_ang/s_ver2 + y*cos_ang/s_hor2); // dG/dmu_hor
            d_gaussian[3] = gaussian1*x2/s_ver2/s_ver; // dG/ds_ver
            d_gaussian[4] = gaussian1*y2/s_hor2/s_hor; // dG/ds_hor
            d_gaussian[5] = gaussian1*x*y*(1/s_hor2 - 1/s_ver2); // dG/dang

            // Calculate chi2
            dist_temp = dist[i*width+j];
            delta = dist_temp - gaussian;
            chi2 += delta*delta;
            norm += dist_temp*dist_temp;

            // Calculate beta and alpha
            for (k=0; k<num_prms; ++k) {
                beta[k] += delta*d_gaussian[k];
                for (m=0; m<num_prms; ++m)
                    alpha[k*num_prms+m] += d_gaussian[k]*d_gaussian[m];
            }
        }

    return chi2/norm;
 }
