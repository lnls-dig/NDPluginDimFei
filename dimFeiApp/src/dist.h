
#pragma once

#include "stdafx.h"

struct gaussian_parameters {
	float amplitude;
	float cent_ver;
	float cent_hor;
	float sigma_ver;
	float sigma_hor;
	float angle;
	float offset;
};

const int num_prms = 7;

class Dist2D {
public:
	Dist2D(intensity *d, const int w, const int h);
	~Dist2D();

	int test_image();
	void calc_default_parameters(gaussian_parameters &prms_out);
	int fit_gaussian(gaussian_parameters prms_in);

	inline float get_amplitude() { return prms.amplitude; }
	inline float get_cent_ver() { return prms.cent_ver; }
	inline float get_cent_hor() { return prms.cent_hor; }
	inline float get_sigma_ver() { return prms.sigma_ver; }
	inline float get_sigma_hor() { return prms.sigma_hor; }
	inline float get_angle() { return prms.angle; }
	inline float get_offset() { return prms.offset; }
private:
	intensity *dist;
	int width, height;
	gaussian_parameters prms;

	int test_number(float x);
	int test_parameters(gaussian_parameters prms_in);
	float calc_chi2(gaussian_parameters prms_in, float *beta, float *alpha);
    void correct_negative_sizes(gaussian_parameters& prms);
    void swap_inverted_sizes(gaussian_parameters& prms);
    void reduce_angle(gaussian_parameters& prms);
};
