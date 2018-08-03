#include "stdafx.h"
#include "dimfei.h"
#include "dist.h"


bool are_prms_defined;
bool use_last_fit;
gaussian_parameters prms, prms_default;

extern "C" DIMFEI_LM_2D_API int calc_parameters(unsigned char *intensity, int width, int height)
{
    /*
     * Fit Gaussian to image.
     *
     * INPUT
     *     intensity -- pointer to array of pixel values (row-major order)
     *     width -- image width, in pixels
     *     height -- image height, in pixels
     * RETURN
     *     0 -- success
     *     1 -- error in Gaussian fit
     *     2 -- error in varying parameters
     *     3 -- chi2 tolerance not achived
     *     4 -- weak image
     *     5 -- saturated image
     */

    int r;
    int w = width;
    int h = height;

    Dist2D image(intensity, w, h);

    r = image.test_image();
    if (r)
        return r;

    if (!are_prms_defined) {
        image.calc_default_parameters(prms_default);
        are_prms_defined = true;
    }

    if (!use_last_fit)
        prms = prms_default;

    r = image.fit_gaussian(prms);
    if (r)
        use_last_fit = false;
    else {
        prms.amplitude = image.get_amplitude();
        prms.cent_hor = image.get_cent_hor();
        prms.cent_ver = image.get_cent_ver();
        prms.sigma_hor = image.get_sigma_hor();
        prms.sigma_ver = image.get_sigma_ver();
        prms.angle = image.get_angle();
        prms.offset = image.get_offset();

        use_last_fit = true;
    }

    return r;
}
extern "C" DIMFEI_LM_2D_API int read_parameters(float *amp, float *cent_hor, float *cent_ver, float *s_hor, float *s_ver, float *ang, float *offset)
{
    /*
     * Read current parameters. After fitting, these will be the fitted parameters.
     *
     * OUTPUT
     *     amp -- amplitude
     *     cent_hor -- horizontal centroid position, in pixels
     *     cent_ver -- vertical centroid position, in pixels
     *     s_hor -- horizontal sigma, in pixels
     *     s_ver -- vertical sigma, in pixels
     *     ang -- calculated angle, in radians (relative to the horizontal axis, positive for counterclockwise rotation)
     *     offset -- offset
     * RETURN
     *     0 -- success
     *     6 -- parameters not defined
     */

    if(!are_prms_defined) {
        *amp = *cent_hor = *cent_ver = *s_hor = *s_ver = *ang = *offset = 0.0;
        return ERROR_UNDETERMINED_VALUE;
    }

    *amp = prms.amplitude;
    *cent_hor = prms.cent_hor;
    *cent_ver = prms.cent_ver;
    *s_hor = prms.sigma_hor;
    *s_ver = prms.sigma_ver;
    *ang = prms.angle;
    *offset = prms.offset;

    return SUCCESS;
}
extern "C" DIMFEI_LM_2D_API int set_parameters(float amp, float cent_hor, float cent_ver, float s_hor, float s_ver, float ang, float offset)
{
    /*
     * Set initial parameters for next fit.
     *
     * INPUT
     *     amp -- amplitude
     *     cent_hor -- horizontal centroid position, in pixels
     *     cent_ver -- vertical centroid position, in pixels
     *     s_hor -- horizontal sigma, in pixels
     *     s_ver -- vertical sigma, in pixels
     *     ang -- calculated angle, in radians (relative to the horizontal axis, positive for counterclockwise rotation)
     *     offset -- offset
     * RETURN
     *     0 -- success
     */

    prms_default.amplitude = amp;
    prms_default.cent_hor = cent_hor;
    prms_default.cent_ver = cent_ver;
    prms_default.sigma_hor = s_hor;
    prms_default.sigma_ver = s_ver;
    prms_default.angle = ang;
    prms_default.offset = offset;

    are_prms_defined = true;
    use_last_fit = false;

    return SUCCESS;
}
extern "C" DIMFEI_LM_2D_API int reset_parameters()
{
    /*
     * Reset current parameters.
     *
     * RETURN
     *     0 -- success
     */

    are_prms_defined = false;
    use_last_fit = false;

    return SUCCESS;
}

// Wrappers for backward compatibility
extern "C" DIMFEI_LM_2D_API int calcula_parametros(unsigned char *intensity, int width, int height)
{
    return calc_parameters(intensity, width, height);
}
extern "C" DIMFEI_LM_2D_API int le_parametros(float *amp, float *cent_hor, float *cent_ver, float *s_hor, float *s_ver, float *ang, float *offset)
{
    return read_parameters(amp, cent_hor, cent_ver, s_hor, s_ver, ang, offset);
}
extern "C" DIMFEI_LM_2D_API int ajusta_parametros(float amp, float cent_hor, float cent_ver, float s_hor, float s_ver, float ang, float offset)
{
    return set_parameters(amp, cent_hor, cent_ver, s_hor, s_ver, ang, offset);
}
extern "C" DIMFEI_LM_2D_API int reset_parametros()
{
    return reset_parameters();
}
