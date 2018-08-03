#ifndef NDPluginDimFei_H
#define NDPluginDimFei_H

#include <epicsTypes.h>

#include "NDPluginDriver.h"

/* Statistics */
#define NDPluginDimFeiComputeStatisticsString  "COMPUTE_STATISTICS"  /* (asynInt32,        r/w) Compute statistics? */
#define NDPluginDimFeiCentroidXString          "CENTROIDX_VALUE"     /* (asynFloat64,      r/o) X centroid */
#define NDPluginDimFeiCentroidYString          "CENTROIDY_VALUE"     /* (asynFloat64,      r/o) Y centroid */
#define NDPluginDimFeiSigmaXString             "SIGMAX_VALUE"        /* (asynFloat64,      r/o) Sigma X */
#define NDPluginDimFeiSigmaYString             "SIGMAY_VALUE"        /* (asynFloat64,      r/o) Sigma Y */
#define NDPluginDimFeiOrientationString        "ORIENTATION_VALUE"   /* (asynFloat64,      r/o) Orientation */


/* Arrays of total and net counts for MCA or waveform record */
#define NDPluginDimFeiCallbackPeriodString     "CALLBACK_PERIOD"     /* (asynFloat64,      r/w) Callback period */

/** Does image statistics.  These include
  * Min, max, mean, sigma
  * X and Y centroid and sigma
  * Histogram
  */
class epicsShareClass NDPluginDimFei : public NDPluginDriver {
public:
    NDPluginDimFei(const char *portName, int queueSize, int blockingCallbacks,
                 const char *NDArrayPort, int NDArrayAddr,
                 int maxBuffers, size_t maxMemory,
                 int priority, int stackSize, int maxThreads=1);
    /* These methods override the virtual methods in the base class */
    void processCallbacks(NDArray *pArray);
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    template <typename epicsType> void convertImage(NDArray *pArray,
            unsigned int width, unsigned int height, unsigned char* image);
    asynStatus doComputeCentroid(NDArray *pArray);

protected:
    int NDPluginDimFeiComputeStatistics;
    #define FIRST_NDPLUGIN_DIMFEI_PARAM NDPluginDimFeiComputeStatistics
    /* Statistics */
    int NDPluginDimFeiCentroidX;
    int NDPluginDimFeiCentroidY;
    int NDPluginDimFeiSigmaX;
    int NDPluginDimFeiSigmaY;
    int NDPluginDimFeiOrientation;

    #define LAST_NDPLUGIN_DIMFEI_PARAM NDPluginDimFeiOrientation

private:
    double  centroidX;
    double  centroidY;
    double  sigmaX;
    double  sigmaY;
    double  orientation;
};
#define NUM_NDPLUGIN_DIMFEI_PARAMS ((int)(&LAST_NDPLUGIN_DIMFEI_PARAM - &FIRST_NDPLUGIN_DIMFEI_PARAM + 1))

#endif
