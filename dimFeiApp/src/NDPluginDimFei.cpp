/*
 * NDPluginDimFei.cpp
 *
 * Image statistics plugin
 * Author: Janito V. F. Filho
 *
 * Created April 18, 2017
 */

#include <limits>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <epicsTypes.h>
#include <epicsMessageQueue.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsTime.h>
#include <iocsh.h>

#include <asynDriver.h>

#include <epicsExport.h>
#include "NDPluginDriver.h"
#include "NDPluginDimFei.h"

#include "dimfei.h"

#define MAX(A,B) (A)>(B)?(A):(B)
#define MIN(A,B) (A)<(B)?(A):(B)

/* Some systems do not define M_PI in math.h */
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

static const char *driverName="NDPluginDimFei";

template <typename epicsType>
void NDPluginDimFei::convertImage(NDArray *pArray, unsigned int width,
        unsigned int height, unsigned char* image)
{
    double min = 0.0;
    double max = std::numeric_limits<epicsType>::max();
    epicsType *arrayPixel = (epicsType *)pArray->pData;

    if (std::numeric_limits<epicsType>::is_signed)
        min = std::numeric_limits<epicsType>::min();

    double normalizationFactor = 255.0 / (max - min);
    double offset = min;
    unsigned char* imagePixel = &image[0];
    arrayPixel = (epicsType *)pArray->pData;

    for (unsigned int y = 0; y < height; y++) {
        for (unsigned int x = 0; x < width; x++) {
            double rawValue = (double)*arrayPixel;
            double value = (rawValue - offset) * normalizationFactor;

            *imagePixel = (unsigned char)value;

            ++arrayPixel;
            ++imagePixel;
        }
    }
}

asynStatus NDPluginDimFei::doComputeCentroid(NDArray *pArray)
{
    if (pArray->ndims != 2)
        return(asynError);

    unsigned int width = pArray->dims[0].size;
    unsigned int height = pArray->dims[1].size;
    unsigned char* image = new unsigned char[width * height];

    switch(pArray->dataType) {
        case NDInt8:
            convertImage<epicsInt8>(pArray, width, height, image);
            break;
        case NDUInt8:
            convertImage<epicsUInt8>(pArray, width, height, image);
            break;
        case NDInt16:
            convertImage<epicsInt16>(pArray, width, height, image);
            break;
        case NDUInt16:
            convertImage<epicsUInt16>(pArray, width, height, image);
            break;
        case NDInt32:
            convertImage<epicsInt32>(pArray, width, height, image);
            break;
        case NDUInt32:
            convertImage<epicsUInt32>(pArray, width, height, image);
            break;
        case NDFloat32:
            convertImage<epicsFloat32>(pArray, width, height, image);
            break;
        case NDFloat64:
            convertImage<epicsFloat64>(pArray, width, height, image);
            break;
        default:
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "NDDimFeiPlugin: failed to convert image\n");

            delete [] image;
            return asynError;
    }

    if (reset_parameters()) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "NDDimFeiPlugin: failed to reset parameters\n");

        delete [] image;
        return asynError;
    }

    int status = calc_parameters(image, width, height);
    delete [] image;
    if (status != 0 && status != 4 && status != 5) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "NDDimFeiPlugin: failed to calculate parameters: %d\n", status);

        return asynError;
    }

    float amp = 0.f;
    float centroidX = 0.f;
    float centroidY = 0.f;
    float sigmaX = 0.f;
    float sigmaY = 0.f;
    float orientation = 0.f;
    float offset = 0.f;

    if (status == 0) {
        status = read_parameters(&amp, &centroidX, &centroidY, &sigmaX, &sigmaY,
                &orientation, &offset);
        if (status) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "NDDimFeiPlugin: failed to read parameters\n");

            return asynError;
        }
    }

    this->centroidX = centroidX;
    this->centroidY = centroidY;
    this->sigmaX = sigmaX;
    this->sigmaY = sigmaY;
    this->orientation = orientation;

    return asynSuccess;
}


/** Callback function that is called by the NDArray driver with new NDArray data.
  * Does image statistics.
  * \param[in] pArray  The NDArray from the callback.
  */
void NDPluginDimFei::processCallbacks(NDArray *pArray)
{
    /* This function does array statistics.
     * It is called with the mutex already locked.  It unlocks it during long calculations when private
     * structures don't need to be protected.
     */
    int computeStatistics;
    size_t sizeX=0, sizeY=0;
    NDArrayInfo arrayInfo;
    static const char* functionName = "processCallbacks";

    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "DIMFEI processCallbacks\n");

    /* Call the base class method */
    NDPluginDriver::beginProcessCallbacks(pArray);

    pArray->getInfo(&arrayInfo);
    getIntegerParam(NDPluginDimFeiComputeStatistics,  &computeStatistics);

    if (pArray->ndims > 0) sizeX = pArray->dims[0].size;
    if (pArray->ndims == 1) sizeY = 1;
    if (pArray->ndims > 1)  sizeY = pArray->dims[1].size;

    // Release the lock.  While it is released we cannot access the parameter library or class member data.
    this->unlock();

    if (computeStatistics) {
        doComputeCentroid(pArray);
    }

    // Take the lock again.  The time-series data need to be protected.
    this->lock();

    if (computeStatistics) {
        setDoubleParam(NDPluginDimFeiCentroidX,     this->centroidX);
        setDoubleParam(NDPluginDimFeiCentroidY,     this->centroidY);
        setDoubleParam(NDPluginDimFeiSigmaX,        this->sigmaX);
        setDoubleParam(NDPluginDimFeiSigmaY,        this->sigmaY);
        setDoubleParam(NDPluginDimFeiOrientation,   this->orientation);
    }


    int arrayCallbacks = 0;
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    if (arrayCallbacks == 1) {
        NDArray *pArrayOut = this->pNDArrayPool->copy(pArray, NULL, 1);
        if (NULL != pArrayOut) {
            this->getAttributes(pArrayOut->pAttributeList);
            this->unlock();
            doCallbacksGenericPointer(pArrayOut, NDArrayData, 0);
            this->lock();
            /* Save a copy of this array for calculations when cursor is moved or threshold is changed */
            if (this->pArrays[0]) this->pArrays[0]->release();
            this->pArrays[0] = pArrayOut;
        }
        else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s: Couldn't allocate output array. Further processing terminated.\n",
                driverName, functionName);
        }
    }

    NDPluginDriver::endProcessCallbacks(pArray, true, true);

    callParamCallbacks();
}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus NDPluginDimFei::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    static const char *functionName = "writeInt32";


    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(function, value);

    if (function < FIRST_NDPLUGIN_DIMFEI_PARAM)
        status = NDPluginDriver::writeInt32(pasynUser, value);

    /* Do callbacks so higher layers see any changes */
    status = (asynStatus) callParamCallbacks();

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d, value=%d",
                  driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: function=%d, value=%d\n",
              driverName, functionName, function, value);
    return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus  NDPluginDimFei::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    static const char *functionName = "writeFloat64";

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(function, value);

    /* If this parameter belongs to a base class call its method */
    if (function < FIRST_NDPLUGIN_DIMFEI_PARAM)
        status = NDPluginDriver::writeFloat64(pasynUser, value);

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%f\n",
              driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: function=%d, value=%f\n",
              driverName, functionName, function, value);
    return status;
}



/** Constructor for NDPluginDimFei; most parameters are simply passed to NDPluginDriver::NDPluginDriver.
  * After calling the base class constructor this method sets reasonable default values for all of the
  * parameters.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] queueSize The number of NDArrays that the input queue for this plugin can hold when
  *            NDPluginDriverBlockingCallbacks=0.  Larger queues can decrease the number of dropped arrays,
  *            at the expense of more NDArray buffers being allocated from the underlying driver's NDArrayPool.
  * \param[in] blockingCallbacks Initial setting for the NDPluginDriverBlockingCallbacks flag.
  *            0=callbacks are queued and executed by the callback thread; 1 callbacks execute in the thread
  *            of the driver doing the callbacks.
  * \param[in] NDArrayPort Name of asyn port driver for initial source of NDArray callbacks.
  * \param[in] NDArrayAddr asyn port driver address for initial source of NDArray callbacks.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] maxThreads The maximum number of threads this driver is allowed to use. If 0 then 1 will be used.
  */
NDPluginDimFei::NDPluginDimFei(const char *portName, int queueSize, int blockingCallbacks,
                         const char *NDArrayPort, int NDArrayAddr,
                         int maxBuffers, size_t maxMemory,
                         int priority, int stackSize, int maxThreads)
    /* Invoke the base class constructor */
    : NDPluginDriver(portName, queueSize, blockingCallbacks,
                   NDArrayPort, NDArrayAddr, 1, maxBuffers, maxMemory,
                   asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
                   asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
                   0, 1, priority, stackSize, maxThreads)
{
    //static const char *functionName = "NDPluginDimFei";
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "DIMFEI ctor\n");

    /* Statistics */
    createParam(NDPluginDimFeiComputeStatisticsString, asynParamInt32,      &NDPluginDimFeiComputeStatistics);
    createParam(NDPluginDimFeiCentroidXString,         asynParamFloat64,    &NDPluginDimFeiCentroidX);
    createParam(NDPluginDimFeiCentroidYString,         asynParamFloat64,    &NDPluginDimFeiCentroidY);
    createParam(NDPluginDimFeiSigmaXString,            asynParamFloat64,    &NDPluginDimFeiSigmaX);
    createParam(NDPluginDimFeiSigmaYString,            asynParamFloat64,    &NDPluginDimFeiSigmaY);
    createParam(NDPluginDimFeiOrientationString,       asynParamFloat64,    &NDPluginDimFeiOrientation);

    /* Set the plugin type string */
    setStringParam(NDPluginDriverPluginType, "NDPluginDimFei");

    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "DIMFEI ctored?\n");
    /* Try to connect to the array port */
    connectToArrayPort();
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "DIMFEI ctored\n");
}

/** Configuration command */
extern "C" int NDDimFeiConfigure(const char *portName, int queueSize, int blockingCallbacks,
                                 const char *NDArrayPort, int NDArrayAddr,
                                 int maxBuffers, size_t maxMemory,
                                 int priority, int stackSize, int maxThreads)
{
    NDPluginDimFei *pPlugin = new NDPluginDimFei(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr,
                                              maxBuffers, maxMemory, priority, stackSize, maxThreads);
    return pPlugin->start();
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0 = { "portName",iocshArgString};
static const iocshArg initArg1 = { "frame queue size",iocshArgInt};
static const iocshArg initArg2 = { "blocking callbacks",iocshArgInt};
static const iocshArg initArg3 = { "NDArrayPort",iocshArgString};
static const iocshArg initArg4 = { "NDArrayAddr",iocshArgInt};
static const iocshArg initArg5 = { "maxBuffers",iocshArgInt};
static const iocshArg initArg6 = { "maxMemory",iocshArgInt};
static const iocshArg initArg7 = { "priority",iocshArgInt};
static const iocshArg initArg8 = { "stackSize",iocshArgInt};
static const iocshArg initArg9 = { "maxThreads",iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0,
                                            &initArg1,
                                            &initArg2,
                                            &initArg3,
                                            &initArg4,
                                            &initArg5,
                                            &initArg6,
                                            &initArg7,
                                            &initArg8,
                                            &initArg9};
static const iocshFuncDef initFuncDef = {"NDDimFeiConfigure",10,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    NDDimFeiConfigure(args[0].sval, args[1].ival, args[2].ival,
                      args[3].sval, args[4].ival, args[5].ival,
                      args[6].ival, args[7].ival, args[8].ival,
                      args[9].ival);
}

extern "C" void NDDimFeiRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

extern "C" {
epicsExportRegistrar(NDDimFeiRegister);
}
