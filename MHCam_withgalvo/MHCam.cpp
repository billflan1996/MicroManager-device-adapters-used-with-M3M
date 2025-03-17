///////////////////////////////////////////////////////////////////////////////
// FILE:          MH_Cam.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   MODIFIED the example implementation of the demo camera.
//                Simulates generic digital camera and associated automated
//                microscope devices and enables testing of the rest of the
//                system without the need to connect to the actual hardware. 
//                
// AUTHOR:        Sunil Kumar, building a tiny bit of stuff on top of a lot of work by [Nenad Amodaj, nenad@amodaj.com, 06/08/2005] 04/10/2022
//
// COPYRIGHT:     Imperial College London 2022
//                University of California, San Francisco, 2006
// 
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.

#define _WINSOCK_DEPRECATED_NO_WARNINGS //DEFINITELY VERY NORMAL
#include <winsock2.h> //Good to do this before other things like windows.h - seems to generate conflicts otherwise
#include <windows.h>

#include "MHCam.h"
#include <cstdio>
#include <string>
#include <math.h>
#include "ModuleInterface.h"
#include <sstream>
#include <algorithm>
#include "WriteCompactTiffRGB.h"
#include <iostream>
#include <direct.h>

#pragma comment(lib, "ws2_32.lib") // Link with Ws2_32.lib for sockets

using namespace std;
const double MH_camera::nominalPixelSizeUm_ = 1.0;
double g_IntensityFactor_ = 1.0;

// External names used used by the rest of the system
// to load particular device from the "DemoCamera.dll" library
const char* g_CameraDeviceName = "MH Camera";
const char* g_SocketGalvoDeviceName = "Socket Galvo";
const char* g_HubDeviceName = "MH as Cam Hub";
const char* g_PROP_EXAMPLE_NAME = "Hub EXAMPLE STRING PROPERTY FROM CAM";

// constants for naming pixel types (allowed values of the "PixelType" property)
const char* g_PixelType_8bit = "8bit";
const char* g_PixelType_16bit = "16bit";

// constants for naming camera modes
const char* g_MH_Test = "MH Test Pattern";
const char* g_MH_Histo = "MH Histogram";
const char* g_MH_Image = "MH Image";

//enum { MODE_ARTIFICIAL_WAVES, MODE_NOISE, MODE_COLOR_TEST, MODE_MH_TEST };
enum { MODE_MH_TEST, MODE_MH_HISTO, MODE_MH_IMAGE};

///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////

MODULE_API void InitializeModuleData()
{
    RegisterDevice(g_CameraDeviceName, MM::CameraDevice, "MH Camera");
    RegisterDevice(g_SocketGalvoDeviceName, MM::GenericDevice, "Socket Galvo");
    RegisterDevice(g_HubDeviceName, MM::HubDevice, "MH as Cam Hub");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (deviceName == 0)
        return 0;

    // decide which device class to create based on the deviceName parameter
    if (strcmp(deviceName, g_CameraDeviceName) == 0)
    {
        // create camera
        return new MH_camera();
    }
    else if (strcmp(deviceName, g_SocketGalvoDeviceName) == 0)
    {
        return new SocketGalvo();
    }
    else if (strcmp(deviceName, g_HubDeviceName) == 0)
    {
        return new Scan_hub();
    }

    // ...supplied name not recognized
    return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
    delete pDevice;
}

///////////////////////////////////////////////////////////////////////////////
// MH_camera implementation
// ~~~~~~~~~~~~~~~~~~~~~~~~~~

/**
* MH_camera constructor.
* Setup default all variables and create device properties required to exist
* before intialization. In this case, no such properties were required. All
* properties will be created in the Initialize() method.
*
* As a general guideline Micro-Manager devices do not access hardware in the
* the constructor. We should do as little as possible in the constructor and
* perform most of the initialization in the Initialize() method.
*/
MH_camera::MH_camera() :
    CCameraBase<MH_camera>(),
    exposureMaximum_(MAX_INTEG_MS),
    dPhase_(0),
    initialized_(false),
    readoutUs_(0.0),
    scanMode_(1),
    bitDepth_(16),
    roiX_(0),
    roiY_(0),
    sequenceStartTime_(0),
    isSequenceable_(false),
    sequenceMaxLength_(100),
    sequenceRunning_(false),
    sequenceIndex_(0),
    binSize_(1),
    n_scanPixels_X_(120),
    n_scanPixels_Y_(80),
    n_beams_X_(2),
    n_beams_Y_(3),
    cameraCCDXSize_(240),
    cameraCCDYSize_(240),
    ccdT_(0.0),
    triggerDevice_(""),
    stopOnOverflow_(false),
    dropPixels_(false),
    fastImage_(false),
    saturatePixels_(false),
    fractionOfPixelsToDropOrSaturate_(0.002),
    shouldRotateImages_(false),
    shouldDisplayImageNumber_(false),
    stripeWidth_(1.0),
    supportsMultiROI_(false),
    multiROIFillValue_(0),
    nComponents_(1),
    mode_(MODE_MH_TEST),
    imgManpl_(0),
    pcf_(1.0),
    photonFlux_(50.0),
    readNoise_(2.5),
    Sim_lifetime_(2000),
    Lifetime_range_(12500),
    rates_or_decays_(false),
    special_mask_(0x80000000),//0b10000000000000000000000000000000
    channel_mask_(0x7E000000),//0b01111110000000000000000000000000
    time_mask_(0x1FFFC00),//0b00000001111111111111110000000000
    nsync_mask_(0x3FF),//0b00000000000000000000001111111111
    pixel_dwelltime_ps_(100000000),
    MeasDesc_GlobalResolution_(12618),//From an old .ptu file
    last_line_start_(0),
    last_line_end_(100000000000000000),
    current_line_(-99),
    n_line_repeats_(1),
    n_frame_repeats_(1),
    n_frame_tracker_(0),
    nBeams(1),
    overflow_counter_(0),
    saving_(false),
    frame_active_(false)
{
    memset(testProperty_, 0, sizeof(testProperty_));

    // call the base class method to set-up default error codes/messages
    InitializeDefaultErrorMessages();
    readoutStartTime_ = GetCurrentMMTime();
    thd_ = new MySequenceThread(this);

    // parent ID display
    CreateHubIDProperty();

    CreateFloatProperty("MaximumExposureMs", exposureMaximum_, false,
        new CPropertyAction(this, &MH_camera::OnMaxExposure),
        true);
}

/**
* MH_camera destructor.
* If this device used as intended within the Micro-Manager system,
* Shutdown() will be always called before the destructor. But in any case
* we need to make sure that all resources are properly released even if
* Shutdown() was not called.
*/
MH_camera::~MH_camera()
{
    StopSequenceAcquisition();
    delete thd_;
}

/**
* Obtains device name.
* Required by the MM::Device API.
*/
void MH_camera::GetName(char* name) const
{
    // Return the name used to referr to this device adapte
    CDeviceUtils::CopyLimitedString(name, g_CameraDeviceName);
}

/**
* Intializes the hardware.
* Required by the MM::Device API.
* Typically we access and initialize hardware at this point.
* Device properties are typically created here as well, except
* the ones we need to use for defining initialization parameters.
* Such pre-initialization properties are created in the constructor.
* (This device does not have any pre-initialization properties)
*/
int MH_camera::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    Scan_hub* pHub = static_cast<Scan_hub*>(GetParentHub());
    if (pHub)
    {
        char hubLabel[MM::MaxStrLength];
        pHub->GetLabel(hubLabel);
        pHub->CreateStringProperty(g_PROP_EXAMPLE_NAME, "EXAMPLE VALUE  FROM CAM", true);
        SetParentID(hubLabel); // for backward comp.
    }
    else
        LogMessage(NoHubError);

    for (int i = 0; i < MAX_N_CHANNELS; i++) {
        live_rates[i] = 0;
    }
        

    // set property list
    // -----------------

    // Name
    int nRet = CreateStringProperty(MM::g_Keyword_Name, g_CameraDeviceName, true);
    if (DEVICE_OK != nRet)
        return nRet;

    // Description
    nRet = CreateStringProperty(MM::g_Keyword_Description, "MultiHarp as Camera Device Adapter", true);
    if (DEVICE_OK != nRet)
        return nRet;

    // CameraName
    nRet = CreateStringProperty(MM::g_Keyword_CameraName, "MultiHarp Cam - MultiMode", true);
    assert(nRet == DEVICE_OK);

    // CameraID
    nRet = CreateStringProperty(MM::g_Keyword_CameraID, "V1.0", true);
    assert(nRet == DEVICE_OK);

    // binning
    CPropertyAction* pAct = new CPropertyAction(this, &MH_camera::OnBinning);
    nRet = CreateIntegerProperty(MM::g_Keyword_Binning, 1, false, pAct);
    assert(nRet == DEVICE_OK);

    nRet = SetAllowedBinning();
    if (nRet != DEVICE_OK)
        return nRet;

    // pixel type
    pAct = new CPropertyAction(this, &MH_camera::OnPixelType);
    nRet = CreateStringProperty(MM::g_Keyword_PixelType, g_PixelType_16bit, false, pAct);
    assert(nRet == DEVICE_OK);

    vector<string> pixelTypeValues;
    pixelTypeValues.push_back(g_PixelType_8bit);
    pixelTypeValues.push_back(g_PixelType_16bit);

    nRet = SetAllowedValues(MM::g_Keyword_PixelType, pixelTypeValues);
    if (nRet != DEVICE_OK)
        return nRet;

    // Bit depth
    pAct = new CPropertyAction(this, &MH_camera::OnBitDepth);
    nRet = CreateIntegerProperty("BitDepth", 8, false, pAct);
    assert(nRet == DEVICE_OK);

    vector<string> bitDepths;
    bitDepths.push_back("8");
    bitDepths.push_back("16");
    nRet = SetAllowedValues("BitDepth", bitDepths);
    if (nRet != DEVICE_OK)
        return nRet;

    // exposure
    nRet = CreateFloatProperty(MM::g_Keyword_Exposure, 250.0, false);
    assert(nRet == DEVICE_OK);
    //NOT THE USUAL DEFAULTS!
    SetPropertyLimits(MM::g_Keyword_Exposure, 100.0, exposureMaximum_);

    CPropertyActionEx* pActX = 0;
    // create an extended (i.e. array) properties 1 through 4

    for (int ij = 1; ij < 7; ++ij)
    {
        std::ostringstream os;
        os << ij;
        std::string propName = "TestProperty" + os.str();
        pActX = new CPropertyActionEx(this, &MH_camera::OnTestProperty, ij);
        nRet = CreateFloatProperty(propName.c_str(), 0., false, pActX);
        if (0 != (ij % 5))
        {
            // try several different limit ranges
            double upperLimit = (double)ij * pow(10., (double)(((ij % 2) ? -1 : 1) * ij));
            double lowerLimit = (ij % 3) ? -upperLimit : 0.;
            SetPropertyLimits(propName.c_str(), lowerLimit, upperLimit);
        }
    }

     // scan mode
    pAct = new CPropertyAction(this, &MH_camera::OnScanMode);
    nRet = CreateIntegerProperty("ScanMode", 1, false, pAct);
    assert(nRet == DEVICE_OK);
    AddAllowedValue("ScanMode", "1");
    AddAllowedValue("ScanMode", "2");
    AddAllowedValue("ScanMode", "3");

    // camera gain
    nRet = CreateIntegerProperty(MM::g_Keyword_Gain, 0, false);
    assert(nRet == DEVICE_OK);
    SetPropertyLimits(MM::g_Keyword_Gain, -5, 8);

    // camera offset
    nRet = CreateIntegerProperty(MM::g_Keyword_Offset, 0, false);
    assert(nRet == DEVICE_OK);

    // camera temperature
    pAct = new CPropertyAction(this, &MH_camera::OnCCDTemp);
    nRet = CreateFloatProperty(MM::g_Keyword_CCDTemperature, 0, false, pAct);
    assert(nRet == DEVICE_OK);
    SetPropertyLimits(MM::g_Keyword_CCDTemperature, -100, 10);

    // camera temperature RO
    pAct = new CPropertyAction(this, &MH_camera::OnCCDTemp);
    nRet = CreateFloatProperty("CCDTemperature RO", 0, true, pAct);
    assert(nRet == DEVICE_OK);

    // readout time
    pAct = new CPropertyAction(this, &MH_camera::OnReadoutTime);
    nRet = CreateFloatProperty(MM::g_Keyword_ReadoutTime, 0, false, pAct);
    assert(nRet == DEVICE_OK);

    // Number of pixels the scanner needs to do
    pAct = new CPropertyAction(this, &MH_camera::Onn_scanPixels_X);
    CreateIntegerProperty(g_N_Scan_Px_X, 120, false, pAct);
    SetPropertyLimits(g_N_Scan_Px_X, 1, 1024);
    pAct = new CPropertyAction(this, &MH_camera::Onn_scanPixels_Y);
    CreateIntegerProperty(g_N_Scan_Px_Y, 80, false, pAct);
    SetPropertyLimits(g_N_Scan_Px_Y, 1, 1024);
    
    //Number of beams in the array
    pAct = new CPropertyAction(this, &MH_camera::Onn_beams_X);
    CreateIntegerProperty(g_N_Beams_X, 2, false, pAct);
    SetPropertyLimits(g_N_Beams_X, 1, MAX_N_CHANNELS);
    pAct = new CPropertyAction(this, &MH_camera::Onn_beams_Y);
    CreateIntegerProperty(g_N_Beams_Y, 3, false, pAct);
    SetPropertyLimits(g_N_Beams_Y, 1, MAX_N_CHANNELS);

    // Trigger device
    pAct = new CPropertyAction(this, &MH_camera::OnTriggerDevice);
    CreateStringProperty("TriggerDevice", "", false, pAct);

    pAct = new CPropertyAction(this, &MH_camera::OnDropPixels);
    CreateIntegerProperty("DropPixels", 0, false, pAct);
    AddAllowedValue("DropPixels", "0");
    AddAllowedValue("DropPixels", "1");

    pAct = new CPropertyAction(this, &MH_camera::OnSaturatePixels);
    CreateIntegerProperty("SaturatePixels", 0, false, pAct);
    AddAllowedValue("SaturatePixels", "0");
    AddAllowedValue("SaturatePixels", "1");

    pAct = new CPropertyAction(this, &MH_camera::OnFastImage);
    CreateIntegerProperty("FastImage", 0, false, pAct);
    AddAllowedValue("FastImage", "0");
    AddAllowedValue("FastImage", "1");

    pAct = new CPropertyAction(this, &MH_camera::OnFractionOfPixelsToDropOrSaturate);
    CreateFloatProperty("FractionOfPixelsToDropOrSaturate", 0.002, false, pAct);
    SetPropertyLimits("FractionOfPixelsToDropOrSaturate", 0., 0.1);

    pAct = new CPropertyAction(this, &MH_camera::OnShouldRotateImages);
    CreateIntegerProperty("RotateImages", 0, false, pAct);
    AddAllowedValue("RotateImages", "0");
    AddAllowedValue("RotateImages", "1");

    pAct = new CPropertyAction(this, &MH_camera::OnShouldDisplayImageNumber);
    CreateIntegerProperty("DisplayImageNumber", 0, false, pAct);
    AddAllowedValue("DisplayImageNumber", "0");
    AddAllowedValue("DisplayImageNumber", "1");

    pAct = new CPropertyAction(this, &MH_camera::OnStripeWidth);
    CreateFloatProperty("StripeWidth", 0, false, pAct);
    SetPropertyLimits("StripeWidth", 0, 10);

    pAct = new CPropertyAction(this, &MH_camera::OnSupportsMultiROI);
    CreateIntegerProperty("AllowMultiROI", 0, false, pAct);
    AddAllowedValue("AllowMultiROI", "0");
    AddAllowedValue("AllowMultiROI", "1");

    pAct = new CPropertyAction(this, &MH_camera::OnMultiROIFillValue);
    CreateIntegerProperty("MultiROIFillValue", 0, false, pAct);
    SetPropertyLimits("MultiROIFillValue", 0, 65536);

    // Whether or not to use exposure time sequencing
    pAct = new CPropertyAction(this, &MH_camera::OnIsSequenceable);
    std::string propName = "UseExposureSequences";
    CreateStringProperty(propName.c_str(), "No", false, pAct);
    AddAllowedValue(propName.c_str(), "Yes");
    AddAllowedValue(propName.c_str(), "No");
     
    // Camera mode: 
    pAct = new CPropertyAction(this, &MH_camera::OnMode);
    propName = "Mode";
    CreateStringProperty(propName.c_str(), g_MH_Image, false, pAct);
    AddAllowedValue(propName.c_str(), g_MH_Test);
    AddAllowedValue(propName.c_str(), g_MH_Histo);
    AddAllowedValue(propName.c_str(), g_MH_Image);

    // Photon Conversion Factor for Noise type camera
    pAct = new CPropertyAction(this, &MH_camera::OnPCF);
    propName = "Photon Conversion Factor";
    CreateFloatProperty(propName.c_str(), pcf_, false, pAct);
    SetPropertyLimits(propName.c_str(), 0.01, 10.0);

    // Read Noise (expressed in electrons) for the Noise type camera
    pAct = new CPropertyAction(this, &MH_camera::OnReadNoise);
    propName = "ReadNoise (electrons)";
    CreateFloatProperty(propName.c_str(), readNoise_, false, pAct);
    SetPropertyLimits(propName.c_str(), 0.25, 50.0);

    // Photon Flux for the Noise type camera
    pAct = new CPropertyAction(this, &MH_camera::OnPhotonFlux);
    propName = "Photon Flux";
    CreateFloatProperty(propName.c_str(), photonFlux_, false, pAct);
    SetPropertyLimits(propName.c_str(), 2.0, 5000.0);

    // Simulate application crash
    pAct = new CPropertyAction(this, &MH_camera::OnCrash);
    CreateStringProperty("SimulateCrash", "", false, pAct);
    AddAllowedValue("SimulateCrash", "");
    AddAllowedValue("SimulateCrash", "Dereference Null Pointer");
    AddAllowedValue("SimulateCrash", "Divide by Zero");

    //###################### ADDED ###################
    pAct = new CPropertyAction(this, &MH_camera::OnLifetime);
    propName = "Simulated lifetime [ps]";
    nRet = CreateIntegerProperty(propName.c_str(), 0, false, pAct);
    SetPropertyLimits(propName.c_str(), 10, 3000);

    pAct = new CPropertyAction(this, &MH_camera::OnDecOrRat);
    propName = "Decay or rates";
    CreateStringProperty("Decay or rates", "", false, pAct);
    AddAllowedValue(propName.c_str(), "Decay");
    AddAllowedValue(propName.c_str(), "Rates");

    nRet = CreateIntegerProperty(g_PropName_Offset_Ch1, 0, false, new CPropertyAction(this, &MH_camera::On_Offset_Ch1));
    if (DEVICE_OK != nRet) {
        return nRet;
    }

    // MAY WISH TO REMOVE?
    nRet = CreateStringProperty(g_PropName_MHStatus, "Idle", false, new CPropertyAction(this, &MH_camera::On_MH_Status));
    if (DEVICE_OK != nRet) {
        return nRet;
    }
    AddAllowedValue(g_PropName_MHStatus, "Idle");
    AddAllowedValue(g_PropName_MHStatus, "Start");
    AddAllowedValue(g_PropName_MHStatus, "Running");
    AddAllowedValue(g_PropName_MHStatus, "Abort");
    // MAY WISH TO REMOVE?

    nRet = CreateStringProperty(g_PropName_Saving, "False", false, new CPropertyAction(this, &MH_camera::On_Save_Enable));
    if (DEVICE_OK != nRet) {
        return nRet;
    }
    AddAllowedValue(g_PropName_Saving, "True");
    AddAllowedValue(g_PropName_Saving, "False");

    LogMessage("Did add allowed statuses", false);

/////UPDATE STATUS WAS HERE

    // setup the buffer
    // ----------------
    nRet = ResizeImageBuffer();
    if (nRet != DEVICE_OK)
        return nRet;

    LogMessage("Image buffer resized", false);

#ifdef TESTRESOURCELOCKING
    TestResourceLocking(true);
    LogMessage("TestResourceLocking OK", true);
#endif

    MH_changedTime_ = GetCurrentMMTime();
    initialized_ = true;
 ///////////////////////////////////////////////////////
    //See if we can access the DLL, make sure it's the same version we have on the development system (3.0)...
    MH_GetLibraryVersion(LIB_Version);
    msgstr = "MultiHarp library version is ";
    LogMessage(msgstr.append(LIB_Version), false);
    int ret = CreateProperty(g_Keyword_Ver, LIB_Version, MM::String, true);
    if (DEVICE_OK != ret)
        return ret;
    tmpstr = "3.0";
    if (tmpstr.compare(LIB_Version) != 0) {
        LogMessage("MultiHarp library version needs to be 3.0 for now - sorry!");
        return DEVICE_INVALID_PROPERTY_VALUE;
    }
    
    //Try to initialise the first MultiHarp we can find
    for (int i = 0; i < MAXDEVNUM; i++)
    {
        retcode = MH_OpenDevice(i, HW_Serial);
        if (retcode == 0) //Grab any device we can open
        {
            if (HW_Serial[0] != '\0') {//If the HW_Serial 'string' isn't an empty one
                LogMessage("HWSerial not empty");
                std::string sernum = convertToString(HW_Serial, true);
                msgstr = "MultiHarp #" + sernum + " opened ok!";
                LogMessage(msgstr, false);
            }
            dev[found] = i; //keep index to devices we want to use
            found++;
        }
        else
        {
            if (retcode == MH_ERROR_DEVICE_OPEN_FAIL)
            {
                msgstr = "No MultiHarp at index " + to_string(i);
                LogMessage(msgstr, false);
            }
            else
            {
                msgstr = "MultiHarp OpenDevice error message: " + MH_GetErrorString(Errorstring, retcode);
                LogMessage(msgstr, false);
            }
        }
    }
    if (found > 0) {
        msgstr = "Total no. of MultiHarps found: " + to_string(found) + " - using first one detected: " + to_string(dev[0]);
    }
    else {
        msgstr = "No MultiHarp found!";
        return DEVICE_NOT_CONNECTED;
    }
    LogMessage(msgstr, false);

    //Try initialisation
    retcode = MH_Initialize(dev[0], Mode, 0);
    if (retcode < 0)
    {
        MH_GetErrorString(Errorstring, retcode);
        msgstr = to_string(printf("MultiHarp: MH_Initialize error %d (%s). Aborted.", retcode, Errorstring));
        LogMessage(msgstr);
        goto fail;
    }
    else {
        LogMessage("MultiHarp initialise call went ok");
    }

    retcode = MH_GetHardwareInfo(dev[0], HW_Model, HW_Partno, HW_Version);
    if (retcode < 0)
    {
        MH_GetErrorString(Errorstring, retcode);
        msgstr = to_string(printf("MH_GetHardwareInfo error %d (%s). Aborted.", retcode, Errorstring));
        LogMessage(msgstr);
        goto fail;
    }
    else
    {
        LogMessage("MultiHarp get info call went ok");
        char dummy[100];
        sprintf(dummy, "Found Model %s Part no %s Version %s", HW_Model, HW_Partno, HW_Version);
        msgstr = dummy;
        //ADD PROPERTY SETTING HERE FOR READ-ONLY
        LogMessage(msgstr);
    }

    retcode = MH_GetNumOfInputChannels(dev[0], &NumChannels);
    if (retcode < 0)
    {
        char dummy[100];
        MH_GetErrorString(Errorstring, retcode);
        sprintf(dummy, "MH_GetNumOfInputChannels error %d (%s). Aborted.", retcode, Errorstring);
        msgstr = dummy;
        LogMessage(msgstr);
        goto fail;
    }
    else
    {
        //ADD PROPERTY SETTING HERE FOR READ-ONLY
        char dummy[100];
        sprintf(dummy, "Device has %i input channels.", NumChannels);
        msgstr = dummy;
        LogMessage(msgstr);
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    retcode = MH_SetSyncDiv(dev[0], SyncDivider);
    if (retcode < 0)
    {
        MH_GetErrorString(Errorstring, retcode);
        char dummy[100];
        sprintf(dummy, "MH_SetSyncDiv error %d (%s). Aborted.", retcode, Errorstring);
        msgstr = dummy;
        LogMessage(msgstr);
        goto fail;
    }
    else {
        LogMessage("MH_SetSyncDiv set Sync Divider to " + to_string(SyncDivider));
    }

    retcode = MH_SetSyncEdgeTrg(dev[0], SyncTriggerLevel, SyncTiggerEdge);
    if (retcode < 0)
    {
        MH_GetErrorString(Errorstring, retcode);
        char dummy[100];
        sprintf(dummy, "MH_SetSyncEdgeTrg error % d(% s).Aborted.", retcode, Errorstring);
        msgstr = dummy;
        LogMessage(msgstr);
        goto fail;
    }
    else {
        LogMessage("MH_SetSyncEdgeTrg set Sync Edge Trigger to " + to_string(SyncTriggerLevel) + " and " + to_string(SyncTiggerEdge));
    }

    retcode = MH_SetSyncChannelOffset(dev[0], 0);
    if (retcode < 0)
    {
        MH_GetErrorString(Errorstring, retcode);
        char dummy[100];
        sprintf(dummy, "MH_SetSyncChannelOffset error %d (%s). Aborted.", retcode, Errorstring);
        msgstr = dummy;
        LogMessage(msgstr);
        goto fail;
    }

    // WE ADDED THIS BIT! >>>  WAS 1,0,1,1
    retcode = MH_SetMarkerEdges(dev[0], 1, 0, 1, 1);
    if (retcode < 0)
    {
        MH_GetErrorString(Errorstring, retcode);
        char dummy[100];
        sprintf(dummy, "MH_SetMarkerEdges error % d(% s).Aborted.\n", retcode, Errorstring);
        msgstr = dummy;
        LogMessage(msgstr);
        goto fail;
    }
    // <<<WE ADDED THIS BIT!

    for (int i = 0; i < NumChannels; i++) //Uses the same input offset for all channels
    {
        retcode = MH_SetInputEdgeTrg(dev[0], i, InputTriggerLevel, InputTriggerEdge);
        if (retcode < 0)
        {
            MH_GetErrorString(Errorstring, retcode);
            char dummy[100];
            sprintf(dummy, "MH_SetInputEdgeTrg error %d (%s). Aborted.", retcode, Errorstring);
            msgstr = dummy;
            LogMessage(msgstr);
            goto fail;
        }

        //int hardcoded_init_offsets[6] = {0,1,2,3,4,5,6,7};

        retcode = MH_SetInputChannelOffset(dev[0], i, hardcoded_init_offsets[i]);
        if (retcode < 0)
        {
            MH_GetErrorString(Errorstring, retcode);
            char dummy[100];
            sprintf(dummy, "MH_SetInputChannelOffset error %d (%s). Aborted.", retcode, Errorstring);
            msgstr = dummy;
            LogMessage(msgstr);
            goto fail;
        }
        else {
            char dummy[100];
            sprintf(dummy, "Input channel %d offset set to %d.", i, hardcoded_init_offsets[i]);
            msgstr = dummy;
            LogMessage(msgstr);
        }

        retcode = MH_SetInputChannelEnable(dev[0], i, 1);
        if (retcode < 0)
        {
            MH_GetErrorString(Errorstring, retcode);
            char dummy[100];
            sprintf(dummy, "MH_SetInputChannelEnable error %d (%s). Aborted.", retcode, Errorstring);
            msgstr = dummy;
            LogMessage(msgstr);
            goto fail;
        }
    }

    if (Mode != MODE_T2)
    {
        retcode = MH_SetBinning(dev[0], Binning);
        if (retcode < 0)
        {
            MH_GetErrorString(Errorstring, retcode);
            char dummy[100];
            sprintf(dummy, "MH_SetBinning error %d (%s). Aborted.", retcode, Errorstring);
            msgstr = dummy;
            LogMessage(msgstr);
            goto fail;
        }

        retcode = MH_SetOffset(dev[0], Offset);
        if (retcode < 0)
        {
            MH_GetErrorString(Errorstring, retcode);
            char dummy[100];
            sprintf(dummy, "MH_SetOffset error %d (%s). Aborted.", retcode, Errorstring);
            msgstr = dummy;
            LogMessage(msgstr);
            goto fail;

        }
    }

    char gummy[100];
    sprintf(gummy, "EARLY OUTPUT");
    msgstr = gummy;
    LogMessage(msgstr);

    retcode = MH_GetResolution(dev[0], &Resolution);
    if (retcode < 0)
    {
        MH_GetErrorString(Errorstring, retcode);
        char dummy[100];
        sprintf(dummy, "MH_GetResolution error %d (%s). Aborted.", retcode, Errorstring);
        msgstr = dummy;
        LogMessage(msgstr);
        goto fail;
    }
    else {
        char dummy[100];
        sprintf(dummy, "MH_GetResolution gave %1.0lfps", retcode);
        msgstr = dummy;
        LogMessage(msgstr);

    }

    // After Init allow 150 ms for valid  count rate readings
    // Subsequently you get new values after every 100ms
    Sleep(150);

    retcode = MH_GetSyncRate(dev[0], &Syncrate);
    if (retcode < 0)
    {
        MH_GetErrorString(Errorstring, retcode);
        char dummy[100];
        sprintf(dummy, "MH_GetSyncRate error%d (%s). Aborted.", retcode, Errorstring);
        msgstr = dummy;
        LogMessage(msgstr);
        goto fail;
    }
    else {
        char dummy[100];
        sprintf(dummy, "Sync rate: %1.0lf", retcode);
        msgstr = dummy;
        LogMessage(msgstr);
    }


    for (int i = 0; i < NumChannels; i++) // for all channels
    {
        sprintf(gummy, "Checking countrate on channel %d", i);
        msgstr = gummy;
        LogMessage(msgstr);

        retcode = MH_GetCountRate(dev[0], i, &Countrate);
        if (retcode < 0)
        {
            MH_GetErrorString(Errorstring, retcode);
            char dummy[100];
            sprintf(dummy, "MH_GetCountRate error %d (%s). Aborted.", retcode, Errorstring);
            msgstr = dummy;
            LogMessage(msgstr);
            goto fail;
        }
        char dummy[100];
        sprintf(dummy, "Countrate[%1d]=%1d/s", i, Countrate);
        msgstr = dummy;
        LogMessage(msgstr);
    }

    //after getting the count rates, we can check for warnings
    retcode = MH_GetWarnings(dev[0], &warnings);
    if (retcode < 0)
    {
        MH_GetErrorString(Errorstring, retcode);
        char dummy[100];
        sprintf(dummy, "MH_GetWarnings error %d (%s). Aborted.", retcode, Errorstring);
        msgstr = dummy;
        LogMessage(msgstr);
        //Just warnings, not errors
    }
    if (warnings)
    {
        MH_GetWarningsText(dev[0], warningstext, warnings);
        char dummy[100];
        sprintf(dummy, "%s", warningstext);
        msgstr = dummy;
        LogMessage(msgstr);
        //Just warnings, not errors
    }
    //Next step in example tttrmode.c is to start upon pressing return, so switch to generating MM UI stuff instead...

    //Set default stored values for offsets and other per-channel bits (all to zero?)
    for (int i = 0; i < NumChannels; i++) {
        offsets.push_back(hardcoded_init_offsets[i]);
    }

////////////////////////////////////////////////////////

    // synchronize all properties
    // --------------------------
    nRet = UpdateStatus();
    if (nRet != DEVICE_OK)
        return nRet;

    LogMessage("UpdateStatus ok", false);

    // initialize image buffer
    GenerateEmptyImage(img_);
    return DEVICE_OK;

    fail:
    LogMessage("MultiHarp initialisation failure! Shutting down in MultiHarp device adapter...");
    Shutdown();
    return DEVICE_CAN_NOT_SET_PROPERTY;
}

/**
* Shuts down (unloads) the device.
* Required by the MM::Device API.
* Ideally this method will completely unload the device and release all resources.
* Shutdown() may be called multiple times in a row.
* After Shutdown() we should be allowed to call Initialize() again to load the device
* without causing problems.
*/
int MH_camera::Shutdown()
{
    initialized_ = false;
    return DEVICE_OK;
}

/**
* Performs exposure and grabs a single image.
* This function should block during the actual exposure and return immediately afterwards
* (i.e., before readout).  This behavior is needed for proper synchronization with the shutter.
* Required by the MM::Camera API.
*/
int MH_camera::SnapImage()
{
    static int callCounter = 0;
    ++callCounter;

    MM::MMTime startTime = GetCurrentMMTime();
    double exp = GetExposure();
    if (sequenceRunning_ && IsCapturing())
    {
        LogMessage("Was sequence", false);
        exp = GetSequenceExposure();
    }

    if (!fastImage_)
    {
        LogMessage("Was !fastImage_", false);
        start_acq();
        GenerateSyntheticImage(img_, exp);
    }

    MM::MMTime s0(0, 0);
    if (s0 < startTime)
    {
        while (exp > (GetCurrentMMTime() - startTime).getMsec())
        {
            CDeviceUtils::SleepMs(1);
        }
    }
    else
    {
        std::cerr << "You are operating this device adapter without setting the core callback, timing functions aren't yet available" << std::endl;
        // called without the core callback probably in off line test program
        // need way to build the core in the test program

    }
    readoutStartTime_ = GetCurrentMMTime();

    return DEVICE_OK;
}


/**
* Returns pixel data.
* Required by the MM::Camera API.
* The calling program will assume the size of the buffer based on the values
* obtained from GetImageBufferSize(), which in turn should be consistent with
* values returned by GetImageWidth(), GetImageHight() and GetImageBytesPerPixel().
* The calling program allso assumes that camera never changes the size of
* the pixel buffer on its own. In other words, the buffer can change only if
* appropriate properties are set (such as binning, pixel type, etc.)
*/
const unsigned char* MH_camera::GetImageBuffer()
{
    MMThreadGuard g(imgPixelsLock_);
    MM::MMTime readoutTime(readoutUs_);
    while (readoutTime > (GetCurrentMMTime() - readoutStartTime_)) {}
    unsigned char* pB = (unsigned char*)(img_.GetPixels());
    return pB;
}

/**
* Returns image buffer X-size in pixels.
* Required by the MM::Camera API.
*/
unsigned MH_camera::GetImageWidth() const
{
    return img_.Width();
}

/**
* Returns image buffer Y-size in pixels.
* Required by the MM::Camera API.
*/
unsigned MH_camera::GetImageHeight() const
{
    return img_.Height();
}

/**
* Returns image buffer pixel depth in bytes.
* Required by the MM::Camera API.
*/
unsigned MH_camera::GetImageBytesPerPixel() const
{
    return img_.Depth();
}

/**
* Returns the bit depth (dynamic range) of the pixel.
* This does not affect the buffer size, it just gives the client application
* a guideline on how to interpret pixel values.
* Required by the MM::Camera API.
*/
unsigned MH_camera::GetBitDepth() const
{
    return bitDepth_;
}

/**
* Returns the size in bytes of the image buffer.
* Required by the MM::Camera API.
*/
long MH_camera::GetImageBufferSize() const
{
    return img_.Width() * img_.Height() * GetImageBytesPerPixel();
}

/**
* Sets the camera Region Of Interest.
* Required by the MM::Camera API.
* This command will change the dimensions of the image.
* Depending on the hardware capabilities the camera may not be able to configure the
* exact dimensions requested - but should try do as close as possible.
* If the hardware does not have this capability the software should simulate the ROI by
* appropriately cropping each frame.
* This demo implementation ignores the position coordinates and just crops the buffer.
* If multiple ROIs are currently set, then this method clears them in favor of
* the new ROI.
* @param x - top-left corner coordinate
* @param y - top-left corner coordinate
* @param xSize - width
* @param ySize - height
*/
int MH_camera::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize)
{
    multiROIXs_.clear();
    multiROIYs_.clear();
    multiROIWidths_.clear();
    multiROIHeights_.clear();
    if (xSize == 0 && ySize == 0)
    {
        // effectively clear ROI
        ResizeImageBuffer();
        roiX_ = 0;
        roiY_ = 0;
    }
    else
    {
        // apply ROI
        img_.Resize(xSize, ySize);
        roiX_ = x;
        roiY_ = y;
    }
    return DEVICE_OK;
}

/**
* Returns the actual dimensions of the current ROI.
* If multiple ROIs are set, then the returned ROI should encompass all of them.
* Required by the MM::Camera API.
*/
int MH_camera::GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize)
{
    x = roiX_;
    y = roiY_;

    xSize = img_.Width();
    ySize = img_.Height();

    return DEVICE_OK;
}

/**
* Resets the Region of Interest to full frame.
* Required by the MM::Camera API.
*/
int MH_camera::ClearROI()
{
    ResizeImageBuffer();
    roiX_ = 0;
    roiY_ = 0;
    multiROIXs_.clear();
    multiROIYs_.clear();
    multiROIWidths_.clear();
    multiROIHeights_.clear();
    return DEVICE_OK;
}

/**
 * Queries if the camera supports multiple simultaneous ROIs.
 * Optional method in the MM::Camera API; by default cameras do not support
 * multiple ROIs.
 */
bool MH_camera::SupportsMultiROI()
{
    return supportsMultiROI_;
}

/**
 * Queries if multiple ROIs have been set (via the SetMultiROI method). Must
 * return true even if only one ROI was set via that method, but must return
 * false if an ROI was set via SetROI() or if ROIs have been cleared.
 * Optional method in the MM::Camera API; by default cameras do not support
 * multiple ROIs, so this method returns false.
 */
bool MH_camera::IsMultiROISet()
{
    return multiROIXs_.size() > 0;
}

/**
 * Queries for the current set number of ROIs. Must return zero if multiple
 * ROIs are not set (including if an ROI has been set via SetROI).
 * Optional method in the MM::Camera API; by default cameras do not support
 * multiple ROIs.
 */
int MH_camera::GetMultiROICount(unsigned int& count)
{
    count = (unsigned int)multiROIXs_.size();
    return DEVICE_OK;
}

/**
 * Set multiple ROIs. Replaces any existing ROI settings including ROIs set
 * via SetROI.
 * Optional method in the MM::Camera API; by default cameras do not support
 * multiple ROIs.
 * @param xs Array of X indices of upper-left corner of the ROIs.
 * @param ys Array of Y indices of upper-left corner of the ROIs.
 * @param widths Widths of the ROIs, in pixels.
 * @param heights Heights of the ROIs, in pixels.
 * @param numROIs Length of the arrays.
 */
int MH_camera::SetMultiROI(const unsigned int* xs, const unsigned int* ys,
    const unsigned* widths, const unsigned int* heights,
    unsigned numROIs)
{
    multiROIXs_.clear();
    multiROIYs_.clear();
    multiROIWidths_.clear();
    multiROIHeights_.clear();
    unsigned int minX = UINT_MAX;
    unsigned int minY = UINT_MAX;
    unsigned int maxX = 0;
    unsigned int maxY = 0;
    for (unsigned int i = 0; i < numROIs; ++i)
    {
        multiROIXs_.push_back(xs[i]);
        multiROIYs_.push_back(ys[i]);
        multiROIWidths_.push_back(widths[i]);
        multiROIHeights_.push_back(heights[i]);
        if (minX > xs[i])
        {
            minX = xs[i];
        }
        if (minY > ys[i])
        {
            minY = ys[i];
        }
        if (xs[i] + widths[i] > maxX)
        {
            maxX = xs[i] + widths[i];
        }
        if (ys[i] + heights[i] > maxY)
        {
            maxY = ys[i] + heights[i];
        }
    }
    img_.Resize(maxX - minX, maxY - minY);
    roiX_ = minX;
    roiY_ = minY;
    return DEVICE_OK;
}

/**
 * Queries for current multiple-ROI setting. May be called even if no ROIs of
 * any type have been set. Must return length of 0 in that case.
 * Optional method in the MM::Camera API; by default cameras do not support
 * multiple ROIs.
 * @param xs (Return value) X indices of upper-left corner of the ROIs.
 * @param ys (Return value) Y indices of upper-left corner of the ROIs.
 * @param widths (Return value) Widths of the ROIs, in pixels.
 * @param heights (Return value) Heights of the ROIs, in pixels.
 * @param numROIs Length of the input arrays. If there are fewer ROIs than
 *        this, then this value must be updated to reflect the new count.
 */
int MH_camera::GetMultiROI(unsigned* xs, unsigned* ys, unsigned* widths,
    unsigned* heights, unsigned* length)
{
    unsigned int roiCount = (unsigned int)multiROIXs_.size();
    if (roiCount > *length)
    {
        // This should never happen.
        return DEVICE_INTERNAL_INCONSISTENCY;
    }
    for (unsigned int i = 0; i < roiCount; ++i)
    {
        xs[i] = multiROIXs_[i];
        ys[i] = multiROIYs_[i];
        widths[i] = multiROIWidths_[i];
        heights[i] = multiROIHeights_[i];
    }
    *length = roiCount;
    return DEVICE_OK;
}

/**
* Returns the current exposure setting in milliseconds.
* Required by the MM::Camera API.
*/
double MH_camera::GetExposure() const
{
    char buf[MM::MaxStrLength];
    int ret = GetProperty(MM::g_Keyword_Exposure, buf);
    if (ret != DEVICE_OK)
        return 0.0;
    return atof(buf);
}

/**
 * Returns the current exposure from a sequence and increases the sequence counter
 * Used for exposure sequences
 */
double MH_camera::GetSequenceExposure()
{
    if (exposureSequence_.size() == 0)
        return this->GetExposure();

    double exposure = exposureSequence_[sequenceIndex_];

    sequenceIndex_++;
    if (sequenceIndex_ >= exposureSequence_.size())
        sequenceIndex_ = 0;

    return exposure;
}

/**
* Sets exposure in milliseconds.
* Required by the MM::Camera API.
*/
void MH_camera::SetExposure(double exp)
{
    SetProperty(MM::g_Keyword_Exposure, CDeviceUtils::ConvertToString(exp));
    GetCoreCallback()->OnExposureChanged(this, exp);;
}

/**
* Returns the current binning factor.
* Required by the MM::Camera API.
*/
int MH_camera::GetBinning() const
{
    char buf[MM::MaxStrLength];
    int ret = GetProperty(MM::g_Keyword_Binning, buf);
    if (ret != DEVICE_OK)
        return 1;
    return atoi(buf);
}

/**
* Sets binning factor.
* Required by the MM::Camera API.
*/
int MH_camera::SetBinning(int binF)
{
    return SetProperty(MM::g_Keyword_Binning, CDeviceUtils::ConvertToString(binF));
}

int MH_camera::IsExposureSequenceable(bool& isSequenceable) const
{
    isSequenceable = isSequenceable_;
    return DEVICE_OK;
}

int MH_camera::GetExposureSequenceMaxLength(long& nrEvents) const
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    nrEvents = sequenceMaxLength_;
    return DEVICE_OK;
}

int MH_camera::StartExposureSequence()
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    // may need thread lock
    sequenceRunning_ = true;
    return DEVICE_OK;
}

int MH_camera::StopExposureSequence()
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    // may need thread lock
    sequenceRunning_ = false;
    sequenceIndex_ = 0;
    return DEVICE_OK;
}

/**
 * Clears the list of exposures used in sequences
 */
int MH_camera::ClearExposureSequence()
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    exposureSequence_.clear();
    return DEVICE_OK;
}

/**
 * Adds an exposure to a list of exposures used in sequences
 */
int MH_camera::AddToExposureSequence(double exposureTime_ms)
{
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    exposureSequence_.push_back(exposureTime_ms);
    return DEVICE_OK;
}

int MH_camera::SendExposureSequence() const {
    if (!isSequenceable_) {
        return DEVICE_UNSUPPORTED_COMMAND;
    }

    return DEVICE_OK;
}

int MH_camera::SetAllowedBinning()
{
    vector<string> binValues;
    binValues.push_back("1");
    binValues.push_back("2");
    if (scanMode_ < 3)
        binValues.push_back("4");
    if (scanMode_ < 2)
        binValues.push_back("8");
    if (binSize_ == 8 && scanMode_ == 3) {
        SetProperty(MM::g_Keyword_Binning, "2");
    }
    else if (binSize_ == 8 && scanMode_ == 2) {
        SetProperty(MM::g_Keyword_Binning, "4");
    }
    else if (binSize_ == 4 && scanMode_ == 3) {
        SetProperty(MM::g_Keyword_Binning, "2");
    }

    LogMessage("Setting Allowed Binning settings", true);
    return SetAllowedValues(MM::g_Keyword_Binning, binValues);
}


/**
 * Required by the MM::Camera API
 * Please implement this yourself and do not rely on the base class implementation
 * The Base class implementation is deprecated and will be removed shortly
 */
int MH_camera::StartSequenceAcquisition(double interval)
{
    return StartSequenceAcquisition(LONG_MAX, interval, false);
}

/**
* Stop and wait for the Sequence thread finished
*/
int MH_camera::StopSequenceAcquisition()
{
    if (!thd_->IsStopped()) {
        thd_->Stop();
        thd_->wait();
    }

    return DEVICE_OK;
}

/**
* Simple implementation of Sequence Acquisition
* A sequence acquisition should run on its own thread and transport new images
* coming of the camera into the MMCore circular buffer.
*/
int MH_camera::StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow)
{
    if (IsCapturing())
        return DEVICE_CAMERA_BUSY_ACQUIRING;

    int ret = GetCoreCallback()->PrepareForAcq(this);
    if (ret != DEVICE_OK)
        return ret;
    sequenceStartTime_ = GetCurrentMMTime();
    imageCounter_ = 0;
    thd_->Start(numImages, interval_ms);
    stopOnOverflow_ = stopOnOverflow;
    return DEVICE_OK;
}

/*
 * Inserts Image and MetaData into MMCore circular Buffer
 */
int MH_camera::InsertImage()
{
    MM::MMTime timeStamp = this->GetCurrentMMTime();
    char label[MM::MaxStrLength];
    this->GetLabel(label);

    // Important:  metadata about the image are generated here:
    Metadata md;
    md.put("Camera", label);
//    md.put(MM::g_Keyword_Metadata_StartTime, CDeviceUtils::ConvertToString(sequenceStartTime_.getMsec()));
    md.put(MM::g_Keyword_Elapsed_Time_ms, CDeviceUtils::ConvertToString((timeStamp - sequenceStartTime_).getMsec()));
    md.put(MM::g_Keyword_Metadata_ROI_X, CDeviceUtils::ConvertToString((long)roiX_));
    md.put(MM::g_Keyword_Metadata_ROI_Y, CDeviceUtils::ConvertToString((long)roiY_));

    imageCounter_++;

    char buf[MM::MaxStrLength];
    GetProperty(MM::g_Keyword_Binning, buf);
    md.put(MM::g_Keyword_Binning, buf);

    MMThreadGuard g(imgPixelsLock_);

    const unsigned char* pI;
    pI = GetImageBuffer();

    unsigned int w = GetImageWidth();
    unsigned int h = GetImageHeight();
    unsigned int b = GetImageBytesPerPixel();

    int ret = GetCoreCallback()->InsertImage(this, pI, w, h, b, nComponents_, md.Serialize().c_str());
    if (!stopOnOverflow_ && ret == DEVICE_BUFFER_OVERFLOW)
    {
        // do not stop on overflow - just reset the buffer
        GetCoreCallback()->ClearImageBuffer(this);
        // don't process this same image again...
        return GetCoreCallback()->InsertImage(this, pI, w, h, b, nComponents_, md.Serialize().c_str(), false);
    }
    else
    {
        return ret;
    }
}

/*
 * Do actual capturing
 * Called from inside the thread
 */
int MH_camera::RunSequenceOnThread(MM::MMTime startTime)
{
    //SEQUENCE OR LIVE HERE?
    int ret = DEVICE_ERR;

    // Trigger
    if (triggerDevice_.length() > 0) {
        MM::Device* triggerDev = GetDevice(triggerDevice_.c_str());
        if (triggerDev != 0) {
            LogMessage("trigger requested");
            triggerDev->SetProperty("Trigger", "+");
        }
    }

    double exposure = GetSequenceExposure();

    if (!fastImage_)
    {
        LogMessage("Sequence non-fast");
        start_acq();
        GenerateSyntheticImage(img_, exposure);
    }

    // Simulate exposure duration
    double finishTime = exposure * (imageCounter_ + 1);
    while ((GetCurrentMMTime() - startTime).getMsec() < finishTime)
    {
        CDeviceUtils::SleepMs(1);
    }

    ret = InsertImage();

    if (ret != DEVICE_OK)
    {
        return ret;
    }
    return ret;
};

bool MH_camera::IsCapturing() {
    return !thd_->IsStopped();
}

/*
 * called from the thread function before exit
 */
void MH_camera::OnThreadExiting() throw()
{
    try
    {
        LogMessage(g_Msg_SEQUENCE_ACQUISITION_THREAD_EXITING);
        GetCoreCallback() ? GetCoreCallback()->AcqFinished(this, 0) : DEVICE_OK;
    }
    catch (...)
    {
        LogMessage(g_Msg_EXCEPTION_IN_ON_THREAD_EXITING, false);
    }
}


MySequenceThread::MySequenceThread(MH_camera* pCam)
    :intervalMs_(default_intervalMS)
    , numImages_(default_numImages)
    , imageCounter_(0)
    , stop_(true)
    , suspend_(false)
    , camera_(pCam)
    , startTime_(0)
    , actualDuration_(0)
    , lastFrameTime_(0)
{};

MySequenceThread::~MySequenceThread() {};

void MySequenceThread::Stop() {
    MMThreadGuard g(this->stopLock_);
    stop_ = true;
}

void MySequenceThread::Start(long numImages, double intervalMs)
{
    MMThreadGuard g1(this->stopLock_);
    MMThreadGuard g2(this->suspendLock_);
    numImages_ = numImages;
    intervalMs_ = intervalMs;
    imageCounter_ = 0;
    stop_ = false;
    suspend_ = false;
    activate();
    actualDuration_ = MM::MMTime(0);
    startTime_ = camera_->GetCurrentMMTime();
    lastFrameTime_ = MM::MMTime(0);
}

bool MySequenceThread::IsStopped() {
    MMThreadGuard g(this->stopLock_);
    return stop_;
}

void MySequenceThread::Suspend() {
    MMThreadGuard g(this->suspendLock_);
    suspend_ = true;
}

bool MySequenceThread::IsSuspended() {
    MMThreadGuard g(this->suspendLock_);
    return suspend_;
}

void MySequenceThread::Resume() {
    MMThreadGuard g(this->suspendLock_);
    suspend_ = false;
}

int MySequenceThread::svc(void) throw()
{
    int ret = DEVICE_ERR;
    try
    {
        do
        {
            ret = camera_->RunSequenceOnThread(startTime_);
        } while (DEVICE_OK == ret && !IsStopped() && imageCounter_++ < numImages_ - 1);
        if (IsStopped())
            camera_->LogMessage("SeqAcquisition interrupted by the user\n");
    }
    catch (...) {
        camera_->LogMessage(g_Msg_EXCEPTION_IN_THREAD, false);
    }
    stop_ = true;
    actualDuration_ = camera_->GetCurrentMMTime() - startTime_;
    camera_->OnThreadExiting();
    return ret;
}


///////////////////////////////////////////////////////////////////////////////
// MH_camera Action handlers
///////////////////////////////////////////////////////////////////////////////

int MH_camera::OnMaxExposure(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(exposureMaximum_);
    }
    else if (eAct == MM::AfterSet)
    {
        pProp->Get(exposureMaximum_);
    }
    return DEVICE_OK;
}


/*
* this Read Only property will update whenever any property is modified
*/

int MH_camera::OnTestProperty(MM::PropertyBase* pProp, MM::ActionType eAct, long indexx)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(testProperty_[indexx]);
    }
    else if (eAct == MM::AfterSet)
    {
        pProp->Get(testProperty_[indexx]);
    }
    return DEVICE_OK;

}


/**
* Handles "Binning" property.
*/
int MH_camera::OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int ret = DEVICE_ERR;
    switch (eAct)
    {
    case MM::AfterSet:
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        // the user just set the new value for the property, so we have to
        // apply this value to the 'hardware'.
        long binFactor;
        pProp->Get(binFactor);
        if (binFactor > 0 && binFactor < 10)
        {
            // calculate ROI using the previous bin settings
            double factor = (double)binFactor / (double)binSize_;
            roiX_ = (unsigned int)(roiX_ / factor);
            roiY_ = (unsigned int)(roiY_ / factor);
            for (unsigned int i = 0; i < multiROIXs_.size(); ++i)
            {
                multiROIXs_[i] = (unsigned int)(multiROIXs_[i] / factor);
                multiROIYs_[i] = (unsigned int)(multiROIYs_[i] / factor);
                multiROIWidths_[i] = (unsigned int)(multiROIWidths_[i] / factor);
                multiROIHeights_[i] = (unsigned int)(multiROIHeights_[i] / factor);
            }
            img_.Resize((unsigned int)(img_.Width() / factor),
                (unsigned int)(img_.Height() / factor));
            binSize_ = binFactor;
            std::ostringstream os;
            os << binSize_;
            OnPropertyChanged("Binning", os.str().c_str());
            ret = DEVICE_OK;
        }
    }break;
    case MM::BeforeGet:
    {
        ret = DEVICE_OK;
        pProp->Set(binSize_);
    }break;
    default:
        break;
    }
    return ret;
}

/**
* Handles "PixelType" property.
*/
int MH_camera::OnPixelType(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int ret = DEVICE_ERR;
    switch (eAct)
    {
    case MM::AfterSet:
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        string pixelType;
        pProp->Get(pixelType);

        if (pixelType.compare(g_PixelType_8bit) == 0)
        {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 1);
            bitDepth_ = 8;
            ret = DEVICE_OK;
        }
        else if (pixelType.compare(g_PixelType_16bit) == 0)
        {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 2);
            bitDepth_ = 16;
            ret = DEVICE_OK;
        }
        else
        {
            // on error switch to default pixel type
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 1);
            pProp->Set(g_PixelType_8bit);
            bitDepth_ = 8;
            ret = ERR_UNKNOWN_MODE;
        }
    }
    break;
    case MM::BeforeGet:
    {
        long bytesPerPixel = GetImageBytesPerPixel();
        if (bytesPerPixel == 1)
        {
            pProp->Set(g_PixelType_8bit);
        }
        else if (bytesPerPixel == 2)
        {
            pProp->Set(g_PixelType_16bit);
        }
        //else if (bytesPerPixel == 4)
        //{
        //    if (nComponents_ == 1)
        //    {
        //        pProp->Set(::g_PixelType_32bit);
        //    }
        //} 
        else
        {
            pProp->Set(g_PixelType_8bit);
        }
        ret = DEVICE_OK;
    } break;
    default:
        break;
    }
    return ret;
}

/**
* Handles "BitDepth" property.
*/
int MH_camera::OnBitDepth(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    int ret = DEVICE_ERR;
    switch (eAct)
    {
    case MM::AfterSet:
    {
        if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

        long bitDepth;
        pProp->Get(bitDepth);

        unsigned int bytesPerComponent;

        switch (bitDepth) {
        case 8:
            bytesPerComponent = 1;
            bitDepth_ = 8;
            ret = DEVICE_OK;
            break;
        case 10:
            bytesPerComponent = 2;
            bitDepth_ = 10;
            ret = DEVICE_OK;
            break;
        case 12:
            bytesPerComponent = 2;
            bitDepth_ = 12;
            ret = DEVICE_OK;
            break;
        case 14:
            bytesPerComponent = 2;
            bitDepth_ = 14;
            ret = DEVICE_OK;
            break;
        case 16:
            bytesPerComponent = 2;
            bitDepth_ = 16;
            ret = DEVICE_OK;
            break;
        case 32:
            bytesPerComponent = 4;
            bitDepth_ = 32;
            ret = DEVICE_OK;
            break;
        default:
            // on error switch to default pixel type
            bytesPerComponent = 1;

            pProp->Set((long)8);
            bitDepth_ = 8;
            ret = ERR_UNKNOWN_MODE;
            break;
        }
        char buf[MM::MaxStrLength];
        GetProperty(MM::g_Keyword_PixelType, buf);
        std::string pixelType(buf);
        unsigned int bytesPerPixel = 1;


        // automagickally change pixel type when bit depth exceeds possible value
        if (pixelType.compare(g_PixelType_8bit) == 0)
        {
            if (2 == bytesPerComponent)
            {
                SetProperty(MM::g_Keyword_PixelType, g_PixelType_16bit);
                bytesPerPixel = 2;
            }
            else
            {
                bytesPerPixel = 1;
            }
        }
        else if (pixelType.compare(g_PixelType_16bit) == 0)
        {
            bytesPerPixel = 2;
        }
        img_.Resize(img_.Width(), img_.Height(), bytesPerPixel);

    } break;
    case MM::BeforeGet:
    {
        pProp->Set((long)bitDepth_);
        ret = DEVICE_OK;
    } break;
    default:
        break;
    }
    return ret;
}
/**
* Handles "ReadoutTime" property.
*/
int MH_camera::OnReadoutTime(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        double readoutMs;
        pProp->Get(readoutMs);

        readoutUs_ = readoutMs * 1000.0;
    }
    else if (eAct == MM::BeforeGet)
    {
        pProp->Set(readoutUs_ / 1000.0);
    }

    return DEVICE_OK;
}

int MH_camera::OnDropPixels(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        long tvalue = 0;
        pProp->Get(tvalue);
        dropPixels_ = (0 == tvalue) ? false : true;
    }
    else if (eAct == MM::BeforeGet)
    {
        pProp->Set(dropPixels_ ? 1L : 0L);
    }

    return DEVICE_OK;
}

int MH_camera::OnFastImage(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        long tvalue = 0;
        pProp->Get(tvalue);
        fastImage_ = (0 == tvalue) ? false : true;
    }
    else if (eAct == MM::BeforeGet)
    {
        pProp->Set(fastImage_ ? 1L : 0L);
    }

    return DEVICE_OK;
}

int MH_camera::OnSaturatePixels(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        long tvalue = 0;
        pProp->Get(tvalue);
        saturatePixels_ = (0 == tvalue) ? false : true;
    }
    else if (eAct == MM::BeforeGet)
    {
        pProp->Set(saturatePixels_ ? 1L : 0L);
    }

    return DEVICE_OK;
}

int MH_camera::OnFractionOfPixelsToDropOrSaturate(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        double tvalue = 0;
        pProp->Get(tvalue);
        fractionOfPixelsToDropOrSaturate_ = tvalue;
    }
    else if (eAct == MM::BeforeGet)
    {
        pProp->Set(fractionOfPixelsToDropOrSaturate_);
    }

    return DEVICE_OK;
}

int MH_camera::OnShouldRotateImages(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        long tvalue = 0;
        pProp->Get(tvalue);
        shouldRotateImages_ = (tvalue != 0);
    }
    else if (eAct == MM::BeforeGet)
    {
        pProp->Set((long)shouldRotateImages_);
    }

    return DEVICE_OK;
}

int MH_camera::OnShouldDisplayImageNumber(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        long tvalue = 0;
        pProp->Get(tvalue);
        shouldDisplayImageNumber_ = (tvalue != 0);
    }
    else if (eAct == MM::BeforeGet)
    {
        pProp->Set((long)shouldDisplayImageNumber_);
    }

    return DEVICE_OK;
}

int MH_camera::OnStripeWidth(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        pProp->Get(stripeWidth_);
    }
    else if (eAct == MM::BeforeGet)
    {
        pProp->Set(stripeWidth_);
    }

    return DEVICE_OK;
}

int MH_camera::OnSupportsMultiROI(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        long tvalue = 0;
        pProp->Get(tvalue);
        supportsMultiROI_ = (tvalue != 0);
    }
    else if (eAct == MM::BeforeGet)
    {
        pProp->Set((long)supportsMultiROI_);
    }

    return DEVICE_OK;
}

int MH_camera::OnMultiROIFillValue(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        long tvalue = 0;
        pProp->Get(tvalue);
        multiROIFillValue_ = (int)tvalue;
    }
    else if (eAct == MM::BeforeGet)
    {
        pProp->Set((long)multiROIFillValue_);
    }

    return DEVICE_OK;
}

/*
* Handles "ScanMode" property.
* Changes allowed Binning values to test whether the UI updates properly
*/
int MH_camera::OnScanMode(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet) {
        pProp->Get(scanMode_);
        SetAllowedBinning();
        if (initialized_) {
            int ret = OnPropertiesChanged();
            if (ret != DEVICE_OK)
                return ret;
        }
    }
    else if (eAct == MM::BeforeGet) {
        LogMessage("Reading property ScanMode", true);
        pProp->Set(scanMode_);
    }
    return DEVICE_OK;
}

int MH_camera::Onn_beams_X(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(n_beams_X_);
    }
    else if (eAct == MM::AfterSet)
    {
        long value;
        pProp->Get(value);
        if ((value < 1) || (8 < value))
            return DEVICE_ERR;  // invalid image size
        if (value != n_beams_X_)
        {
            n_beams_X_ = value;
            cameraCCDXSize_ = n_scanPixels_X_ * n_beams_X_;
            img_.Resize(cameraCCDXSize_ / binSize_, cameraCCDYSize_ / binSize_);
        }
    }
    return DEVICE_OK;
}

int MH_camera::Onn_beams_Y(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(n_beams_Y_);
    }
    else if (eAct == MM::AfterSet)
    {
        long value;
        pProp->Get(value);
        if ((value < 1) || (8 < value))
            return DEVICE_ERR;  // invalid image size
        if (value != n_beams_Y_)
        {
            n_beams_Y_ = value;
            cameraCCDYSize_ = n_scanPixels_Y_ * n_beams_Y_;
            img_.Resize(cameraCCDXSize_ / binSize_, cameraCCDYSize_ / binSize_);
        }
    }
    return DEVICE_OK;
}

int MH_camera::Onn_scanPixels_X(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(n_scanPixels_X_);
    }
    else if (eAct == MM::AfterSet)
    {
        long value;
        pProp->Get(value);
        if ((value < 16) || (33000 < value))
            return DEVICE_ERR;  // invalid image size
        if (value != n_scanPixels_X_)
        {
            n_scanPixels_X_ = value;
            cameraCCDXSize_ = n_scanPixels_X_ * n_beams_X_;
            img_.Resize(cameraCCDXSize_ / binSize_, cameraCCDYSize_ / binSize_);
        }
    }
    MM::Hub* pHub = GetParentHub();  
    int newval = (int)n_scanPixels_X_;
    char buf[MM::MaxStrLength];
    _itoa(newval, buf, 10);
    pHub->SetProperty(g_N_Hub_Scan_Px_X,buf);
    pHub->GetProperty(g_N_Hub_Scan_Px_X, buf);
    LogMessage("real value should be next");
    std::string tmp(buf);
    LogMessage(tmp.c_str());

    return DEVICE_OK;
}

int MH_camera::Onn_scanPixels_Y(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(n_scanPixels_Y_);
    }
    else if (eAct == MM::AfterSet)
    {
        long value;
        pProp->Get(value);
        if ((value < 16) || (33000 < value))
            return DEVICE_ERR;  // invalid image size
        if (value != n_scanPixels_Y_)
        {
            n_scanPixels_Y_ = value;
            cameraCCDYSize_ = n_scanPixels_Y_ * n_beams_Y_;
            img_.Resize(cameraCCDXSize_ / binSize_, cameraCCDYSize_ / binSize_);
        }
    }
    return DEVICE_OK;
}

int MH_camera::OnTriggerDevice(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(triggerDevice_.c_str());
    }
    else if (eAct == MM::AfterSet)
    {
        pProp->Get(triggerDevice_);
    }
    return DEVICE_OK;
}


int MH_camera::OnCCDTemp(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(ccdT_);
    }
    else if (eAct == MM::AfterSet)
    {
        pProp->Get(ccdT_);
    }
    return DEVICE_OK;
}

int MH_camera::OnIsSequenceable(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    std::string val = "Yes";
    if (eAct == MM::BeforeGet)
    {
        if (!isSequenceable_)
        {
            val = "No";
        }
        pProp->Set(val.c_str());
    }
    else if (eAct == MM::AfterSet)
    {
        isSequenceable_ = false;
        pProp->Get(val);
        if (val == "Yes")
        {
            isSequenceable_ = true;
        }
    }

    return DEVICE_OK;
}


int MH_camera::OnMode(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    std::string val;
    if (eAct == MM::BeforeGet)
    {
        switch (mode_)
        {
        case MODE_MH_TEST:
            val = g_MH_Test;
            break;
        case MODE_MH_HISTO:
            val = g_MH_Histo;
            break;
        case MODE_MH_IMAGE:
            val = g_MH_Image;
            break;
        default:
            val = g_MH_Test;
            break;
        }
        pProp->Set(val.c_str());
    }
    else if (eAct == MM::AfterSet)
    {
        pProp->Get(val);
        if (val == g_MH_Test)
        {
            mode_ = MODE_MH_TEST;
        }
        else if (val == g_MH_Histo)
        {
            mode_ = MODE_MH_HISTO;
        }
        else if (val == g_MH_Image)
        {
            mode_ = MODE_MH_IMAGE;
        }
        else
        {
            mode_ = MODE_MH_TEST;
        }
    }
    return DEVICE_OK;
}

int MH_camera::OnPCF(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(pcf_);
    }
    else if (eAct == MM::AfterSet)
    {
        pProp->Get(pcf_);
    }
    return DEVICE_OK;
}

int MH_camera::OnPhotonFlux(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(photonFlux_);
    }
    else if (eAct == MM::AfterSet)
    {
        pProp->Get(photonFlux_);
    }
    return DEVICE_OK;
}

int MH_camera::OnReadNoise(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(readNoise_);
    }
    else if (eAct == MM::AfterSet)
    {
        pProp->Get(readNoise_);
    }
    return DEVICE_OK;
}


int MH_camera::OnCrash(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    AddAllowedValue("SimulateCrash", "");
    AddAllowedValue("SimulateCrash", "Dereference Null Pointer");
    AddAllowedValue("SimulateCrash", "Divide by Zero");
    if (eAct == MM::BeforeGet)
    {
        pProp->Set("");
    }
    else if (eAct == MM::AfterSet)
    {
        std::string choice;
        pProp->Get(choice);
        if (choice == "Dereference Null Pointer")
        {
            int* p = 0;
            volatile int i = *p;
            i++;
        }
        else if (choice == "Divide by Zero")
        {
            volatile int i = 1, j = 0, k;
            k = i / j;
        }
    }
    return DEVICE_OK;
}

int MH_camera::OnLifetime(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(Sim_lifetime_);
    }
    else if (eAct == MM::AfterSet)
    {
        long value;
        pProp->Get(value);
        Sim_lifetime_ = value;
    }
    return DEVICE_OK;

}

int MH_camera::OnDecOrRat(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        std::string current_val;
        if (rates_or_decays_) {
            current_val = "Rates";
        }
        else {
            current_val = "Decays";
        }
        pProp->Set(current_val.c_str());
    }
    else if (eAct == MM::AfterSet)
    {
        std::string choice;
        pProp->Get(choice);
        if (choice == "Decay") {
            rates_or_decays_ = false;
        }
        else {
            rates_or_decays_ = true;
        }
    }
    return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Private MH_camera methods
///////////////////////////////////////////////////////////////////////////////

/**
* Sync internal image buffer size to the chosen property values.
*/
int MH_camera::ResizeImageBuffer()
{
    char buf[MM::MaxStrLength];
    //int ret = GetProperty(MM::g_Keyword_Binning, buf);
    //if (ret != DEVICE_OK)
    //   return ret;
    //binSize_ = atol(buf);

    int ret = GetProperty(MM::g_Keyword_PixelType, buf);
    if (ret != DEVICE_OK)
        return ret;

    std::string pixelType(buf);
    int byteDepth = 0;

    if (pixelType.compare(g_PixelType_8bit) == 0)
    {
        byteDepth = 1;
    }
    else if (pixelType.compare(g_PixelType_16bit) == 0)
    {
        byteDepth = 2;
    }

    img_.Resize(cameraCCDXSize_ / binSize_, cameraCCDYSize_ / binSize_, byteDepth);
    return DEVICE_OK;
}

void MH_camera::GenerateDecay(ImgBuffer &img)
{

}

uint64_t MH_camera::TimestampDeltaToPs(uint64_t timestamp_delta) {
    return (timestamp_delta * MeasDesc_GlobalResolution_);
}


//#   +----------------------+ T3 32 bit record  +---------------------+
//#   |x|x|x|x|x|x|x|x|x|x|x|x|x|x|x|x|  |x|x|x|x|x|x|x|x|x|x|x|x|x|x|x|x| --> 32 bit record
//#   +-------------------------------+  +-------------------------------+
//#   +-------------------------------+  +-------------------------------+
//#   | | | | | | | | | | | | | | | | |  | | | | | | |x|x|x|x|x|x|x|x|x|x|  --> Sync
//#   +-------------------------------+  +-------------------------------+
//#   +-------------------------------+  +-------------------------------+
//#   | | | | | | | |x|x|x|x|x|x|x|x|x|  |x|x|x|x|x|x| | | | | | | | | | |  --> TCSPC bin
//#   +-------------------------------+  +-------------------------------+
//#   +-------------------------------+  +-------------------------------+
//#   | |x|x|x|x|x|x| | | | | | | | | |  | | | | | | | | | | | | | | | | |  --> Spectral/TCSPC input Channel
//#   +-------------------------------+  +-------------------------------+
//#   +-------------------------------+  +-------------------------------+
//#   |x| | | | | | | | | | | | | | | |  | | | | | | | | | | | | | | | | |  --> Special markers
//#   +-------------------------------+  +-------------------------------+

void MH_camera::Interpret_TTTR(unsigned int& record, unsigned int& sync, unsigned int& tcspc, unsigned int& channel, bool& special) {
    //Try to extract relevant information with a minimum amount of duplication
    //Bit-mask a copy, then shift as appropriate for each element?
    //Reminder: For MultiHarp
    //val will be one record from the unsigned int buffer[TTREADMAX]
    //Multiharp: ## special: 1 ## channel: 6 ## dtime : 15 ## nsync : 10
    //Do some shifting for direct comparisons
    unsigned int nsync_mask = 0x00003ff; // 10x 1
    unsigned int tcspc_mask = 0x0007fff; // 15x 1
    unsigned int chan_mask = 0x000003f; // 6x 1
    unsigned int special_mask = 0x0000001; // 1x 1
    int nsync_shift = 0;
    int tcspc_shift = 10;
    int chan_shift = 25;
    int special_shift = 31;
    sync = (record >> nsync_shift) & nsync_mask;
    tcspc = (record >> tcspc_shift) & tcspc_mask;
    channel = (record >> chan_shift) & chan_mask;
    special = (record >> special_shift) & special_mask;
}

void MH_camera::TranslateRecord(unsigned int val, ImgBuffer& img) {
    bool special;
    unsigned int tcspc;
    unsigned int channel;
    unsigned int nsync;
    uint64_t timestamp_ps;
    uint64_t pulse_interval_ps = MeasDesc_GlobalResolution_;//in ps?
    
    Interpret_TTTR(val,nsync,tcspc,channel,special);
    //NEED TO ACCOUNT FOR ROLLOVERS - Every 1024 syncs it'll overflow, as it is a 10-bit counter
    uint64_t overflowtime = (uint64_t)overflow_counter_ * ((uint64_t)1024);
    timestamp_ps = (((uint64_t)nsync + overflowtime)* MeasDesc_GlobalResolution_);//this should convert to ps
    if (special) {
        //Handle special tagged channel numbers here:
        //Line start - 000010 :: Line end - 000001 :: Frame start - 000011
        switch (channel) {
            case 1:
                //Use for pixel timing
                last_line_end_ = timestamp_ps;
                //This will be invalid for the first line
                pixel_dwelltime_ps_ = (last_line_end_ - last_line_start_) / (cameraCCDXSize_);
                break;
            case 2:
                last_line_start_ = timestamp_ps;//this is in ps
                if (frame_active_) {
                    //Once we've had a frame clock...
                    current_line_++;
                }
                if (current_line_> cameraCCDYSize_) {
                    frame_active_ = false;
                }
                break;
            case 3:
            case 4: //frame clock
                current_line_ = -1;//First line clock will then correspond to line 0, assuming it comes right after the frame clock?
                frame_active_ = true;
                n_frame_tracker_++;
                break;
            case 63:
                overflow_counter_ += nsync;
                break;
            default:
                break;
        }
        if (overflow_counter_ % 10 == 0) {
            //char dummy1[100];
            //sprintf(dummy1, "Syncs : %" PRIu64 " syncs", nsync);
            //msgstr = dummy1;
            //LogMessage(msgstr); 
        }
    }
    else {
        
        if (last_line_end_ < last_line_start_ && channel !=6) {//If not in X flyback... ignore NDD for now
            if (current_line_ >= 0) {//Or Y flyback?
                //For multibeam add switch for channel here which tells us the x and y shift needed to add to x_px and y_px below? test code done below
                //Making the assumption that channels correspond in order to beams as follows (e.g. for a 3x2 array - X assumed to always go first):
                //X1Y1, X2Y1, X3Y1, X1Y2, X2Y2, X3Y2
                int x_shift = 0;
                int y_shift = 0;
                //int tmpchan = ((n_beams_X_ * n_beams_Y_)-1) - channel;//change this to change order that beams are shown in
                int tmpchan =  5-channel;//change this to change order that beams are shown in
                x_shift = (tmpchan / n_beams_Y_) * n_scanPixels_X_;
                y_shift = (tmpchan % n_beams_Y_) * n_scanPixels_Y_;
                

                //If we have had a frame clock...
                //int x_px = GetPixnumInLine(timestamp_ps, last_line_start_);
                int x_px = GetPixnumInLine(timestamp_ps, last_line_start_)+ x_shift;
                //int x_px = std::rand()*cameraCCDXSize_/RAND_MAX;// "simulation"
                int y_px = current_line_+y_shift;

                //These parts to be moved into superfunction. Based off the memory structure rather than the variable interpretations, which may change out of sync?
                char buf[MM::MaxStrLength];
                GetProperty(MM::g_Keyword_PixelType, buf);
                std::string pixelType(buf);
                int maxValue = 1 << GetBitDepth();
                long nrPixels = img.Width() * img.Height();
                int target_px = x_px + y_px * cameraCCDXSize_;

                if (overflow_counter_ % 10 == 0) {
                    char dummy1[100];
                    sprintf(dummy1, "px : %d, channel %" PRIu64 " . ", y_px, channel);
                    msgstr = dummy1;
                    LogMessage(msgstr);
                }

                if (channel >= 0) {//DELETE THIS LATER
                    if (pixelType.compare(g_PixelType_8bit) == 0) {//REALLY shouldn't be using this... too few DN per pixel
                        maxValue = 255;//Future overflow check? Current behaviour is to saturate
                        unsigned char* rawShorts = (unsigned char*) const_cast<unsigned char*>(img.GetPixels());
                        if (rawShorts[target_px] != maxValue) {
                            rawShorts[target_px] += 1;
                        }
                    }
                    else if (pixelType.compare(g_PixelType_16bit) == 0) {
                        maxValue = 65535;//Future overflow check? Current behaviour is to saturate
                        unsigned short* rawShorts = (unsigned short*) const_cast<unsigned char*>(img.GetPixels());
                        //channel * cameraCCDXSize_* (cameraCCDYSize_ / nBeams);
                        if (rawShorts[target_px] != maxValue) {
                            rawShorts[target_px] += 1;
                        }
                    }
                }
            }
            else {
                //Not sure what to do if we are in an unknown position in a scan... could accumulate in separate memory and shift/fix retrospectively?
                //Ignore it for now and just lose the counts. Worst case is just loning one line's worth?
            }
        }
    }
}

int MH_camera::GetPixnumInLine(uint64_t timestamp, uint64_t linestart_timestamp) {
    //Try to calculate which pixel in a line a photon belongs in
    //uint64_t time_into_line_ps = TimestampDeltaToPs(timestamp - linestart_timestamp);
    uint64_t time_into_line_ps = (timestamp - linestart_timestamp);
    uint64_t pn = (time_into_line_ps / pixel_dwelltime_ps_);
    int pixnum = (int)pn/ n_beams_X_;
    if (overflow_counter_ % 1 == 0) {
        char dummy1[100];
        sprintf(dummy1, "pn : %" PRIu64 " px, dwelltime: %" PRIu64 ", y line num: %dpx", pn, pixel_dwelltime_ps_, current_line_);
        msgstr = dummy1;
        //LogMessage(msgstr);
    }
    //Assume the fast scan axis is in x...
    if (pixnum >= cameraCCDXSize_) {
        //If there's too many pixels in a line, dump things here
        pixnum = cameraCCDXSize_-1;
    }
    return pixnum;
}

void MH_camera::GenerateEmptyImage(ImgBuffer& img)
{
    MMThreadGuard g(imgPixelsLock_);
    if (img.Height() == 0 || img.Width() == 0 || img.Depth() == 0)
        return;
    unsigned char* pBuf = const_cast<unsigned char*>(img.GetPixels());
    memset(pBuf, 0, img.Height() * img.Width() * img.Depth());
}



/**
* Generates an image.
*
*/

void MH_camera::GenerateSyntheticImage(ImgBuffer& img, double exp)
{

    MMThreadGuard g(imgPixelsLock_);

    if (mode_ == MODE_MH_TEST)
    {
        if (GenerateMHTestPattern(img))
            return;
    }
    else if (mode_ == MODE_MH_HISTO)
    {
        if (GenerateMHHisto(img))
            return;
    }
    else if (mode_ == MODE_MH_IMAGE) {
        if (GenerateMHImage(img))
            return;
    }

    //std::string pixelType;
    char buf[MM::MaxStrLength];
    GetProperty(MM::g_Keyword_PixelType, buf);
    std::string pixelType(buf);

    if (img.Height() == 0 || img.Width() == 0 || img.Depth() == 0) {
        return;
    }
}

bool MH_camera::GenerateMHTestPattern(ImgBuffer& img) {
    unsigned width = img.Width(), height = img.Height();
    //const unsigned char maxVal = 65535;
    char buf[MM::MaxStrLength];
    GetProperty(MM::g_Keyword_PixelType, buf);
    std::string pixelType(buf);

    int maxValue = 1 << GetBitDepth();
    long nrPixels = img.Width() * img.Height();
    int check_stride = 50;

    if (pixelType.compare(g_PixelType_8bit) == 0)
    {
        maxValue = 255;
        unsigned char* rawShorts = (unsigned char*) const_cast<unsigned char*>(img.GetPixels());
        for (unsigned y = 0; y < height; ++y)
        {
            for (unsigned x = 0; x < width; ++x)
            {
                if (y == 0)
                {
                    if (int(x / check_stride) % 2 == 0) {
                        rawShorts[x] = (unsigned char)0;
                    }
                    else {
                        rawShorts[x] = (unsigned char)maxValue;
                    }
                }
                else {
                    if (int(y / check_stride) % 2 == 0) {
                        rawShorts[x + y * width] = (unsigned char)rawShorts[x];
                    }
                    else {
                        rawShorts[x + y * width] = (unsigned char)(maxValue - rawShorts[x]);
                    }
                }
            }
        }
    }
    else if (pixelType.compare(g_PixelType_16bit) == 0)
    {
        maxValue = 65535;
        unsigned short* rawShorts = (unsigned short*) const_cast<unsigned char*>(img.GetPixels());
        for (unsigned y = 0; y < height; ++y)
        {
            for (unsigned x = 0; x < width; ++x)
            {
                if (y == 0)
                {
                    if (int(x / check_stride) % 2 == 0) {
                        rawShorts[x] = (unsigned short)0;
                    }
                    else {
                        rawShorts[x] = (unsigned short)maxValue;
                    }
                }
                else {
                    if (int(y / check_stride) % 2 == 0) {
                        rawShorts[x + y * width] = (unsigned short)rawShorts[x];
                    }
                    else {
                        rawShorts[x + y * width] = (unsigned short)(maxValue - rawShorts[x]);
                    }
                }
            }
        }
    }
    return true;
}

bool MH_camera::GenerateMHImage(ImgBuffer& img) {
    unsigned width = img.Width(), height = img.Height();
    char buf[MM::MaxStrLength];
    GetProperty(MM::g_Keyword_PixelType, buf);
    std::string pixelType(buf);
    int maxValue = 1 << GetBitDepth();

    return true;
}

bool MH_camera::GenerateMHHisto(ImgBuffer& img) {
    //Either display a bar chart with count rates or a decay curve
    unsigned width = img.Width(), height = img.Height();
    char buf[MM::MaxStrLength];
    GetProperty(MM::g_Keyword_PixelType, buf);
    std::string pixelType(buf);
    int maxValue = 1 << GetBitDepth();
    std::vector<int>old_bins;
    if (!bins_.empty()) {
        old_bins = bins_;
        //store values
    }
    else {

    }
    bins_.clear();
    //Represent everything as having the same number of bins as the width of the image
    int nbins = width;
    //Maximum displayable rate
    int max_rate = 500;

    //For chunks, e.g. one bar per actual channel in a bar chart...
    int n_channels = MAX_N_CHANNELS;

    if (rates_or_decays_) {//True is rates
        if (counts_.empty()) {//Initialise
            //We have an array of count rates equal in length to the image width - worry about chunking into bars later
            for (int i = 0; i < nbins; i++) {
                counts_.push_back(0);
            }
        }
    }

    double exposure_scaling_factor = (1000 / GetExposure());

    for (unsigned int i = 0; i < width; i++) {
        int which_ch = (int)((float)i * n_channels / (float)width);
        if (rates_or_decays_) {
            int threshold = (int)(live_rates[which_ch]*exposure_scaling_factor);
            bins_.push_back(threshold);
        } else {
            //int threshold = (int)((float)width * i / (float)height);
            int t = (int)((float)Lifetime_range_ * (float)i / (float)width);
            int threshold = (int)(height * exp((-1 * t) / (float)Sim_lifetime_));
            int noise = (int)((height / 10) * (double)rand() / (double)RAND_MAX);
            //threshold = threshold + noise;
            bins_.push_back(threshold + noise);
        }
    }

    if (pixelType.compare(g_PixelType_8bit) == 0)
    {
        maxValue = 255;
        unsigned char* rawShorts = (unsigned char*) const_cast<unsigned char*>(img.GetPixels());
        for (unsigned y = 0; y < height; ++y)
        {
            for (unsigned x = 0; x < width; ++x)
            {
                if (y > bins_[x]) {
                    rawShorts[x + y * width] = (unsigned char)0;
                }
                else {
                    rawShorts[x + y * width] = (unsigned char)maxValue;
                }
            }
        }
    }
    else if (pixelType.compare(g_PixelType_16bit) == 0)
    {
        maxValue = 65535;
        unsigned short* rawShorts = (unsigned short*) const_cast<unsigned char*>(img.GetPixels());
        for (unsigned y = 0; y < height; ++y)
        {
            for (unsigned x = 0; x < width; ++x)
            {
                if (y > bins_[x]) {
                    rawShorts[x + y * width] = (unsigned short)0;
                }
                else {
                    rawShorts[x + y * width] = (unsigned short)maxValue;
                }
            }
        }
    }
    return true;
}


void MH_camera::TestResourceLocking(const bool recurse)
{
    if (recurse)
        TestResourceLocking(false);
}


/**
* Generate an image with offset plus noise
*/

int MH_camera::RegisterImgManipulatorCallBack(ImgManipulator* imgManpl)
{
    imgManpl_ = imgManpl;
    return DEVICE_OK;
}


// Added bitsfrom MH Device Adapter
int MH_camera::On_Offset_Ch1(MM::PropertyBase* pProp, MM::ActionType eAct) {
    {
        long test_val = 1;
        if (eAct == MM::BeforeGet) {
            pProp->Set(offsets[0]);
            // nothing to do, let the caller use cached property
        }
        else if (eAct == MM::AfterSet)
        {
            char dummy[100];
            // Set timer for the Busy signal
            MH_changedTime_ = GetCurrentMMTime();
            if (Mode == MODE_T2) {
                sprintf(dummy, "Offset cannot be set in T2 mode!");
                msgstr = dummy;
                LogMessage(msgstr);
            }
            else {

                //No ints, just longs
                long val_toset;
                pProp->Get(offsets[0]);
                retcode = MH_SetOffset(dev[0], offsets[0]);
                if (retcode < 0)
                {
                    MH_GetErrorString(Errorstring, retcode);
                    sprintf(dummy, "MH_SetOffset error %d on channel %d (%s). Aborted.", retcode, 0, Errorstring);
                    msgstr = dummy;
                    LogMessage(msgstr);
                    return retcode;
                }
                else {
                    sprintf(dummy, "MH_SetOffset on channel %d set to %d ps.", 0, offsets[0]);
                    msgstr = dummy;
                    LogMessage(msgstr);
                }
            }
        }
        return DEVICE_OK;
    }
}

int MH_camera::On_Integ_t(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(Tacq);
    }
    else if (eAct == MM::AfterSet)
    {
        pProp->Get(Tacq);
    }
    return DEVICE_OK;
}

int MH_camera::On_MH_Status(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    //PROPERTY SETTING LOGIC NEEDS A CHECK OR 5...
    if (eAct == MM::BeforeGet)
    {
        if (MH_Status_ == 0)
        {
            pProp->Set("Idle");
        }
        else if (MH_Status_ == 1) {
            pProp->Set("Start");
        }
        else if (MH_Status_ == 2) {
            pProp->Set("Running");
        }
        else if (MH_Status_ == 3) {
            pProp->Set("Abort");
        }
        else {
            pProp->Set("Idle");//duplication for clarity
        }
        // nothing to do, let the caller use cached property
    }
    else if (eAct == MM::AfterSet)
    {
        LogMessage("AN ACTION WAS DONE");
        // Set timer for the Busy signal
        MH_changedTime_ = GetCurrentMMTime();
        std::string MH_Status_str;
        //Returns a std::string
        pProp->Get(MH_Status_str);
        if (MH_Status_str.compare("Idle") == 0)// 0 means a string matches
        {
            MH_Status_ = 0;
            LogMessage("Set Idle");
            pProp->Set("Idle");
        }
        else if (MH_Status_str.compare("Start") == 0) {
            MH_Status_ = 2;
            LogMessage("Set Start");
            pProp->Set("Running");
            start_acq();
        }
        else if (MH_Status_str.compare("Running") == 0) {
            LogMessage("Set Running");
            pProp->Set("Idle");
            MH_Status_ = 2;
        }
        else if (MH_Status_str.compare("Abort") == 0) {
            MH_Status_ = 0;
            LogMessage("Set Abort");
            pProp->Set("Idle");
        }
        else {//Idle
            LogMessage("Set Idle");
            MH_Status_ = 0;
        }
        int ret = OnPropertiesChanged();
        if (ret != DEVICE_OK)
        {
            return ret;
        }
    }
    return DEVICE_OK;
}


int MH_camera::On_Save_Enable(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    //PROPERTY SETTING LOGIC NEEDS A CHECK OR 5...
    if (eAct == MM::BeforeGet)
    {
        if (MH_Saving_ == 0)
        {
            pProp->Set("False");
            saving_ = false;
        }
        else if (MH_Saving_ == 1) {
            pProp->Set("True");
            saving_ = true;
        }
        else {
            pProp->Set("False");//duplication for clarity
            saving_ = false;
        }
        // nothing to do, let the caller use cached property
    }
    else if (eAct == MM::AfterSet)
    {
        LogMessage("AN ACTION WAS DONE");
        // Set timer for the Busy signal
        MH_changedTime_ = GetCurrentMMTime();
        std::string MH_Saving_str;
        //Returns a std::string
        pProp->Get(MH_Saving_str);
        if (MH_Saving_str.compare("False") == 0)// 0 means a string matches
        {
            MH_Saving_ = 0;
            LogMessage("Saving set to False");
            pProp->Set("False");
            saving_ = false;
        }
        else if (MH_Saving_str.compare("True") == 0) {
            MH_Saving_ = 1;
            LogMessage("Saving set to True");
            pProp->Set("True");
            saving_ = true;
            //start_acq();//do we want to do an acquisition every time we set saving to true?
        }
        else {//Idle
            LogMessage("Saving set to False");
            MH_Status_ = 0;
        }
        int ret = OnPropertiesChanged();
        if (ret != DEVICE_OK)
        {
            return ret;
        }
    }
    return DEVICE_OK;
}

int MH_camera::start_acq()
{
    if (n_frame_tracker_ % n_frame_repeats_ == 0) {//Allows the accumulation and reset of frame clock to both work?
        img_.ResetPixels();
    }
    //LogMessage("Ran start acq function");
    char dummy[100];
    //Reset trackers
    overflow_counter_ = 0;
    current_line_ = -1;
    frame_active_ = false;
    last_line_start_ = 0;
    last_line_end_ = 0;

    for (int i = 0; i < MAX_N_CHANNELS; i++) {
        live_rates[i] = 0;
    }

    //Processing stuff
    unsigned int chan_mask = 0x000003f;
    int chan_shift = 25;
    int tot_rec = 0;
    int acq_duration_ms = (int)GetExposure();

    retcode = MH_StartMeas(dev[0], acq_duration_ms);


    //original
    /*char fpathbuffer[180];

    time_t rawtime;
    struct tm* timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(fpathbuffer, sizeof(fpathbuffer), "%d-%m-%Y_%H-%M-%S", timeinfo);
    std::string fpath(fpathbuffer);
    fpath.append("_tttr.out");
    LogMessage(fpath.c_str());*/
    //till here


    // Billy code START
    char fpathbuffer[180];
    time_t rawtime;
    struct tm* timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(fpathbuffer, sizeof(fpathbuffer), "%d-%m-%Y_%H-%M-%S", timeinfo);
    
    // Step 3: Create the file path with a subfolder
    std::string subfolder = "tttr_raws/"; // Corrected: Removed leading slash for relative path
    std::string fpath(subfolder);  // Add subfolder to the path
    fpath.append(fpathbuffer);     // Append the formatted date and time
    fpath.append("_tttr.out");     // Append the file extension

    // Step 4: Ensure the subfolder exists (create it if necessary)
    const char* folderPath = "tttr_raws";
    _mkdir(folderPath);  // Windows-specific: create directory
    // Billy code END

    LogMessage(fpath.c_str()); // Log the file path



    int loopctr = 0;
    if (retcode < 0)
    {
        MH_GetErrorString(Errorstring, retcode);
        //printf("\nMH_StartMeas error %d (%s). Aborted.\n", retcode, Errorstring);
        sprintf(dummy, "Error at 546");
        msgstr = dummy;
        LogMessage(msgstr);
        goto fail;
    }
    //
    sprintf(dummy, "About to start while loop");
    msgstr = dummy;
    LogMessage(msgstr);


    if (saving_) {
        if ((fpout = fopen(fpath.c_str(), "wb")) == NULL)
        {
            sprintf(dummy, "Failed to open a file!");
            msgstr = dummy;
            LogMessage(msgstr);
            goto fail;
        }
    }
    
    while (1)
    {
        retcode = MH_GetFlags(dev[0], &flags);
        if (retcode < 0)
        {
            MH_GetErrorString(Errorstring, retcode);
            //printf("\nMH_GetFlags error %d (%s). Aborted.\n", retcode, Errorstring);
            sprintf(dummy, "Get Flags failed");
            msgstr = dummy;
            LogMessage(msgstr);
            goto fail;
        }

        if (flags & FLAG_FIFOFULL)
        {
            sprintf(dummy, "Flags and FIFOFull");
            msgstr = dummy;
            LogMessage(msgstr);
            goto stoptttr;
        }

        retcode = MH_ReadFiFo(dev[0], buffer, &nRecords);	//may return less!  
        if (retcode < 0)
        {
            MH_GetErrorString(Errorstring, retcode);
            //printf("\nMH_ReadFiFo error %d (%s). Aborted.\n", retcode, Errorstring);
            sprintf(dummy, "Read Fifo failed");
            msgstr = dummy;
            LogMessage(msgstr);
            goto stoptttr;
        }

        if (nRecords)
        {
            //TRY UPDATING RATES
            unsigned int* ptrToBuf = buffer;
            //https://wiki.sei.cmu.edu/confluence/display/c/EXP08-C.+Ensure+pointer+arithmetic+is+used+correctly
            for (int i = 0; i < nRecords; i++) {
                unsigned int record = ptrToBuf[i];
                unsigned int chan = (record >> chan_shift) & chan_mask;//shift 25 bits down to get record at the start
                TranslateRecord(record, img_);//for just the MH acquisition we don't need to do any translating... to
                if (chan < MAX_N_CHANNELS) {
                    live_rates[chan] += 1;
                }
            }
            //WRITE TO DISK
            //tot_rec += nRecords;
            if (saving_) {
                if (fwrite(buffer, 4, nRecords, fpout) != (unsigned)nRecords)//The 4 is bytes per element - https://cplusplus.com/reference/cstdio/fwrite/
                {
                    //printf("\nfile write error\n");
                    sprintf(dummy, "nRecords failed?");
                    msgstr = dummy;
                    LogMessage(msgstr);
                    goto stoptttr;
                }
            }
            Progress += nRecords;
            //printf("\b\b\b\b\b\b\b\b\b\b\b\b%12u", Progress);
            fflush(stdout);
        }

        else
        {
            retcode = MH_CTCStatus(dev[0], &ctcstatus);
            if (retcode < 0)
            {
                MH_GetErrorString(Errorstring, retcode);
                //printf("\nMH_CTCStatus error %d (%s). Aborted.\n", retcode, Errorstring);
                sprintf(dummy, "CTCstatus failed");
                msgstr = dummy;
                LogMessage(msgstr);
                goto fail;
            }
            if (ctcstatus)
            {
                //printf("\nDone\n");
                goto stoptttr;
            }
        }
        //Add other check code here e.g. look for abort set?

        loopctr++;

    }
    //within this loop you can also read the count rates if needed.
stoptttr:
    for (int i = 0; i < MAX_N_CHANNELS; i++) {
        sprintf(dummy, "Rate for channel %d: %d", i,live_rates[i]);
        msgstr = dummy;
        LogMessage(msgstr);        
    }
    sprintf(dummy, "Got to stoptttr");
    msgstr = dummy;
    LogMessage(msgstr);

    retcode = MH_StopMeas(dev[0]);
    if (retcode < 0)
    {
        MH_GetErrorString(Errorstring, retcode);
        sprintf(dummy, "Tried to stop measurement");
        msgstr = dummy;
        LogMessage(msgstr);
        //printf("\nMH_StopMeas error %d (%s). Aborted.\n", retcode, Errorstring);
        //goto ex;
    }

fail:
    //Shutdown();
    current_line_ = -99;
    sprintf(dummy, "Got to fail");
    msgstr = dummy;
    LogMessage(msgstr);

    //sprintf(dummy, "nRecords latest: %d and total: %d", nRecords, tot_rec);
    //msgstr = dummy;
    //LogMessage(msgstr);

    if (saving_) {
        if (fpout)
        {
            fclose(fpout);
        }
    }

    return DEVICE_OK;

}

////////// BEGINNING OF POORLY ORGANIZED CODE //////////////
//////////  CLEANUP NEEDED ////////////////////////////

int SocketGalvo::Initialize()
{
    if (initialized_) {
        return DEVICE_OK;
    }
    //>>>BEGIN  INSERT
    Scan_hub* pHub = static_cast<Scan_hub*>(GetParentHub());
    if (pHub)
    {
        int nRet = CreateStringProperty(g_PropName_ScanStatus, "Idle", false, new CPropertyAction(this, &SocketGalvo::On_SG_Status));
        AddAllowedValue(g_PropName_ScanStatus, "Idle");
        AddAllowedValue(g_PropName_ScanStatus, "Running");
        AddAllowedValue(g_PropName_ScanStatus, "Start");
        if (DEVICE_OK != nRet) {
            return nRet;
        }

        char hubLabel[MM::MaxStrLength];
        pHub->GetLabel(hubLabel);
        std::string camexample_str;
        char propval[MM::MaxStrLength];
        char propval2[MM::MaxStrLength];
        strncpy(propval2, "HA HA NEW VALUE!!!", MM::MaxStrLength);
        pHub->GetProperty(g_PROP_EXAMPLE_NAME, propval);
        pHub->CreateStringProperty("Hub EXAMPLE STRING PROPERTY FROM GALVO", propval, true);
        //pHub->SetProperty("Hub EXAMPLE STRING PROPERTY FROM GALVO", propval2, true);
        SetParentID(hubLabel); // for backward comp.
    }
    else
        LogMessage(NoHubError);
    //<<<END INSERT

    galvo_control_port_ = 54321;
    galvo_control_IP_address_ = "127.0.0.1";
    json_template_ = "{\"pixels_per_axisX\":p_p_a_X_value,\"microns_per_pixel\":m_p_p_value,\"time_per_image\":t_p_i_value,\"images\":n_im_value,\"flyback_fraction\":f_frac_value,\"magnification\":mag_value,\"scans_per_image\":s_p_i_value,\"pixels_per_axisY\":p_p_a_Y_value}";
    sg_command_template_ = "command1";
    SG_Status_ = 0;
    return DEVICE_OK;
}

// action interface
// ----------------
int SocketGalvo::OnSocketSend(MM::PropertyBase* pProp, MM::ActionType eAct) {
    
    //>>>REPLACE THESE TEMP VALUES ONCE CALLING THE PARENT HUB IS SORTED
    float m_p_p = 1.0;//MH_camera::GetPixelSizeUm();//BINNING?!
    float t_p_i = 1000;//MH_camera::GetExposure();//Time per image
    float flyback_fraction = 0.1;
    float magnification = 20;
    int n_scanPixels_X_ = 120;
    int n_scanPixels_Y_ = 80;
    int n_frame_repeats_ = 1;
    //<<<REPLACE
    std::string galvo_json_string = prep_json(json_template_, n_scanPixels_X_, n_scanPixels_Y_, m_p_p, t_p_i, 1, flyback_fraction, magnification, n_frame_repeats_);
    send_on_socket(galvo_json_string, galvo_control_IP_address_, galvo_control_port_);
    SG_changedTime_ = GetCurrentMMTime();
    return DEVICE_OK;//test
}

int SocketGalvo::On_SG_Status(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    //PROPERTY SETTING LOGIC NEEDS A CHECK OR 5...
    if (eAct == MM::BeforeGet)
    {
        if (SG_Status_ == 0)
        {
            pProp->Set("Idle");
        }
        else if (SG_Status_ == 1) {
            pProp->Set("Running");
        }
        else if (SG_Status_ == 2) {
            pProp->Set("Start");
        }
        else {
            pProp->Set("Idle");//duplication for clarity
        }
        // nothing to do, let the caller use cached property
    }
    else if (eAct == MM::AfterSet)
    {
        LogMessage("AN ACTION WAS DONE");
        // Set timer for the Busy signal
        SG_changedTime_ = GetCurrentMMTime();
        std::string SG_Status_str;
        //Returns a std::string
        pProp->Get(SG_Status_str);
        if (SG_Status_str.compare("Idle") == 0)// 0 means a string matches
        {
            LogMessage("Set Idle");
            pProp->Set("Idle");
        }
        else if (SG_Status_str.compare("Running") == 0) {
            LogMessage("Set Running");
            pProp->Set("Idle");
            SG_Status_ = 1;
            start_scan();
        }
        else if (SG_Status_str.compare("Start") == 0) {
            LogMessage("Set Start");
            pProp->Set("Running");
            SG_Status_ = 1;
        }
        else {//Idle
            LogMessage("Set Idle");
            SG_Status_ = 0;
        }
        int ret = OnPropertiesChanged();
        if (ret != DEVICE_OK)
        {
            return ret;
        }
    }
    return DEVICE_OK;
}

int SocketGalvo::OnMsgChange(MM::PropertyBase* pProp, MM::ActionType eAct) {
    send_on_socket(sg_command_template_, galvo_control_IP_address_, galvo_control_port_);
    return DEVICE_OK;
}


Scan_hub::Scan_hub() :
    HubBase<Scan_hub>(),
    hub_n_scanPixels_X_(120),
    hub_n_scanPixels_Y_(80)
{

}

int Scan_hub::Initialize()
{
    initialized_ = true;
    CPropertyAction* pAct = new CPropertyAction(this, &Scan_hub::Onn_hubscanPixels_X);
    CreateIntegerProperty(g_N_Hub_Scan_Px_X, 120, false, pAct);
    //SetPropertyLimits(g_N_Scan_Px_X, 1, 1024);
    pAct = new CPropertyAction(this, &Scan_hub::Onn_hubscanPixels_Y);
    CreateIntegerProperty(g_N_Hub_Scan_Px_Y, 80, false, pAct);
    //SetPropertyLimits(g_N_Scan_Px_Y, 1, 1024);
    return DEVICE_OK;
}

int Scan_hub::DetectInstalledDevices()
{
    ClearInstalledDevices();

    // make sure this method is called before we look for available devices
    InitializeModuleData();

    char hubName[MM::MaxStrLength];
    GetName(hubName); // this device name
    for (unsigned i = 0; i < GetNumberOfDevices(); i++)
    {
        char deviceName[MM::MaxStrLength];
        bool success = GetDeviceName(i, deviceName, MM::MaxStrLength);
        if (success && (strcmp(hubName, deviceName) != 0))
        {
            MM::Device* pDev = CreateDevice(deviceName);
            AddInstalledDevice(pDev);
        }
    }
    return DEVICE_OK;
}

void Scan_hub::GetName(char* pName) const
{
    CDeviceUtils::CopyLimitedString(pName, g_HubDeviceName);
}

int Scan_hub::Onn_hubscanPixels_X(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
       
    }
    else if (eAct == MM::AfterSet)
    {
       
    }
    return DEVICE_OK;
}

int Scan_hub::Onn_hubscanPixels_Y(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        
    }
    else if (eAct == MM::AfterSet)
    {
         
    }
    return DEVICE_OK;
}


///////////////////////////////////////////////////////////////////////////////////////////////
//Utility functions																			 //
///////////////////////////////////////////////////////////////////////////////////////////////

int SocketGalvo::send_on_socket(std::string message_string, std::string IP_address, int port_number) {
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
        std::cerr << "WSAStartup failed: " << result << std::endl;
    }
    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Error creating socket: " << WSAGetLastError() << std::endl;
        WSACleanup();
    }
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port_number);
    serverAddress.sin_addr.s_addr = inet_addr(IP_address.c_str());

    result = connect(sock, (sockaddr*)&serverAddress, sizeof(serverAddress));
    if (result == SOCKET_ERROR) {
        std::cerr << "Connection failed: " << WSAGetLastError() << std::endl;
        closesocket(sock);
        WSACleanup();
    }

    //FIX ZERO PADDING FOR 4 BYTES
    int cmd_len = message_string.size();
    std::string cmd_len_padded = std::to_string(cmd_len);
    unsigned int number_of_zeros = 4 - cmd_len_padded.length(); // pad with zeros until length is 4 (expected in LabVIEW)
    cmd_len_padded.insert(0, number_of_zeros, '0');
    message_string = cmd_len_padded + message_string + "\r\n";
    const char* message = message_string.c_str();

    result = send(sock, message, strlen(message), 0);
    if (result == SOCKET_ERROR) {
        std::cerr << "Send failed: " << WSAGetLastError() << std::endl;
        closesocket(sock);
        WSACleanup();
    }

    closesocket(sock);
    WSACleanup();
    return 0;
}

std::string SocketGalvo::replace_str(std::string source, std::string target, std::string replacement)
{
    //https://stackoverflow.com/questions/3418231/replace-part-of-a-string-with-another-string
    std::string destination = source;
    while (source.find(target) != string::npos) destination.replace(source.find(target), target.size(), replacement);
    return destination;
}

std::string SocketGalvo::prep_json(std::string json_template, int scan_pixels_per_axis_X, int scan_pixels_per_axis_Y, float microns_per_pixel, float time_per_image, int n_images, float flyback_fraction, float magnification, int scans_per_image)
{
    std::string json_output = json_template;
    //https://stackoverflow.com/questions/5590381/how-to-convert-int-to-string-in-c
    replace_str(json_output, "p_p_a_X_value", std::to_string(scan_pixels_per_axis_X));
    replace_str(json_output, "p_p_a_Y_value", std::to_string(scan_pixels_per_axis_Y));
    replace_str(json_output, "m_p_p_value", std::to_string(microns_per_pixel));
    replace_str(json_output, "t_p_i_value", std::to_string(time_per_image));
    replace_str(json_output, "n_im_value", std::to_string(n_images));
    replace_str(json_output, "f_frac_value", std::to_string(flyback_fraction));
    replace_str(json_output, "mag_value", std::to_string(magnification));
    replace_str(json_output, "s_p_i_value", std::to_string(scans_per_image));
    return json_output;
}

int SocketGalvo::start_scan()
{
    send_on_socket(sg_command_template_, galvo_control_IP_address_, galvo_control_port_);
    return DEVICE_OK;
}

std::string MH_camera::convertToString(char* a, boolean drop_last)
{
    int ca_size = sizeof(a) / sizeof(char);
    int i;
    string s = "";
    if (drop_last) {//WILL DROP TERMINATION CHARACTER FOR A NULL STRING TOO!
        ca_size -= 1;
    }
    for (i = 0; i < ca_size; i++) {
        s = s + a[i];
    }
    return s;
}

std::string MH_camera::formulate_message() {
    return "";
}

int MH_camera::dummyfunc() {
    //Need the MH:: !!!!
    return 0;
}