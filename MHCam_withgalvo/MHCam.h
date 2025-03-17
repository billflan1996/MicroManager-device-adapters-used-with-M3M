#pragma once
#ifndef _WIN32
#define _stdcall
#endif

//For printing 64-bit unsigned ints
#define __STDC_FORMAT_MACROS 1
#include <inttypes.h>

//MM BITS
#include "../../MMDevice/MMDevice.h"
#include "../../MMDevice/DeviceBase.h"
#include "../../MMDevice/DeviceUtils.h"
#include "../../MMDevice/ModuleInterface.h"

//MULTIHARP BITS
#include "errorcodes.h"
#include "mhdefin.h"
#include "mhlib.h"

//DA BITS
#include <vector>

static const char* g_MHDeviceName = "MultiHarp";
static const char* g_Keyword_Ver = "MultiHarp library version";
static const char* g_PropName_Offset_Ch1 = "Channel 1 time offset [ps]";
static const char* g_PropName_Offset_Ch2 = "Channel 2 time offset [ps]";
static const char* g_PropName_Offset_Ch3 = "Channel 3 time offset [ps]";
static const char* g_PropName_Offset_Ch4 = "Channel 4 time offset [ps]";
static const char* g_PropName_Offset_Ch5 = "Channel 5 time offset [ps]";
static const char* g_PropName_Offset_Ch6 = "Channel 6 time offset [ps]";
static const char* g_PropName_MHStatus = "MultiHarp Status";
static const char* g_PropName_Saving = "MultiHarp Save enable";
static const char* g_Window_t = "Windowing time [ms]"; //~Still to be implemented
static const char* g_Max_rate = "Max rate to display [Hz]"; //~Still to be implemented
static const char* g_N_Scan_Px_X = "Number of scan points in X";
static const char* g_N_Scan_Px_Y = "Number of scan points in Y";
static const char* g_N_Beams_X = "Number of beams in array along X direction";
static const char* g_N_Beams_Y = "Number of beams in array along Y direction";
static const char* g_PropName_Socket_Msg_To_Send = "Message to send on socket";
static const char* g_Keyword_Socket_State = "Socket state";
static const char* g_N_Hub_Scan_Px_X = "Number of hub scan points in X";
static const char* g_N_Hub_Scan_Px_Y = "Number of hub scan points in Y";
static const char* g_PropName_ScanStatus = "Scanner status";

#define command_wait_time				100

/////////////////////////////////////////////
// Predefined constants
/////////////////////////////////////////////

#define MAX_OFFSET_PS					10000
#define MIN_INTEG_MS					1000
#define MAX_INTEG_MS					100000
#define MAX_N_CHANNELS                  8

unsigned int buffer[TTREADMAX];

///////////////////////////////////////////////////////////////////////////////
// FILE:          MH_Cam.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   MODIFIED the example implementation of the demo camera.
//                Simulates generic digital camera and associated automated
//                microscope devices and enables testing of the rest of the
//                system without the need to connect to the actual hardware. 
//                
// AUTHOR:        Sunil Kumar, 04/10/2022, building on work by:
//                Nenad Amodaj, nenad@amodaj.com, [06/08/2005] 
//                Karl Hoover (stuff such as programmable CCD size  & the various image processors)
//                Arther Edelstein ( equipment error simulation)
//
// COPYRIGHT:     Imperial College London 2022
//                University of California, San Francisco, 2006-2015
//                100X Imaging Inc, 2008
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

#ifndef _DEMOCAMERA_H_
#define _DEMOCAMERA_H_

#include "DeviceBase.h"
#include "ImgBuffer.h"
#include "DeviceThreads.h"
#include <string>
#include <map>
#include <algorithm>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////
// Error codes
//
#define ERR_UNKNOWN_MODE         102
#define ERR_UNKNOWN_POSITION     103
#define ERR_IN_SEQUENCE          104
#define ERR_SEQUENCE_INACTIVE    105
#define ERR_STAGE_MOVING         106
#define HUB_NOT_AVAILABLE        107

const char* NoHubError = "Parent Hub not defined.";

class ImgManipulator
{
public:
    virtual int ChangePixels(ImgBuffer& img) = 0;
};

////////////////////////
// DemoHub
//////////////////////

class Scan_hub : public HubBase<Scan_hub>
{
public:
    Scan_hub();
    ~Scan_hub() {}

    // Device API
    // ---------
    int Initialize();
    int Shutdown() { return DEVICE_OK; };
    void GetName(char* pName) const;
    bool Busy() { return busy_; };

    // HUB api
    int DetectInstalledDevices();

    //HUB action interface
    int Onn_hubscanPixels_X(MM::PropertyBase*, MM::ActionType);
    int Onn_hubscanPixels_Y(MM::PropertyBase*, MM::ActionType);

private:
    void GetPeripheralInventory();
    long hub_n_scanPixels_X_;
    long hub_n_scanPixels_Y_;

    std::vector<std::string> peripherals_;
    bool initialized_;
    bool busy_;
};


//////////////////////////////////////////////////////////////////////////////
// MH_camera class
// Simulation of the Camera device
//////////////////////////////////////////////////////////////////////////////

class MySequenceThread;

class MH_camera : public CCameraBase<MH_camera>
{
public:
    MH_camera();
    ~MH_camera();

    // MMDevice API
    // ------------
    int Initialize();
    int Shutdown();

    void GetName(char* name) const;

    // MMCamera API
    // ------------
    int SnapImage();
    const unsigned char* GetImageBuffer();
    unsigned GetImageWidth() const;
    unsigned GetImageHeight() const;
    unsigned GetImageBytesPerPixel() const;
    unsigned GetBitDepth() const;
    long GetImageBufferSize() const;
    double GetExposure() const;
    void SetExposure(double exp);
    int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize);
    int GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize);
    int ClearROI();
    bool SupportsMultiROI();
    bool IsMultiROISet();
    int GetMultiROICount(unsigned& count);
    int SetMultiROI(const unsigned* xs, const unsigned* ys,
        const unsigned* widths, const unsigned* heights,
        unsigned numROIs);
    int GetMultiROI(unsigned* xs, unsigned* ys, unsigned* widths,
        unsigned* heights, unsigned* length);
    int PrepareSequenceAcqusition() { return DEVICE_OK; }
    int StartSequenceAcquisition(double interval);
    int StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow);
    int StopSequenceAcquisition();
    int InsertImage();
    int RunSequenceOnThread(MM::MMTime startTime);
    bool IsCapturing();
    void OnThreadExiting() throw();
    double GetNominalPixelSizeUm() const { return nominalPixelSizeUm_; }
    double GetPixelSizeUm() const { return nominalPixelSizeUm_ * GetBinning(); }
    int GetBinning() const;
    int SetBinning(int bS);

    int IsExposureSequenceable(bool& isSequenceable) const;
    int GetExposureSequenceMaxLength(long& nrEvents) const;
    int StartExposureSequence();
    int StopExposureSequence();
    int ClearExposureSequence();
    int AddToExposureSequence(double exposureTime_ms);
    int SendExposureSequence() const;

    unsigned  GetNumberOfComponents() const { return nComponents_; };

    // action interface
    // ----------------
    int OnMaxExposure(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnTestProperty(MM::PropertyBase* pProp, MM::ActionType eAct, long);
    int OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnPixelType(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnBitDepth(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnReadoutTime(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnScanMode(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnErrorSimulation(MM::PropertyBase*, MM::ActionType);
    
    
    int Onn_scanPixels_X(MM::PropertyBase*, MM::ActionType);
    int Onn_scanPixels_Y(MM::PropertyBase*, MM::ActionType);
    int Onn_beams_X(MM::PropertyBase*, MM::ActionType);
    int Onn_beams_Y(MM::PropertyBase*, MM::ActionType);
    

    int OnCameraCCDXSize(MM::PropertyBase*, MM::ActionType);
    int OnCameraCCDYSize(MM::PropertyBase*, MM::ActionType);
    int OnTriggerDevice(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnDropPixels(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnFastImage(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnSaturatePixels(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnFractionOfPixelsToDropOrSaturate(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnShouldRotateImages(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnShouldDisplayImageNumber(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnStripeWidth(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnSupportsMultiROI(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnMultiROIFillValue(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnCCDTemp(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnIsSequenceable(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnMode(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnPCF(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnPhotonFlux(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnReadNoise(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnCrash(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnLifetime(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnDecOrRat(MM::PropertyBase* pProp, MM::ActionType eAct);
    //From MH device adapter

    int On_Offset_Ch1(MM::PropertyBase* pProp, MM::ActionType eAct);
    int On_MH_Status(MM::PropertyBase* pProp, MM::ActionType eAct);
    int On_Integ_t(MM::PropertyBase* pProp, MM::ActionType eAct);

    //Things we added
    int On_Save_Enable(MM::PropertyBase* pProp, MM::ActionType eAct);

    // Special public DemoCamera methods
    int RegisterImgManipulatorCallBack(ImgManipulator* imgManpl);
    long GetCCDXSize() { return cameraCCDXSize_; }
    long GetCCDYSize() { return cameraCCDYSize_; }


private:
    int SetAllowedBinning();
    void TestResourceLocking(const bool);
    void Interpret_TTTR(unsigned int& record, unsigned int& sync, unsigned int& tcspc, unsigned int& channel, bool& special);
    void GenerateEmptyImage(ImgBuffer& img);
    void GenerateSyntheticImage(ImgBuffer& img, double exp);
    void AcquireMHImage(ImgBuffer& img, double exp);
    bool GenerateMHTestPattern(ImgBuffer& img);
    bool GenerateMHHisto(ImgBuffer& img);
    bool GenerateMHImage(ImgBuffer& img);
    int ResizeImageBuffer();
    void GenerateDecay(ImgBuffer& img);
    uint64_t TimestampDeltaToPs(uint64_t timestamp_delta);
    void TranslateRecord(unsigned int val, ImgBuffer& img);
    int GetPixnumInLine(uint64_t timestamp, uint64_t linestart_timestamp);
    //Utility
    std::string format_JSON_for_galvo();

    static const double nominalPixelSizeUm_;

    double exposureMaximum_;
    double dPhase_;
    ImgBuffer img_;
    bool busy_;
    bool stopOnOverFlow_;
    bool initialized_;
    double readoutUs_;
    MM::MMTime readoutStartTime_;
    long scanMode_;
    int bitDepth_;
    unsigned roiX_;
    unsigned roiY_;
    MM::MMTime sequenceStartTime_;
    bool isSequenceable_;
    long sequenceMaxLength_;
    bool sequenceRunning_;
    unsigned long sequenceIndex_;
    double GetSequenceExposure();
    std::vector<double> exposureSequence_;
    long imageCounter_;
    long binSize_;
    long n_scanPixels_X_;
    long n_scanPixels_Y_;
    long n_beams_X_;
    long n_beams_Y_;
    long cameraCCDXSize_;
    long cameraCCDYSize_;
    double ccdT_;
    std::string triggerDevice_;

    bool stopOnOverflow_;

    bool dropPixels_;
    bool fastImage_;
    bool saturatePixels_;
    double fractionOfPixelsToDropOrSaturate_;
    bool shouldRotateImages_;
    bool shouldDisplayImageNumber_;
    double stripeWidth_;
    bool supportsMultiROI_;
    int multiROIFillValue_;
    std::vector<unsigned> multiROIXs_;
    std::vector<unsigned> multiROIYs_;
    std::vector<unsigned> multiROIWidths_;
    std::vector<unsigned> multiROIHeights_;
    std::vector<int> bins_;
    std::vector<int> counts_;

    double testProperty_[10];
    MMThreadLock imgPixelsLock_;
    friend class MySequenceThread;
    int nComponents_;
    MySequenceThread* thd_;
    int mode_;
    ImgManipulator* imgManpl_;
    double pcf_;
    double photonFlux_;
    double readNoise_;
    long Sim_lifetime_;
    long Lifetime_range_;
    bool rates_or_decays_;
    unsigned int special_mask_;
    unsigned int channel_mask_;
    unsigned int time_mask_;
    unsigned int nsync_mask_;
    uint64_t pixel_dwelltime_ps_;
    uint64_t MeasDesc_GlobalResolution_;
    uint64_t last_line_start_;
    uint64_t last_line_end_;
    int current_line_;
    int n_line_repeats_;
    int n_frame_repeats_;
    int n_frame_tracker_;
    int nBeams;
    unsigned int overflow_counter_;
    bool saving_;
    bool frame_active_;

    //From MH Device Adapter
    MM::MMTime MH_changedTime_;
    std::string msgstr;
    std::string tmpstr;
    std::string convertToString(char* a, boolean drop_last);
    std::string formulate_message();
    int dummyfunc();

    //Items from the PicoQuant tttrmode.c demo - some comments modified for my purposes
    int dev[MAXDEVNUM];
    int found = 0;
    FILE* fpout;
    int retcode;
    int ctcstatus;
    char LIB_Version[8];
    char HW_Model[32];
    char HW_Partno[8];
    char HW_Serial[9];
    char HW_Version[16];
    char Errorstring[40];
    int NumChannels;
    int Mode = MODE_T3; //set T2 or T3 here, observe suitable Sync divider and Range!
    int Binning = 0; //can change this, meaningful only in T3 mode
    int Offset = 0;  //can change this, meaningful only in T3 mode
    double Tacq = 10000; //Measurement time in millisec, can change this
    int SyncDivider = 2; //can change this, observe Mode! READ MANUAL!

    int SyncTiggerEdge = 0; //can change this
    int SyncTriggerLevel = 30; //can change this
    int InputTriggerEdge = 0; //can change this
    int InputTriggerLevel = -200; //can change this
    int hardcoded_init_offsets[8] = { 8060,7930,7180,7150,7990,7860,2000,0 };
    double Resolution;
    int Syncrate;
    int Countrate;
    //int i;
    int flags;
    int warnings;
    char warningstext[16384]; //must have 16384 byte text buffer
    int nRecords;
    unsigned int Progress;
    unsigned int live_rates[MAX_N_CHANNELS];

    int start_acq();
    //int On_Offset_General(MM::PropertyBase* pProp, MM::ActionType eAct, int which_channel);
    std::vector<long> offsets;
    int MH_Status_;
    int MH_Saving_;
};

class MySequenceThread : public MMDeviceThreadBase
{
    friend class MH_camera;
    enum { default_numImages = 1, default_intervalMS = 100 };
public:
    MySequenceThread(MH_camera* pCam);
    ~MySequenceThread();
    void Stop();
    void Start(long numImages, double intervalMs);
    bool IsStopped();
    void Suspend();
    bool IsSuspended();
    void Resume();
    double GetIntervalMs() { return intervalMs_; }
    void SetLength(long images) { numImages_ = images; }
    long GetLength() const { return numImages_; }
    long GetImageCounter() { return imageCounter_; }
    MM::MMTime GetStartTime() { return startTime_; }
    MM::MMTime GetActualDuration() { return actualDuration_; }
private:
    int svc(void) throw();
    double intervalMs_;
    long numImages_;
    long imageCounter_;
    bool stop_;
    bool suspend_;
    MH_camera* camera_;
    MM::MMTime startTime_;
    MM::MMTime actualDuration_;
    MM::MMTime lastFrameTime_;
    MMThreadLock stopLock_;
    MMThreadLock suspendLock_;
};

//////////////////////////////////////////////////////////////////////////////
// SocketGalvo class
// Tries to talk to a galvo via a socket mostly using JSON strings
// S.K.
//////////////////////////////////////////////////////////////////////////////
class SocketGalvo : public CGenericBase<SocketGalvo>
{
public:
    SocketGalvo() : busy_(false) {
        //
    }

    ~SocketGalvo() {
        //
    }

    int Shutdown() {
        return DEVICE_OK; 
    }

    void GetName(char* name) const {
        strcpy(name, "Socket Galvo"); 
    }

    int Initialize();
    
    bool Busy(void) { 
        return busy_; 

    };

    // action interface
    // ----------------
    int On_SG_Status(MM::PropertyBase* pProp, MM::ActionType eAct);
    //For socket comms
    int OnSocketSend(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnMsgChange(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
    bool busy_;
    bool initialized_;
    int galvo_control_port_;
    std::string galvo_control_IP_address_;
    std::string json_template_;
    std::string sg_command_template_;
    int SG_Status_;
    MM::MMTime SG_changedTime_;
    int start_scan();
    std::string replace_str(std::string source, std::string target, std::string replacement);
    std::string prep_json(std::string json_template, int scan_pixels_per_axis_X, int scan_pixels_per_axis_Y, float microns_per_pixel, float time_per_image, int n_images, float flyback_fraction, float magnification, int scans_per_image);
    int send_on_socket(std::string message_string, std::string IP_address, int port_number);
};

#endif //_DEMOCAMERA_H_