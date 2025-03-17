#include "MySLM.h"
#include "DeviceBase.h"  // Micro-Manager Device API
#include "ModuleInterface.h"  // Interface to create the device
#include "SLMFunc.h"  // The header provided by the SLM SDK

#define ERR_SLM_NOT_FOUND 10001
#define ERR_SLM_INIT_FAILED 10002
#define ERR_SLM_SET_WAVELENGTH_FAILED 10003
// Additional error codes as needed...


class SLMMicroscope : public CGenericBase<SLMMicroscope>
{
public:
    SLMMicroscope();
    ~SLMMicroscope();

    // Device API methods
    int Initialize();
    int Shutdown();

    void GetName(char* pszName) const;
    bool Busy();

    // Property handlers
    int OnWavelength(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnImage(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
    bool initialized_;
    DWORD slmNumber_;
    DWORD displayNumber_;
    // Additional variables...
};
