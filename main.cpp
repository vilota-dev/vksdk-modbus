/* ----------------------- Standard includes --------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <atomic>
#include <chrono>
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/*------------------------- SDK includes-------------------------------------*/
#include <memory>
#include <iostream>
#include <vk_sdk/Sdk.hpp>   
#include <optional>

/* ----------------------- Defines ------------------------------------------*/
#define PROG            "freemodbus"
#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 4
#define REG_HOLDING_START 2000
#define REG_HOLDING_NREGS 130

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];

static enum ThreadState
{
    STOPPED,
    RUNNING,
    SHUTDOWN
} ePollThreadState;

static pthread_mutex_t xLock = PTHREAD_MUTEX_INITIALIZER;
static BOOL     bDoExit;
std::atomic<int> detected(0);

enum WantHeartbeat {
    None,
    System,
    CameraDriver,
    Vio,
};
const int TIMEOUT_MILLI_SECONDS = 3000;

std::atomic<WantHeartbeat> want = WantHeartbeat::None;
std::atomic<uint32_t> want_index = WantHeartbeat::None;

std::mutex heartbeat_mutex;
std::condition_variable heartbeat_cv;
bool received_heartbeat = false;

/* ----------------------- Static functions ---------------------------------*/
static BOOL     bCreatePollingThread( void );
static enum ThreadState eGetPollingThreadState( void );
static void     vSetPollingThreadState( enum ThreadState eNewState );
static void    *pvPollingThread( void *pvParameter );
static void *heartbeatThread(void * params);

// Custom receiver for detections messages.
class DetectionsReceiver : public vkc::Receiver<vkc::Detections2d> {
    // Override this method to handle messages.
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Detections2d>>& message) override {
        auto all_detections = message.payload.reader();
        auto labels = all_detections.getLabels();
        detected = 0;
        for (const auto& detection : all_detections.getDetections()){
            std::string d = labels[detection.getLabelIdx()].cStr();
            if(d == "person"){
                std::cout <<"person detected" << std::endl;
                detected = 1;
            }
        }
        return vkc::ReceiverStatus::Open;
    }
};

class MessageReceiver : public vkc::Receiver<vkc::ManagerMessage> {
public:
    // Override this method to handle messages.
    vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::ManagerMessage>>& message) override {
        auto desired = want.load();
        auto index = want_index.load();

        auto heartbeat = message.payload.reader();
        auto variant = heartbeat.getVariant();

        std::lock_guard lock(heartbeat_mutex);

        if (received_heartbeat) {
            return vkc::ReceiverStatus::Closed;
        } else if (variant.isSystem() && desired == WantHeartbeat::System) {
            auto system = variant.getSystem().getVariant();

            if (system.isInfo()) {
                auto info = system.getInfo();
                std::cout << "Avg. Temperature: " << info.getTemperature().getAverageCelsius() << std::endl;
                std::cout << "Max. Temperature: " << info.getTemperature().getMaximumCelsius() << std::endl;
                std::cout << "RAM: " << info.getRam().getUsed() << " / " << info.getRam().getTotal() << std::endl;
                for (auto cpu = info.getCpu().begin(); cpu != info.getCpu().end(); ++cpu) {
                    std::cout << "Core Usage: " << cpu->getUsage() << "%" << std::endl;
                }
                received_heartbeat = true;
            }
        } else if (variant.isCameraDriver() && desired == WantHeartbeat::CameraDriver && variant.getCameraDriver().getTarget() == index) {
            auto camera = variant.getCameraDriver().getVariant();

            if (camera.isHeartbeat()) {
                /*
                auto heartbeat = camera.getHeartbeat();
                std::cout << "Status: " << static_cast<int>(heartbeat.getStatus()) << std::endl;
                std::cout << "VPU Temperature: " << heartbeat.getTemperature() << std::endl;
                std::cout << "Current Configuration: " << heartbeat.getConfiguration().cStr() << std::endl;
                */
                received_heartbeat = true;
            }
        } else if (variant.isVio() && desired == WantHeartbeat::Vio && variant.getVio().getTarget() == index) {
            auto vio = variant.getVio().getVariant();

            if (vio.isHeartbeat()) {
                auto heartbeat = vio.getHeartbeat();

                std::cout << "Status: " << static_cast<int>(heartbeat.getStatus()) << std::endl;
                std::cout << "Current Configuration: " << heartbeat.getConfiguration().cStr() << std::endl;
                received_heartbeat = true;
            }
        }

        if (received_heartbeat) {
            heartbeat_cv.notify_one();
            return vkc::ReceiverStatus::Closed;
        } else {
            return vkc::ReceiverStatus::Open;
        }

    }
};

/* ----------------------- MODBUS implementation -----------------------------*/
BOOL
bSetSignal( int iSignalNr, void ( *pSigHandler ) ( int ) )
{
    BOOL            bResult;
    struct sigaction xNewSig, xOldSig;

    xNewSig.sa_handler = pSigHandler;
    sigemptyset( &xNewSig.sa_mask );
    xNewSig.sa_flags = 0;
    if( sigaction( iSignalNr, &xNewSig, &xOldSig ) != 0 )
    {
        bResult = FALSE;
    }
    else
    {
        bResult = TRUE;
    }
    return bResult;
}

void
vSigShutdown( int xSigNr )
{
    switch ( xSigNr )
    {
    case SIGQUIT:
    case SIGINT:
        vSetPollingThreadState( SHUTDOWN );
        bDoExit = TRUE; 
    case SIGTERM:
        vSetPollingThreadState( SHUTDOWN );
        bDoExit = TRUE;
    }
}

int
main( int argc, char *argv[] )
{
    int             iExitCode;
    CHAR            cCh;
    std::string remote = "127.0.0.1";
        

    if(argc < 2 || argc > 2){
        std::cout << "Improper use see below for usage\n"
                << "./main <modbus_id> "
                << std::endl;
        return -1;
    }

    int modbus_id = atoi(argv[1]);

    const UCHAR     ucSlaveID[] = { 0xAA, 0xBB, 0xCC };
    if( !bSetSignal( SIGQUIT || SIGINT, vSigShutdown ) || !bSetSignal( SIGTERM, vSigShutdown ) )
    {
        fprintf( stderr, "%s: can't install signal handlers: %s!\n", PROG, strerror( errno ) );
        iExitCode = EXIT_FAILURE;
    }
    else if( eMBInit( MB_RTU, modbus_id, 4, 115200, MB_PAR_EVEN ) != MB_ENOERR )
    {
        fprintf( stderr, "%s: can't initialize modbus stack!\n", PROG );
        iExitCode = EXIT_FAILURE;
    }   
    else if( eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3 ) != MB_ENOERR )
    {
        fprintf( stderr, "%s: can't set slave id!\n", PROG );
        iExitCode = EXIT_FAILURE;
    }
    else
    {
        vSetPollingThreadState( STOPPED );
        //initalize sdk stuff 
        auto visualkit = vkc::VisualKit::create(remote);

        // Check that the object has been created successfully before proceeding.
        if (visualkit == nullptr) {
            std::cout << "Failed to create visualkit connection." << std::endl;
            return -1;
        }
        
        // Create custom receiver
        auto detectionReceiver = std::make_unique<DetectionsReceiver>();
        auto messageReceiver = std::make_unique<MessageReceiver>();
       // auto heartbeatReceiver = std::make_unique<HeartbeatReceiver>();

        // Install the receiver into the data source so that the receiver can receive messages.
        visualkit->source().install("S0/cama/yolo", std::move(detectionReceiver));
        auto messages = visualkit->source().install("ws/message", std::move(messageReceiver));
        
        // Start the data source so messages can be received by `myReceiver`.
        visualkit->source().start();
        pthread_t hThread;
        pthread_create(&hThread, NULL, heartbeatThread,NULL);
        
         if( bCreatePollingThread(  ) != TRUE )
                {
                    printf( "Can't start protocol stack! Already running?\n" );
                    
                }
        vkc::waitForCtrlCSignal();

        /* Release hardware resources. */
        ( void )eMBClose(  );
        visualkit->source().stop(false);
        iExitCode = EXIT_SUCCESS;
    }
    return iExitCode;
}

BOOL
bCreatePollingThread( void )
{
    BOOL            bResult;
    pthread_t       xThread;

    if( eGetPollingThreadState(  ) == STOPPED )
    {
        if( pthread_create( &xThread, NULL, pvPollingThread, NULL ) != 0 )
        {
            bResult = FALSE;
        }
        else
        {
            bResult = TRUE;
        }
    }
    else
    {
        bResult = FALSE;
    }

    return bResult;
}

void           *
pvPollingThread( void *pvParameter )
{
    vSetPollingThreadState( RUNNING );

    if( eMBEnable(  ) == MB_ENOERR )
    {
        do
        {
            if( eMBPoll(  ) != MB_ENOERR )
                break;

            ( void )pthread_mutex_lock( &xLock );    
            usRegInputBuf[0] = ( USHORT ) detected;
            received_heartbeat = false;
            ( void )pthread_mutex_unlock( &xLock );
        }
        while( eGetPollingThreadState(  ) != SHUTDOWN );
    }

    ( void )eMBDisable(  );

    vSetPollingThreadState( STOPPED );

    return 0;
}

enum ThreadState
eGetPollingThreadState(  )
{
    enum ThreadState eCurState;

    ( void )pthread_mutex_lock( &xLock );
    eCurState = ePollThreadState;
    ( void )pthread_mutex_unlock( &xLock );

    return eCurState;
}

void
vSetPollingThreadState( enum ThreadState eNewState )
{
    ( void )pthread_mutex_lock( &xLock );
    ePollThreadState = eNewState;
    ( void )pthread_mutex_unlock( &xLock );
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= usRegInputStart  + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}


void * heartbeatThread(void * params){
    while(1){
        want = WantHeartbeat::CameraDriver;
        std::unique_lock lock(heartbeat_mutex);
        if (!heartbeat_cv.wait_for(lock, std::chrono::milliseconds(TIMEOUT_MILLI_SECONDS), [&]{ return received_heartbeat; })) {
            std::cout << "Did not receive any heartbeat from camera " << "s0/cama" << "." << std::endl; 
            detected = 2;
        }
    }
}