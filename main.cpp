#include <iostream>
#include <vector>
#include <map>
#include <string>

// xsense sdk
#include <xscommon.h>   // exclude "xscommon/common_qdebug.h"
#include <xscontroller.h>
#include <xsensdot_pc_sdk.h>
#include <xstypes.h>

// Matlab lib
#include "MatlabEngine.hpp"
#include "MatlabDataArray.hpp"

#include "./include/settings.h"
#include "./include/util.h"
#include "./include/simple_skeleton_model.h"

std::unique_ptr<matlab::engine::MATLABEngine> matlabPtr = matlab::engine::startMATLAB();
matlab::data::Array plotHandle;
matlab::data::ArrayFactory factory;

// TODO: replace it with faster plotting libaraies (QT?)
inline void plotInMatlab(){
    std::vector<ARCS2::Vector3> pts =  ARCS2::model1.getCoordinates(ARCS2::model1.c1);
    
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    
    for(auto & pt: pts){
        x.push_back(pt.x);
        y.push_back(pt.y);
        z.push_back(pt.z);
    }

    // data for matlab
    matlab::data::TypedArray<double> mArray_x = factory.createArray({x.size(),1},x.begin(),x.end());
    matlab::data::TypedArray<double> mArray_y = factory.createArray({y.size(),1},y.begin(),y.end());
    matlab::data::TypedArray<double> mArray_z = factory.createArray({z.size(),1},z.begin(),z.end());

    matlabPtr->setProperty(plotHandle,u"XData", mArray_y);
    matlabPtr->setProperty(plotHandle,u"YData", mArray_z);
    
    //pts[0].print();
    //pts[1].print();
    //pts[2].print();
    //printf("\r");
    //matlabPtr->eval(u"drawnow");

};

xsens::Mutex m_mutex;


class myCallBackHandler : public XsDotCallback{
    public:
        XsPortInfoArray portInfoList = XsPortInfoArray();
    protected:

        void onAdvertisementFound(const XsPortInfo *portInfo){
            std::string ble_addr = portInfo->bluetoothAddress().toStdString();
            std::cout<<"Found "<<ble_addr<<"\n";
            xsens::Lock locky(&m_mutex);
            portInfoList.push_back(*portInfo);
        }

        void onLiveDataAvailable(XsDotDevice* device, const XsDataPacket* packet){
            xsens::Lock locky(&m_mutex);
            std::string ble_addr  = device->portInfo().bluetoothAddress().toStdString(); // get bluetooth address in string
            auto it = ARCS2::model1.sensor_packets.find(ble_addr); // sensor_packets is a std::map<std::string, XsDataPacket>
            if(it!=ARCS2::model1.sensor_packets.end()){
                it->second = *packet;       // save the packet
                //printf("%ld: %s: %ld\n",XsTime::timeStampNow(),device->portInfo().bluetoothAddress().c_str(), packet->packetId());
            }
        }
};

int main(int argc, char** argv) {


    // setup connection manager
    XsDotConnectionManager *connectionManager = XsDotConnectionManager::construct();
    myCallBackHandler *cb = new myCallBackHandler();
    connectionManager->addXsDotCallbackHandler(cb);

    // setup bluetooth adpater
    printf("Avaiable bluetooth adapters:\n");
    for (auto &adapter: connectionManager->getAvailableBluetoothAdapters()){
        printf("%s\n", adapter.c_str());
    }

    if(!connectionManager->setPreferredBluetoothAdapter("hci1")){
        printf("Failed to connect to adapter hci1.\n");
        return -1;
    }
        

    // scanning
    bool continueScanning = true;
    bool timeout = false;
    std::cout<<"Start scanning...\n";
    connectionManager->enableDeviceDetection();
    int64_t startTime = XsTime::timeStampNow();
    
    while(continueScanning && !timeout){
        timeout = (XsTime::timeStampNow()-startTime >10000);
    }
    std::cout<<"Stop scanning...\n";
    connectionManager->disableDeviceDetection();

    // connect
    for (auto &it :cb->portInfoList){
        std::string ble_addr = it.bluetoothAddress().toStdString();
        //std::cout<<ble_addr<<std::endl;
        auto it2 = ARCS2::model1.sensor_packets.find(ble_addr); // find if the sensor (address) is registered
        if(it2!= ARCS2::model1.sensor_packets.end()){
            if(!(connectionManager->openPort(it))){
                std::cout<<connectionManager->lastResultText()<<std::endl;
                return -1;
            }else{
                std::cout<<"connected "<<ble_addr<<std::endl;
                ARCS2::model1.sensors[ble_addr]=connectionManager->device(it.deviceId()); // save the sensor device ptr to the model
            }
        } 
    }

    //set data output rate
    for (auto &s: ARCS2::model1.sensors){
        if(s.second!=nullptr){
            if(!(s.second->setOutputRate(ODR))){
                std::cout<<connectionManager->lastResultText()<<std::endl;
                return -2;
            }else{
                printf("Set ODR of %s to %d Hz\n",s.first.c_str(),ODR);
            }
        }
    }
    
    //start streaming data
    for (auto &s: ARCS2::model1.sensors){
        if(s.second!=nullptr){
            if (!(s.second->startMeasurement(XsPayloadMode::CompleteEuler))){
                std::cout<<connectionManager->lastResultText()<<std::endl;
                return -3;
            }
        }
    }
    
    // calibration
    // char input;
    // std::cout<<"Press 'c' to calibrate."<<std::endl;
    // while(std::cin>>input){
    //     if(input=='c'){
    //         std::cout<<"Calibration will start in 5 sec."<<std::endl;
    //         XsTime::msleep(5000);
    //         std::cout<<"Calibration started."<<std::endl;
    //         ARCS2::model1.calibration();
    //         std::cout<<"Calibration finished."<<std::endl;
    //         break;
    //     }      
    // }

    // real-time visuilization
    matlabPtr->eval(u"figure;");
    matlabPtr->eval(u"axis equal;");
    matlabPtr->eval(u"xlim([-2,2]);");
    matlabPtr->eval(u"ylim([-2,2]);");
    matlabPtr->eval(u"grid on;");
    matlabPtr->eval(u"hold on;");
    matlab::data::Array yData = factory.createArray<double>({ 1, 1 }, {0.0});
    plotHandle = matlabPtr->feval(u"plot",yData);
    matlab::data::CharArray marker = factory.createCharArray("o");
    matlabPtr->setProperty(plotHandle,u"Marker", marker);


    std::cout<<"streaming data...\n";
    int64_t t = XsTime::timeStampNow();
    int64_t time_next = XsTime::timeStampNow()+dt;
    while(1){
        // process data every dt ms
        t = XsTime::timeStampNow();
        int64_t waitTime = time_next-t;
        time_next = t+dt;
        //printf("%ld\n",waitTime);
        if(waitTime>0) XsTime::msleep(waitTime);

        ARCS2::model1.computeTransformation_S2E(); 
        plotInMatlab();
        //ARCS2::model1.printPackets();
        //printf("\r");
    
        //fflush(stdout);
    }
    
    for (auto s: ARCS2::model1.sensors){
        if(s.second!=nullptr) s.second->stopMeasurement();
    }

    connectionManager->close();
    std::cout<<"All devices disconnected.\n";
    connectionManager->destruct();
    
    return 0;
};