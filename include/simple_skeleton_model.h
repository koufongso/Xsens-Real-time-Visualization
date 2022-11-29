#ifndef SIMPLE_SKELETON_MODEL_H
#define SIMPLE_SKELETON_MODEL_H 1

namespace ARCS2{

    #include "util.h"
    #include <vector>
    #include <string.h>

    // body parameters [m]
    #define L_FOREAMR       0.254
    #define L_ARM           0.254
    #define L_THIGH         0.457
    #define L_LEG           0.406
    #define L_SHOULDER      0.216
    #define L_BODY_U        0.254
    #define L_BODY_L        0.178  
    #define L_HIPS          0.127

    // DOT sensor address and the corresponding body parts (sensor registration)
    #define ADDR_LEFT_ARM       "D4:22:CD:00:56:7B"
    #define ADDR_LEFT_FARM      "D4:22:CD:00:56:9B"
    #define ADDR_LEFT_THIGH     "D4:22:CD:00:55:C4"
    #define ADDR_LEFT_LEG       "D4:22:CD:00:55:6E"

    #define ADDR_RIGHT_ARM      "D4:22:CD:00:56:88"
    #define ADDR_RIGHT_FARM     "D4:22:CD:00:56:85"
    #define ADDR_RIGHT_THIGH    "D4:22:CD:00:55:C3"
    #define ADDR_RIGHT_LEG      "D4:22:CD:00:56:2F"

    #define ADDR_BODY_CORE      "D4:22:CD:00:56:85"


    // assume sensor frame is attach to the body frame
    const ARCS2::Vector3 lfa = ARCS2::Vector3(-L_FOREAMR,0,0);      // forearm
    const ARCS2::Vector3 la = ARCS2::Vector3(-L_ARM,0,0);           // arm
    const ARCS2::Vector3 lt = ARCS2::Vector3(-L_THIGH,0,0);         // thigh
    const ARCS2::Vector3 ll = ARCS2::Vector3(-L_LEG,0,0);           // leg

    const ARCS2::Vector3 rfa = ARCS2::Vector3(-L_FOREAMR,0,0);      // forearm
    const ARCS2::Vector3 ra = ARCS2::Vector3(-L_ARM,0,0);           // arm
    const ARCS2::Vector3 rt = ARCS2::Vector3(-L_THIGH,0,0);         // thigh
    const ARCS2::Vector3 rl = ARCS2::Vector3(-L_LEG,0,0);           // leg

    const ARCS2::Vector3 b1 = ARCS2::Vector3(0,-L_SHOULDER,0);      // body core, to the right shoulder
    const ARCS2::Vector3 b2 = ARCS2::Vector3(0,L_SHOULDER,0);       // body core, to the left shoulder
    const ARCS2::Vector3 b3 = ARCS2::Vector3(L_BODY_U,0,0);         // body core, to the upper part
    const ARCS2::Vector3 b4 = ARCS2::Vector3(-L_BODY_L,0,0);        // body core, to the lower part
    const ARCS2::Vector3 b5 = ARCS2::Vector3(0,-L_HIPS,0);          // body core, 
    const ARCS2::Vector3 b6 = ARCS2::Vector3(0,L_HIPS,0);           // body core


    
    // model
    struct simple_skeleton_model
    {
        simple_skeleton_model(){
            c1.base = ARCS2::Vector3(0,0,0);
            //c1.links.push_back(std::pair<ARCS2::Vector3,ARCS2::TransMatrices>(b3,TransMatrices(&(sensor_transformation[ADDR_BODY_CORE]),&(sensor_transformation_calib[ADDR_BODY_CORE]))));
            //c1.links.push_back(std::pair<ARCS2::Vector3,ARCS2::TransMatrices>(b2,TransMatrices(&(sensor_transformation[ADDR_BODY_CORE]),&(sensor_transformation_calib[ADDR_BODY_CORE]))));
            c1.links.push_back(std::pair<ARCS2::Vector3,ARCS2::TransMatrices>(la,TransMatrices(&(sensor_transformation[ADDR_LEFT_ARM]),&(sensor_transformation_calib[ADDR_LEFT_ARM]))));
            c1.links.push_back(std::pair<ARCS2::Vector3,ARCS2::TransMatrices>(lfa,TransMatrices(&(sensor_transformation[ADDR_LEFT_FARM]),&(sensor_transformation_calib[ADDR_LEFT_FARM]))));
        }

        ARCS2::OpenChain c1 = ARCS2::OpenChain();
        std::map<std::string, XsDotDevice*> sensors = {
            {ADDR_LEFT_ARM,nullptr},
            {ADDR_LEFT_FARM,nullptr},
            {ADDR_LEFT_THIGH,nullptr},
            {ADDR_LEFT_LEG,nullptr},

            {ADDR_RIGHT_ARM,nullptr},
            {ADDR_RIGHT_FARM,nullptr},
            {ADDR_RIGHT_THIGH,nullptr},
            {ADDR_RIGHT_LEG,nullptr},

            {ADDR_BODY_CORE,nullptr},
        };

        std::map<std::string, XsDataPacket> sensor_packets = {
            {ADDR_LEFT_ARM,XsDataPacket(nullptr)},
            {ADDR_LEFT_FARM,XsDataPacket(nullptr)},
            {ADDR_LEFT_THIGH,XsDataPacket(nullptr)},
            {ADDR_LEFT_LEG,XsDataPacket(nullptr)},

            {ADDR_RIGHT_ARM,XsDataPacket(nullptr)},
            {ADDR_RIGHT_FARM,XsDataPacket(nullptr)},
            {ADDR_RIGHT_THIGH,XsDataPacket(nullptr)},
            {ADDR_RIGHT_LEG,XsDataPacket(nullptr)},

            {ADDR_BODY_CORE,XsDataPacket(nullptr)},
        };

        std::map<std::string, ARCS2::Matrix3> sensor_transformation = {
            {ADDR_LEFT_ARM,ARCS2::Matrix3()},
            {ADDR_LEFT_FARM,ARCS2::Matrix3()},
            {ADDR_LEFT_THIGH,ARCS2::Matrix3()},
            {ADDR_LEFT_LEG,ARCS2::Matrix3()},

            {ADDR_RIGHT_ARM,ARCS2::Matrix3()},
            {ADDR_RIGHT_FARM,ARCS2::Matrix3()},
            {ADDR_RIGHT_THIGH,ARCS2::Matrix3()},
            {ADDR_RIGHT_LEG,ARCS2::Matrix3()},

            {ADDR_BODY_CORE,ARCS2::Matrix3()},
        };

        std::map<std::string, ARCS2::Matrix3> sensor_transformation_calib = {
            {ADDR_LEFT_ARM,ARCS2::Matrix3()},
            {ADDR_LEFT_FARM,ARCS2::Matrix3()},
            {ADDR_LEFT_THIGH,ARCS2::Matrix3()},
            {ADDR_LEFT_LEG,ARCS2::Matrix3()},

            {ADDR_RIGHT_ARM,ARCS2::Matrix3()},
            {ADDR_RIGHT_FARM,ARCS2::Matrix3()},
            {ADDR_RIGHT_THIGH,ARCS2::Matrix3()},
            {ADDR_RIGHT_LEG,ARCS2::Matrix3()},

            {ADDR_BODY_CORE,ARCS2::Matrix3()},   
        };

        //compute the transform matrix from sensor euler angle (sensor coord to earth coord)
        void computeTransformation_S2E(){
            for (auto &it: sensor_packets){
                    XsEuler euler =  it.second.orientationEuler();
                    sensor_transformation[it.first] = ARCS2::T_ZYX_d(euler.yaw(),euler.pitch(),euler.roll());
            }
        }

        // return the coordinate from a open chain w.r.t. the base 
        // the observation model
        std::vector<Vector3> getCoordinates(const OpenChain &c){
            std::vector<Vector3> out;
            out.push_back(c.base);  // add base coordinates
            Vector3 prev_vect = Vector3(0,0,0);
            for (auto &it :c.links){
                    prev_vect = (*(it.second.T_S2E))*(*(it.second.T_B2S))*(it.first)+prev_vect;
                    out.push_back(prev_vect-c.base);
            };
            return out;
        }

        inline void printPackets(){
            for (auto &it: sensor_packets){
                XsEuler euler =  it.second.orientationEuler();
                printf("%.2f, %.2f, %.2f ",euler.yaw(),euler.pitch(),euler.roll());
            }
        }

        inline void calibration(){
            computeTransformation_S2E();
            for(auto &it: sensor_transformation){
                sensor_transformation_calib[it.first] = it.second.transpose();
                it.second.print();
                it.second.transpose().print();
                std::cout<<"---------------------------------\n";
            }
        }
        

    }model1;
    
    
};
#endif