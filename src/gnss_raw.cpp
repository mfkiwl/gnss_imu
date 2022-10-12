#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <algorithm> 
#include <ublox_msgs/RxmRAWX.h>
#include <ublox_msgs/NavSAT.h>

using namespace std;

// based on total satellites in constellation
vector<float> gps_rxm[32];
vector<float> galileo_rxm[30];
vector<float> beidou_rxm[35];
// vector<float> qzss_rxm[5];
// vector<float> glonass_rxm[24];

vector<float> gps_nav[32];
vector<float> galileo_nav[30];
vector<float> beidou_nav[35];
// vector<float> qzss_nav[5];
// vector<float> glonass_nav[24];

vector<int> gps_svid(32);
vector<int> galileo_svid(30);
vector<int> beidou_svid(35);


void getSatelliteRxm(const ublox_msgs::RxmRAWX& sat){
    static int gps_num, galileo_num,  beidou_num, qzss_num, glonass_num;

    // gps clear vector
    for(int i = 0;i < gps_num;i++){
        for(int j = 0;j < 4;j++){
            gps_rxm[gps_svid[i]].pop_back();
        } 
    }
    gps_svid.clear();
    // beidou clear vector
    for(int i = 0;i < beidou_num;i++){
        for(int j = 0;j < 4;j++){
            beidou_rxm[beidou_svid[i]].pop_back();
        } 
    }
    beidou_svid.clear();

    int sat_num = sat.numMeas;
    int sat_type;
    gps_num = 0, galileo_num = 0, beidou_num = 0, qzss_num = 0, glonass_num = 0;

    cout << "Total satellite number: " << sat_num << endl;
    for(int i = 0;i < sat_num;i++){
        sat_type = sat.meas[i].gnssId;
        
        switch(sat_type){
            case 0: // GPS
            {
                // multipath exclution
                vector<int>::iterator it = find(gps_svid.begin(), gps_svid.end(), sat.meas[i].svId);
                if(it != gps_svid.end() && gps_num != 0){
                    break;
                }
                else{
                    gps_svid.push_back(int(sat.meas[i].svId));
                    gps_rxm[gps_svid[gps_num]].push_back(sat.meas[i].svId);       // sv id
                    gps_rxm[gps_svid[gps_num]].push_back(sat.meas[i].cno);        // carrier to noise ratio
                    gps_rxm[gps_svid[gps_num]].push_back(sat.meas[i].prMes);      // pseudorange measurement
                    gps_rxm[gps_svid[gps_num]].push_back(sat.meas[i].prStdev);    // pseudorange std
                }                                                                                          
                
                cout << "gps number: " << gps_num << endl;
                cout << "others: ";
                for(int j = 0;j < gps_rxm[gps_svid[gps_num]].size();j++){
                    cout << gps_rxm[gps_svid[gps_num]].at(j) << ", ";
                }
                cout << endl;
                gps_num++;
            }
                 
            case 2: // Galileo
                ;
            case 3: // BeiDou
            {
                // multipath exclution
                vector<int>::iterator it = find(beidou_svid.begin(), beidou_svid.end(), sat.meas[i].svId);
                if(it != beidou_svid.end() && beidou_num != 0){
                    break;
                }
                else{
                    beidou_svid.push_back(int(sat.meas[i].svId));
                    beidou_rxm[beidou_svid[beidou_num]].push_back(sat.meas[i].svId);       // sv id
                    beidou_rxm[beidou_svid[beidou_num]].push_back(sat.meas[i].cno);        // carrier to noise ratio
                    beidou_rxm[beidou_svid[beidou_num]].push_back(sat.meas[i].prMes);      // pseudorange measurement
                    beidou_rxm[beidou_svid[beidou_num]].push_back(sat.meas[i].prStdev);    // pseudorange std
                }  
                
                // cout << "BDS: " << beidou_num << endl;
                // cout << "others: ";
                // for(int j = 0;j < beidou_rxm[beidou_svid[beidou_num]].size();j++){
                //     cout << beidou_rxm[beidou_svid[beidou_num]].at(j) << ", ";
                // }
                // cout << endl;
                beidou_num++;
            }
                
            case 5: // QZSS
                ;
            case 6: // GLONASS
                ;
        }
       
    }
    cout << "-----------------------------" << endl;
    return;
}

void getSatelliteNav(const ublox_msgs::NavSAT& sat){

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GNSS");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/ublox_f9p/rxmraw", 1, getSatelliteRxm);
    ros::Subscriber sub2 = n.subscribe("/ublox_f9p/navsat", 1, getSatelliteNav);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();
    }
    return 0;
}