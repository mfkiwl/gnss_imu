#include "ros/ros.h"
#include <gnss/tightly_coupled.hpp>
#include <nm33/raw.h>
#include <nm33/gnss_raw.h>

using namespace std;

float vehicle_heading;
bool rec_navsat = false; 

vector<int> gps_svid(32);
vector<int> gps_multi(32);
vector<int> galileo_svid(30);
vector<int> beidou_svid(35);

vector<gps_raw_t> gps;
gps_raw_t sat_rxm;
gps_raw_t sat_nav;

nm33::raw raw;
nm33::gnss_raw gps_raw;

void getSatelliteRxmraw(const ublox_msgs::RxmRAWX& sat){

    if(rec_navsat){
        static int gps_num, galileo_num,  beidou_num;
        int sat_num = sat.numMeas;
        int sat_type, index;
        gps_num = 0, galileo_num = 0, beidou_num = 0;

        gps_multi.clear();
        gps_raw.meas.clear();

        cout << GREEN << "Update Ublox Rxmraw" << RESET << endl;

        for(int i = 0;i < sat_num;i++){
            sat_type = sat.meas[i].gnssId;
            
            switch(sat_type){
                case 0: // GPS
                {   
                    // get same raw data from svId
                    vector<int>::iterator it = find(gps_svid.begin(), gps_svid.end(), sat.meas[i].svId);
                    if(it != gps_svid.end()){

                        // multipath exclution
                        vector<int>::iterator it_multi = find(gps_multi.begin(), gps_multi.end(), sat.meas[i].svId);
                        if(it_multi != gps_multi.end()){
                            break;
                        }
                        else{
                            // cout << "it: " << *it << ", index: " << distance(gps_svid.begin(), it) << endl;
                            index = distance(gps_svid.begin(), it);

                            gps_multi.push_back(*it);
                            gps[index].cno = sat.meas[i].cno;
                            gps[index].prMes = sat.meas[i].prMes;
                            gps[index].prStdev = sat.meas[i].prStdev;

                            // rostopic msg
                            gps_raw.stamp = ros::Time::now();
                            gps_raw.heading = vehicle_heading;
                            raw.svId = gps[index].svId;
                            raw.elev = gps[index].elev;
                            raw.azim = gps[index].azim;
                            raw.cno = gps[index].cno;
                            raw.prMes = gps[index].prMes;
                            raw.prStdev = gps[index].prStdev;

                            gps_raw.meas.push_back(raw);

                        }
                    }
                    

                    cout << "gps number: " << gps_num << endl;
                    cout << "svId: " <<  gps[index].svId <<",elev: " <<  gps[index].elev <<",azim: " <<  gps[index].azim 
                    << ",cno: " <<  gps[index].cno << ",prMes: " <<  gps[index].prMes << ",prStdev: " <<  gps[index].prStdev << endl;

                    gps_num++;
                }
                    
                case 2: // Galileo
                    ;
                case 3: // BeiDou
                {
                    ;
                }
                    
            }
        
        }
        
        cout << "-----------------------------" << endl;

    }
    else{
        cout << YELLOW << "Waiting for Ublox Navsat data to start localization..." << RESET << endl;
    }
    return;
}

void getSatelliteNavsat(const ublox_msgs::NavSAT& sat){

    // gps clear vector
    if(rec_navsat){
        for(int i = 0;i < gps_svid.size();i++){
            gps.pop_back();
        }
    }
    
    rec_navsat = true;
    int sat_avalible = sat.numSvs;
    int sat_type;
    static int gps_num, galileo_num,  beidou_num;
    gps_num = 0, galileo_num = 0, beidou_num = 0;

    
    gps_svid.clear();

    cout << RED << "Update Ublox Navsat" << RESET << endl;

    for(int i = 0;i < sat_avalible;i++){
        sat_type = sat.sv[i].gnssId;
        
        switch(sat_type){
            case 0: // GPS
                gps_svid.push_back(sat.sv[i].svId);
        
                sat_nav.svId = sat.sv[i].svId;
                sat_nav.elev = sat.sv[i].elev;
                sat_nav.azim = sat.sv[i].azim;
                sat_nav.cno = 0;
                sat_nav.prMes = 0.;
                sat_nav.prStdev = 0;

                cout << "gps number: " << gps_num << endl;
                cout << "svId: " << sat_nav.svId <<",elev: " << sat_nav.elev <<",azim: " << sat_nav.azim 
                << ",cno: " << sat_nav.cno << ",prMes: " << sat_nav.prMes << ",prStdev: " << sat_nav.prStdev << endl;

                gps.push_back(sat_nav);
                gps_num++;

            case 2: // Galileo
            ;
            case 3: // BeiDou
            ;
        }
    }
    cout << "-----------------------------" << endl;
    return;
}

void getSatelliteNavpvt(const ublox_msgs::NavPVT& sat){
    vehicle_heading = sat.heading * 0.00001;
    cout << "vehicle heading: " << vehicle_heading << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GNSS");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<nm33::gnss_raw>("/gps_raw", 1); 

    ros::Subscriber sub[3];
    sub[0] = n.subscribe("/ublox_f9p/rxmraw", 1, getSatelliteRxmraw);
    sub[1] = n.subscribe("/ublox_f9k/navsat", 1, getSatelliteNavsat);
    sub[2] = n.subscribe("/ublox_f9k/navpvt", 1, getSatelliteNavpvt);

    ros::Rate loop_rate(1);

    while(ros::ok()){

        pub.publish(gps_raw);
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}