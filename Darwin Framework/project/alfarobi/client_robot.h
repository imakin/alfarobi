#ifndef ALFA_COMMUNICATION_H
#define ALFA_COMMUNICATION_H

#include <iostream>
#include "string.h"
#include "threading.h"
#include "LinuxNetwork.h"
#include "libUDPclient.h"
#include "data_AlfarobiCommunication.h"
#include "data_RoboCupGameControll.h"

// =================== GENERAL SETTING =========================
// +> For Config
#define mSLEEP_COM              10
#define THREAD_PRIORITY_COM     30
// =============================================

// =================== KOMUNIKASI STRATEGI =========================
// +> For Config
#define PORT        3939
#define HOST        "192.168.1.122"

// +> For Use
void establishCommunication();
struct Alfarobi::Player recv_data;
struct Alfarobi::Player send_data;
// ==========================================

// ====================== GAMECONTROLLER =====================
// +> For Config
#define referreBox_broadcast_ip_port    "255.255.255.255:3838"
//~ #define TEAM_ALFAROBI   				TEAM_CYAN
//~ #define TEAM_MUSUH      				TEAM_MAGENTA
#define NO_TEAM_ALFAROBI                3
#define NO_PLAYER_ALFAROBI				3

// +> For Use
unsigned short state, warnaGawang;
#define NOT_FOUND       404
// ===============================================

using namespace Robot;
using namespace std;

#define juri_myteam			juri.teams[warna_team_alfarobi]
#define juri_sebagai(self)  juri.teams[warna_team_alfarobi].players[self]
#define juri_myself         juri_sebagai(NO_PLAYER_ALFAROBI-1)

struct RoboCupGameControlData juri;
uint8 warna_team_alfarobi;

// pengaman
static bool globalCom_isActivated = false;
static bool alfaCom_isActivated = false;
static bool komunikasiJuri_aktive = false;
static bool cariWarnaTeam_aktive = false;


uint8 whatColorIsMyTeam()
{
    for (int var = 0; var <= 2; ++var) {
        if (juri.teams[var].teamNumber == NO_TEAM_ALFAROBI)
            return juri.teams[var].teamColour;
    }
	state = 190;
    return 190;
}

void *referreBox_communication(void*arg)
{
    globalCom_isActivated = true;
    /// CLIENT UDP GAMECONTROLLER
    UDPClient client_gcd;
	state = 404;
    while(1)
    {
///    / INIT :  CLIENT UDP ==============================
        if(!komunikasiJuri_aktive)
        {
            //cout << "\n---------------[Waiting for GAMECONTROLLER to active]---------------\n";
            komunikasiJuri_aktive = client_gcd.Initilaize(referreBox_broadcast_ip_port, "udp");
            cout << "-----------------------[GAMECONTROLLER Connected]---------------------------";
			juri_myself.penalty = PENALTY_NONE;
        }
///    ============================================================

///        / LOOP :  CLIENT UDP ==============================
        if(komunikasiJuri_aktive)
        {
            client_gcd.receive(Struct2String(juri), sizeof(juri));

            if(!cariWarnaTeam_aktive)
            {
                warna_team_alfarobi = whatColorIsMyTeam();
                cariWarnaTeam_aktive = true;
            }

            if(juri.version == GAMECONTROLLER_STRUCT_VERSION)
            {
                // cekking status pertandingan
                if(juri_myself.penalty == PENALTY_HL_KID_REQUEST_FOR_PICKUP)
                    state = 9;
                else if(juri_myself.penalty == PENALTY_HL_KID_REQUEST_FOR_SERVICE)
                    state = 10;
                else if(juri_myself.penalty != PENALTY_NONE)
                    state = 6;
                else if(juri.dropInTeam == warna_team_alfarobi && juri.dropInTime < 10 && juri.dropInTime > 0)
                    state = 12; // 10 detik setelah OUT
                else if(juri.state == STATE_INITIAL)
                    state = 0;
                else if(juri.state == STATE_READY)
                    state = 1;
                else if(juri.state == STATE_SET)
                    state = 2;
                else if(juri.kickOffTeam == DROPBALL && juri.state == STATE_PLAYING && (juri.secondaryState == STATE2_NORMAL || juri.secondaryState == STATE2_OVERTIME))
                    state = 11; // kickoff siapa cepat dia dapat (langsung tendang gawang coy)
                else if(juri.kickOffTeam == warna_team_alfarobi && juri.state == STATE_PLAYING && (juri.secondaryState == STATE2_NORMAL || juri.secondaryState == STATE2_OVERTIME))
                    state = 3; // alfarobi kickoff
                else if(juri.kickOffTeam != warna_team_alfarobi && juri.state == STATE_PLAYING && (juri.secondaryState == STATE2_NORMAL|| juri.secondaryState == STATE2_OVERTIME))
                    state = 4; // musuh kickoff
                else if(juri.kickOffTeam == warna_team_alfarobi && juri.secondaryState == STATE2_PENALTYSHOOT)
                    state = 7;
                else if(juri.kickOffTeam != warna_team_alfarobi && juri.secondaryState == STATE2_PENALTYSHOOT)
                    state = 8;
                else if(juri.state == STATE_FINISHED)
                    state = 5;
                else
                    state = NOT_FOUND;

                // cek gawang
                if(juri.teams[warna_team_alfarobi].goalColour == GOAL_BLUE)
                    warnaGawang = 11;
                else if(juri.teams[warna_team_alfarobi].goalColour == GOAL_YELLOW)
                    warnaGawang = 12;
            }
            else
            {
                state = NOT_FOUND;
            }
        }
///        ============================================================
        usleep(mSLEEP_COM*1000);
    }
    komunikasiJuri_aktive = false;
    globalCom_isActivated = false;
    cariWarnaTeam_aktive = false;
}

void *client_communication(void *arg)
{
    globalCom_isActivated = true;
    /// CLIENT TCP KIPER
    LinuxClient *socket;

    while (1)
    {
///    / INIT : CLIENT TCP ==============================
        if(!alfaCom_isActivated)
        {
            try
            {
				socket = new LinuxClient(PORT);
                //cout << "\n-----------------[Waiting for kiper connection]-----------------\n";
                socket->connect(HOST, PORT);
                cout << "\n-----------------[kiper connected]-----------------\n";
                alfaCom_isActivated = true;
            } catch(LinuxSocketException e)
            {
                //cerr << "\n======================kiper===+>" << e.description() << endl;
                alfaCom_isActivated = false;
            }
        }
///    ============================================================

///        / LOOP : CLIENT TCP ==============================
        if(alfaCom_isActivated)
        {
            try
            {
                socket->send(AlfaStruct2String(send_data), sizeof(send_data));
                socket->recv(AlfaStruct2String(recv_data), sizeof(recv_data));
            } catch(LinuxSocketException e)
            {
                //cerr << "=========================+>" << e.description() << endl;
                alfaCom_isActivated = false;
            }
        }
///        ============================================================

        usleep(mSLEEP_COM*1000);
    }
    alfaCom_isActivated = false;
    globalCom_isActivated = false;
}

void establishCommunication()
{
    static pthread_t client_t, referreBox_t;

    //~ strcpy(send_data.header, ALFAROBI_STRUCT_HEADER);
    if(!globalCom_isActivated)
    {
        threadInitialize(referreBox_t, referreBox_communication, THREAD_PRIORITY_COM);
        threadInitialize(client_t, client_communication, THREAD_PRIORITY_COM);
    }
}

#endif // ALFA_COMMUNICATION_H
