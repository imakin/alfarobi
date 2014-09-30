/*	Komunikasi Juri
 * 	
 * 	Version 7
 * 		Penalty SHOOT siapa ya ??
 * 		pickup & service
 */

#ifndef KOMUNIKASI_H
#define KOMUNIKASI_H

#include <iostream>
#include "threading.h"

#include "libUDPclient.h"
//#include "libDarwinNetwork.h"

//#include "data_AlfarobiCommunication.h"
#include "data_RoboCupGameControll.h"

// ========= MACRO buat kalibrasi ================
#define referreBox_broadcast_ip_port    "255.255.255.255:3838"
#define TEAM_ALFAROBI   				TEAM_MAGENTA
#define TEAM_MUSUH      				TEAM_CYAN
#define NO_PLAYER_ALFAROBI				2

//#define kiper_port      798
//#define kiper_ip        "192.168.1.22"
// --------------------------------------------------------------------------------------
// buat ngecek kalo ga ada status (disconnect)
#define NOT_FOUND       404


#define juri_sebagai(self) juri.teams[TEAM_ALFAROBI].players[self]
//#define abi_sebagai(self) abi_data[self]

#define juri_myself      juri_sebagai(NO_PLAYER_ALFAROBI)
//#define abi_myself      abi_sebagai(bek_alfarobi)

//using namespace Robot;

//LinuxClient *client_kiper;

struct RoboCupGameControlData juri;
//struct Alfarobi::Player abi_data[3];

static bool komunikasiJuri_aktive = false;
static pthread_t komunikasi_t;
unsigned short state, warnaGawang;

void *patuh_peraturan(void* arg)
{
	UDPClient client_gcd;
    if(!komunikasiJuri_aktive)
    {
        client_gcd.Initilaize(referreBox_broadcast_ip_port, "udp");
        komunikasiJuri_aktive = true;
    }
    juri_myself.penalty = PENALTY_NONE;
    state = NOT_FOUND;

//~ exit(1);
	while(1)
	{
		cout << "[======================kl========waiting==]";
		client_gcd.receive(Struct2String(juri), sizeof(juri));
		cout << "=====================================================" << juri.version;
		//~ exit(1);
		if(juri.version == GAMECONTROLLER_STRUCT_VERSION)
		{
			// cekking status pertandingan
			if(juri_myself.penalty == PENALTY_HL_KID_REQUEST_FOR_PICKUP)
				state = 9;
			else if(juri_myself.penalty == PENALTY_HL_KID_REQUEST_FOR_SERVICE)
				state = 10;
			else if(juri_myself.penalty != PENALTY_NONE)
				state = 6;
			else if(juri.state == STATE_INITIAL)
				state = 0;
			else if(juri.state == STATE_READY)
				state = 1;
			else if(juri.state == STATE_SET)
				state = 2;
			else if(juri.kickOffTeam == TEAM_ALFAROBI && juri.state == STATE_PLAYING && (juri.secondaryState == STATE2_NORMAL || juri.secondaryState == STATE2_OVERTIME))
				state = 3;
			else if(juri.kickOffTeam != TEAM_ALFAROBI && juri.state == STATE_PLAYING && (juri.secondaryState == STATE2_NORMAL|| juri.secondaryState == STATE2_OVERTIME))
				state = 4;
			else if(juri.kickOffTeam != TEAM_MUSUH && juri.secondaryState == STATE2_PENALTYSHOOT)
				state = 7;
			else if(juri.kickOffTeam == TEAM_MUSUH && juri.secondaryState == STATE2_PENALTYSHOOT)
				state = 8;
			else if(juri.state == STATE_FINISHED)
				state = 5;
			//~ else
				//~ state = NOT_FOUND;
				
			// cek gawang
			if(juri.teams[TEAM_ALFAROBI].goalColour == GOAL_BLUE)
				warnaGawang = 11;
			if(juri.teams[TEAM_ALFAROBI].goalColour == GOAL_YELLOW)
				warnaGawang = 12;
		} else {
			state = NOT_FOUND;
		}
		usleep(100000);
	}
}

void patuhiPeraturan()
{
	if(!komunikasiJuri_aktive)
		threadInitialize(komunikasi_t, patuh_peraturan, 30);
}

#endif // KOMUNIKASI_H
