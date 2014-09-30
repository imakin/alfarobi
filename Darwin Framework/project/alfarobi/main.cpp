/*
 * Makin
 */

#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <SerialStream.h>
#include <iostream>

#include "Camera.h"
#include "Point.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "ColorFinder.h"
#include "CompassSerial.h"
#include "client_robot.h"

#define INI_FILE_PATH       "bola.ini"

using namespace LibSerial;
using namespace std;

#define PIDSPEED			7
#define X_TENGAH_FRAME 160
#define Y_TENGAH_FRAME 120

#define KEPALA_KIRI_BESAR 3400
#define KEPALA_KANAN_KECIL 600
//~ #define KEPALA_KIRI_BESAR 2330
//~ #define KEPALA_KANAN_KECIL 1700
//~ #define KEPALA_ATAS_BESAR 3110
//~ #define KEPALA_ATAS_BESAR 2000
#define KEPALA_ATAS_BESAR 1900
//~ #define KEPALA_BAWAH_KECIL 1730
#define KEPALA_BAWAH_KECIL 1000
#define KEPALA_TENGAH_X 2048

//~ #define JARAK_SAPU_X 354
#define JARAK_SAPU_X 354
//~ #define JARAK_SAPU_X 100
#define JARAK_SAPU_Y 345
//~ #define JARAK_SAPU_Y 50

//fungsi swipe_line()
#define SWIPE_X 0
#define SWIPE_Y 1

#define GERAKAN_BALANCE 				5001
#define GERAKAN_BANGUN_DEPAN 			5002
#define GERAKAN_BANGUN_BELAKANG 		5003

#define GERAKAN_JALAN_DITEMPAT_LAMBAT 	5005
#define GERAKAN_JALAN 					5006
#define GERAKAN_GESER_KANAN 			5007
#define GERAKAN_GESER_KIRI 				5008
#define GERAKAN_TENDANG_KANAN 			5009
#define GERAKAN_ROTASI_KANAN 			5010
#define GERAKAN_ROTASI_KIRI 			5011
#define GERAKAN_JALAN_PELAN 			5012
#define GERAKAN_REVOLUSI_KANAN			5013
#define GERAKAN_REVOLUSI_KIRI			5014
#define GERAKAN_SERONG_KANAN			5015
#define GERAKAN_SERONG_KIRI				5016
#define GERAKAN_TENDANG_KIRI 			5017
#define GERAKAN_STOP_CLOSE		 		5018
#define GERAKAN_TANGAN		 			5019
#define GERAKAN_TENDANG_BELAKANG_KANAN	5020
#define PERINTAH_MINTA_KONDISI_TOMBOL	5021
#define GERAKAN_JALAN_DITEMPAT	 		5022
#define GERAKAN_MUNDUR					5023
#define GERAKAN_STOP					5024
#define GERAKAN_KICKOFF_KANAN			5025



#define LOOP 1

#define ACTION_KEJAR					0
#define ACTION_BALANCE 					1
#define ACTION_BANGUN_DEPAN 			2
#define ACTION_BANGUN_BELAKANG 			3

#define ACTION_JALAN_DITEMPAT_LAMBAT 	5
#define ACTION_JALAN					6
#define ACTION_GESER_KANAN 				7
#define ACTION_GESER_KIRI 				8
#define ACTION_TENDANG_KANAN 			9
#define ACTION_ROTASI_KANAN 			10
#define ACTION_ROTASI_KIRI 				11
#define ACTION_JALAN_PELAN 				12
#define ACTION_REVOLUSI_KANAN			13
#define ACTION_REVOLUSI_KIRI			14
#define ACTION_SERONG_KANAN				15
#define ACTION_SERONG_KIRI				16
#define ACTION_TENDANG_KIRI 			17
#define ACTION_STOP_CLOSE 				18
#define ACTION_TANGAN 					19
#define ACTION_TENDANG_BELAKANG_KANAN	20
#define ACTION_MINTA_KONDISI_TOMBOL		21
#define ACTION_JALAN_DITEMPAT 			22
#define ACTION_MUNDUR					23
#define ACTION_STOP						24
#define ACTION_KICKOFF_KANAN			25


#define TENDANG_KIRI					0
#define TENDANG_KANAN					1
#define TENDANG_SIDEKICK_KIRI			2
#define TENDANG_SIDEKICK_KANAN			3
#define TENDANG_BELAKANG_KIRI			4
#define TENDANG_BELAKANG_KANAN			5



//playing sate softwarenya juri
#define INITIAL					0
#define READY					1
#define SET						2	
#define PLAY_ALFAROBI_KICK0FF	3	
#define	PLAY_ENEMY_KICKOFF		4
#define PLAY_STOP				5
#define PELANGGARAN				6
#define P_ALFAROBI_KICK			7
#define P_ENEMY_KICK			8 
#define PICK_UP					9
#define SERVICE					10
#define DROP_BALL				11 ///KICK OFF BEBAS siapa cepat dia dapat (langsung tendang gawang coy)
#define BALL_OUT				12

#define GAWANG_MUSUH_KUNING		11
#define GAWANG_MUSUH_BIRU		12


///Makro-makro kurang ajar
#define cmps10 CompassSerial::GetInstance()
#define atur_kepala_x(n) kirim_data((n/4));
#define atur_kepala_y(n) kirim_data((n/4)+10000);
#define compass_baca() CompassSerial::GetInstance()->get_angle()

///Global Warming
SerialStream porting;
int pala_y,last_palay; ///last_palay/x buat pala terakhir sebelum perintah motion
int pala_x,last_palax;
int errorx,last_errorx,errory,last_errory,Px,Dx,Py,Dy;
int sudut_skrg,sudut_acuan,sudut_error;
char sudah_kickoff;
char keputusan_tendang;
int kondisitombol;
#define TOMBOL_ATAS_ON_BAWAH_ON 		2
#define TOMBOL_ATAS_ON_BAWAH_OFF		1
#define TOMBOL_ATAS_OFF_BAWAH_ON		4
#define TOMBOL_ATAS_OFF_BAWAH_OFF 		3
//~ #define TOMBOL_ATAS_ON_BAWAH_ON 		1
//~ #define TOMBOL_ATAS_ON_BAWAH_OFF		2
//~ #define TOMBOL_ATAS_OFF_BAWAH_ON		3
//~ #define TOMBOL_ATAS_OFF_BAWAH_OFF 		4

int arah_gawang; ///NILAINYA ADALAH SUDUT ERROR
bool arah_gawang_sudah_berubah=true;

int mode_kickoff=0;
#define MODE_KICKOFF_BIASA 0
#define MODE_KICKOFF_KANAN 1
int mode_main=0;
#define MODE_MAIN_LANGSUNG	0
#define MODE_MAIN_GIRING	1

struct timeval waktu_struct;
int waktu_acuan;
int waktu_sekarang;
int waktu_selisih;
unsigned char waktu_dipake;
#define WAKTU_AUTO_PICKUP_SKIPPING	0
int waktu_terakhir_selesai_pickup;
int force_play_flag;
#define FORCE_PLAY_NONE				0
#define FORCE_PLAY_PLAYNOKICKOFF	1
#define FORCE_PLAY_PLAYFROMPICKUP	2

void waktu_set_acuan(void);
int waktu_hitung_acuan(void);
int waktu_hitung_sekarang(void);


void arahkan_kepala(void);
void batasi_kepala(void);
void batasi_kepala(char fix);
char batasi_kepala_y_aktiv=1;
char batasi_kepala_x_aktiv=1;
void arahkan_kepala(char leher);
void stare(int posx, int posy);
void swipe_line(char SWIPETYPE, int start, int end);
void sapu_total();
void kejar();
void port_init(char _ttynum);
void port_init_prevent(char _ttynum);
void kirim_data(int data);
void change_current_dir();
int makin_get_index(int cx, int cy);
int makin_get_x(int indeks);
int makin_get_y(int indeks);
void compass_set_acuan(void);
void compass_hitung_acuan(void);
int compass_hitung_acuan_ret(void);

char last_action;

void motion_jalan();
void motion_jalan_pelan();
void motion_jalan_ditempat();
void motion_rotasi_kanan();
void motion_rotasi_kiri();
void motion_bangun_depan();
void motion_bangun_belakang();
void motion_balance();
void motion_tendang_kanan();
void motion_tendang_kiri();
void motion_tendang_belakang_kanan();
//~ void motion_tendang_belakang_kiri();
void motion_geser_kanan();
void motion_geser_kiri();
void motion_revolusi_kanan();
void motion_revolusi_kiri();
void motion_serong_kiri();
void motion_serong_kanan();
void motion_stop_close();
void motion_tangan();
void motion_mundur();
void motion_stop();
void motion_kickoff_kanan();


int baca_tombol();
int  baca_data(void);

int main(int argc, char* argv[])
{
	last_errorx  = 0;
	last_errory  = 0;
	errorx  = 0;
	errory  = 0;
	sudah_kickoff = 0;
	
	pala_x = 2000;
	pala_y = (1670);
	

	int portserial;
	portserial = CompassSerial::GetInstance()->AutoInitialize(4);
	cout<<"Compass ambil ttyUSB"<<portserial<<"--"<<endl;
	port_init_prevent(portserial);
	
	
	arahkan_kepala();
	usleep(1000*1000);
	//~ sudut_acuan = 36;
	//~ sudut_acuan = 306;
	///serang gawang biru 98
	///serang gawang kuning 297
    //~ pala_x = 2048;
	//~ pala_y = 2030;
	//~ compass_set_acuan();
	//~ arahkan_kepala();
	//~ motion_jalan();
	//~ usleep(2000*1000);
	//~ sudut_acuan=278;
	//~ while (1)
	//~ {
		//~ usleep(200*1000);
		//~ compass_hitung_acuan();
		//~ cout<<"acuan: "<<sudut_acuan<<endl;
		//~ cout<<"skrg: "<<sudut_skrg<<endl;
		//~ cout<<"eror: "<<sudut_error<<endl;
		//~ usleep(200*1000);
	//~ }
	int harus_jalan_pickup=0;
	
	
    minIni* ini;
    change_current_dir();
    if (argc>1)
    {
		if ((argv[1][0]=='f'))
		{
			ini = new minIni(argv[2]);
			cout<<"kalibrasi";
		}
		else
		{	
			ini = new minIni("bola.ini");
			cout<<"biasa";
		}
	}
	else
	{
		ini = new minIni("bola.ini");
		cout<<"biasa";	
	}
    
    Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    
    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* finder_bola = new ColorFinder();
    ColorFinder* finder_gawang = new ColorFinder();
    ColorFinder* finder_gawang_musuh = new ColorFinder();
    finder_bola->LoadINISettings(ini);
    httpd::ball_finder = finder_bola;
    //~ httpd::ball_finder = finder_gawang;

	finder_bola->m_min_percent=0.01;
	finder_bola->m_max_percent=100;
	
	minIni* ini_gawang;
	minIni* ini_gawang_musuh;
	minIni* ini_gawang_kuning;
	minIni* ini_gawang_biru;
	
	int perintahkangerakan = 0;
	int batasperintahkangerakan = 25;
	int samplinglihatgawang= 1;
	int tunggukickoff=0;
	int hilang_pertamakali=1;
	int hilang_arahrotasi=0;
	#define HILANG_ARAH_ROTASI_KANAN 0
	#define HILANG_ARAH_ROTASI_KIRI 1
	Point2D pos;
	Point2D gawang_pos;
	Point2D gawang_musuh_pos;
	
	pala_x = 2048;
	//~ pala_y = KEPALA_BAWAH_KECIL+100;
	//~ pala_y = 2000;
	arahkan_kepala();
	//~ establishCommunication();
	
	
	
	state = PLAY_ALFAROBI_KICK0FF;
	warnaGawang = GAWANG_MUSUH_BIRU;
	
	ini_gawang_kuning = new minIni("gawang_kuning.ini");
	ini_gawang_biru = new minIni("gawang_biru.ini");
	
		
	#define SUDUT_ACUAN_MUSUH_KUNING 351
	#define SUDUT_ACUAN_MUSUH_BIRU 182
	
	if (warnaGawang==GAWANG_MUSUH_KUNING)
	{
		ini_gawang = ini_gawang_kuning;
		ini_gawang_musuh = ini_gawang_biru;
		finder_gawang->LoadINISettings(ini_gawang);
		finder_gawang_musuh->LoadINISettings(ini_gawang_musuh);
		sudut_acuan = SUDUT_ACUAN_MUSUH_KUNING;
	}
	else
	{
		ini_gawang = ini_gawang_biru;
		ini_gawang_musuh = ini_gawang_kuning;
		finder_gawang->LoadINISettings(ini_gawang);
		finder_gawang_musuh->LoadINISettings(ini_gawang_musuh);
		sudut_acuan = SUDUT_ACUAN_MUSUH_BIRU;
	}
{
	
	if ((argc>1) && (argv[1][0]=='f'))
	{
		pala_x=KEPALA_TENGAH_X;
		pala_y=1030;
		//~ pala_y=KEPALA_ATAS_BESAR;
		arahkan_kepala();
		while (1)
		{
			LinuxCamera::GetInstance()->CaptureFrame();
			pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
			gawang_pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
			memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
			/*if (pos.X>0)
			{
				stare(pos.X,pos.Y);
				if (pala_x>2500)
				{
					cout<<"rotasi_kiri "<<pala_x<<":"<<pala_y<<"\t";
				}
				else if (pala_x>2200)
				{
					cout<<"serong_kiri "<<pala_x<<":"<<pala_y<<"\t";
				}
				else if (pala_x<1500)
				{
					cout<<"rotasi_kanan "<<pala_x<<":"<<pala_y<<"\t";
				}
				else if (pala_x<1800)
				{
					cout<<"serong_kanan "<<pala_x<<":"<<pala_y<<"\t";
				}
				else
				{
					cout<<"jalan "<<pala_x<<":"<<pala_y<<"\t";
				}
			}*/
			for(int i = 0; i < rgb_ball->m_NumberOfPixels; i++)
			{
				if(finder_gawang->m_result->m_ImageData[i] == 1)
				{
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255-rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0];
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 255-rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1];
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 255-rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2];
					
				}
				if(finder_bola->m_result->m_ImageData[i] == 1)
				{
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255-rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0];
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 255-rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1];
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 255-rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2];
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 255;
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 255;
				}
				else
				{
					/*//~ int avrage=rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0]+rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1]+rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2];
					avrage=avrage/3;
					rgb_ball->m_ImageData[(i)*rgb_ball->m_PixelSize + 0] = avrage;
					rgb_ball->m_ImageData[(i)*rgb_ball->m_PixelSize + 1] = avrage;
					rgb_ball->m_ImageData[(i)*rgb_ball->m_PixelSize + 2] = avrage;*/
				}
			}
			streamer->send_image(rgb_ball);
			cout<<pos.X<<" "<<pos.Y<<" er"<<errorx<<","<<errory<<"\n";//<<"  compass skrg:"<<sudut_skrg<<"\n";
		}
	}
			pala_x = 2048;
			pala_y = KEPALA_BAWAH_KECIL+100;
			arahkan_kepala();
			
			//~ int x;
			//~ while (1)
			//~ {
				//~ LinuxCamera::GetInstance()->CaptureFrame();
				//~ pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				//~ cout<<"("<<pos.X<<":"<<pos.Y<<")\n";
				//~ cin>>x;
				//~ pala_y = x;
				//~ arahkan_kepala();
				//~ if (x==1)
				//~ {
					//~ pala_x+=20;
					//~ cout<<pala_x<<","<<pala_y<<"\n";
					//~ atur_kepala_x(pala_x);
					//~ atur_kepala_y(pala_y);
				//~ }
				//~ else if (x==2)
				//~ {
					//~ pala_x-=20;
					//~ cout<<pala_x<<","<<pala_y<<"\n";
					//~ atur_kepala_x(pala_x);
					//~ atur_kepala_y(pala_y);
//~ 
				//~ }
				//~ else if (x==3)
				//~ {
					//~ pala_y+=20;
					//~ cout<<pala_x<<","<<pala_y<<"\n";
					//~ atur_kepala_x(pala_x);
					//~ atur_kepala_y(pala_y);
//~ 
				//~ }
				//~ else 
				//~ {
					//~ pala_y-=20;
					//~ cout<<pala_x<<","<<pala_y<<"\n";
					//~ atur_kepala_x(pala_x);
					//~ atur_kepala_y(pala_y);
//~ 
				//~ }
			//~ }
			
			/*
			///LIAT GAWANG
			pala_y = KEPALA_ATAS_BESAR+100;
			pala_x = KEPALA_KANAN_KECIL;
			arahkan_kepala();
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
			int sGawang_a=0,sGawang_b=0,sGawang_c=0,sGawang_d=0,sGawang_jbaru=0,sGawang_jlama=0,sGawang_jjarak=0;
			while (pala_x<=KEPALA_KIRI_BESAR-1)
			{
				if ( ((KEPALA_KIRI_BESAR-pala_x)<200) || ((pala_x-KEPALA_KANAN_KECIL)<300) )
				{
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
				}
				LinuxCamera::GetInstance()->CaptureFrame();
				pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				
				if ((pos.X>0) && (pos.X<320))
				{
					if (sGawang_a==0)
						sGawang_a=pala_x;
					sGawang_jbaru=pala_x;
					if (sGawang_jlama==0)
						sGawang_jlama=pala_x;
					
					if ((sGawang_jbaru-sGawang_jlama)>sGawang_jjarak)
					{
						sGawang_jjarak=sGawang_jbaru-sGawang_jlama;
						sGawang_b=sGawang_jlama;
						sGawang_c=sGawang_jbaru;
					}
					sGawang_jlama = pala_x;
					if (sGawang_c!=0)
						sGawang_d=pala_x;
					cout<<pala_x<<":"<<pala_y<<"\t"<<pos.X<<","<<pos.Y<<endl;
				}
				pala_x += 80;
				arahkan_kepala();
			}
			cout<<"a b c d jarak = "<<sGawang_a<<" "<<sGawang_b<<" "<<sGawang_c<<" "<<sGawang_d<<" "<<sGawang_jjarak<<endl;
			while (1)
			{
				pala_x = (sGawang_a+sGawang_b)/2;
				arahkan_kepala();
				usleep(1000*1000);
				pala_x = (sGawang_c+sGawang_d)/2;
				arahkan_kepala();
				usleep(1000*1000);
			}*/
			
			//~ while (1)
			//~ {
				//~ cout<<endl;
				//~ baca_tombol();
			//~ }


	
}		
	int perlukickoff=0;//klo lewaat initial /ready /set perlukickoff==1 klo gak perlu kickoff=0
	
	
///DIPAKE DENGAN TELITI, TOMBOL DICEK PLEASE
	if (baca_tombol()==TOMBOL_ATAS_ON_BAWAH_ON || baca_tombol()==TOMBOL_ATAS_OFF_BAWAH_ON)
	{
		//~ force_play_flag = FORCE_PLAY_PLAYNOKICKOFF;
		//~ mode_kickoff = MODE_KICKOFF_KANAN;
		mode_main = MODE_MAIN_GIRING;
		cout<<"mode giring";
	}
	else
	{
		cout<<"mode biasa";
	}
	
	sudah_kickoff=0;
	harus_jalan_pickup = 0;
    while(1)
    {
			
		cout<<"state : "<< state <<endl; 
#define AA__________INITIAL
		if(state == INITIAL)
		{
			perlukickoff=1;
			cout<<"initial, baca gawang musuh\t";
			LinuxCamera::GetInstance()->CaptureFrame();
			pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
			if (pos.X>=0)
			{
				stare(pos.X,pos.Y);
			}
			else
			{
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				while(pos.X<0)
				{
					for (pala_y = KEPALA_BAWAH_KECIL; pala_y<KEPALA_ATAS_BESAR;pala_y+=JARAK_SAPU_Y/4)
					{
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
						arahkan_kepala();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
					}
					for (pala_y = KEPALA_ATAS_BESAR; pala_y>KEPALA_BAWAH_KECIL;pala_y-=JARAK_SAPU_Y/4)
					{
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
						arahkan_kepala();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
					}
					
				}
			}
				//~ sudah_kickoff=1;
				
			if (warnaGawang==GAWANG_MUSUH_KUNING)
			{
				ini_gawang = ini_gawang_kuning;
				finder_gawang->LoadINISettings(ini_gawang);
				sudut_acuan = SUDUT_ACUAN_MUSUH_KUNING;
				cout<<"Gawang musuh kuning\n";
			}
			else
			{
				ini_gawang = ini_gawang_biru;
				finder_gawang->LoadINISettings(ini_gawang);
				sudut_acuan = SUDUT_ACUAN_MUSUH_BIRU;
				cout<<"Gawang musuh biru\n";
			}
			
		}
#define AA__________READY
		else if (state == READY)
		{
			if (warnaGawang==GAWANG_MUSUH_KUNING)
			{
				ini_gawang = ini_gawang_kuning;
				finder_gawang->LoadINISettings(ini_gawang);
				sudut_acuan = SUDUT_ACUAN_MUSUH_KUNING;
				cout<<"Gawang musuh kuning\n";
			}
			else
			{
				ini_gawang = ini_gawang_biru;
				finder_gawang->LoadINISettings(ini_gawang);
				sudut_acuan = SUDUT_ACUAN_MUSUH_BIRU;
				cout<<"Gawang musuh biru\n";
			}
			perlukickoff=1;
			cout<<"ready"<<endl;
			LinuxCamera::GetInstance()->CaptureFrame();
			pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
			if (pos.X>=0)
			{
				stare(pos.X,pos.Y);
			}
			else
			{
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				while(pos.X<0)
				{
					for (pala_y = KEPALA_BAWAH_KECIL; pala_y<KEPALA_ATAS_BESAR;pala_y+=JARAK_SAPU_Y/8)
					{
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
						arahkan_kepala();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
					}
					for (pala_y = KEPALA_ATAS_BESAR; pala_y>KEPALA_BAWAH_KECIL;pala_y-=JARAK_SAPU_Y/8)
					{
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
						arahkan_kepala();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
					}
					
				}
			}
				//~ sudah_kickoff=1;
		}
#define AA__________SET
		else if(state == SET)
		{
			perlukickoff=1;
			cout<<"set"<<endl;
			LinuxCamera::GetInstance()->CaptureFrame();
			pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
			if (pos.X>=0)
			{
				stare(pos.X,pos.Y);
			}
			else
			{
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				while(pos.X<0)
				{
					for (pala_y = KEPALA_BAWAH_KECIL; pala_y<KEPALA_ATAS_BESAR;pala_y+=JARAK_SAPU_Y/8)
					{
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
						arahkan_kepala();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
					}
					for (pala_y = KEPALA_ATAS_BESAR; pala_y>KEPALA_BAWAH_KECIL;pala_y-=JARAK_SAPU_Y/8)
					{
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
						arahkan_kepala();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
					}
					
				}
			}
				//~ sudah_kickoff=1;
			
		}
#define AA__________PLAY_STOP
		else if(state == PLAY_STOP)
		{
			perlukickoff=1;
			///abis goal
			sudah_kickoff=0;
			tunggukickoff=0;
			cout<<"play stop"<<endl;
			LinuxCamera::GetInstance()->CaptureFrame();
			pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
			if (pos.X>=0)
			{
				stare(pos.X,pos.Y);
			}
			else
			{
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				while(pos.X<0)
				{
					for (pala_y = KEPALA_BAWAH_KECIL; pala_y<KEPALA_ATAS_BESAR;pala_y+=JARAK_SAPU_Y/8)
					{
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
						arahkan_kepala();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
					}
					for (pala_y = KEPALA_ATAS_BESAR; pala_y>KEPALA_BAWAH_KECIL;pala_y-=JARAK_SAPU_Y/8)
					{
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
						arahkan_kepala();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
						if (pos.X>=0)
						{
							stare(pos.X,pos.Y);
							break;
						}
					}
					
				}
			}
		}
#define AA__________PLAY
		else if(state == PLAY_ALFAROBI_KICK0FF || state == PLAY_ENEMY_KICKOFF || state == BALL_OUT || state == DROP_BALL || (force_play_flag != FORCE_PLAY_NONE && ((waktu_hitung_sekarang()-waktu_terakhir_selesai_pickup)<=10) ) )
		{
			if (!perlukickoff)
			{
				sudah_kickoff=1;
			}
			cout<<"play";
			if (harus_jalan_pickup==2)
			{
				pala_x = KEPALA_TENGAH_X;
				pala_y = KEPALA_ATAS_BESAR-100;
				arahkan_kepala();
				waktu_set_acuan();
				motion_jalan();
				perintahkangerakan = 0;
				while (waktu_hitung_acuan()<20)
				{
					cout<<"jalan_pickep"<<waktu_acuan<<"-"<<waktu_sekarang<<"="<<waktu_selisih<<"\t";
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
					pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					if (pos.X>0)
					{
						stare(pos.X,pos.Y);							
						if (perintahkangerakan>40)
						{
							if (pala_y>1300)
							{
								if (pala_x>2500)
								{
									motion_rotasi_kiri();
									cout<<"rotasi_kiri "<<pala_x<<":"<<pala_y;
									//~ usleep(1000*1000);
								}
								else if (pala_x>2300)
								{
									motion_serong_kiri();
									cout<<"serong_kiri "<<pala_x<<":"<<pala_y;
								}
								else if (pala_x<1500)
								{
									motion_rotasi_kanan();
									cout<<"rotasi_kanan "<<pala_x<<":"<<pala_y;
									//~ usleep(1000*1000);
								}
								else if (pala_x<1700)
								{
									motion_serong_kanan();
									cout<<"serong_kanan "<<pala_x<<":"<<pala_y;
								}
								else
								{
									motion_jalan();
									cout<<"jalan "<<pala_x<<":"<<pala_y;
								}
							}
							else
							{
								break;
							}
							perintahkangerakan=0;
						}
						else
						{
							perintahkangerakan++;
						}
					}
					else
					{
						motion_jalan();
						cout<<"jalan "<<pala_x<<":"<<pala_y;
					}
					cout<<endl;
				}
				motion_stop_close();
				harus_jalan_pickup = 0;
			}
			else if (harus_jalan_pickup==1)
			{
				pala_x = KEPALA_TENGAH_X;
				pala_y = KEPALA_ATAS_BESAR-100;
				arahkan_kepala();
				waktu_set_acuan();
				motion_jalan();
				perintahkangerakan = 0;
				while (waktu_hitung_acuan()<5)
				{
					cout<<"jalan_pickep"<<waktu_acuan<<"-"<<waktu_sekarang<<"="<<waktu_selisih<<"\t";
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
					pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					if (pos.X>0)
					{
						stare(pos.X,pos.Y);							
						if (perintahkangerakan>40)
						{
							if (pala_y>1300)
							{
								if (pala_x>2500)
								{
									motion_rotasi_kiri();
									cout<<"rotasi_kiri "<<pala_x<<":"<<pala_y;
									//~ usleep(1000*1000);
								}
								else if (pala_x>2300)
								{
									motion_serong_kiri();
									cout<<"serong_kiri "<<pala_x<<":"<<pala_y;
								}
								else if (pala_x<1500)
								{
									motion_rotasi_kanan();
									cout<<"rotasi_kanan "<<pala_x<<":"<<pala_y;
									//~ usleep(1000*1000);
								}
								else if (pala_x<1700)
								{
									motion_serong_kanan();
									cout<<"serong_kanan "<<pala_x<<":"<<pala_y;
								}
								else
								{
									motion_jalan();
									cout<<"jalan "<<pala_x<<":"<<pala_y;
								}
							}
							else
							{
								break;
							}
							perintahkangerakan=0;
						}
						else
						{
							perintahkangerakan++;
						}
					}
					else
					{
						motion_jalan();
						cout<<"jalan "<<pala_x<<":"<<pala_y;
					}
					cout<<endl;
				}
				motion_stop_close();
				harus_jalan_pickup = 0;
			}
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			//~ cout<<"acuan "<<sudut_acuan<<"\n";
			memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

			pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

			//~ fprintf(stderr, "posx: %f, posy: %f num: %d-----\r", pos.X, pos.Y,rgb_ball->m_NumberOfPixels);
			//~ fprintf(stderr, "             \t\tposx: %f, posy: %f num: %d----\r", pos.X, pos.Y,finder_bola->numpixel);
			//~ cout<<"posx: "<<pos.X<<", posy: "<<pos.Y<<" num: "<<finder_bola->numpixel<<"\n";
			
			//tunggu di kickoff
			if (force_play_flag==FORCE_PLAY_PLAYFROMPICKUP)
			{
				tunggukickoff=40;
				sudah_kickoff=1;
				if ((waktu_hitung_sekarang()-waktu_terakhir_selesai_pickup)>10)
					force_play_flag = FORCE_PLAY_NONE;
			}
			else if (force_play_flag==FORCE_PLAY_PLAYNOKICKOFF)
			{
				tunggukickoff=40;
				sudah_kickoff=1;
				force_play_flag = FORCE_PLAY_NONE;
			}
			
			
			if ((state == PLAY_ENEMY_KICKOFF)&&(tunggukickoff<=36))
			{
				sudah_kickoff=1;
				cout<<"play enemy kickoff\n";
				int xkuncikickoff,ykuncikickoff,palaxt,palayt;
				char foundflag=0;
				pala_x = 2000;
				pala_y = (2280);
				arahkan_kepala();
				palaxt=pala_x;
				palayt=pala_y;
				while (tunggukickoff<=37)
				{
					LinuxCamera::GetInstance()->CaptureFrame();
					memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
					pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					cout<<pos.X<<","<<pos.Y<<"\n";
					cout<<pala_x<<","<<pala_y<<"\\"<<palaxt<<","<<palayt<<"\n";
					if (pos.X>=0)
					{
						stare(pos.X,pos.Y);
						xkuncikickoff = pos.X;
						ykuncikickoff = pos.Y;
						foundflag=1;
					}	
					else
					{
						if (foundflag==1)
						{
							tunggukickoff=40;
							cout<<"kicked off\n";
						}
					}
					//~ if ((foundflag==1)&& (
						//~ (abs(pala_x-palaxt)>100) | (abs(pala_y-palayt)>80))
						//~ )
					usleep(250*1000);
					tunggukickoff++;
					palaxt=pala_x;
					palayt=pala_y;
					streamer->send_image(rgb_ball);
				}
				
			}
			else if (pos.X>0)
			{
				hilang_pertamakali=1;
				stare(pos.X,pos.Y);
				if (perintahkangerakan>batasperintahkangerakan)
				{
					//~ cout<<pala_x<<"::"<<pala_y<<"\n";cout<<"posx: "<<pos.X<<", posy: "<<pos.Y<<" num: "<<finder->numpixel<<"\n";
					if (
						(pala_y>1300)
						|| ((pala_x>2750)&&(pala_y<1450))
						|| ((pala_x<1350)&&(pala_y<1450))
						)
					{
						if ((pala_x>2750)&&(pala_y<1450)&&(abs(compass_hitung_acuan_ret())<45))
						{
							motion_geser_kiri();
							cout<<"geser_kiri "<<pala_x<<":"<<pala_y;
						}
						else if (pala_x>2500)
						{
							motion_rotasi_kiri();
							cout<<"rotasi_kiri "<<pala_x<<":"<<pala_y;
							//~ usleep(1000*1000);
						}
						else if (pala_x>2200)
						{
							motion_serong_kiri();
							cout<<"serong_kiri "<<pala_x<<":"<<pala_y;
						}
						else if ((pala_x<1350)&&(pala_y<1450)&&(abs(compass_hitung_acuan_ret())<45))
						{	
							motion_geser_kanan();
							cout<<"geser_kanan "<<pala_x<<":"<<pala_y;
						}
						else if (pala_x<1500)
						{
							motion_rotasi_kanan();
							cout<<"rotasi_kanan "<<pala_x<<":"<<pala_y;
							//~ usleep(1000*1000);
						}
						else if (pala_x<1800)
						{
							motion_serong_kanan();
							cout<<"serong_kanan "<<pala_x<<":"<<pala_y;
						}
						else
						{
							motion_jalan();
							cout<<"jalan "<<pala_x<<":"<<pala_y;
						}
					}
					else
					{
						if (sudah_kickoff) ///MAU TENDANGIN BOLA KE GAWANG
						{


///MODE MAIN BIASA LANGSUNG TENDANG



							if (mode_main==MODE_MAIN_LANGSUNG)
							{
								motion_stop_close();
								compass_hitung_acuan();
								///BACKSHOOT KICK
								cout<<sudut_error<<"---"<<endl;
								if (abs(sudut_error)>130)
								{
									cout<<"backshoot\n";
									int sampling_compass=0;
									while ((abs(sudut_error)<=160)&&(pos.X>0))
									{
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
										pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
										streamer->send_image(rgb_ball);
										compass_hitung_acuan();
										
										if (sudut_error<=160)
										{
											motion_revolusi_kiri();
											cout<<"revolusi kiri\n";
										}
										else 
										{
											motion_revolusi_kanan();
											cout<<"revolusi kanan\n";
										}
										//~ usleep(500*1000);
										//~ motion_balance();
										//~ usleep(1000*1000);
									}
												
												
									pala_y = KEPALA_BAWAH_KECIL;
									///MAU TENDANG BACKSHOOT
									{
										for (pala_x=1600; pala_x<=2048; pala_x+=JARAK_SAPU_X/2)
										{
											arahkan_kepala();
											usleep(100*1000);
										}
										pala_x = 1900;
										pala_y = 1200;
										arahkan_kepala();
										usleep(100*1000);
										pala_x = 2040;
										pala_y = 1000;
										arahkan_kepala();
										usleep(100*1000);
										pala_x = 2048;
										pala_y = 1030;
										arahkan_kepala();
										
										//~ motion_balance();
										//~ motion_jalan_ditempat();
										//~ motion_jalan_pelan();
										//~ atur_kepala_x(pala_x);
										//~ arahkan_kepala();
										//~ arahkan_kepala();
										//~ usleep(500*1000);
										//~ atur_kepala_y(pala_y);
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
										
											while (pos.X>0)
											{
												pala_x = 1900;
												pala_y = 1200;
												arahkan_kepala();
												usleep(100*1000);
												pala_x = 2100;
												pala_y = 1020;
												arahkan_kepala();
												usleep(100*1000);
												pala_x = 2048;
												pala_y = 1030;
												arahkan_kepala();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
												pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												streamer->send_image(rgb_ball);
												if (pos.X<0)
													break;
												cout << pos.X<<","<<pos.Y;
												if (pos.X<195)
												{
													motion_geser_kiri();
													cout<<" geser kiri\n";
													//~ usleep(1000*1000);
												}
												else if (pos.X>230)
												{
													motion_geser_kanan();
													cout<<" geser kanan\n";
													//~ usleep(1000*1000);
												}
												else if (pos.Y<140)
												{
													motion_jalan_pelan();
													cout<<" jalan pelan\n";
													//~ usleep(1000*1000);
												}
												else
												{
													cout<<"mau tendang belakang\n";
													motion_tendang_belakang_kanan();
													usleep(3000*1000);
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													//~ motion_balance();
												}
												//~ usleep(1000*1000);
											}
										
									}
								}
								else
								{
									if (arah_gawang_sudah_berubah)
									{
										cout<<"preparasi tendang tendang "<<pala_x<<":"<<pala_y;
										//~ motion_jalan_ditempat();
										compass_hitung_acuan();
										///samplinglihatgawang initialnya 5
										///SAMPLING LIHAT GAWANG kalo samplinglihatgawang = 5
										///berarti LIHAT GAWANG, klo kurang masih belum
										if (samplinglihatgawang>=0)
										{
											if ((abs(sudut_error)>=60))
											{
												if (
														(
															(abs(sudut_error)>=20) &&
															(last_action!=ACTION_REVOLUSI_KANAN) &&
															(last_action!=ACTION_REVOLUSI_KIRI)
														) || (abs(sudut_error)>=40)
													)
												{
													int sampling_compass=0;
													while ((abs(sudut_error)>=20)&&(pos.X>0))
													{
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
														pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
														streamer->send_image(rgb_ball);
														
														if (sampling_compass<1)
														{
															compass_hitung_acuan();
															sampling_compass = ((abs(sudut_error)/10)-1)*8;
															cout<<"."<<sampling_compass<<endl;
														}
														else
														{
															sampling_compass--;
															cout<<"."<<sampling_compass<<endl;
														}
														
														if (sudut_error>20)
														{
															motion_revolusi_kanan();
															cout<<"revolusi kanan\n";
														}
														else 
														{
															motion_revolusi_kiri();
															cout<<"revolusi kiri\n";
														}
														//~ usleep(500*1000);
														//~ motion_balance();
														//~ usleep(1000*1000);
													}
												}
											}
											
			///LIHAT GAWANG LAMA
						/*		
											///CEK GAWANG
											batasi_kepala_y_aktiv=0;
											compass_hitung_acuan();
											//1. Cari gawang
											if (sudut_error<0)
											{
												pala_y = KEPALA_ATAS_BESAR+100;
												if (abs(sudut_error)<40)
													pala_x = KEPALA_TENGAH_X-100;
												else
													pala_x = KEPALA_KANAN_KECIL;
												arahkan_kepala();
												arahkan_kepala();
												usleep(400*1000);
												//1. Cari gawang
												LinuxCamera::GetInstance()->LoadINISettings(ini_gawang);
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												while ((pos.X<0)&&(pala_x<KEPALA_KIRI_BESAR))
												{
													cout<<"CARI GAWANGNYA "<<pos.X<<","<<pos.Y<<"\n";
													pala_x+=JARAK_SAPU_X/2;
													arahkan_kepala();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
													if (pos.X>0)
													{
														cout<<"Gawang seems found\n";
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
													}
												}
											}
											else
											{
												pala_y = KEPALA_ATAS_BESAR+100;
												if (abs(sudut_error)<40)
													pala_x = KEPALA_TENGAH_X+100;
												else
													pala_x = KEPALA_KIRI_BESAR;
												arahkan_kepala();
												arahkan_kepala();
												usleep(400*1000);
												//1. Cari gawang
												LinuxCamera::GetInstance()->LoadINISettings(ini_gawang);
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												while ((pos.X<0)&&(pala_x>KEPALA_KANAN_KECIL))
												{
													cout<<"CARI GAWANGNYA "<<pos.X<<","<<pos.Y<<"\n";
													pala_x-=JARAK_SAPU_X/2;
													arahkan_kepala();
													//~ usleep(200*1000);
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
													if (pos.X>0)
													{
														cout<<"Gawang seems found\n";
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
													}
												}
											}
											//2. Stare gawang sambil revolusi sampe sejajar
											if (pos.X>0)//gawang terlihat
											{
												
												
												arahkan_kepala();
												cout<<"GAWANG TERLIHAT\n";
												cout<<"stare gawang\n";
												//2. Stare gawang sambil revolusi sampe sejajar
												perintahkangerakan=0;
												int arahkan_gawang_batas_search=400;
												if (abs(pala_x-2048)<=400)
													arahkan_gawang_batas_search=140;
												while ((abs(pala_x-2048)>arahkan_gawang_batas_search)&&(pos.X>0))
												{
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
													if (pos.X>0)
													{
														stare(pos.X,pos.Y);
													}
													else
													{
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);		
														if (pos.X>0)
														{
															stare(pos.X,pos.Y);
														}
														else
														{
															LinuxCamera::GetInstance()->CaptureFrame();
															LinuxCamera::GetInstance()->CaptureFrame();
															LinuxCamera::GetInstance()->CaptureFrame();
															LinuxCamera::GetInstance()->CaptureFrame();
															pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);											
															if (pos.X>0)
															{
																stare(pos.X,pos.Y);
															}
															else
															{
																perintahkangerakan=0;
															}
														}
													}
													if (perintahkangerakan>10)
													{
														if (pala_x>2000)
														{
															motion_revolusi_kanan();
															cout<<"revolusi kanan sejajarin gawang";
														}
														else //pala_x<2000
														{
															motion_revolusi_kiri();
															cout<<"revolusi kiri sejajarin gawang";
														}
														perintahkangerakan=0;
													}
													else
													{
														perintahkangerakan++;
													}
												}
												if (pos.X<0)
													cout<<"ilang meneh\n";
											}
											else//andalin compass
											{
												while ((abs(sudut_error)>=20)&&(pos.X>0))
												{
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
													pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
													if (pos.X<0)
															break;
													streamer->send_image(rgb_ball);
														
													compass_hitung_acuan();
													if (sudut_error>20)
													{
														motion_revolusi_kanan();
														cout<<"revolusi kanan--gawang gak terlihat\n";
													}
													else 
													{
														motion_revolusi_kiri();
														cout<<"revolusi kiri--gawang gak terlihat\n";
													}
													//~ usleep(1000*1000);
													//~ motion_balance();
													//~ usleep(1000*1000);
												}
											}
											//kembaliin settingan ke kalibrasi bola
											LinuxCamera::GetInstance()->LoadINISettings(ini);
											samplinglihatgawang = 0;
										
							*/				
			///LIHAT GAWANG BARU 
											
											///LIAT GAWANG
											LinuxCamera::GetInstance()->LoadINISettings(ini_gawang);
											pala_y = KEPALA_ATAS_BESAR+100;
											pala_x = KEPALA_KANAN_KECIL+200;
											arahkan_kepala();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
											gawang_musuh_pos = finder_gawang_musuh->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
											int sGawang_a=0,sGawang_b=0,sGawang_c=0,sGawang_d=0,sGawang_jbaru=0,sGawang_jlama=0,sGawang_jjarak=0;
											int milih;
											while (pala_x<=KEPALA_KIRI_BESAR-200)
											{
												if ( ((KEPALA_KIRI_BESAR-pala_x)<50) || ((pala_x-KEPALA_KANAN_KECIL)<50) )
												{
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
												}
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												gawang_musuh_pos = finder_gawang_musuh->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												if ((
													( (pala_x>KEPALA_KANAN_KECIL+300)&&(pala_x<KEPALA_KIRI_BESAR-300)&&(pos.X>0)) ||
													( (pala_x<(KEPALA_KANAN_KECIL+300)) && (pos.X<80)  &&(pos.X>0) ) ||
													( (pala_x>(KEPALA_KIRI_BESAR-300) ) && (pos.X>240))
												)&&(gawang_musuh_pos.X<0))
												{
													if (sGawang_a==0)
														sGawang_a=pala_x;
													sGawang_jbaru=pala_x;
													if (sGawang_jlama==0)
														sGawang_jlama=pala_x;
													
													if ((sGawang_jbaru-sGawang_jlama)>sGawang_jjarak)
													{
														sGawang_jjarak=sGawang_jbaru-sGawang_jlama;
														sGawang_b=sGawang_jlama;
														sGawang_c=sGawang_jbaru;
													}
													sGawang_jlama = pala_x;
													if (sGawang_c!=0)
														sGawang_d=pala_x;
													cout<<pala_x<<":"<<pala_y<<"\t"<<pos.X<<","<<pos.Y<<endl;
												}
												pala_x += 80;
												arahkan_kepala();
											}
											
											
											//2. Stare gawang sambil revolusi sampe sejajar
											if (sGawang_a>0)//gawang terlihat
											{
												
												//MILIH
												if (abs(((sGawang_a+sGawang_b)/2)-KEPALA_TENGAH_X) < abs(((sGawang_c+sGawang_d)/2)-KEPALA_TENGAH_X)) ///LEBIH DEKAT TIANG KANAN
												{
													pala_x = (sGawang_a+sGawang_b)/2;
													milih=0;
													cout<<"AMBIL TIANG KANAN\n";
												}
												else
												{
													pala_x = (sGawang_c+sGawang_d)/2;
													milih=1;
													cout<<"AMBIL TIANG KIRI\n";
												}
												
												for (int dbgksr=40;dbgksr>0;dbgksr-=4)
												{
													pala_x = pala_x -dbgksr;
													arahkan_kepala();
													usleep(150*1000);
													cout<<pala_x<<endl;
													pala_x = pala_x +dbgksr;
												}
												cout<<"GAWANG TERLIHAT\n";
												cout<<"stare gawang\n";
												//2. Stare gawang sambil revolusi sampe sejajar
												perintahkangerakan=0;
												int jarak_antar_tiang=400;
												//~ if (abs(pala_x-2048)<=400)
													//~ jarak_antar_tiang=140;
												jarak_antar_tiang = ((sGawang_c+sGawang_d)/2) - ((sGawang_a+sGawang_b)/2);
												//~ while ((abs(pala_x-2048)>jarak_antar_tiang)&&(pos.X>0))
												
												cout<<sGawang_a<<" "<<sGawang_b<<" "<<sGawang_c<<" "<<sGawang_d<<endl;
												cout<<"milih "<<milih<<"\t abs((KEPALA_TENGAH_X+jarak_antar_tiang)-pala_x)"
													<<abs((KEPALA_TENGAH_X+jarak_antar_tiang)-pala_x)<<"\tabs(pala_x - (KEPALA_TENGAH_X-jarak_antar_tiang))"
													<<abs(pala_x - (KEPALA_TENGAH_X-jarak_antar_tiang))<<"\t (jarak_antar_tiang/2)"<<(jarak_antar_tiang/2)<<endl;
												if (!
														(
															(((sGawang_a+sGawang_b)/2)<(KEPALA_TENGAH_X-100)) &&
															(((sGawang_c+sGawang_d)/2)>(KEPALA_TENGAH_X+100))
														)
													)///SUDaH HADAP TENGAH TIANG
												while (
														((milih==0) &&  //ACUAN tiang kanan, krn tubuh mengarah ke sebelah kanannya. condongin kiri
																	( 
																		(pala_x <(KEPALA_TENGAH_X-(jarak_antar_tiang/2)-(jarak_antar_tiang/2))) || //REV KIRI
																		(pala_x >(KEPALA_TENGAH_X-(jarak_antar_tiang/2)+(jarak_antar_tiang/2)))    //REV KAN
																	)
														) || 
														
														((milih==1) &&  //ACUAN tiang kiri, krn tubuh mengarah ke sebelah kirinya. condongin kanan
																	(
																		(pala_x <(KEPALA_TENGAH_X+(jarak_antar_tiang/2)-(jarak_antar_tiang/2))) || //REV KIRI
																		(pala_x >(KEPALA_TENGAH_X+(jarak_antar_tiang/2)+(jarak_antar_tiang/2)))    //REV KAN
																	)
														) 
												)
												{
													//~ cout<<"masuk";
													arahkan_kepala();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
													if (pos.X>0)
													{
														stare(pos.X,pos.Y);
													}
													else
													{
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);		
														if (pos.X>0)
														{
															stare(pos.X,pos.Y);
														}
														else
														{
															int bates=pala_x;
															for (pala_x=(bates-300);pala_x<=(bates+300);pala_x+=JARAK_SAPU_X/8)
															{
																cout<<"nyari";
																arahkan_kepala();
																LinuxCamera::GetInstance()->CaptureFrame();
																LinuxCamera::GetInstance()->CaptureFrame();
																pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
																if (pos.X>0)
																{
																	cout<<"gawang seems found (sweep)\n";
																	//~ usleep(500*1000);				
																	LinuxCamera::GetInstance()->CaptureFrame();
																	LinuxCamera::GetInstance()->CaptureFrame();
																	LinuxCamera::GetInstance()->CaptureFrame();
																	LinuxCamera::GetInstance()->CaptureFrame();
																	LinuxCamera::GetInstance()->CaptureFrame();
																	pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
																	if (pos.X>0)
																	{
																		cout<<"gawang found (sweep)\n";
																		break;
																	}
																}
															}
															if (pos.X>0)
															{
																cout<<"ok";
															}
															else
															{
																perintahkangerakan=0;
																cout<<"ilang meneh\n";
																break;
															}
														}
													}
													if (perintahkangerakan>10)
													{
														cout<<pala_x<<"\t";
														if (milih==0)  //condongin kiri
														{
															cout<<"milih tiang kanan";
															cout<<"\t"<<(KEPALA_TENGAH_X-(jarak_antar_tiang/2)+(jarak_antar_tiang/2));
															if (pala_x >(KEPALA_TENGAH_X-(jarak_antar_tiang/2)+(jarak_antar_tiang/2)))
															{
																motion_revolusi_kanan();
																cout<<"   revolusi kanan sejajarin gawang\n";
															}
															else
															{
																motion_revolusi_kiri();
																cout<<"   revolusi kiri sejajarin gawang\n";
															}
														}
														else   //condongin kanan
														{
															cout<<"milih tiang kiri";
															cout<<"\t"<<(KEPALA_TENGAH_X+(jarak_antar_tiang/2)-(jarak_antar_tiang/2));
															if (pala_x <(KEPALA_TENGAH_X+(jarak_antar_tiang/2)-(jarak_antar_tiang/2)))
															{
																motion_revolusi_kiri();
																cout<<"   revolusi kiri sejajarin gawang\n";
															}
															else
															{
																motion_revolusi_kanan();
																cout<<"   revolusi kanan sejajarin gawang\n";
															}
														}
														perintahkangerakan=0;
													}
													else
													{
														perintahkangerakan++;
													}
												}
												if (pos.X<0)
													cout<<"ilang meneh\n";

											}
											else//andalin compass
											{
												while ((abs(sudut_error)>=20)&&(pos.X>0))
												{
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
													pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
													if (pos.X<0)
															break;
													streamer->send_image(rgb_ball);
														
													compass_hitung_acuan();
													if (sudut_error>20)
													{
														motion_revolusi_kanan();
														cout<<"revolusi kanan--gawang gak terlihat\n";
													}
													else 
													{
														motion_revolusi_kiri();
														cout<<"revolusi kiri--gawang gak terlihat\n";
													}
													//~ usleep(1000*1000);
													//~ motion_balance();
													//~ usleep(1000*1000);
												}
											}
											//kembaliin settingan ke kalibrasi bola
											LinuxCamera::GetInstance()->LoadINISettings(ini);
											samplinglihatgawang = 0;
			///END LIHAT GAWANG BARU 

										}
										else
										{
											samplinglihatgawang++;
											if (
														(
															(abs(sudut_error)>=25) &&
															(last_action!=ACTION_REVOLUSI_KANAN) &&
															(last_action!=ACTION_REVOLUSI_KIRI)
														) || (abs(sudut_error)>=40)
													)
												{
													int sampling_compass=0;
													while ((abs(sudut_error)>=20)&&(pos.X>0))
													{
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
														pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
														streamer->send_image(rgb_ball);
														
														if (sampling_compass<1)
														{
															compass_hitung_acuan();
															sampling_compass = ((abs(sudut_error)/10)-1)*8;
															cout<<"."<<sampling_compass<<endl;
														}
														else
														{
															sampling_compass--;
															cout<<"."<<sampling_compass<<endl;
														}
														
														if (sudut_error>20)
														{
															motion_revolusi_kanan();
															cout<<"revolusi kanan\n";
														}
														else 
														{
															motion_revolusi_kiri();
															cout<<"revolusi kiri\n";
														}
														//~ usleep(500*1000);
														//~ motion_balance();
														//~ usleep(1000*1000);
													}
												}
										}
										batasi_kepala_y_aktiv =1;
									}
									arah_gawang_sudah_berubah=false;
									
									///MAU TENDANG
									{
										pala_y = KEPALA_BAWAH_KECIL;
										for (pala_x=1600; pala_x<=2048; pala_x+=JARAK_SAPU_X/2)
										{
											arahkan_kepala();
											usleep(100*1000);
										}
										pala_x = 1900;
										pala_y = 1200;
										arahkan_kepala();
										usleep(100*1000);
										pala_x = 2040;
										pala_y = 1000;
										arahkan_kepala();
										usleep(100*1000);
										pala_x = 2048;
										pala_y = 1030;
										arahkan_kepala();
										
										//~ motion_balance();
										//~ motion_jalan_ditempat();
										//~ motion_jalan_pelan();
										//~ atur_kepala_x(pala_x);
										//~ arahkan_kepala();
										//~ arahkan_kepala();
										//~ usleep(500*1000);
										//~ atur_kepala_y(pala_y);
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
										if (pos.X>167)
											keputusan_tendang = TENDANG_KANAN;
										else
											keputusan_tendang = TENDANG_KIRI;
										cout<<"preparasi tendang \n";
										if (keputusan_tendang == TENDANG_KANAN)
										{
											while (pos.X>0)
											{
												pala_x = 1900;
												pala_y = 1200;
												arahkan_kepala();
												usleep(100*1000);
												pala_x = 2100;
												pala_y = 1020;
												arahkan_kepala();
												usleep(100*1000);
												pala_x = 2048;
												pala_y = 1030;
												arahkan_kepala();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
												pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												streamer->send_image(rgb_ball);
												if (pos.X<0)
													break;
												cout << pos.X<<","<<pos.Y;
												if (pos.X<180)
												{
													motion_geser_kiri();
													cout<<" geser kiri\n";
													//~ usleep(1000*1000);
												}
												else if (pos.X>220)
												{
													motion_geser_kanan();
													cout<<" geser kanan\n";
													//~ usleep(1000*1000);
												}
												else if (pos.Y<150)
												{
													motion_jalan_pelan();
													cout<<" jalan pelan\n";
													//~ usleep(1000*1000);
												}
												else
												{
													cout<<"mau tendang\n";
													motion_tendang_kanan();
													usleep(3000*1000);
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													//~ motion_balance();
												}
												//~ usleep(1000*1000);
											}
										}
										else ///TENDANG KIRI
										{
											while (pos.X>0)
											{
												pala_x = 1900;
												pala_y = 1200;
												arahkan_kepala();
												usleep(100*1000);
												pala_x = 2100;
												pala_y = 1020;
												arahkan_kepala();
												usleep(100*1000);
												pala_x = 2048;
												pala_y = 1030;
												arahkan_kepala();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
												pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												streamer->send_image(rgb_ball);
												if (pos.X<0)
													break;
												cout << pos.X<<","<<pos.Y;
												if (pos.X<110)
												{
													motion_geser_kiri();
													cout<<" geser kiri\n";
													//~ usleep(1000*1000);
												}
												else if (pos.X>150)
												{
													motion_geser_kanan();
													cout<<" geser kanan\n";
													//~ usleep(1000*1000);
												}
												else if (pos.Y<140)
												{
													motion_jalan_pelan();
													cout<<" jalan pelan\n";
													//~ usleep(1000*1000);
												}
												else
												{
													cout<<"mau tendang\n";
													motion_tendang_kiri();
													usleep(3000*1000);
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													//~ motion_balance();
												}
												//~ usleep(1000*1000);
											}
										}
									}
								}
							}
							else ///MODE GIRING BOLA
							{
///MODE GIRING BOLA

								cout<<"preparasi tendang tendang "<<pala_x<<":"<<pala_y;
								//~ motion_jalan_ditempat();
								
								if (arah_gawang_sudah_berubah)
								{
									
									compass_hitung_acuan();
									///samplinglihatgawang initialnya 5
									///SAMPLING LIHAT GAWANG kalo samplinglihatgawang = 5
									///berarti LIHAT GAWANG, klo kurang masih belum
									if (samplinglihatgawang>=0)
									{
										if ((abs(sudut_error)>=60))
										{
											if (
													(
														(abs(sudut_error)>=20) &&
														(last_action!=ACTION_REVOLUSI_KANAN) &&
														(last_action!=ACTION_REVOLUSI_KIRI)
													) || (abs(sudut_error)>=40)
												)
											{
												int sampling_compass=0;
												while ((abs(sudut_error)>=20)&&(pos.X>0))
												{
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
													pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
													streamer->send_image(rgb_ball);
													
													if (sampling_compass<1)
													{
														compass_hitung_acuan();
														sampling_compass = ((abs(sudut_error)/10)-1)*8;
														cout<<"."<<sampling_compass<<endl;
													}
													else
													{
														sampling_compass--;
														cout<<"."<<sampling_compass<<endl;
													}
													
													if (sudut_error>20)
													{
														motion_revolusi_kanan();
														cout<<"revolusi kanan\n";
													}
													else 
													{
														motion_revolusi_kiri();
														cout<<"revolusi kiri\n";
													}
													//~ usleep(500*1000);
													//~ motion_balance();
													//~ usleep(1000*1000);
												}
											}
										}
										
										/*
		///LIHAT GAWANG LAMA
						
										///CEK GAWANG
										batasi_kepala_y_aktiv=0;
										compass_hitung_acuan();
										//1. Cari gawang
											pala_y = KEPALA_ATAS_BESAR+100;
												pala_x = KEPALA_KANAN_KECIL;
											arahkan_kepala();
											arahkan_kepala();
											usleep(400*1000);
											//1. Cari gawang
											LinuxCamera::GetInstance()->LoadINISettings(ini_gawang);
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
											while ((pos.X<0)&&(pala_x<KEPALA_KIRI_BESAR))
											{
												cout<<"CARI GAWANGNYA "<<pos.X<<","<<pos.Y<<"\n";
												pala_x+=JARAK_SAPU_X/2;
												arahkan_kepala();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												if (pos.X>0)
												{
													cout<<"Gawang seems found\n";
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												}
											}
										//2. Stare gawang sambil revolusi sampe sejajar
										if (pos.X>0)//gawang terlihat
										{
											
											
											arahkan_kepala();
											cout<<"GAWANG TERLIHAT\n";
											cout<<"stare gawang\n";
											//2. Stare gawang sambil revolusi sampe sejajar
											perintahkangerakan=0;
											int arahkan_gawang_batas_search=400;
											if (abs(pala_x-2048)<=400)
												arahkan_gawang_batas_search=140;
											while ((abs(pala_x-2048)>arahkan_gawang_batas_search)&&(pos.X>0))
											{
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												if (pos.X>0)
												{
													stare(pos.X,pos.Y);
												}
												else
												{
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);		
													if (pos.X>0)
													{
														stare(pos.X,pos.Y);
													}
													else
													{
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														LinuxCamera::GetInstance()->CaptureFrame();
														pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);											
														if (pos.X>0)
														{
															stare(pos.X,pos.Y);
														}
														else
														{
															perintahkangerakan=0;
														}
													}
												}
												if (perintahkangerakan>10)
												{
													if (pala_x>2000)
													{
														motion_revolusi_kanan();
														cout<<"revolusi kanan sejajarin gawang";
													}
													else //pala_x<2000
													{
														motion_revolusi_kiri();
														cout<<"revolusi kiri sejajarin gawang";
													}
													perintahkangerakan=0;
												}
												else
												{
													perintahkangerakan++;
												}
											}
											if (pos.X<0)
												cout<<"ilang meneh\n";
										}
										else//andalin compass
										{
											while ((abs(sudut_error)>=20)&&(pos.X>0))
											{
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
												pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												if (pos.X<0)
														break;
												streamer->send_image(rgb_ball);
													
												compass_hitung_acuan();
												if (sudut_error>20)
												{
													motion_revolusi_kanan();
													cout<<"revolusi kanan--gawang gak terlihat\n";
												}
												else 
												{
													motion_revolusi_kiri();
													cout<<"revolusi kiri--gawang gak terlihat\n";
												}
												//~ usleep(1000*1000);
												//~ motion_balance();
												//~ usleep(1000*1000);
											}
										}
										//kembaliin settingan ke kalibrasi bola
										LinuxCamera::GetInstance()->LoadINISettings(ini);
										samplinglihatgawang = 0;
									
									*/
		///LIHAT GAWANG BARU 
										///LIAT GAWANG
										LinuxCamera::GetInstance()->LoadINISettings(ini_gawang);
										pala_y = KEPALA_ATAS_BESAR+100;
										pala_x = KEPALA_KANAN_KECIL;
										arahkan_kepala();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
										gawang_musuh_pos = finder_gawang_musuh->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
										
										int sGawang_a=0,sGawang_b=0,sGawang_c=0,sGawang_d=0,sGawang_jbaru=0,sGawang_jlama=0,sGawang_jjarak=0;
										int milih;
										while (pala_x<=KEPALA_KIRI_BESAR-1)
										{
											if ( ((KEPALA_KIRI_BESAR-pala_x)<50) || ((pala_x-KEPALA_KANAN_KECIL)<50) )
											{
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
											}
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
											gawang_musuh_pos = finder_gawang_musuh->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
										
											if ((
												( (pala_x>KEPALA_KANAN_KECIL+300)&&(pala_x<KEPALA_KIRI_BESAR-300)&&(pos.X>0)) ||
												( (pala_x<(KEPALA_KANAN_KECIL+300)) && (pos.X<80)  &&(pos.X>0) ) ||
												( (pala_x>(KEPALA_KIRI_BESAR-300) ) && (pos.X>240))
											)&&(gawang_musuh_pos.X<0))
											{
												if (sGawang_a==0)
													sGawang_a=pala_x;
												sGawang_jbaru=pala_x;
												if (sGawang_jlama==0)
													sGawang_jlama=pala_x;
												
												if ((sGawang_jbaru-sGawang_jlama)>sGawang_jjarak)
												{
													sGawang_jjarak=sGawang_jbaru-sGawang_jlama;
													sGawang_b=sGawang_jlama;
													sGawang_c=sGawang_jbaru;
												}
												sGawang_jlama = pala_x;
												if (sGawang_c!=0)
													sGawang_d=pala_x;
												cout<<pala_x<<":"<<pala_y<<"\t"<<pos.X<<","<<pos.Y<<endl;
											}
											pala_x += 80;
											arahkan_kepala();
										}/*
										///LIAT GAWANG
										LinuxCamera::GetInstance()->LoadINISettings(ini_gawang);
										pala_y = KEPALA_ATAS_BESAR+100;
										pala_x = KEPALA_KANAN_KECIL;
										arahkan_kepala();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
										int sGawang_a=0,sGawang_b=0,sGawang_c=0,sGawang_d=0,sGawang_jbaru=0,sGawang_jlama=0,sGawang_jjarak=0;
										int milih;
										int pala_adagawang[28];
										int index_pala=0;
										while (pala_x<=KEPALA_KIRI_BESAR-1)
										{
											if ( ((KEPALA_KIRI_BESAR-pala_x)<50) || ((pala_x-KEPALA_KANAN_KECIL)<50) )
											{
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
											}
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
											
											if (
												( (pala_x>KEPALA_KANAN_KECIL+300)&&(pala_x<KEPALA_KIRI_BESAR-300)&&(pos.X>0)) ||
												( (pala_x<(KEPALA_KANAN_KECIL+300)) && (pos.X<160)  &&(pos.X>0) ) ||
												( (pala_x>(KEPALA_KIRI_BESAR-300) ) && (pos.X>160))
											)
											{
												pala_adagawang[index_pala] = pala_x;
												index_pala++;
												
											}
											pala_x += 100;
											arahkan_kepala();
										}
										
										///HITUNG HEHE
										//~ for (int mulai=1;mulai<index_pala;mulai++)
										int mulai=0;
										while (sGawang_a==sGawang_b && mulai<1)
										{
											mulai++;
											sGawang_a=0;sGawang_b=0;sGawang_c=0;sGawang_d=0;sGawang_jbaru=0;sGawang_jlama=0;sGawang_jjarak=0;
											for (int ii=mulai;ii<index_pala;ii++)
											{
													if (sGawang_a==0)
														sGawang_a=pala_adagawang[ii];
													sGawang_jbaru=pala_adagawang[ii];
													if (sGawang_jlama==0)
														sGawang_jlama=pala_adagawang[ii];
													
													if ((sGawang_jbaru-sGawang_jlama)>sGawang_jjarak)
													{
														sGawang_jjarak=sGawang_jbaru-sGawang_jlama;
														sGawang_b=sGawang_jlama;
														sGawang_c=sGawang_jbaru;
													}
													sGawang_jlama = pala_adagawang[ii];
													if (sGawang_c!=0)
														sGawang_d=pala_adagawang[ii];
													cout<<pala_adagawang[ii]<<":"<<pala_y<<"\t"<<pos.X<<","<<pos.Y<<ii<<endl;
											}
											cout<<(sGawang_a+sGawang_b)/2<<"\t"<<(sGawang_c+sGawang_d)/2<<endl;
										}
										*/
										
										//2. Stare gawang sambil revolusi sampe sejajar
										if (sGawang_a>0)//gawang terlihat
										{
											
											//MILIH
											if (abs(((sGawang_a+sGawang_b)/2)-KEPALA_TENGAH_X) < abs(((sGawang_c+sGawang_d)/2)-KEPALA_TENGAH_X)) ///LEBIH DEKAT TIANG KANAN
											{
												pala_x = (sGawang_a+sGawang_b)/2;
												milih=0;
												cout<<"AMBIL TIANG KANAN\n";
											}
											else
											{
												pala_x = (sGawang_c+sGawang_d)/2;
												milih=1;
												cout<<"AMBIL TIANG KIRI\n";
											}
											
											for (int dbgksr=40;dbgksr>0;dbgksr-=4)
											{
												pala_x = pala_x -dbgksr;
												arahkan_kepala();
												usleep(150*1000);
												cout<<pala_x<<endl;
												pala_x = pala_x +dbgksr;
											}
											cout<<"GAWANG TERLIHAT\n";
											cout<<"stare gawang\n";
											//2. Stare gawang sambil revolusi sampe sejajar
											perintahkangerakan=0;
											int jarak_antar_tiang=400;
											//~ if (abs(pala_x-2048)<=400)
												//~ jarak_antar_tiang=140;
											jarak_antar_tiang = ((sGawang_c+sGawang_d)/2) - ((sGawang_a+sGawang_b)/2);
											//~ while ((abs(pala_x-2048)>jarak_antar_tiang)&&(pos.X>0))
											
											cout<<sGawang_a<<" "<<sGawang_b<<" "<<sGawang_c<<" "<<sGawang_d<<endl;
											cout<<"milih "<<milih<<"\t abs((KEPALA_TENGAH_X+jarak_antar_tiang)-pala_x)"
												<<abs((KEPALA_TENGAH_X+jarak_antar_tiang)-pala_x)<<"\tabs(pala_x - (KEPALA_TENGAH_X-jarak_antar_tiang))"
												<<abs(pala_x - (KEPALA_TENGAH_X-jarak_antar_tiang))<<"\t (jarak_antar_tiang/2)"<<(jarak_antar_tiang/2)<<endl;
											
											
											
											if (!
													(
														(((sGawang_a+sGawang_b)/2)<(KEPALA_TENGAH_X-100)) &&
														(((sGawang_c+sGawang_d)/2)>(KEPALA_TENGAH_X+100))
													)
												)///SUDaH HADAP TENGAH TIANG
											while (
													((milih==0) &&  //ACUAN tiang kanan, krn tubuh mengarah ke sebelah kanannya. condongin kiri
																( 
																	(pala_x <(KEPALA_TENGAH_X-(jarak_antar_tiang/2)-(jarak_antar_tiang/2))) || //REV KIRI
																	(pala_x >(KEPALA_TENGAH_X-(jarak_antar_tiang/2)+(jarak_antar_tiang/2)))    //REV KAN
																)
													) || 
													
													((milih==1) &&  //ACUAN tiang kiri, krn tubuh mengarah ke sebelah kirinya. condongin kanan
																(
																	(pala_x <(KEPALA_TENGAH_X+(jarak_antar_tiang/2)-(jarak_antar_tiang/2))) || //REV KIRI
																	(pala_x >(KEPALA_TENGAH_X+(jarak_antar_tiang/2)+(jarak_antar_tiang/2)))    //REV KAN
																)
													) 
											)
											{
												cout<<"masuk";
												arahkan_kepala();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												if (pos.X>0)
												{
													stare(pos.X,pos.Y);
												}
												else
												{
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);		
													if (pos.X>0)
													{
														stare(pos.X,pos.Y);
													}
													else
													{
														int bates=pala_x;
														for (pala_x=(bates-300);pala_x<=(bates+300);pala_x+=JARAK_SAPU_X/8)
														{
															cout<<"nyari";
															arahkan_kepala();
															LinuxCamera::GetInstance()->CaptureFrame();
															LinuxCamera::GetInstance()->CaptureFrame();
															pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
															if (pos.X>0)
															{
																cout<<"gawang seems found (sweep)\n";
																//~ usleep(500*1000);				
																LinuxCamera::GetInstance()->CaptureFrame();
																LinuxCamera::GetInstance()->CaptureFrame();
																LinuxCamera::GetInstance()->CaptureFrame();
																LinuxCamera::GetInstance()->CaptureFrame();
																LinuxCamera::GetInstance()->CaptureFrame();
																pos = finder_gawang->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
																if (pos.X>0)
																{
																	cout<<"gawang found (sweep)\n";
																	break;
																}
															}
														}
														if (pos.X>0)
														{
															cout<<"ok";
														}
														else
														{
															perintahkangerakan=0;
															cout<<"ilang meneh\n";
															break;
														}
													}
												}
												if (perintahkangerakan>10)
												{
													cout<<pala_x<<"\t";
													if (milih==0)  //condongin kiri
													{
														cout<<"milih tiang kanan";
														cout<<"\t"<<(KEPALA_TENGAH_X-(jarak_antar_tiang/2)+(jarak_antar_tiang/2));
														if (pala_x >(KEPALA_TENGAH_X-(jarak_antar_tiang/2)+(jarak_antar_tiang/2)))
														{
															motion_revolusi_kanan();
															cout<<"   revolusi kanan sejajarin gawang\n";
														}
														else
														{
															motion_revolusi_kiri();
															cout<<"   revolusi kiri sejajarin gawang\n";
														}
													}
													else   //condongin kanan
													{
														cout<<"milih tiang kiri";
														cout<<"\t"<<(KEPALA_TENGAH_X+(jarak_antar_tiang/2)-(jarak_antar_tiang/2));
														if (pala_x <(KEPALA_TENGAH_X+(jarak_antar_tiang/2)-(jarak_antar_tiang/2)))
														{
															motion_revolusi_kiri();
															cout<<"   revolusi kiri sejajarin gawang\n";
														}
														else
														{
															motion_revolusi_kanan();
															cout<<"   revolusi kanan sejajarin gawang\n";
														}
													}
													perintahkangerakan=0;
												}
												else
												{
													perintahkangerakan++;
												}
											}
											if (pos.X<0)
											{
												cout<<"ilang meneh\n";
											}

										}
										else//andalin compass
										{
											while ((abs(sudut_error)>=20)&&(pos.X>0))
											{
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
												pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
												if (pos.X<0)
														break;
												streamer->send_image(rgb_ball);
													
												compass_hitung_acuan();
												if (sudut_error>20)
												{
													motion_revolusi_kanan();
													cout<<"revolusi kanan--gawang gak terlihat\n";
												}
												else 
												{
													motion_revolusi_kiri();
													cout<<"revolusi kiri--gawang gak terlihat\n";
												}
												//~ usleep(1000*1000);
												//~ motion_balance();
												//~ usleep(1000*1000);
											}
										}
										//kembaliin settingan ke kalibrasi bola
										LinuxCamera::GetInstance()->LoadINISettings(ini);
										samplinglihatgawang = 0;
		///END LIHAT GAWANG BARU 

									}
									else
									{
										samplinglihatgawang++;
										if (
													(
														(abs(sudut_error)>=25) &&
														(last_action!=ACTION_REVOLUSI_KANAN) &&
														(last_action!=ACTION_REVOLUSI_KIRI)
													) || (abs(sudut_error)>=40)
												)
											{
												int sampling_compass=0;
												while ((abs(sudut_error)>=20)&&(pos.X>0))
												{
													LinuxCamera::GetInstance()->CaptureFrame();
													LinuxCamera::GetInstance()->CaptureFrame();
													memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
													pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
													streamer->send_image(rgb_ball);
													
													if (sampling_compass<1)
													{
														compass_hitung_acuan();
														sampling_compass = ((abs(sudut_error)/10)-1)*8;
														cout<<"."<<sampling_compass<<endl;
													}
													else
													{
														sampling_compass--;
														cout<<"."<<sampling_compass<<endl;
													}
													
													if (sudut_error>20)
													{
														motion_revolusi_kanan();
														cout<<"revolusi kanan\n";
													}
													else 
													{
														motion_revolusi_kiri();
														cout<<"revolusi kiri\n";
													}
													//~ usleep(500*1000);
													//~ motion_balance();
													//~ usleep(1000*1000);
												}
											}
									}
									batasi_kepala_y_aktiv =1;
								}
								arah_gawang_sudah_berubah=false;
								
								///MAU TENDANG
								{
									pala_y = KEPALA_BAWAH_KECIL;
									for (pala_x=1600; pala_x<=2048; pala_x+=JARAK_SAPU_X/2)
									{
										arahkan_kepala();
										usleep(50*1000);
									}
									pala_x = 1900;
									pala_y = 1200;
									arahkan_kepala();
									usleep(100*1000);
									pala_x = 2040;
									pala_y = 1000;
									arahkan_kepala();
									usleep(100*1000);
									pala_x = 2048;
									pala_y = 1030;
									arahkan_kepala();
									
									//~ motion_balance();
									//~ motion_jalan_ditempat();
									//~ motion_jalan_pelan();
									//~ atur_kepala_x(pala_x);
									//~ arahkan_kepala();
									//~ arahkan_kepala();
									//~ usleep(500*1000);
									//~ atur_kepala_y(pala_y);
									LinuxCamera::GetInstance()->CaptureFrame();
									LinuxCamera::GetInstance()->CaptureFrame();
									LinuxCamera::GetInstance()->CaptureFrame();
									LinuxCamera::GetInstance()->CaptureFrame();
									LinuxCamera::GetInstance()->CaptureFrame();
									LinuxCamera::GetInstance()->CaptureFrame();
									pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
									if (pos.X>167)
										keputusan_tendang = TENDANG_KANAN;
									else
										keputusan_tendang = TENDANG_KIRI;
									cout<<"preparasi tendang \n";
									if (keputusan_tendang == TENDANG_KANAN)
									{
										while (pos.X>0)
										{
											pala_x = 1900;
											pala_y = 1200;
											arahkan_kepala();
											usleep(100*1000);
											pala_x = 2100;
											pala_y = 1020;
											arahkan_kepala();
											usleep(100*1000);
											pala_x = 2048;
											pala_y = 1030;
											arahkan_kepala();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
											pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
											streamer->send_image(rgb_ball);
											if (pos.X<0)
												break;
											cout << pos.X<<","<<pos.Y;
											if (pos.X<175)
											{
												motion_geser_kiri();
												cout<<" geser kiri\n";
												//~ usleep(1000*1000);
											}
											else if (pos.X>225)
											{
												motion_geser_kanan();
												cout<<" geser kanan\n";
												//~ usleep(1000*1000);
											}
											else if (pos.Y<145)
											{
												motion_jalan();
												cout<<" jalan\n";
											}
											else if (pos.Y<150)
											{
												motion_jalan_pelan();
												cout<<" jalan pelan\n";
												//~ usleep(1000*1000);
											}
											else
											{
												cout<<"mau tendang giringan\n";
												motion_jalan_pelan();
												//~ usleep(3000*1000);
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												//~ motion_balance();
												break;
											}
											//~ usleep(1000*1000);
										}
									}
									else ///TENDANG KIRI
									{
										while (pos.X>0)
										{
											pala_x = 1900;
											pala_y = 1200;
											arahkan_kepala();
											usleep(100*1000);
											pala_x = 2100;
											pala_y = 1020;
											arahkan_kepala();
											usleep(100*1000);
											pala_x = 2048;
											pala_y = 1030;
											arahkan_kepala();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
											pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
											streamer->send_image(rgb_ball);
											if (pos.X<0)
												break;
											cout << pos.X<<","<<pos.Y;
											if (pos.X<105)
											{
												motion_geser_kiri();
												cout<<" geser kiri\n";
												//~ usleep(1000*1000);
											}
											else if (pos.X>155)
											{
												motion_geser_kanan();
												cout<<" geser kanan\n";
												//~ usleep(1000*1000);
											}
											else if (pos.Y<130)
											{
												motion_jalan_pelan();
												cout<<" jalan pelan\n";
												//~ usleep(1000*1000);
											}
											else
											{
												cout<<"mau tendang giringan\n";
												motion_jalan();
												//~ usleep(3000*1000);
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												//~ motion_balance();
												break;
											}
											//~ usleep(1000*1000);
										}
									}
								}
							




								
							}
						}
						else
						{
							if (mode_kickoff==MODE_KICKOFF_KANAN)
							{
									///KICK OFF
									cout<<"kickoff kanan\n";
									pala_y = KEPALA_BAWAH_KECIL;
									for (pala_x=1600; pala_x<=2048; pala_x+=JARAK_SAPU_X/2)
									{
										arahkan_kepala();
										usleep(100*1000);
									}
									pala_x = 1900;
									pala_y = 1200;
									arahkan_kepala();
									usleep(100*1000);
									pala_x = 2040;
									pala_y = 1000;
									arahkan_kepala();
									usleep(100*1000);
									pala_x = 2048;
									pala_y = 1030;
									arahkan_kepala();
									
									//~ motion_balance();
									//~ motion_jalan_ditempat();
									//~ motion_jalan_pelan();
									//~ atur_kepala_x(pala_x);
									//~ arahkan_kepala();
									//~ arahkan_kepala();
									//~ usleep(500*1000);
									//~ atur_kepala_y(pala_y);
									LinuxCamera::GetInstance()->CaptureFrame();
									LinuxCamera::GetInstance()->CaptureFrame();
									LinuxCamera::GetInstance()->CaptureFrame();
									LinuxCamera::GetInstance()->CaptureFrame();
									LinuxCamera::GetInstance()->CaptureFrame();
									LinuxCamera::GetInstance()->CaptureFrame();
									pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
									
									while (pos.X>0)
									{
										pala_x = 1900;
										pala_y = 1200;
										arahkan_kepala();
										usleep(100*1000);
										pala_x = 2100;
										pala_y = 1020;
										arahkan_kepala();
										usleep(100*1000);
										pala_x = 2048;
										pala_y = 1030;
										arahkan_kepala();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
										pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
										streamer->send_image(rgb_ball);
										if (pos.X<0)
											break;
										cout << pos.X<<","<<pos.Y;
										if (pos.X<165)
										{
											motion_geser_kiri();
											cout<<" geser kiri\n";
											//~ usleep(1000*1000);
										}
										else if (pos.X>195)
										{
											motion_geser_kanan();
											cout<<" geser kanan\n";
											//~ usleep(1000*1000);
										}
										else if (pos.Y<160)
										{
											motion_jalan_pelan();
											cout<<" jalan pelan\n";
											//~ usleep(1000*1000);
										}
										else
										{
											usleep(1000*1000);
											cout<<"mau tendang kickoff\n";
											motion_kickoff_kanan();
											//~ while (1);
											//~ for (int hitungankickoff=0;hitungankickoff<=1000;hitungankickoff++)
											//~ {
												//~ motion_kickoff_kanan();
												//~ usleep(2*1000);
											//~ }
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
											//~ while (pos.X>0)
											//~ {
												//~ stare(pos.X,pos.Y);
												//~ LinuxCamera::GetInstance()->CaptureFrame();
												//~ LinuxCamera::GetInstance()->CaptureFrame();
												//~ LinuxCamera::GetInstance()->CaptureFrame();
												//~ LinuxCamera::GetInstance()->CaptureFrame();
												//~ pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);											
												//~ cout<<pala_x<<":"<<pala_y<<endl;
											//~ }
											cout<<"done loss"<<endl;
											pala_y=1200;
											swipe_line(SWIPE_X,KEPALA_TENGAH_X,KEPALA_KANAN_KECIL);
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
											perintahkangerakan = 0;
											while (pos.X>0)
											{
												stare(pos.X,pos.Y);
												LinuxCamera::GetInstance()->CaptureFrame();
												LinuxCamera::GetInstance()->CaptureFrame();
												pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);											
												//~ cout<<pala_x<<":"<<pala_y<<endl;
												if (perintahkangerakan>3)
												{
													if (pala_x<(KEPALA_TENGAH_X-400))
														motion_geser_kanan();
													else
														break;
													perintahkangerakan=0;
													cout<<"geser";
												}
												else
												{
													perintahkangerakan++;
												}
											}
											//~ while(1);
											cout<<"test1";
											sudah_kickoff = 1;
											break;
											cout<<"test2";
										}
											cout<<"test3";
									}
											cout<<"test4";
							}
							else
							{
								///KICK OFF
								cout<<"kickoff BIASA\n";
								pala_y = KEPALA_BAWAH_KECIL;
								for (pala_x=1600; pala_x<=2048; pala_x+=JARAK_SAPU_X/2)
								{
									arahkan_kepala();
									usleep(100*1000);
								}
								pala_x = 1900;
								pala_y = 1200;
								arahkan_kepala();
								usleep(100*1000);
								pala_x = 2040;
								pala_y = 1000;
								arahkan_kepala();
								usleep(100*1000);
								pala_x = 2048;
								pala_y = 1030;
								arahkan_kepala();
								
								//~ motion_balance();
								//~ motion_jalan_ditempat();
								//~ motion_jalan_pelan();
								//~ atur_kepala_x(pala_x);
								//~ arahkan_kepala();
								//~ arahkan_kepala();
								//~ usleep(500*1000);
								//~ atur_kepala_y(pala_y);
								LinuxCamera::GetInstance()->CaptureFrame();
								LinuxCamera::GetInstance()->CaptureFrame();
								LinuxCamera::GetInstance()->CaptureFrame();
								LinuxCamera::GetInstance()->CaptureFrame();
								LinuxCamera::GetInstance()->CaptureFrame();
								LinuxCamera::GetInstance()->CaptureFrame();
								pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
								if (pos.X>167)
									keputusan_tendang = TENDANG_KANAN;
								else
									keputusan_tendang = TENDANG_KIRI;
								cout<<"preparasi kickoff \n";
								if (keputusan_tendang == TENDANG_KANAN)
								{
									while (pos.X>0)
									{
										pala_x = 1900;
										pala_y = 1200;
										arahkan_kepala();
										usleep(100*1000);
										pala_x = 2100;
										pala_y = 1020;
										arahkan_kepala();
										usleep(100*1000);
										pala_x = 2048;
										pala_y = 1030;
										arahkan_kepala();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
										pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
										streamer->send_image(rgb_ball);
										if (pos.X<0)
											break;
										cout << pos.X<<","<<pos.Y;
										if (pos.X<180)
										{
											motion_geser_kiri();
											cout<<" geser kiri\n";
											//~ usleep(1000*1000);
										}
										else if (pos.X>220)
										{
											motion_geser_kanan();
											cout<<" geser kanan\n";
											//~ usleep(1000*1000);
										}
										else if (pos.Y<140)
										{
											motion_jalan_pelan();
											cout<<" jalan pelan\n";
											//~ usleep(1000*1000);
										}
										else
										{
											cout<<"mau tendang kickoff\n";
											for (int hitungankickoff=0;hitungankickoff<=1000;hitungankickoff++)
											{
												motion_jalan_pelan();
												usleep(2*1000);
											}
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											sudah_kickoff = 1;
											break;
											//~ motion_balance();
										}
										//~ usleep(1000*1000);
									}
								}
								else ///TENDANG KIRI
								{
									while (pos.X>0)
									{
										pala_x = 1900;
										pala_y = 1200;
										arahkan_kepala();
										usleep(100*1000);
										pala_x = 2100;
										pala_y = 1020;
										arahkan_kepala();
										usleep(100*1000);
										pala_x = 2048;
										pala_y = 1030;
										arahkan_kepala();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										LinuxCamera::GetInstance()->CaptureFrame();
										memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
										pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
										streamer->send_image(rgb_ball);
										if (pos.X<0)
											break;
										cout << pos.X<<","<<pos.Y;
										if (pos.X<110)
										{
											motion_geser_kiri();
											cout<<" geser kiri\n";
											//~ usleep(1000*1000);
										}
										else if (pos.X>150)
										{
											motion_geser_kanan();
											cout<<" geser kanan\n";
											//~ usleep(1000*1000);
										}
										else if (pos.Y<140)
										{
											motion_jalan_pelan();
											cout<<" jalan pelan\n";
											//~ usleep(1000*1000);
										}
										else
										{
											cout<<"mau tendang kickoff\n";
											for (int hitungankickoff=0;hitungankickoff<=1000;hitungankickoff++)
											{
												motion_jalan_pelan();
												usleep(2*1000);
											}
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											LinuxCamera::GetInstance()->CaptureFrame();
											sudah_kickoff = 1;
											break;
											//~ motion_balance();
										}
										//~ usleep(1000*1000);
									}
								}
							}
						}
					}
					cout<<endl;
					perintahkangerakan = 0;
				}
				else
				{
					//~ motion_jalan_ditempat();
					perintahkangerakan++;
					if ((last_palax+pala_x)<abs(pala_x)) //SALAH SATU NEGATIF, LAIN POSITIF
					{
						perintahkangerakan = batasperintahkangerakan+10;
					}
					else if ((last_action==ACTION_ROTASI_KANAN) || (last_action==ACTION_ROTASI_KIRI))
					{
						perintahkangerakan = perintahkangerakan+5;
					}
					else if (pala_y<=1300)
					{
						if (last_action!=ACTION_KEJAR)
							perintahkangerakan = batasperintahkangerakan+10;
					}
					else if ((pala_x>2500) && (last_action!=ACTION_ROTASI_KIRI))
					{
						perintahkangerakan = perintahkangerakan+5;
					}
					else if ((pala_x>2200) && (last_action!=ACTION_SERONG_KIRI))
					{
						perintahkangerakan = perintahkangerakan+5;
					}
					else if ((pala_x<1500) && (last_action!=ACTION_ROTASI_KANAN))
					{
						perintahkangerakan = perintahkangerakan+5;
					}
					else if ((pala_x<1800) && (last_action!=ACTION_SERONG_KANAN))
					{
						perintahkangerakan = perintahkangerakan+5;
					}
					else if (last_action!=ACTION_JALAN)
					{
						perintahkangerakan = perintahkangerakan+5;
					}
					
				}
			}
			else
			{
				perintahkangerakan = 0;
				cout<<"lost\n";
				//~ usleep(500*1000);				
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				if (pos.X<0)
				{
					cout<<"surely lost\n";
					last_errorx = 0;
					last_errory = 0;
					motion_stop_close();
					//~ motion_balance();
					cout<<"balance \n";
					//~ usleep(400*1000);
					if (mode_main==MODE_MAIN_GIRING)
					{
						swipe_line(SWIPE_Y,pala_y,KEPALA_ATAS_BESAR);
					}
					else if (hilang_pertamakali)
					{
						kejar();	
						hilang_pertamakali=0;	
						if (pala_x>KEPALA_TENGAH_X)
							hilang_arahrotasi = HILANG_ARAH_ROTASI_KIRI;
						else
							hilang_arahrotasi = HILANG_ARAH_ROTASI_KANAN;
					}
					else
					{
						sapu_total();
					}
					
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					if (pos.X<0)
					{
						pala_x=2000;
						pala_y=1400;
						arahkan_kepala();
						for (int rot=0;rot<=60;rot++)
						{
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					
							if (pos.X<0)
							{
								if (hilang_arahrotasi == HILANG_ARAH_ROTASI_KANAN)
								{
									motion_rotasi_kanan();
									cout<<" rotasi kanan cari bola\n";
									usleep(50*1000);
								}
								else
								{
									motion_rotasi_kiri();
									cout<<" rotasi kiri cari bola\n";
									usleep(50*1000);
								}
							}
							else
							{
								rot=61;
							}
						}
					}
				}
				
			}
			streamer->send_image(rgb_ball);
		}
#define AA__________PELANGGARAN
		else if ((state == PELANGGARAN)&&((waktu_hitung_sekarang()-waktu_terakhir_selesai_pickup)>=10))
		{
			harus_jalan_pickup = 1;
			//Pelanggaran state
			motion_stop_close();
			//service robot bek
			pala_x = KEPALA_TENGAH_X;
			pala_y = KEPALA_ATAS_BESAR-100;
			arahkan_kepala();
			int lamahilang=0;
			waktu_set_acuan();
			motion_stop_close();
			
			//ternyata stop kudu sering2 dicek, krn kadang ngebug
			int detik_terakhir_cek_stop=0;
			
			//~ while (state == PELANGGARAN)
			while ((waktu_hitung_acuan()<29) && (state == PELANGGARAN))
			{
				//~ motion_stop_close();
				if ((waktu_hitung_acuan()%2)==0)
				{
					if (waktu_hitung_acuan()!=detik_terakhir_cek_stop)
					{
						cout<<"stop cek=========================================================";
						motion_balance();
						detik_terakhir_cek_stop=waktu_hitung_acuan();
					}
				}
				LinuxCamera::GetInstance()->CaptureFrame();
				pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				if (pos.X>=0)
				{
					stare(pos.X,pos.Y);
					lamahilang=0;
				}
				else
				{
					usleep(200);
					if (lamahilang>50)
					{
						pala_x = KEPALA_TENGAH_X;
						pala_y = KEPALA_ATAS_BESAR-100;
						arahkan_kepala();
					}
					else
						lamahilang++;
					cout<<lamahilang<<endl;
				}
			}
			///UDAH KELUAR DR PICKUP, meskipun blm dipencet Upennalize player
			waktu_terakhir_selesai_pickup = waktu_hitung_sekarang();
			force_play_flag = FORCE_PLAY_PLAYFROMPICKUP;
			int nilaitombol=baca_tombol();
			if ((nilaitombol==TOMBOL_ATAS_ON_BAWAH_ON) || (nilaitombol==TOMBOL_ATAS_ON_BAWAH_OFF))
			{
				harus_jalan_pickup = 2;
			}
			else
			{
				harus_jalan_pickup = 1;
			}
					
			if (nilaitombol==TOMBOL_ATAS_ON_BAWAH_ON || nilaitombol==TOMBOL_ATAS_OFF_BAWAH_ON)
			{
				//~ force_play_flag = FORCE_PLAY_PLAYNOKICKOFF;
				//~ mode_kickoff = MODE_KICKOFF_KANAN;
				mode_main = MODE_MAIN_GIRING;
			}
		}
///**
///**	PENALTI KICK
///**	PENALTY KYCK
///**
///**
///**

#define AA__________P_ALFAROBI_KICK
		else if(state == P_ALFAROBI_KICK)
		{
			//~ motion_jalan();
			//~ usleep(1000*1000);
			//penaltyshoot : alfarobi kicker, enemy kiper
			cout<<"penaltyshoot : alfarobi kicker";
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

			pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
			if (pos.X>=0)
			{
				if (pala_y>1930)
				{
					if (pala_x>2400)
					{
						motion_rotasi_kiri();
						cout<<"rotasi_kiri "<<pala_x<<":"<<pala_y;
						//~ usleep(1000*1000);
					}
					else if (pala_x>2100)
					{
						motion_serong_kiri();
						cout<<"serong_kiri "<<pala_x<<":"<<pala_y;
					}
					else if (pala_x<1600)
					{
						motion_rotasi_kanan();
						cout<<"rotasi_kanan "<<pala_x<<":"<<pala_y;
						//~ usleep(1000*1000);
					}
					else if (pala_x<1900)
					{
						motion_serong_kanan();
						cout<<"serong_kanan "<<pala_x<<":"<<pala_y;
					}
					else
					{
						motion_jalan_pelan();
						cout<<"jalan "<<pala_x<<":"<<pala_y;
					}
				}
				else
				{
					cout<<"preparasi tendang "<<pala_x<<":"<<pala_y;
					motion_rotasi_kiri();
					usleep(500*1000);
					motion_rotasi_kiri();
					usleep(500*1000);
					motion_rotasi_kiri();
					usleep(500*1000);
					motion_rotasi_kiri();
					compass_hitung_acuan();
					if (
						(
							(abs(sudut_error)>=25) &&
							(last_action!=ACTION_REVOLUSI_KANAN) &&
							(last_action!=ACTION_REVOLUSI_KIRI)
						) || (abs(sudut_error)>=40)
					)
					{
						while ((abs(sudut_error)>=25)&&(pos.X>0))
						{
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

							compass_hitung_acuan();
							if (sudut_error>=25)
							{
								if (sudut_error>=120)
								{
									motion_revolusi_kanan();
									usleep(1000*1000);
								}
								if (sudut_error>=90)
								{
									motion_revolusi_kanan();
									usleep(1000*1000);
								}
								if (sudut_error>=60)
								{
									motion_revolusi_kanan();
									usleep(1000*1000);
								}
								motion_revolusi_kanan();
								cout<<"revolusi kanan\n";
							}
							else 
							{
								if (sudut_error<=-120)
								{
									motion_revolusi_kiri();
									usleep(1000*1000);
								}
								if (sudut_error<=-90)
								{
									motion_revolusi_kiri();
									usleep(1000*1000);
								}
								if (sudut_error<=-60)
								{
									motion_revolusi_kiri();
									usleep(1000*1000);
								}
								motion_revolusi_kiri();
								cout<<"revolusi kiri\n";
							}
						}
					}///end arahin manut kompas
					///MAU TENDANG
					{
					pala_y = KEPALA_BAWAH_KECIL;
					for (pala_x=1600; pala_x<=2048; pala_x+=JARAK_SAPU_X/2)
					{
						arahkan_kepala();
						usleep(100*1000);
					}
					pala_x = 1900;
					pala_y = 1200;
					arahkan_kepala();
					usleep(100*1000);
					pala_x = 2040;
					pala_y = 1000;
					arahkan_kepala();
					usleep(100*1000);
					pala_x = 2048;
					pala_y = 1030;
					arahkan_kepala();
					
					//~ motion_balance();
					//~ motion_jalan_ditempat();
					//~ motion_jalan_pelan();
					//~ atur_kepala_x(pala_x);
					//~ arahkan_kepala();
					//~ arahkan_kepala();
					//~ usleep(500*1000);
					//~ atur_kepala_y(pala_y);
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					if (pos.X>167)
						keputusan_tendang = TENDANG_KANAN;
					else
						keputusan_tendang = TENDANG_KIRI;
					cout<<"preparasi tendang \n";
					if (keputusan_tendang == TENDANG_KANAN)
					{
						while (pos.X>0)
						{
							pala_x = 1900;
							pala_y = 1200;
							arahkan_kepala();
							usleep(100*1000);
							pala_x = 2100;
							pala_y = 1020;
							arahkan_kepala();
							usleep(100*1000);
							pala_x = 2048;
							pala_y = 1030;
							arahkan_kepala();
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
							pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
							streamer->send_image(rgb_ball);
							if (pos.X<0)
								break;
							cout << pos.X<<","<<pos.Y;
							if (pos.X<180)
							{
								motion_geser_kiri();
								cout<<" geser kiri\n";
								//~ usleep(1000*1000);
							}
							else if (pos.X>220)
							{
								motion_geser_kanan();
								cout<<" geser kanan\n";
								//~ usleep(1000*1000);
							}
							else if (pos.Y<110)
							{
								motion_jalan_pelan();
								cout<<" jalan pelan\n";
								//~ usleep(1000*1000);
							}
							else
							{
								cout<<"mau tendang\n";
								motion_tendang_kanan();
								usleep(3000*1000);
								LinuxCamera::GetInstance()->CaptureFrame();
								LinuxCamera::GetInstance()->CaptureFrame();
								LinuxCamera::GetInstance()->CaptureFrame();
								//~ motion_balance();
							}
							//~ usleep(1000*1000);
						}
					}
					else ///TENDANG KIRI
					{
						while (pos.X>0)
						{
							pala_x = 1900;
							pala_y = 1200;
							arahkan_kepala();
							usleep(100*1000);
							pala_x = 2100;
							pala_y = 1020;
							arahkan_kepala();
							usleep(100*1000);
							pala_x = 2048;
							pala_y = 1030;
							arahkan_kepala();
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							LinuxCamera::GetInstance()->CaptureFrame();
							memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
							pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
							streamer->send_image(rgb_ball);
							if (pos.X<0)
								break;
							cout << pos.X<<","<<pos.Y;
							if (pos.X<110)
							{
								motion_geser_kiri();
								cout<<" geser kiri\n";
								//~ usleep(1000*1000);
							}
							else if (pos.X>150)
							{
								motion_geser_kanan();
								cout<<" geser kanan\n";
								//~ usleep(1000*1000);
							}
							else if (pos.Y<110)
							{
								motion_jalan_pelan();
								cout<<" jalan pelan\n";
								//~ usleep(1000*1000);
							}
							else
							{
								cout<<"mau tendang\n";
								motion_tendang_kiri();
								usleep(3000*1000);
								LinuxCamera::GetInstance()->CaptureFrame();
								LinuxCamera::GetInstance()->CaptureFrame();
								LinuxCamera::GetInstance()->CaptureFrame();
								//~ motion_balance();
							}
							//~ usleep(1000*1000);
						}
					}
				}
					
				}///end preparasi tendang
			}
			else
			{
				cout<<"lost\n";
				usleep(500*1000);				
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				if (pos.X<0)
				{
					cout<<"surely lost\n";
					last_errorx = 0;
					last_errory = 0;
					{
						//~ motion_balance();
						motion_stop_close();
						cout<<"balance \n";
						usleep(400*1000);
					}
					kejar();			
					
					LinuxCamera::GetInstance()->CaptureFrame();
					pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					if (pos.X<0)
					{
						pala_x=2000;
						arahkan_kepala();
						for (int rot=0;rot<=8;rot++)
						{
							LinuxCamera::GetInstance()->CaptureFrame();
							pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					
							if (pos.X<0)
							{
								motion_rotasi_kanan();
								cout<<" rotasi kanan cari bola\n";
								usleep(1000*1000);
							}
							else
							{
								rot=10;
							}
						}
					}
				}
			}
			streamer->send_image(rgb_ball);
		}
#define AA__________P_ENEMY_KICK
		else if(state == P_ENEMY_KICK)
		{
			//penaltyshoot : enemy kicker, alfarobi kiper
		}
#define AA__________PICK_UP
		else if ((state == PICK_UP)&& ((waktu_hitung_sekarang()-waktu_terakhir_selesai_pickup)>=10) )
		{
			harus_jalan_pickup = 1;
			//Pelanggaran state
			motion_stop_close();
			pala_x = KEPALA_TENGAH_X;
			pala_y = KEPALA_ATAS_BESAR-100;
			arahkan_kepala();
			int lamahilang=0;
			waktu_set_acuan();
			motion_stop_close();
			int detik_terakhir_cek_stop=0;
			//~ while (state == PICK_UP)
			while ((state == PICK_UP) && (waktu_hitung_acuan()<29))
			{
				cout<<"pick up"<<waktu_selisih<<endl;
				if ((waktu_hitung_acuan()%2)==0)
				{
					if (waktu_hitung_acuan()!=detik_terakhir_cek_stop)
					{
						cout<<"stop cek=========================================================";
						motion_balance();
						detik_terakhir_cek_stop=waktu_hitung_acuan();
					}
				}
				//~ motion_stop_close();
				
				LinuxCamera::GetInstance()->CaptureFrame();
				pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				if (pos.X>=0)
				{
					stare(pos.X,pos.Y);
					lamahilang=0;
				}
				else
				{
					usleep(200);
					if (lamahilang>50)
					{
						pala_x = KEPALA_TENGAH_X;
						pala_y = KEPALA_ATAS_BESAR-100;
						arahkan_kepala();
					}
					else
						lamahilang++;
					cout<<lamahilang<<endl;
				}
			}
			///UDAH KELUAR DR PICKUP, meskipun blm dipencet Upennalize player
			waktu_terakhir_selesai_pickup = waktu_hitung_sekarang();
			force_play_flag = FORCE_PLAY_PLAYFROMPICKUP;
			
			int nilaitombol=baca_tombol();
			if ((nilaitombol==TOMBOL_ATAS_ON_BAWAH_ON) || (nilaitombol==TOMBOL_ATAS_ON_BAWAH_OFF))
			{
				harus_jalan_pickup = 2;
			}
			else
			{
				harus_jalan_pickup = 1;
			}
					
			if (nilaitombol==TOMBOL_ATAS_ON_BAWAH_ON || nilaitombol==TOMBOL_ATAS_OFF_BAWAH_ON)
			{
				//~ force_play_flag = FORCE_PLAY_PLAYNOKICKOFF;
				//~ mode_kickoff = MODE_KICKOFF_KANAN;
				mode_main = MODE_MAIN_GIRING;
			}
		}
#define AA__________SERVICE
		else if ((state == SERVICE) && ((waktu_hitung_sekarang()-waktu_terakhir_selesai_pickup)>=10))
		{
			harus_jalan_pickup = 1;
			//Pelanggaran state
			motion_stop_close();
			pala_x = KEPALA_TENGAH_X;
			pala_y = KEPALA_ATAS_BESAR-100;
			arahkan_kepala();
			int lamahilang=0;
			motion_stop_close();
			waktu_set_acuan();
			int detik_terakhir_cek_stop;
			while ((state == SERVICE) && (waktu_hitung_acuan()<58))
			{
				cout<<"pick up"<<waktu_selisih<<endl;
				//~ motion_stop_close();
				if ((waktu_hitung_acuan()%2)==0)
				{
					if (waktu_hitung_acuan()!=detik_terakhir_cek_stop)
					{
						cout<<"stop cek=========================================================";
						motion_balance();
						detik_terakhir_cek_stop=waktu_hitung_acuan();
					}
				}
				
				LinuxCamera::GetInstance()->CaptureFrame();
				pos = finder_bola->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				if (pos.X>=0)
				{
					stare(pos.X,pos.Y);
					lamahilang=0;
				}
				else
				{
					usleep(200);
					if (lamahilang>50)
					{
						pala_x = KEPALA_TENGAH_X;
						pala_y = KEPALA_ATAS_BESAR-100;
						arahkan_kepala();
					}
					else
						lamahilang++;
					cout<<lamahilang<<endl;
				}
			}
			///UDAH KELUAR DR PICKUP, meskipun blm dipencet Upennalize player
			waktu_terakhir_selesai_pickup = waktu_hitung_sekarang();
			force_play_flag = FORCE_PLAY_PLAYFROMPICKUP;
			
			int nilaitombol=baca_tombol();
			if ((nilaitombol==TOMBOL_ATAS_ON_BAWAH_ON) || (nilaitombol==TOMBOL_ATAS_ON_BAWAH_OFF))
			{
				harus_jalan_pickup = 2;
			}
			else
			{
				harus_jalan_pickup = 1;
			}
					
			if (nilaitombol==TOMBOL_ATAS_ON_BAWAH_ON || nilaitombol==TOMBOL_ATAS_OFF_BAWAH_ON)
			{
				//~ force_play_flag = FORCE_PLAY_PLAYNOKICKOFF;
				//~ mode_kickoff = MODE_KICKOFF_KANAN;
				mode_main = MODE_MAIN_GIRING;
			}
		}
		else
		{
			cout<<"mboh"<<endl;
			motion_tangan();
		}
    }

    return 0;
}

void batasi_kepala(void)
{
	
	if (pala_x>KEPALA_KIRI_BESAR)
		pala_x=KEPALA_KIRI_BESAR;
	else if (pala_x<KEPALA_KANAN_KECIL)
		pala_x=KEPALA_KANAN_KECIL;
		
	if (!(batasi_kepala_y_aktiv))
		return;
	if (pala_y>KEPALA_ATAS_BESAR)
		pala_y=KEPALA_ATAS_BESAR;
	else if (pala_y<KEPALA_BAWAH_KECIL)
		pala_y=KEPALA_BAWAH_KECIL;
	return;
}

void batasi_kepala(char fix)
{
	if (pala_x>KEPALA_KIRI_BESAR)
		pala_x=KEPALA_KIRI_BESAR;
	else if (pala_x<KEPALA_KANAN_KECIL)
		pala_x=KEPALA_KANAN_KECIL;
		
		
	if (pala_y>KEPALA_ATAS_BESAR)
		pala_y=KEPALA_ATAS_BESAR;
	else if (pala_y<KEPALA_BAWAH_KECIL)
	{
		pala_y=KEPALA_BAWAH_KECIL;
		
	}
	/*
	if (pala_x>3000)
	{
		pala_x=3000;
	}
	else if (pala_x<1100)
	{
		pala_x=1100;
	}
		
	if (pala_y>2500)
	{
		pala_y=2500;
	}
	else if (pala_y<1930)
	{
		if (pala_x<1700)
		{
			pala_x = 1700;
		}
		else if (pala_x>2330)
		{
			pala_x = 2330;
		}
		
		if (pala_y<1730)
		{
			pala_y = 1730;
		}
	}*/
}
void arahkan_kepala(void)
{
	batasi_kepala();
	//~ arahkan_kepala();
	atur_kepala_x(pala_x);
	atur_kepala_y(pala_y);
}
void arahkan_kepala(char leher)
{
	if (leher==0)
	{
		atur_kepala_x(pala_x);
	}
	else
	{
		atur_kepala_y(pala_y);
	}
	//~ atur_kepala_x(pala_x);
	//~ atur_kepala_y(pala_y);
}

void stare(int posx, int posy)
{
	//nilai kecil = kanan atas
	errorx = posx - X_TENGAH_FRAME; //kanan positif, kiri negativ
	errory = posy - Y_TENGAH_FRAME; //bawah positif, atas negativ
	if (abs(errorx)<=50)
	{
		Px = (PIDSPEED-4) * errorx/15;
		Dx = (PIDSPEED-4) * (errorx - last_errorx)/14;
	}
	else
	{
		Px = PIDSPEED * errorx/15;
		Dx = PIDSPEED * (errorx - last_errorx)/14;
	}
	if(abs(errory<=22))
	{
		Py = (PIDSPEED-4) * errory/25;
		Dy = (PIDSPEED-4) * (errory - last_errory)/24;
	}
	else
	{
		Py = PIDSPEED * errory/25;
		Dy = PIDSPEED * (errory - last_errory)/24;
	}
	
	pala_x = pala_x - Px - Dx; //kiri servo pala_x gede, kanan pala_x kecil
	pala_y = pala_y - Py - Dy; //kiri servo pala_x gede, kanan pala_x kecil
	arahkan_kepala();
}

void kejar()
{
	last_action = ACTION_KEJAR;
	//kanan positif, kiri negativ
	//bawah positif, atas negativ
	if (errorx<0) //tentang kejar kekiri dan keatas/kebawah (errorx negativ)
	{
		if (errory<0) //errorx<0 errory<0 --> kejar kekiri atau keatas
		{
			if (errorx<errory) //kejar kekiri nek errorx lebih extrim dr errory
			{
				//kejar ke kiri
				swipe_line(SWIPE_X,pala_x,KEPALA_KIRI_BESAR);
			}
			else 
			{
				//kejar ke atas
				swipe_line(SWIPE_Y,pala_y,KEPALA_ATAS_BESAR);
			}
		}
		else // --> kejar kekiri atau kebawah (errory positif)
		{
			if (abs(errorx)>errory) //kejar kekiri nek errorx lebih extrim dr errory
			{
				//kejar ke kiri
				swipe_line(SWIPE_X,pala_x,KEPALA_KIRI_BESAR);
			}
			else
			{
				//kejar kebawah
				swipe_line(SWIPE_Y,pala_y,KEPALA_BAWAH_KECIL);
			}
		}
	}
	else // tentang kejar kekanan dan keatas/kebawah (errorx positif)
	{
		if (errory<0) //errorx<0 errory<0 --> kejar kekanan atau keatas
		{
			if (errorx>abs(errory)) //kejar kekanan nek errorx lebih extrim dr errory
			{
				//kejar ke kanan
				swipe_line(SWIPE_X,pala_x,KEPALA_KANAN_KECIL);
			}
			else 
			{
				//kejar ke atas
				swipe_line(SWIPE_Y,pala_y,KEPALA_ATAS_BESAR);
			}
		}
		else // --> kejar kekanan atau kebawah (error y positif)
		{
			if (errorx>errory) //kejar kekanan nek errorx lebih extrim dr errory
			{
				//kejar ke kanan
				swipe_line(SWIPE_X,pala_x,KEPALA_KANAN_KECIL);
			}
			else
			{
				//kejar kebawah
				swipe_line(SWIPE_Y,pala_y,KEPALA_BAWAH_KECIL);
			}
		}
	}
}

void swipe_line(char SWIPETYPE, int start, int end)
{
	ColorFinder* l_finder = new ColorFinder();
	minIni* l_ini = new minIni("bola.ini");
    l_finder->LoadINISettings(l_ini);
    
	l_finder->m_min_percent=0.1;
	l_finder->m_max_percent=100;
	
	Point2D l_pos;
    
	
	if (SWIPETYPE==SWIPE_X)
	{
		if (end>start)
		{
			for (pala_x=start;pala_x<=end;pala_x+=JARAK_SAPU_X/8)
			{
				//~ batasi_kepala();
				arahkan_kepala();
				//~ usleep(200*1000);
				LinuxCamera::GetInstance()->CaptureFrame();
				l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				if (l_pos.X>0)
				{
					cout<<"seems found (sweep)\n";
					//~ usleep(500*1000);				
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					if (l_pos.X>0)
					{
						cout<<"found (sweep)\n";
						stare(l_pos.X,l_pos.Y);
						return;
					}
				}
			}
		}
		else
		{
			for (pala_x=end;pala_x>=start;pala_x-=JARAK_SAPU_X/8)
			{
				//~ batasi_kepala();
				arahkan_kepala();
				//~ usleep(200*1000);
				LinuxCamera::GetInstance()->CaptureFrame();
				l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				if (l_pos.X>0)
				{
					cout<<"seems found (sweep)\n";
					//~ usleep(500*1000);				
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					if (l_pos.X>0)
					{
						cout<<"found (sweep)\n";
						stare(l_pos.X,l_pos.Y);
						return;
					}
				}
			}
		}
	}
	else
	{
		if (end>start)
		{
			for (pala_y=start;pala_y<=end;pala_y+=JARAK_SAPU_Y/4)
			{
				//~ batasi_kepala();
				arahkan_kepala();
				//~ usleep(200*1000);
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				if (l_pos.X>0)
				{
					cout<<"seems found (sweep)\n";
					//~ usleep(500*1000);				
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					if (l_pos.X>0)
					{
						cout<<"found (sweep)\n";
						stare(l_pos.X,l_pos.Y);
						return;
					}
				}
			}	
		}
		else
		{
			for (pala_y=end;pala_y>=start;pala_y-=JARAK_SAPU_Y/4)
			{
				batasi_kepala();
				arahkan_kepala();
				//~ usleep(200*1000);
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				LinuxCamera::GetInstance()->CaptureFrame();
				l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
				if (l_pos.X>0)
				{
					cout<<"seems found (sweep)\n";
					//~ usleep(500*1000);				
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					if (l_pos.X>0)
					{
						cout<<"found (sweep)\n";
						stare(l_pos.X,l_pos.Y);
						return;
					}
				}
			}
		}
	}
	//sapu total
	sapu_total();
	return;
}

void sapu_total()
{
	ColorFinder* l_finder = new ColorFinder();
	minIni* l_ini = new minIni("bola.ini");
	l_finder->LoadINISettings(l_ini);
	char cari_jauh=0;
	l_finder->m_min_percent=0.01;
	l_finder->m_max_percent=100;
	
	Point2D l_pos;
	for (char ulang =0;ulang<=1;ulang++)
	{
		LinuxCamera::GetInstance()->CaptureFrame();
		LinuxCamera::GetInstance()->CaptureFrame();
		LinuxCamera::GetInstance()->CaptureFrame();
		LinuxCamera::GetInstance()->CaptureFrame();
		LinuxCamera::GetInstance()->CaptureFrame();
		l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		if (l_pos.X>0)
		{
			//~ usleep(1000*1000);
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			LinuxCamera::GetInstance()->CaptureFrame();
			l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
			if (l_pos.X>0)
			{
				return;
			}
		}
		int dbg;
		int kk=0;
		for (pala_y=KEPALA_BAWAH_KECIL;pala_y<KEPALA_ATAS_BESAR;pala_y+=(JARAK_SAPU_Y*0.6))
		{
			if (kk==0)
			{
				for (pala_x=KEPALA_KANAN_KECIL;pala_x<=KEPALA_KIRI_BESAR;pala_x+=JARAK_SAPU_X/4)
				{
					if ((state==PICK_UP) || (state==SERVICE))
						return;
			
					batasi_kepala();
					arahkan_kepala();
					if ((KEPALA_ATAS_BESAR-pala_y)<300)
					{
						//~ usleep(200*1000);
						cari_jauh=1;
						cout<<"cari bola jauh";
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
					}
					else
					{
						cari_jauh=0;
						//~ usleep(100*1000);
						cout<<"cari bola dekat";
					}
					LinuxCamera::GetInstance()->CaptureFrame();
					//~ LinuxCamera::GetInstance()->CaptureFrame();
					//~ LinuxCamera::GetInstance()->CaptureFrame();
					l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					//~ usleep(500*1000);
					
					cout<<l_pos.X<<","<<l_pos.Y<<"\t"<<pala_x<<":"<<pala_y<<"\n";
					//~ cin>>dbg;
					
					if (l_pos.X>0)
					{
						stare(l_pos.X,l_pos.Y);
						cout<<"found\n";
						if ((cari_jauh==1)&&(abs(pala_x-2048)<600))
							motion_jalan();
						//~ usleep(200*1000);				
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
						if (l_pos.X>0)
						{
							cout<<"found\n";
							stare(l_pos.X,l_pos.Y);
							return;
						}
						//~ usleep(250*1000);
						if (cari_jauh==1)
							motion_stop_close();
					}
				}
				kk=1;
			}
			else
			{
				for (pala_x=KEPALA_KIRI_BESAR;pala_x>=KEPALA_KANAN_KECIL;pala_x-=JARAK_SAPU_X/4)
				{
					if ((state==PICK_UP) || (state==SERVICE))
						return;
			
					batasi_kepala(0);
					arahkan_kepala();
					if ((KEPALA_ATAS_BESAR-pala_y)<300)
					{
						cari_jauh=1;
							
						//~ usleep(200*1000);
						cout<<"cari bola jauh";
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					LinuxCamera::GetInstance()->CaptureFrame();
					}
					else
					{
						cari_jauh=0;
						//~ usleep(100*1000);
						cout<<"cari bola dekat";
					}
					LinuxCamera::GetInstance()->CaptureFrame();
					//~ LinuxCamera::GetInstance()->CaptureFrame();
					//~ LinuxCamera::GetInstance()->CaptureFrame();
					l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
					//~ usleep(500*1000);
					
					cout<<l_pos.X<<","<<l_pos.Y<<"\t"<<pala_x<<":"<<pala_y<<"\n";
					//~ cin>>dbg;
					
					if (l_pos.X>0)
					{
						stare(l_pos.X,l_pos.Y);
						cout<<"found\n";
						if ((cari_jauh==1)&&(abs(pala_x-2048)<600))
							motion_jalan();
						//~ usleep(200*1000);				
						LinuxCamera::GetInstance()->CaptureFrame();
						LinuxCamera::GetInstance()->CaptureFrame();
						l_pos = l_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
						if (l_pos.X>0)
						{
							cout<<"found\n";
							stare(l_pos.X,l_pos.Y);
							return;
						}
						//~ usleep(250*1000);
						if (cari_jauh==1)
							motion_stop_close();
					}
				}
				kk=0;
			}
		}
		if (l_pos.X<0)
		{
			//~ motion_rotasi_kanan();
		}
	}
}

void compass_set_acuan(void)
{
	sudut_acuan = CompassSerial::GetInstance()->get_angle();
	return;
}

void compass_hitung_acuan(void)
{
	///mengubah nilai sudut_error
	/// dari sudut acuan, 180 derajat searah jarum jam nilai positif, 180derajat lawan jarum jam nilai negatif
	///CONTOH sudut_error:
	///		**(acuan)
	///   -45		 45
	///	 -90	 		90
	///   -120		120
	///		   180
	cout<<"hitung acuan, arahkan kepala\n";
	motion_stop();
	usleep(200*1000);
	atur_kepala_x(2048);
	atur_kepala_y(2503);
	usleep(1000*1000);
	int	selisih_sudut;
	sudut_skrg = CompassSerial::GetInstance()->get_angle();
	selisih_sudut 		= sudut_skrg - sudut_acuan;
//	sudut_error = 0;
//	return;
	if (selisih_sudut > 0)
	{
		if (selisih_sudut > 180)
			selisih_sudut = selisih_sudut - 360;
	}
	else
	{
		if (selisih_sudut < -180)
			selisih_sudut = 360 + selisih_sudut;
	}
	sudut_error = selisih_sudut;
	cout<<"skrg :"<<sudut_skrg<<"  eror: "<<sudut_error<<endl;
	arahkan_kepala();
	usleep(500*1000);
	return;
}


int compass_hitung_acuan_ret(void)
{
	///mengubah nilai sudut_error
	/// dari sudut acuan, 180 derajat searah jarum jam nilai positif, 180derajat lawan jarum jam nilai negatif
	///CONTOH sudut_error:
	///		**(acuan)
	///   -45		 45
	///	 -90	 		90
	///   -120		120
	///		   180
	cout<<"hitung acuan, arahkan kepala\n";
	motion_stop_close();
	usleep(200*1000);
	atur_kepala_x(2048);
	atur_kepala_y(2503);
	usleep(1000*1000);
	int	selisih_sudut;
	sudut_skrg = CompassSerial::GetInstance()->get_angle();
	selisih_sudut 		= sudut_skrg - sudut_acuan;
//	sudut_error = 0;
//	return;
	if (selisih_sudut > 0)
	{
		if (selisih_sudut > 180)
			selisih_sudut = selisih_sudut - 360;
	}
	else
	{
		if (selisih_sudut < -180)
			selisih_sudut = 360 + selisih_sudut;
	}
	sudut_error = selisih_sudut;
	cout<<"skrg :"<<sudut_skrg<<"  eror: "<<sudut_error<<endl;
	arahkan_kepala();
	usleep(500*1000);
	return sudut_error;
}

void port_init(char _ttynum)
{
	//~ porting.Open("/dev/ttyUSB1");
	
	if (_ttynum == 0)
		porting.Open("/dev/ttyUSB0");
	else 
		porting.Open("/dev/ttyUSB1");
		
	if(!porting.good())
	{
		cout<<"serial facepalm"<< endl;
		exit(1);
	}
	porting.SetBaudRate(SerialStreamBuf::BAUD_57600);
    if (!porting.good())
	{
		cout<<"serial baud facepalm"<< endl;
		exit(1);
	}
	//set the number of data bits
	porting.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the character size." <<
		endl ;
		exit(1) ;
	}
	//disable parity
	porting.SetParity(SerialStreamBuf::PARITY_NONE);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the parity" <<
		endl ;
		exit(1) ;
	}
	//stop bit = 1
	porting.SetNumOfStopBits(1);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the stop bit" <<
		endl ;
		exit(1) ;
	}
	//set flow control = none
	porting.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the flow control." <<
		endl;
		exit(1) ;
	}
}

void port_init_prevent(char _ttynum)
{
	//~ porting.Open("/dev/ttyUSB1");
	
	if (_ttynum == 0) ///klo prevetnya ttyUSB0, mulai dari 1
	{
		porting.Open("/dev/ttyUSB1");
		cout<<"cm700 ambil ttyUSB1"<<endl;
	}
	else ///klo preventnya bukan ttyUSB0, mulainya dari 0
	{
		porting.Open("/dev/ttyUSB0");
		cout<<"cm700 ambil ttyUSB0"<<endl;
	}
	
	if ((!porting.good()) && (_ttynum!=1))
	{
		porting.Open("/dev/ttyUSB1");
		cout<<"cm700 ambil ttyUSB1"<<endl;
	}
	if ((!porting.good()) && (_ttynum!=2))
	{
		porting.Open("/dev/ttyUSB2");
		cout<<"cm700 ambil ttyUSB2"<<endl;
	}
	if ((!porting.good()) && (_ttynum!=3))
	{
		porting.Open("/dev/ttyUSB3");
		cout<<"cm700 ambil ttyUSB3"<<endl;
	}
	if ((!porting.good()) && (_ttynum!=4))
	{
		porting.Open("/dev/ttyUSB4");
		cout<<"cm700 ambil ttyUSB4"<<endl;
	}
	if (!porting.good())
	{
		cout<<"0 sampe 4 error smw :(";
		exit(1);
	}
	cout<<" dan sukses"<<endl;
	
	porting.SetBaudRate(SerialStreamBuf::BAUD_57600);
    if (!porting.good())
	{
		cout<<"serial baud facepalm"<< endl;
		exit(1);
	}
	//set the number of data bits
	porting.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the character size." <<
		endl ;
		exit(1) ;
	}
	//disable parity
	porting.SetParity(SerialStreamBuf::PARITY_NONE);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the parity" <<
		endl ;
		exit(1) ;
	}
	//stop bit = 1
	porting.SetNumOfStopBits(1);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the stop bit" <<
		endl ;
		exit(1) ;
	}
	//set flow control = none
	porting.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the flow control." <<
		endl;
		exit(1) ;
	}
}
void kirim_data(int data)
{
    char first_head = 0xFF;
    char sec_head = 0x55;
    char data_msb = (data>>8);
    char data_lsb = data & ~(0xFF<<8);
    char inv_msb = ~data_msb;
    char inv_lsb = ~data_lsb;

    porting.write(&first_head,1);
    porting.write(&sec_head,1);
    porting.write(&data_lsb,1);
    porting.write(&inv_lsb,1);
    porting.write(&data_msb,1);
    porting.write(&inv_msb,1);
}

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int makin_get_index(int cx, int cy)
{
	return cy*Camera::WIDTH + cx;
}

int makin_get_x(int indeks)
{
	return (indeks%Camera::WIDTH);
}
int makin_get_y(int indeks)
{
	return (int(indeks/Camera::WIDTH))+1;
}

void motion_jalan()
{
	kirim_data(GERAKAN_JALAN);
	last_action = ACTION_JALAN;
	last_palax=pala_x;
	last_palay=pala_y;
}
void motion_jalan_pelan()
{
	kirim_data(GERAKAN_JALAN_PELAN);
	last_action = ACTION_JALAN_PELAN;
	last_palax=pala_x;
	last_palay=pala_y;
}
void motion_jalan_ditempat()
{
	kirim_data(GERAKAN_JALAN_DITEMPAT);
	last_action = ACTION_JALAN_DITEMPAT;
	last_palax=pala_x;
	last_palay=pala_y;
}
void motion_rotasi_kanan()
{
	kirim_data(GERAKAN_ROTASI_KANAN);
	last_action = ACTION_ROTASI_KANAN;
	last_palax=pala_x;
	last_palay=pala_y;
	arah_gawang_sudah_berubah=true;
}
void motion_rotasi_kiri()
{
	kirim_data(GERAKAN_ROTASI_KIRI);
	last_action = ACTION_ROTASI_KIRI;
	last_palax=pala_x;
	last_palay=pala_y;
	arah_gawang_sudah_berubah=true;
}
void motion_geser_kiri()
{
	kirim_data(GERAKAN_GESER_KIRI);
	last_action = ACTION_GESER_KIRI;
	last_palax=pala_x;
	last_palay=pala_y;
}
void motion_geser_kanan()
{
	kirim_data(GERAKAN_GESER_KANAN);
	last_action = ACTION_GESER_KANAN;
	last_palax=pala_x;
	last_palay=pala_y;
}
void motion_revolusi_kanan()
{
	kirim_data(GERAKAN_REVOLUSI_KANAN);
	last_action = ACTION_REVOLUSI_KANAN;
	last_palax=pala_x;
	last_palay=pala_y;
	arah_gawang_sudah_berubah=true;
}
void motion_revolusi_kiri()
{
	kirim_data(GERAKAN_REVOLUSI_KIRI);
	last_action = ACTION_REVOLUSI_KIRI;
	last_palax=pala_x;
	last_palay=pala_y;
	arah_gawang_sudah_berubah=true;
}
void motion_serong_kiri()
{
	kirim_data(GERAKAN_SERONG_KIRI);
	last_action = ACTION_SERONG_KIRI;
	last_palax=pala_x;
	last_palay=pala_y;
	arah_gawang_sudah_berubah=true;
}
void motion_serong_kanan()
{
	kirim_data(GERAKAN_SERONG_KANAN);
	last_action = ACTION_SERONG_KANAN;
	last_palax=pala_x;
	last_palay=pala_y;
	arah_gawang_sudah_berubah=true;
}
void motion_stop_close()
{
	kirim_data(GERAKAN_STOP_CLOSE);
	last_action=ACTION_STOP_CLOSE;
}
void motion_tangan()
{
	kirim_data(GERAKAN_TANGAN);
	last_action=ACTION_TANGAN;
}

void motion_balance()
{
	kirim_data(GERAKAN_BALANCE);
	last_action = ACTION_BALANCE;
	last_palax=pala_x;
	last_palay=pala_y;
}

void motion_tendang_kanan()
{
	kirim_data(GERAKAN_TENDANG_KANAN);
	last_action = ACTION_TENDANG_KANAN;
	last_palax=pala_x;
	last_palay=pala_y;
}

void motion_tendang_kiri()
{
	kirim_data(GERAKAN_TENDANG_KIRI);
	last_action = ACTION_TENDANG_KIRI;
	last_palax=pala_x;
	last_palay=pala_y;
}
void motion_tendang_belakang_kanan()
{
	kirim_data(GERAKAN_TENDANG_BELAKANG_KANAN);
	last_action = ACTION_TENDANG_BELAKANG_KANAN;
	last_palax=pala_x;
	last_palay=pala_y;
}


void motion_bangun_belakang()
{
	kirim_data(GERAKAN_BANGUN_BELAKANG);
	last_action = ACTION_BANGUN_BELAKANG;
	last_palax=pala_x;
	last_palay=pala_y;
}
void motion_bangun_depan()
{
	kirim_data(GERAKAN_BANGUN_DEPAN);
	last_action = ACTION_BANGUN_DEPAN;
	last_palax=pala_x;
	last_palay=pala_y;
}

void motion_mundur()
{
	kirim_data(GERAKAN_MUNDUR);
	last_action = ACTION_MUNDUR;
	last_palax = pala_x;
	last_palay = pala_y;
}
void motion_stop()
{
	kirim_data(GERAKAN_STOP);
	last_action = ACTION_STOP;
	last_palax = pala_x;
	last_palay = pala_y;
}
void motion_kickoff_kanan()
{
	kirim_data(GERAKAN_KICKOFF_KANAN);
	last_action = ACTION_KICKOFF_KANAN;
	last_palax = pala_x;
	last_palay = pala_y;
}

int baca_tombol()
{
	kirim_data(PERINTAH_MINTA_KONDISI_TOMBOL);
	return baca_data();
}


int baca_data(void)
{
    char first_head = 0xFF;
    char sec_head = 0x55;
    char data_msb;
    char data_lsb;
    char inv_msb;
    char inv_lsb;

	//~ kirim_data(PERINTAH_MINTA_KONDISI_TOMBOL);
    porting.read(&first_head,1);
    porting.read(&sec_head,1);
    porting.read(&data_lsb,1);
    porting.read(&inv_lsb,1);
    porting.read(&data_msb,1);
    porting.read(&inv_msb,1);
    cout<<int(first_head)<<" "<<int(sec_head)<<" "<<int(data_lsb)<<" "<<int(inv_lsb)<<" "<<int(data_msb)<<" "<<int(inv_msb)<<endl;
    
    return int(data_lsb);
}


void waktu_set_acuan(void)
{
	gettimeofday(&waktu_struct, NULL);
	waktu_acuan = waktu_struct.tv_sec;
}
int waktu_hitung_acuan(void)
{
	gettimeofday(&waktu_struct, NULL);
	waktu_sekarang = waktu_struct.tv_sec;
	waktu_selisih = waktu_sekarang - waktu_acuan;
	return waktu_selisih;
}
int waktu_hitung_sekarang(void)
{
	gettimeofday(&waktu_struct, NULL);
	waktu_sekarang = waktu_struct.tv_sec;
	return waktu_sekarang;
}
