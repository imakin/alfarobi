//BISMILLAHIRROHMANIRROHIM
//
// Allah


//#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <SerialStream.h>


using namespace cv;
using namespace std;
using namespace LibSerial;

#define atur_kepala_x(n) kirim_data((n/4));
#define atur_kepala_y(n) kirim_data((n/4)+10000);


#define PIDSPEED			8
#define X_TENGAH_FRAME 160
#define Y_TENGAH_FRAME 120
#define KEPALA_KIRI_BESAR 3400
#define KEPALA_KANAN_KECIL 600
#define KEPALA_ATAS_BESAR 1800
#define KEPALA_BAWAH_KECIL 1000
#define KEPALA_TENGAH_X 2048
#define JARAK_SAPU_X 354
#define JARAK_SAPU_Y 345
//fungsi swipe_line()
#define SWIPE_X 0
#define SWIPE_Y 1

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
struct timeval waktu_struct;
int waktu_acuan;
int waktu_sekarang;
int waktu_selisih;
void kirim_data(int data);
void arahkan_kepala(void);
void arahkan_kepala(char leher);
char batasi_kepala_y_aktiv=1;
char batasi_kepala_x_aktiv=1;
void batasi_kepala(void);
void port_init_prevent(char _ttynum);
void stare(int posx, int posy);
int radius;
int perintahkangerakan,batasperintahkangerakan;
Mat get_thresh(Mat img_input, int h_min, int h_max, int s_min, int s_max, int v_min, int v_max);
int samplingdpt=0;int samplinggkdpt=0; int sweeparah=0;



#define GERAKAN_BALANCE 				5001//ALFAROBI Certified
#define GERAKAN_BANGUN_DEPAN 			5002//ALFAROBI Certified
#define GERAKAN_BANGUN_BELAKANG 		5003//ALFAROBI Certified

#define GERAKAN_JALAN_DITEMPAT_LAMBAT 	5005
#define GERAKAN_JALAN 					5004//ALFAROBI Certified
#define GERAKAN_GESER_KANAN 			5011//ALFAROBI Certified
#define GERAKAN_GESER_KIRI 				5012//ALFAROBI Certified
#define GERAKAN_TENDANG_KANAN 			5017//ALFAROBI Certified
#define GERAKAN_ROTASI_KANAN 			5013//ALFAROBI Certified
#define GERAKAN_ROTASI_KIRI 			5014//ALFAROBI Certified
#define GERAKAN_JALAN_PELAN 			5012
#define GERAKAN_REVOLUSI_KANAN			5013
#define GERAKAN_REVOLUSI_KIRI			5014
#define GERAKAN_SERONG_KANAN			5006//ALFAROBI Certified
#define GERAKAN_SERONG_KANAN_20			5005//ALFAROBI Certified
#define GERAKAN_SERONG_KANAN_40			5006//ALFAROBI Certified
#define GERAKAN_SERONG_KANAN_60			5007//ALFAROBI Certified
#define GERAKAN_SERONG_KIRI_20			5008//ALFAROBI Certified
#define GERAKAN_SERONG_KIRI_40			5009//ALFAROBI Certified
#define GERAKAN_SERONG_KIRI_60			5010//ALFAROBI Certified
#define GERAKAN_SERONG_KIRI				5009//ALFAROBI Certified
#define GERAKAN_TENDANG_KIRI 			5018//ALFAROBI Certified
#define GERAKAN_STOP					5019//ALFAROBI Certified
#define GERAKAN_STOP_CLOSE		 		5018
#define GERAKAN_TANGAN		 			5019
#define GERAKAN_TENDANG_BELAKANG_KANAN	5020
#define PERINTAH_MINTA_KONDISI_TOMBOL	5021
#define GERAKAN_JALAN_DITEMPAT	 		5022
#define GERAKAN_MUNDUR					5023
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
#define ACTION_TENDANG_KIRI 			24
#define ACTION_STOP_CLOSE 				18
#define ACTION_TANGAN 					19
#define ACTION_TENDANG_BELAKANG_KANAN	20
#define ACTION_MINTA_KONDISI_TOMBOL		21
#define ACTION_JALAN_DITEMPAT 			22
#define ACTION_MUNDUR					23
#define ACTION_STOP						17
#define ACTION_KICKOFF_KANAN			25
int arah_gawang; ///NILAINYA ADALAH SUDUT ERROR
bool arah_gawang_sudah_berubah=true;
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


int main(int argc, char* argv[])
{
	port_init_prevent(30);
	
	
	VideoCapture cap(0);
	if(!cap.isOpened())
	{
		cout<<"kamera eror";
		return -1;
	}
    cap.set(CV_CAP_PROP_FRAME_WIDTH,320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,240);		

	Mat frame, thresh,thresh_item,thresh_putih, itemputih, hough, gray,gray_i,gray_p;
	vector<vector<Point> > contours; 
	vector<Vec4i> hierarchy;
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect(contours.size());

	if (argc>1)
	{
		namedWindow("gray");
		namedWindow("thresh_item");
		namedWindow("thresh_putih");
		namedWindow("item_putih");
		namedWindow("camera");
	}/**end if(argc>1)**/
	//~ 
	pala_x = KEPALA_TENGAH_X;
	
	pala_y = KEPALA_ATAS_BESAR;
	arahkan_kepala();
	usleep(300*1000);
	pala_y = KEPALA_ATAS_BESAR-300;
	arahkan_kepala();
	usleep(300*1000);
	
	perintahkangerakan = 0;
	batasperintahkangerakan=40;
	
	while (true)
	{
		cap>>frame;

		//~ thresh = get_thresh(frame,0,255,0,25,135,255);
		thresh_item = get_thresh(frame,0,255,0,100,0,120);
		thresh_putih = get_thresh(frame,0,255,0,100,135,255);
		//~ GaussianBlur(thresh, thresh, Size(11,11),2,2);
		GaussianBlur(thresh_item, thresh_item, Size(11,11),2,2);
		GaussianBlur(thresh_item, thresh_item, Size(11,11),2,2);
		GaussianBlur(thresh_putih, thresh_putih, Size(11,11),2,2);
		
		//~ findContours(thresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		//~ drawContours(thresh, contours, -1, Scalar::all(255), CV_FILLED);
		
		findContours(thresh_item, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		drawContours(thresh_item, contours, -1, Scalar::all(255), CV_FILLED);
		
		findContours(thresh_putih, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		drawContours(thresh_putih, contours, -1, Scalar::all(255), CV_FILLED);
		
		//konvert frame ke gray
		cvtColor(frame,gray,CV_BGR2GRAY);
		cvtColor(frame,gray_i,CV_BGR2GRAY);
		cvtColor(frame,gray_p,CV_BGR2GRAY);
		
		bitwise_and(thresh_item,thresh_putih,itemputih);
		bitwise_and(itemputih,gray,gray);
		bitwise_and(thresh_item,gray_i,gray_i);
		bitwise_and(thresh_putih,gray_p,gray_p);

		vector<Vec3f> circles;
		vector<Vec3f> circles_i;
		vector<Vec3f> circles_p;
		HoughCircles(gray, circles,CV_HOUGH_GRADIENT, 2, thresh_item.rows, 200, 120, 5);
		HoughCircles(gray_i, circles_i,CV_HOUGH_GRADIENT, 2, thresh_item.rows, 200, 120, 5);
		HoughCircles(gray_p, circles_p,CV_HOUGH_GRADIENT, 2, thresh_item.rows, 200, 120, 5);
		
		int compare_p_c_x=0,compare_p_c_y=0,compare_p_r=0;
		int compare_i_c_x=0,compare_i_c_y=0,compare_i_r=0;
		if (circles_i.size() != 0)
		{
			int maxi_rad = 0;	
			for(size_t i = 0;i < circles_i.size();i++)
			{
				Point center(cvRound(circles_i[i][0]),cvRound(circles_i[i][1]));
				radius = cvRound(circles_i[i][2]);
	
				if(radius > maxi_rad)
				{
					//~ circle(frame, center, 3, Scalar(0,255,0), 1);
					//~ circle(frame, center, radius, Scalar(0,255, 0),3);					
					cout<<"[i] cent x : "<<center.x<<" cent y : "<<center.y<<" rad :"<<radius<<" \n";		
					compare_i_c_x = center.x;
					compare_i_c_y = center.y;
					compare_i_r = radius;
				}
			}
		}
		if (circles_p.size() != 0)
		{
			int maxi_rad = 0;	
			for(size_t i = 0;i < circles_p.size();i++)
			{
				Point center(cvRound(circles_p[i][0]),cvRound(circles_p[i][1]));
				radius = cvRound(circles_p[i][2]);
	
				if(radius > maxi_rad)
				{
					//~ circle(frame, center, 3, Scalar(0,255,0), 1);
					//~ circle(frame, center, radius, Scalar(255,0, 0),3);					
					cout<<"[p] cent x : "<<center.x<<" cent y : "<<center.y<<" rad :"<<radius<<" \n";						
					compare_p_c_x = center.x;
					compare_p_c_y = center.y;
					compare_p_r = radius;
				}
			}
		}
		cout <<"[item]"<<compare_i_c_x<<","<<compare_i_c_y<<"\t[putih]"<<compare_p_c_x<<","<<compare_p_c_y<<"\n";
		cout <<"[item]"<<compare_i_r<<"\t[putih]"<<compare_p_r<<"\n";
		int jaraktitikkuadrat = ((abs(compare_i_c_x-compare_p_c_x)^2)+(abs(compare_i_c_y-compare_p_c_y)^2));
		cout<<"jaraktitik kuadrat : "<<jaraktitikkuadrat<<endl;
		if((circles.size() != 0) && (compare_i_r!=0) && (compare_p_r!=0) && (jaraktitikkuadrat<compare_p_r))
		{
		samplingdpt++;
		if (samplingdpt>2)
		{
			int max_rad = 0;	
			for(size_t i = 0;i < circles.size();i++)
			{
				Point center(cvRound(circles[i][0]),cvRound(circles[i][1]));
				radius = cvRound(circles[i][2]);
	
				if(radius > max_rad)
				{
					circle(frame, center, 3, Scalar(0,255,0), 1);
					circle(frame, center, radius, Scalar(0,0, 255),3);					

					cout<<"[b] cent x : "<<center.x<<" cent y : "<<center.y<<" rad :"<<radius<<" \n";						
///BOLA KEDETEKSI					
					if (samplingdpt>12)
						samplinggkdpt = -8;
					else if (samplingdpt>8)
						samplinggkdpt=0;
					stare(center.x,center.y);
					if (perintahkangerakan>batasperintahkangerakan)
					{
						if (
							(pala_y>1300)
							|| ((pala_x>2750)&&(pala_y<1450))
							|| ((pala_x<1350)&&(pala_y<1450))
						)
						{
							if ((pala_x>2750)&&(pala_y<1450))
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
							else if ((pala_x<1350)&&(pala_y<1450))
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
							
						}
						perintahkangerakan=0;
					}
					else
					{
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
				}/**end if(radius > max_rad)**/
			}/**end for(size_t i = 0;i < circles.size();i++)**/
		}
		else
		{
			//~ pala_x = KEPALA_TENGAH_X;
			//~ pala_y = KEPALA_ATAS_BESAR;
			//~ arahkan_kepala();
			//~ samplinggkdpt++;
			//~ if (samplinggkdpt>5)
			//~ {
				//~ cout<<"mode gak dapet cuy\n";
				//~ if (samplinggkdpt<7)
				//~ {
					//~ cout<<"INITIATE sweep\n";
					//~ pala_x = KEPALA_KANAN_KECIL;
					//~ pala_y = KEPALA_BAWAH_KECIL+200;
					//~ arahkan_kepala();
				//~ }
				//~ else
				//~ {
					//~ if (pala_y>KEPALA_ATAS_BESAR)
					//~ {
						//~ cout<<"EXE RAMPUNG";
					//~ }
					//~ else if (sweeparah==0)
					//~ {
						//~ if (pala_x<=KEPALA_KIRI_BESAR)
						//~ {
							//~ pala_x += (JARAK_SAPU_X/6);
							//~ cout<<"EXE sweep, arah KIRI";
							//~ arahkan_kepala();
						//~ }
						//~ else
						//~ {
							//~ sweeparah = 1;
							//~ pala_y += (JARAK_SAPU_Y*0.6);
							//~ cout<<"EXE sweep, arah NAIK";
							//~ arahkan_kepala();
						//~ }
					//~ }
					//~ else
					//~ {
						//~ if (pala_x>=KEPALA_KANAN_KECIL)
						//~ {
							//~ pala_x -= (JARAK_SAPU_X/6);
							//~ cout<<"EXE sweep, arah kanan";
							//~ arahkan_kepala();
						//~ }
						//~ else
						//~ {
							//~ sweeparah = 0;
							//~ pala_y += (JARAK_SAPU_Y*0.6);
							//~ cout<<"EXE sweep, arah NAIK";
							//~ arahkan_kepala();
						//~ }
					//~ }
				//~ }
			//~ }
			//~ else
			{
				motion_stop();
			}//~ usleep(3000*1000);
			
		}
		}
		else/**else if(circles.size() != 0)**/
		{
			samplingdpt=0;
			//~ Point center(0,0);
			radius = 0;			
			//~ center.x = center.y = 0;
			//~ cout<<"cent x : "<<center.x<<" cent y : "<<center.y<<" rad :"<<radius<<" \n";
			//~ pala_x = KEPALA_TENGAH_X;
			//~ pala_y = KEPALA_ATAS_BESAR;
			//~ arahkan_kepala();
			samplinggkdpt++;
			if (samplinggkdpt>5)
			{
				cout<<"mode gak dapet cuy\n";
				if (samplinggkdpt<7)
				{
					if (last_action==ACTION_ROTASI_KANAN)
					{					
						cout<<"INITIATE abis rotasi kanan\n";
						pala_x += 300;
						//~ pala_y = KEPALA_BAWAH_KECIL+200;
						arahkan_kepala();
					}
					else if (last_action==ACTION_ROTASI_KIRI)
					{					
						cout<<"INITIATE abis rotasi kiri\n";
						pala_x -= 300;
						//~ pala_y = KEPALA_BAWAH_KECIL+200;
						arahkan_kepala();
					}
					else
					{
						cout<<"INITIATE sweep\n";
						pala_x = KEPALA_KANAN_KECIL;
						pala_y = KEPALA_BAWAH_KECIL+200;
						arahkan_kepala();
					}
				}
				else
				{
					if (pala_y>KEPALA_ATAS_BESAR)
					{
						cout<<"EXE RAMPUNG";
						samplinggkdpt = 5;
						motion_stop();
					}
					else if (sweeparah==0)
					{
						if (pala_x<=KEPALA_KIRI_BESAR)
						{
							if (pala_y>1400)
								pala_x += (JARAK_SAPU_X/6);
							else
								pala_x += (JARAK_SAPU_X/3);
							cout<<"EXE sweep, arah KIRI";
							arahkan_kepala();
						}
						else
						{
							sweeparah = 1;
							pala_y += (JARAK_SAPU_Y*0.6);
							cout<<"EXE sweep, arah NAIK";
							arahkan_kepala();
						}
					}
					else
					{
						if (pala_x>=KEPALA_KANAN_KECIL)
						{
							if (pala_y>1400)
								pala_x -= (JARAK_SAPU_X/6);
							else
								pala_x -= (JARAK_SAPU_X/3);
							cout<<"EXE sweep, arah kanan";
							arahkan_kepala();
						}
						else
						{
							sweeparah = 0;
							pala_y += (JARAK_SAPU_Y*0.6);
							cout<<"EXE sweep, arah NAIK";
							arahkan_kepala();
						}
					}
				}
			}
			else
			{
				motion_stop();
			}
			//~ cout<<"stop";
			//~ usleep(3000*1000);
		}/**end if(circles.size() != 0)**/
		
		//cout<<"cent x : "<<center.x<<" cent y : "<<center.y<<" \n";	
		if (argc>1)
		{
			imshow("gray",gray);
			imshow("thresh_item",thresh_item);
			imshow("thresh_putih",thresh_putih);
			imshow("item_putih",itemputih);
			imshow("camera",frame);
		}/**end if (argc>1)**/
		if(waitKey(20) != -1);
			//~ break;
			//~ aaa=0;
	}/**end while(true)**/

	return 0;
}

Mat get_thresh(Mat img_input, int h_min, int h_max, int s_min, int s_max, int v_min, int v_max)
{
	Mat img_hsv, img_bw;
	cvtColor(img_input, img_hsv, CV_BGR2HSV);
	inRange(img_hsv,Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max),img_bw); //putih - abu2			
	//~ inRange(img_hsv,Scalar(h_min, 0, 135), Scalar(h_max, 25, 255),img_bw); //putih - abu2			
	//inRange(img_hsv,Scalar(hsv_min, 140, 140), Scalar(hsv_max, 255, 255),img_bw); //oranye
	
	return img_bw;
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
	
	pala_x = pala_x + Px + Dx; //kiri servo pala_x gede, kanan pala_x kecil
	pala_y = pala_y + Py + Dy; //kiri servo pala_x gede, kanan pala_x kecil
	arahkan_kepala();
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

void arahkan_kepala(void)
{
	batasi_kepala();
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
}

void batasi_kepala(void)
{
	
	if (pala_x>KEPALA_KIRI_BESAR)
		pala_x=KEPALA_KIRI_BESAR+1;
	else if (pala_x<KEPALA_KANAN_KECIL)
		pala_x=KEPALA_KANAN_KECIL-1;
		
	if (!(batasi_kepala_y_aktiv))
		return;
	if (pala_y>KEPALA_ATAS_BESAR)
		pala_y=KEPALA_ATAS_BESAR+1;
	else if (pala_y<KEPALA_BAWAH_KECIL)
		pala_y=KEPALA_BAWAH_KECIL-1;
	return;
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
		//~ exit(1);
	}
	cout<<" dan sukses"<<endl;
	
	porting.SetBaudRate(SerialStreamBuf::BAUD_57600);
    if (!porting.good())
	{
		cout<<"serial baud facepalm"<< endl;
		//exit(1);
	}
	//set the number of data bits
	porting.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the character size." <<
		endl ;
		//exit(1);
	}
	//disable parity
	porting.SetParity(SerialStreamBuf::PARITY_NONE);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the parity" <<
		endl ;
		//exit(1);
	}
	//stop bit = 1
	porting.SetNumOfStopBits(1);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the stop bit" <<
		endl ;
		//exit(1);
	}
	//set flow control = none
	porting.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
	if ( ! porting.good() )
	{
		cout << "Error: Could not set the flow control." <<
		endl;
		//exit(1);
	}
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
	//~ last_action = ACTION_STOP;
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
