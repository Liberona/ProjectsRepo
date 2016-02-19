//--------propiedad de Austral-Robotics.com, realizado por felipe castro nieny, fcastro@austral-robotics.com / 29-10-2015
//--------Modificado por Pablo Liberna, pliberona@austral-robotics.com / 29-10-2015

#define _CRT_SECURE_NO_DEPRECATE//Ignora error C4996 "fopen"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <cv.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <windows.h>
#include <conio.h>
#include <sstream>
#include <string>


using namespace cv;
using namespace std;

GUID uuid;//Direccion MAC
ofstream myfile;//Crea archivo txt 




int distanciareco = 0, estado = 0, posX = 0, posY = 0, iLastX = -1, iLastY = -1, areaMenor = 500, areaMayor = 2e3;
char centrotxt[50], centrotxtmm[50], centro[50], distancia[50], text[255], mac_addr[18];
char cantcontornos[50];

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		vector<Point> approx;

int lowerH = 0, lowerS = 0, lowerV = 107;
int upperH = 90, upperS = 256, upperV = 256;
int Vertice1_x = 58, Vertice2_x = 280, Vertice3_x = 80, Vertice1_y = 43, Vertice2_y = 40, Vertice3_y = 105;  //Vértices triángulo ROI
bool flipCamara = false;
bool flipParameters = false;
bool flipFile = false;

namespace{

	void DireccionMAC(GUID uuid, char mac_addr[18])
	{

		CoCreateGuid(&uuid);
		sprintf(mac_addr, "%02X:%02X:%02X:%02X:%02X:%02X",
			uuid.Data4[2], uuid.Data4[3], uuid.Data4[4],
			uuid.Data4[5], uuid.Data4[6], uuid.Data4[7]);
	}

	void ventanayTrackbar()
	{
		namedWindow("AntesDeEqualizar");
		namedWindow("DespuesDeEqualizar");
		namedWindow("HSV");
		namedWindow("THRESH");
		//namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE);

		moveWindow("AntesDeEqualizar", 0, 460);
		moveWindow("DespuesDeEqualizar", 0, 320);
		moveWindow("HSV", 680, 460);
		moveWindow("THRESH", 1020, 460);
		//moveWindow("calcHist Demo", 700, 400);
	}

	const string currentDateTime()//FUNCION 1: TIMESTAMP
	{
		time_t     now = time(0);
		struct tm  tstruct;
		char       buf[80];
		tstruct = *localtime(&now);
		strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
		return buf;
	}
	
	Mat graficarHistograma(Mat& src)
	{    /// Separate the image in 3 places ( B, G and R )
		vector<Mat> bgr_planes;
		split(src, bgr_planes);

		/// Establish the number of bins
		int histSize = 256;

		/// Set the ranges ( for B,G,R) )
		float range[] = { 0, 256 };
		const float* histRange = { range };

		bool uniform = true; bool accumulate = false;

		Mat b_hist, g_hist, r_hist;

		/// Compute the histograms:
		calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);

		// Draw the histograms for B, G and R
		int hist_w = 512; int hist_h = 200;
		int bin_w = cvRound((double)hist_w / histSize);

		Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

		/// Normalize the result to [ 0, histImage.rows ]
		normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
		normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
		normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

		/// Draw for each channel
		for (int i = 1; i < histSize; i++)
		{
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(b_hist.at<float>(i))),
				Scalar(255, 0, 0), 2, 8, 0);
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(g_hist.at<float>(i))),
				Scalar(0, 255, 0), 2, 8, 0);
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(r_hist.at<float>(i))),
				Scalar(0, 0, 255), 2, 8, 0);
		}
		/// Display
		imshow("calcHist Demo", histImage);
		
		return histImage;
	}
	
	Mat equalizarHistograma(const Mat& inputImage)//FUNCION 4: EQUALIZACION HISTOGRAMA
	{
		if (inputImage.channels() >= 3)
		{
			Mat ycrcb;

			cvtColor(inputImage, ycrcb, CV_BGR2YCrCb);

			vector<Mat> channels;
			split(ycrcb, channels);

			equalizeHist(channels[0], channels[0]);

			Mat result;
			merge(channels, ycrcb);

			cvtColor(ycrcb, result, CV_YCrCb2BGR);
			imshow("DespuesDeEqualizar", inputImage);
			return result;
		}
		return Mat();
	}

	Mat agregarROI(Mat& frame)
	{
		Point poly_points1[1][3];//POLIGONO 1
		//NOTA: Definir coordenadas en sentido horario
		//Según sentido ubicación aristas del polígono
		poly_points1[0][0] = Point(Vertice1_x, Vertice1_y);
		poly_points1[0][1] = Point(Vertice2_x, Vertice2_y);
		poly_points1[0][2] = Point(Vertice3_x, Vertice3_y);
		const Point* ppt1[1] = { poly_points1[0] };
		int npt1[] = { 3 };

		Point poly_points2[1][4];//POLIGONO 2
		poly_points2[0][0] = Point(0, 0);
		poly_points2[0][1] = Point(0, 57);
		poly_points2[0][2] = Point(320, 57);
		poly_points2[0][3] = Point(320, 0);
		const Point* ppt2[1] = { poly_points2[0] };
		int npt2[] = { 4 };

		Point poly_points3[1][4];//POLIGONO 3
		poly_points3[0][0] = Point(0, 0);
		poly_points3[0][1] = Point(0, 280);
		poly_points3[0][2] = Point(20, 280);
		poly_points3[0][3] = Point(20, 0);
		const Point* ppt3[1] = { poly_points3[0] };
		int npt3[] = { 4 };

		Point poly_points4[1][4];//POLIGONO 4
		poly_points4[0][0] = Point(0, 150);
		poly_points4[0][1] = Point(0, 250);
		poly_points4[0][2] = Point(320, 250);
		poly_points4[0][3] = Point(320, 150);
		const Point* ppt4[1] = { poly_points4[0] };
		int npt4[] = { 4 };

		Point poly_points5[1][4];//POLIGONO 5
		poly_points5[0][0] = Point(300, 0);
		poly_points5[0][1] = Point(300, 280);
		poly_points5[0][2] = Point(320, 280);
		poly_points5[0][3] = Point(320, 0);
		const Point* ppt5[1] = { poly_points5[0] };
		int npt5[] = { 4 };

		fillPoly(frame, ppt1, npt1, 1, Scalar(0, 0, 0), 8); //Dibuja triangulo negro N.1
		fillPoly(frame, ppt2, npt2, 1, Scalar(0, 0, 0), 8); //Dibuja poligono negro N.2
		fillPoly(frame, ppt3, npt3, 1, Scalar(0, 0, 0), 8); //Dibuja poligono negro N.3
		fillPoly(frame, ppt4, npt4, 1, Scalar(0, 0, 0), 8); //Dibuja poligono negro N.4
		fillPoly(frame, ppt5, npt5, 1, Scalar(0, 0, 0), 8); //Dibuja poligono negro N.5
		return frame;
	}

	Mat GetThresholdedImage(Mat& imgHSV)//This function threshold the HSV image and create a binary image
	{

		//IplImage* imgThresh = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
		Mat imgThresh = Mat(imgHSV.size(), CV_8UC1);
		inRange(imgHSV, Scalar(lowerH, lowerS, lowerV), Scalar(upperH, upperS, upperV), imgThresh);
		cv::imshow("HSV", imgHSV);
		return imgThresh;
	}

	Mat operacionCierre(const Mat& imgThresh){

		int erosion_elem = 0;
		int erosion_size = 1;
		int erosion_type;

		if (erosion_elem == 0){ erosion_type = MORPH_RECT; }
		else if (erosion_elem == 1){ erosion_type = MORPH_CROSS; }
		else if (erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

		Mat elementErode = getStructuringElement(erosion_type,
			Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			Point(erosion_size, erosion_size));
		erode(imgThresh, imgThresh, elementErode);
		erode(imgThresh, imgThresh, elementErode);

		int dilation_elem = 0;
		int dilation_size = 1;
		int dilation_type;

		if (dilation_elem == 0){ dilation_type = MORPH_RECT; }
		else if (dilation_elem == 1){ dilation_type = MORPH_CROSS; }
		else if (dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

		Mat elementDilation = getStructuringElement(dilation_type,
			Size(2 * dilation_size + 1, 2 * dilation_size + 1),
			Point(dilation_size, dilation_size));

		dilate(imgThresh, imgThresh, elementDilation);
		dilate(imgThresh, imgThresh, elementDilation);
		GaussianBlur(imgThresh, imgThresh, Size(3, 3), 0, 0);
		cv::imshow("THRESH", imgThresh);
		return Mat();
	}

	void MyFilledCircle(Mat img, Point2f center)
	{
		int thickness = 1;
		int lineType = 8;
		int w = 600.0;

		circle(img,
			center,
			w / 32.0,
			Scalar(0, 0, 255),
			thickness,
			lineType);
	}

}

int main(int argc, char** argv){
	Mat frame;
	Mat imgHSV = Mat(frame.size(), CV_8UC3);
	DireccionMAC(uuid, mac_addr);

	
	cout << "Bienvenido a Raton Vision Software, de Austral Robotics.-" << endl;
	Sleep(1000);
	cout << "Seguridad: Inserte la clave del software" << endl;

	string clave;
	getline(cin, clave);
	string ClaveReal = "arcimis2016";
	if (clave == ClaveReal)
	{
		cout << "Producto activado! Puede continuar" << endl;
	}
	else{
		cout << "Clave incorrecta! El software se cierra..." << endl;
		Sleep(1000);
		return -1;
	}
	
	MENU:

	cout << "Favor ingresar el numero de camara que desea ver (1-16):" << endl;
	string NumeroCamara;
	getline(cin, NumeroCamara);
	cout << "Se analizara la camara No."+ NumeroCamara << endl;
	Sleep(1000);
	cout << "Creando archivo de registro txt..." << endl;
	Sleep(1000);
	string BasePath = "C:\\RatonVision-v1.4\\";
	LPCWSTR LPCWSTRBasePath = L"C:\\RatonVision-v1.4\\";

	string TimeStamp = currentDateTime();
	
	string Año = TimeStamp.substr(0, 4);
	string Mes = TimeStamp.substr(5, 2);
	string Dia = TimeStamp.substr(8, 2);
	string Hora = TimeStamp.substr(11, 2);
	string Minuto = TimeStamp.substr(14, 2);
	string Archivo = "C"+NumeroCamara+"-"+Año+Mes+Dia+"-"+Hora+Minuto+".txt";
	
	string RutaCompleta = BasePath + Archivo;
	myfile.open(RutaCompleta);
	cout << "Directorio actual: " + RutaCompleta << endl;
	cout << "Listo! Los datos se guardaran en el archivo " + Archivo << endl;
	cout << "Ubicado en la carpeta " + BasePath << endl;
	//Sleep(1000);
	cout << "El programa comienza en breve..." << endl;

	std::string arg = argv[1];
	VideoCapture capture(arg);

	/*
	const string videoStreamAddress = "rtsp://192.168.1.70:554/user=admin&password=&channel="+NumeroCamara+"&stream=0.sdp?";
	VideoCapture capture(videoStreamAddress);
	if (!capture.open(videoStreamAddress)) {
		cout << "Error opening video stream or file" << endl;
		return -1;
	}
	else { cout << "Acceso correcto a DVR y streaming camara"<< endl; }
	*/
	
	ventanayTrackbar();

	Mat imageMenu;
	string MenuNum = "Menu Camara N." + NumeroCamara;
	string CamaraNum = "Camara N." + NumeroCamara;
	string ParametrosNum = "Parametros Camara N." + NumeroCamara;
	imageMenu = imread("Menu.png", CV_LOAD_IMAGE_COLOR);   // Read the file

	namedWindow(MenuNum, WINDOW_AUTOSIZE);// Create a window for display.

	imshow(MenuNum, imageMenu);                   // Show our image inside it.

	while(1) {

		int key = waitKey(10);

		if ((char)key == 27) //Si presiona tecla ESC "Cierra Programa y volver al Menú principal"
		{
			break;
		}
		if ((char)key == 99) //Si presiona tecla c "Abre Cámara/Cierra Cámara"
		{
			flipCamara = !flipCamara;
		}
		if ((char)key == 112) //Si presiona tecla p "Despliega/esconde menú de parámetros"
		{
			if (flipParameters == 1){
				cout << "Mostrar Parametros camara N." << NumeroCamara << endl;
				namedWindow(ParametrosNum);
				moveWindow(ParametrosNum, 0, 0);
				createTrackbar("Hmin", ParametrosNum, &lowerH, 180, NULL);
				createTrackbar("Hmax", ParametrosNum, &upperH, 180, NULL);

				createTrackbar("Smin", ParametrosNum, &lowerS, 256, NULL);
				createTrackbar("Smax", ParametrosNum, &upperS, 256, NULL);

				createTrackbar("Vmin", ParametrosNum, &lowerV, 256, NULL);
				createTrackbar("Vmax", ParametrosNum, &upperV, 256, NULL);

				createTrackbar("AreaMin", ParametrosNum, &areaMenor, 4e3, NULL);
				createTrackbar("AreaMax", ParametrosNum, &areaMayor, 4e4, NULL);

				createTrackbar("Vertice1x", ParametrosNum, &Vertice1_x, 300, NULL);
				createTrackbar("Vertice2x", ParametrosNum, &Vertice2_x, 300, NULL);
				createTrackbar("Vertice3x", ParametrosNum, &Vertice3_x, 300, NULL);

				createTrackbar("Vertice1y", ParametrosNum, &Vertice1_y, 300, NULL);
				createTrackbar("Vertice2y", ParametrosNum, &Vertice2_y, 300, NULL);
				createTrackbar("Vertice3y", ParametrosNum, &Vertice3_y, 300, NULL);

			}
			else
			{
				destroyWindow(ParametrosNum);
			}
			flipParameters = !flipParameters;
		}
		if ((char)key == 102) //Si presiona tecla f "Despliega/esconde Directorio de registro"
		{
			if (flipFile == 1){
				cout << "Abrir carpeta de registro" << endl;
				ShellExecute(NULL, NULL, LPCWSTRBasePath, NULL, NULL, SW_SHOWNOACTIVATE);
			}
			else{
				cout << "Esconder carpeta de registro" << endl;
				ShellExecute(NULL, NULL, LPCWSTRBasePath, NULL, NULL, SW_MINIMIZE);
			}
			flipFile = !flipFile;
		}

		capture >> frame;
		//Sleep(100);
		if (frame.empty())
			break;

		bool bSuccess = capture.read(frame); // read a new frame from video
		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read the frame from video file" << endl;
		}
		frame = frame.clone();
		imshow("AntesDeEqualizar", frame);
		equalizarHistograma(frame);	
		graficarHistograma(frame);
		agregarROI(frame);
		GaussianBlur(frame, frame, Size(3, 3), 0, 0);
		cvtColor(frame, imgHSV, CV_BGR2HSV);
		Mat imgThresh = GetThresholdedImage(imgHSV);
		operacionCierre(imgThresh);
		

		findContours(imgThresh, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		
		//myfile << currentDateTime() << centrotxt << estado << centrotxtmm << " " << mac_addr << "\n";
		if (contours.size() == 0)
			continue;//Si no hay ratón, dejará de hacer lo que está más abajo
		
		//Calcula moment solamente del ratón = contours[0]
		Moments oMoments = moments(contours[0]);
		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		//Calcula centro de masa ratón

		int posX = dM10 / dArea;
		int posY = dM01 / dArea;

		//Calcula variación de movimiento entre frames
		int deltaX = abs(iLastX - posX);
		int deltaY = abs(iLastY - posY);
		
		//Guarda datos de posicion en distintos formatos (Point y Char)
		Point xy = Point(posX, posY);
		sprintf(centro, " %d %d ", posX, posY);
		sprintf(centrotxt, " %d %d %d ", posX, posY, distanciareco);
		
		//Calcula area ratón
		double area = contourArea(contours[0]);
		if (area < areaMenor || areaMayor < area) continue;//Filta areas muy pequeñas y muy grandes

		putText(frame, centro, xy, FONT_HERSHEY_SIMPLEX, 0.5, (0, 143, 255));
		drawContours(frame, contours, 0, CV_RGB(255, 0, 0), 2, 8, hierarchy, 0);

		//if (deltaX >= 9 && deltaY >= 5)
		//Registra solo cambios considerables de movimiento
		if (deltaX >= 7 && deltaY >= 5)
		{
			//cout << "distanciareco= " << distanciareco << endl;
			distanciareco += sqrt(((iLastX - posX)*(iLastX - posX)) + ((iLastY - posY)*(iLastY - posY)));

		}
		if (deltaX > 20 && deltaY > 15)
		{
			cout << "Se agitó al ratón!!!" << endl;
		}
		iLastX = posX;
		iLastY = posY;		

		CvFont font, fontbig;
		cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, .6, .6, 0, 1.5, CV_AA);
		cvInitFont(&fontbig, CV_FONT_HERSHEY_COMPLEX, 3, .6, 0, 3, CV_AA);

		Point cvpoint = Point(posX, posY);
		putText(frame, CamaraNum, Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.6, (0, 242, 255), 2);
		sprintf(distancia, "Distancia=%d", distanciareco);
		putText(frame, distancia, Point(180, 200), FONT_HERSHEY_SIMPLEX, 0.5, (0, 142, 255));
		
		//putText(frame, centro, cvpoint, FONT_HERSHEY_SIMPLEX, 0.5, (0, 143, 255));
		line(frame, cvPoint(0, 105), cvPoint(320, 105), cvScalar(0, 143, 255), 1.5); //---linea de parado/caminando

		if (posY < 105)
		{
			putText(frame, "parado", cvPoint(30, 200), FONT_HERSHEY_SIMPLEX, 0.5, (0, 142, 255));
			estado = 1;
		}
		else
		{
			putText(frame, "NO parado", cvPoint(30, 200), FONT_HERSHEY_SIMPLEX, 0.5, (0, 142, 255));
			estado = 0;
		}

		//Dibuja target sobre raton
		MyFilledCircle(frame, xy);
		line(frame, cvPoint(posX - 30, posY), cvPoint(posX + 30, posY), cvScalar(0, 255, 103), 1.5);
		line(frame, cvPoint(posX, posY - 30), cvPoint(posX, posY + 30), cvScalar(0, 255, 103), 1.5);

		if (flipCamara == true){
			namedWindow(CamaraNum);
			imshow(CamaraNum, frame);//Muestra el resultado final
		}
		else{
			destroyWindow(CamaraNum);
		}
		
		void displayOverlay(const string& THRESH, const string& text, int delayms = 1000);
		//Escribe las variables en un txt
		//myfile << currentDateTime() << centrotxt << estado << centrotxtmm << " " << mac_addr << "\n";
		//cout << centrotxt << " " << estado << " " << endl;

	}		

	myfile.close();
	cvDestroyAllWindows();
	cout << "Cerrando RatonVision-v1.4..." << endl;
	Sleep(1000);
	cout << "Gracias por utilizar RatonVision-v1.4" << endl;
	Sleep(800);
	cout << "No olvide visitar www.austral-robotics.com" << endl;
	Sleep(800);
	cout << "Volviendo al Menu Principal" << endl;
	Sleep(800);
	cout << " " << endl;
	cout << " " << endl;
	goto MENU;
	return 0;
}



