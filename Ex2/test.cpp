#include <iostream>
#include "CImg.h"
#include <fstream>
#include <cmath>
using namespace cimg_library;
using namespace std;

int main(int argc, char ** argv )
{

	int w;
	int h;
	int SUM; 
	int	sumX, sumY;
	int y;
	int x;

	int	GX[3][3];
	int GY[3][3];

	//Sobel Matrices Horizontal
	GX[0][0] = 1; GX[0][1] = 0; GX[0][2] = -1;
	GX[1][0] = 2; GX[1][1] = 0; GX[1][2] = -2;
	GX[2][0] = 1; GX[2][1] = 0; GX[2][2] = -1;

	//Sobel Matrices Vertical
	GY[0][0] =  1; GY[0][1] =	 2; GY[0][2] =   1;
	GY[1][0] =  0; GY[1][1] =	 0; GY[1][2] =   0;
	GY[2][0] = -1; GY[2][1] =	-2;	GY[2][2] =  -1;

	cout << argv[1] << endl;
	CImg <unsigned char> image(argv[1]); 
	w = image.width();
	h = image.height();
	
	const unsigned int width = w;
	const unsigned int height = h;
	
	cout << "Width is : " << width << " and height is : " << height << endl;

	ofstream myfile;
	myfile.open("pxl.txt", ios::out);
	//Getting the raster data
	int **px;
	px = new  int *[width];
	for(y = 0; y< height; y++)
	{
		px[y] = new  int [height];
		for(x = 0; x<width; x++)
		{
			px[y][x] = image(x,y);
			myfile << (unsigned char)image(x,y) << "\t";
		}
		myfile << "\n";
	}
	myfile.close();
	/*Edge detection using Sobel Algorithm*/
	
	myfile.open("conv.txt", ios::out);
	for(y = 0; y < image.height() ; y++)
	{
		for(x = 0; x < image.width() ; x++)
		{
			sumX	= 0;
			sumY	= 0;
			/*Image Boundaries*/
			if(y == 0 || y == image.height()-1)
				SUM = 0;
			else if(x == 0 || x == image.width() - 1)
				SUM = 0;
			/**************************************/
			else
			{
			/*Convolution for X*/
				for(int i = -1; i < 2; i++)
				{
					for(int j = -1; j < 2; j++)
					{
						sumX = sumX + GX[j+1][i+1] * (int)image(x+j,y+i);
					}
				}
			/*Convolution for Y*/

				for(int i = -1; i < 3; i++)
				{
					for(int j = -1; j < 3; j++)
					{
						sumY = sumY + GY[j+1][i+1] * (int)image(x+j,y+i);
					}
				}
				/*Edge strength*/
				SUM = sqrt(pow((double)sumX,2) + pow((double)sumY,2));
				//SUM = sumX + sumY;
			}		
			
			if(SUM > 255) SUM = 255;
			if(SUM < 0) SUM = 0;
			//unsigned char newPixel = (255 - (unsigned char)(SUM));
			px[y][x] = SUM;
			myfile << px[y][x] << "\t";
		}
		myfile << "\n";
	}

	myfile.close();
	
	CImg<unsigned char> oimage(width,height);
	for(y = 0; y < image.height(); y++)
	{
		for(x = 0; x<image.width(); x++)
		{
			oimage(x,y) = px[y][x];
		}
	}
	oimage.save_bmp("oima2ge.bmp");
	return 0;
	
}