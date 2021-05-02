/*
 * @file sum_of_absolute_diff.h
 *
 * Created : 25 April, 2021
 * Author  : Chandravaran Kunjeti
 */

#ifndef __SUM_OF_ABSOLUTE_DIFF__
#define __SUM_OF_ABSOLUTE_DIFF__

#include <stdio.h>
#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;

namespace disparity
{
    class SumOfAbsoluteDiff
    {
        public:

            /**
             * 
             * \brief This function will find use the sum of absolute difference to 
             * to find the disparity values for a gray image 
             * 
             * */
            std::vector<Mat> sumOfAbsoluteDiffMbm(Mat& imgl, Mat& imgr, int disparity_levels)
            {
                int r,c,h;
                int i,j,k;
                Mat kernel_61_1(61,1,CV_64F,Scalar(1));
                Mat kernel_1_61(1,61,CV_64F,Scalar(1));
                Mat kernel_11_11(11,11,CV_64F,Scalar(1));
                Mat kernel_3_3(3,3,CV_64F,Scalar(1));
                Mat temp(imgl.rows,imgl.cols,CV_64F);
    
                Mat error_61_1;
                Mat error_1_61;
                Mat error_11_11;
                Mat error_3_3;

                std::vector<Mat> errors_61_1;
                std::vector<Mat> errors_1_61;
                std::vector<Mat> errors_11_11;
                std::vector<Mat> errors_3_3;
                std::vector<Mat> out;
    
                r = imgl.rows;
                c = imgl.cols;
                h = 1;
    
                uint64* row_right;
                uint64* row_left;
                uint64* row_temp;
    
                for( i=0 ; i<disparity_levels; i++)
                {
                    for(j=0; j<imgl.rows ; j++)
                    {
                        row_right = imgr.ptr<uint64>(j);
                        row_left  = imgl.ptr<uint64>(j);
                        row_temp  = temp.ptr<uint64>(j);
    
                        for(k=0;k<c-i;k++)
                        {
                            row_temp[k] = abs(row_right[k] - row_left[k+i]);
                        }
    
                    }
    
                    filter2D(temp,error_61_1,-1,kernel_61_1,Point(-1,-1),BORDER_CONSTANT);
                    filter2D(temp,error_1_61,-1,kernel_1_61,Point(-1,-1),BORDER_CONSTANT);
                    filter2D(temp,error_11_11,-1,kernel_11_11,Point(-1,-1),BORDER_CONSTANT);
                    filter2D(temp,error_3_3,-1,kernel_3_3,Point(-1,-1),BORDER_CONSTANT);
    
                    errors_61_1.push_back(error_61_1);
                    errors_1_61.push_back(error_1_61);
                    errors_11_11.push_back(error_11_11);
                    errors_3_3.push_back(error_3_3);

                }

                Mat temp1;
                for(int i=0; i<errors_1_61.size(); i++)
                {
                    temp1 = min(errors_61_1[i],errors_1_61[i]);
                    temp1 = temp1 * errors_11_11[i];
                    out.push_back(temp1*errors_3_3[i]);
                }
    
                return out;    
            }

            /**
             * 
             * \brief This function will find use the sum of absolute difference to 
             * to find the disparity values for a multy dimention image. The function 
             * is overloaded 
             * 
             * */
            std::vector<Mat> sumOfAbsoluteDiffMbm(std::vector<Mat>& img_array_left, std::vector<Mat>& img_array_right, int disparity_levels)
            {
                int r,c,h;
                int i,j,k,l;
                Mat kernel_61_1(61,1,CV_64F,Scalar(1));
                Mat kernel_1_61(1,61,CV_64F,Scalar(1));
                Mat kernel_11_11(11,11,CV_64F,Scalar(1));
                Mat kernel_3_3(3,3,CV_64F,Scalar(1));
                Mat temp(img_array_left[0].rows,img_array_left[0].cols,CV_64F);
    
                Mat error_61_1;
                Mat error_1_61;
                Mat error_11_11;
                Mat error_3_3;

                std::vector<Mat> errors_61_1;
                std::vector<Mat> errors_1_61;
                std::vector<Mat> errors_11_11;
                std::vector<Mat> errors_3_3;
                std::vector<Mat> out;

                errors_61_1.resize(disparity_levels);
                errors_1_61.resize(disparity_levels);
                errors_11_11.resize(disparity_levels);
                errors_3_3.resize(disparity_levels);
    
                r = img_array_left[0].rows;
                c = img_array_left[0].cols;
                h = img_array_left.size();
    
                uint64* row_right;
                uint64* row_left;
                uint64* row_temp;
    
                for( i=0 ; i<disparity_levels; i++)
                {
                   for(l=0; l<img_array_left.size() ; l++)
                   {     
                        for(j=0; j<img_array_left[l].rows ; j++)
                        {
                            row_right = img_array_right[l].ptr<uint64>(j);
                            row_left = img_array_left[l].ptr<uint64>(j);
                            row_temp = temp.ptr<uint64>(j);
        
                            for(k=0;k<c-i;k++)
                            {
                                row_temp[k] = abs(row_right[k] - row_left[k+i]);
                            }
        
                        }
        
                        filter2D(temp,error_61_1,-1,kernel_61_1,Point(-1,-1),BORDER_CONSTANT);
                        filter2D(temp,error_1_61,-1,kernel_1_61,Point(-1,-1),BORDER_CONSTANT);
                        filter2D(temp,error_11_11,-1,kernel_11_11,Point(-1,-1),BORDER_CONSTANT);
                        filter2D(temp,error_3_3,-1,kernel_3_3,Point(-1,-1),BORDER_CONSTANT);
        
                        errors_61_1[i]  += error_61_1;
                        errors_1_61[i]  += error_1_61;
                        errors_11_11[i] += error_11_11;
                        errors_3_3[i]   += error_3_3;
                   }
                }

                Mat temp1;
                for(int i=0; i<errors_1_61.size(); i++)
                {
                    temp1 = min(errors_61_1[i],errors_1_61[i]);
                    temp1 = temp1 * errors_11_11[i];
                    out.push_back(temp1*errors_3_3[i]);
                }
    
                return out;    
            }

    };
}
#endif // __SUM_OF_ABSOLUTE_DIFF__