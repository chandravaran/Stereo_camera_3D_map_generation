/*
 * @file sum_of_absolute_diff.h
 *
 * Created : 25 April, 2021
 * Author  : Chandravaran Kunjeti
 */

#ifndef __TOTAL_MATCHING__
#define __TOTAL_MATCHING__

#include <stdio.h>
#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "depth_imaging/cencus_trasnform.h"
#include "depth_imaging/sum_of_absolute_diff.h"

using namespace cv;
using namespace std;

namespace disparity
{
    class TotalMatching 
    {
        public:
            Mat totalMatchingCostGray(Mat imgl, Mat imgr)
            {
                int disparity_levels=16;
                Mat img_sad;    // output image for sum of absolute difference 
                Mat img_cencus; // output image for cencus transform 
                Mat imgLx1(imgl.rows,imgl.cols,CV_64F);
                Mat imgLy1(imgl.rows,imgl.cols,CV_64F);
                Mat imgRx1(imgr.rows,imgr.cols,CV_64F);
                Mat imgRy1(imgr.rows,imgr.cols,CV_64F);
                Mat combined_disp(imgr.rows,imgr.cols,CV_64F);
                Mat output(imgr.rows,imgr.cols,CV_8U);

                std::vector<Mat> imgLxy(3);
                std::vector<Mat> imgRxy(3);
                std::vector<Mat> cencus_mbm;
                std::vector<Mat> sad_gray;
                std::vector<Mat> sad_multi;
                std::vector<Mat> combined_disp_array(disparity_levels);
                

                double laplacian_cencus_mbm_weight = 45;
                double laplacian_sad_gray_weight   = 5;
                double laplacian_sad_multi_weight  = 18;

                // Finding the X and Y gradient images for left 
                Sobel(imgl,imgLx1,CV_64F,1,0,3);
                Sobel(imgl,imgLy1,CV_64F,0,1,3);

                // Finding the X and Y gradient images for right 
                Sobel(imgr,imgRx1,CV_64F,1,0,3);
                Sobel(imgr,imgRy1,CV_64F,0,1,3);

                // New multi dimensional left image
                imgLxy[0] = imgl;
                imgLxy[1] = imgLx1;
                imgLxy[2] = imgLy1;

                // New multi dimensional right image
                imgRxy[0] = imgr;
                imgRxy[1] = imgRx1;
                imgRxy[2] = imgRy1;

                // Finding errors using census transform by combining color and gradient and forming a 3d image 
                cencus_mbm = cencus.censusTransformMbm(imgLxy,imgRxy,disparity_levels);
                cout << "Finished Cencus" << endl;

                // Finding errors using SAD in color space 
                sad_gray = sad.sumOfAbsoluteDiffMbm(imgl,imgr,disparity_levels);
                cout << "Finished SAD in gray space" << endl;

                // Finding errors using SAD for gradient images
                // This function is overloaded
                sad_multi = sad.sumOfAbsoluteDiffMbm(imgLxy,imgRxy,disparity_levels);
                cout << "Finished SAD in gradient space" << endl;

                uint64* row_sad_gray;
                uint64* row_sad_multi;
                uint64* row_cencus_mbm;
                uint64* row_combined_disp;
                uint64* row_ouput;


                // Using the laplacian equation to combine the results
                for(int i=0; i<disparity_levels ; i++)
                {
                    cencus_mbm[i] = cencus_mbm[i]/laplacian_cencus_mbm_weight;
                    sad_gray[i] = sad_gray[i]/laplacian_sad_gray_weight;
                    sad_multi[i] = sad_multi[i]/laplacian_sad_multi_weight;

                    combined_disp.setTo(0); 

                    for(int j=0; j<imgl.rows ; j++)
                    {
                        row_sad_gray       = sad_gray[i].ptr<uint64>(j);
                        row_sad_multi      = sad_multi[i].ptr<uint64>(j);
                        row_cencus_mbm     = cencus_mbm[i].ptr<uint64>(j);
                        row_combined_disp  = combined_disp.ptr<uint64>(j);
                        for(int k=0; k<imgl.cols ; k++)
                        {
                            row_combined_disp[k] = 3 - 1/exp(row_cencus_mbm[k]) - 1/exp(row_sad_gray[k]) - 1/exp(row_sad_multi[k]);
                        }
                    }

                    combined_disp_array[i] = combined_disp;
                }

                // Getting the disparity map
                int mid =  11/2;
                std::vector<uint64> disparity_values(disparity_levels);

                for(int j=mid; j<(imgl.rows - mid) ; j++)
                {
                    for(int k=mid; k<min(imgl.cols-64,imgl.cols-mid) ; k++)
                    {
                        for(int i=0; i<disparity_levels ; i++)
                        {
                            disparity_values[i] = *combined_disp_array[i].ptr<uint64>(j,k);
                        }

                        *output.ptr<uchar>(j,k) = (std::min_element(disparity_values.begin(),disparity_values.end()) - disparity_values.begin())/disparity_levels*255;
                    }
                }

                return output;
            }
        private:
            CencusTransform cencus;
            SumOfAbsoluteDiff sad;


            
    };
}
#endif //__TOTAL_MATCHING__