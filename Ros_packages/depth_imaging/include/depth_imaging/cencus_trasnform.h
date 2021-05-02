/*
 * @file cencus_transform.h
 *
 * Created : 25 April, 2021
 * Author  : Chandravaran Kunjeti
 */

#ifndef __CENCUS_TRANSFORM__
#define __CENCUS_TRANSFORM__

#include <stdio.h>
#include <iostream>
#include <bits/stdc++.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;

namespace disparity
{
    class CencusTransform
    {
        public:
    
            Mat image;
    
            Mat census_convolution(Mat imgl, Mat imgr,int block_size, int disparity_levels)
            {
            
                Mat output_image(imgr.rows,imgr.cols,CV_8U);
                int csr[block_size*block_size];
                int csl[block_size*block_size];
                int channels = imgr.channels();
                int r      = imgr.rows;
                int c      = imgr.cols*channels;
                int mid    = block_size/2;
                int count  = 0;
                int count1 = 0;
                int min    = 0;
                int cmp    = 0;
                int size   = 0;
                int i      = 0; 
                int j      = 0; 
                int k      = 0; 
                int ii     = 0; 
                int jj     = 0; 
                int kk     = 0; 
    
                uchar* row_right_outer;
                uchar* row_right_inner;
                uchar* row_left_outer;
                uchar* row_left_inner;
                uchar* row_output;
    
                double filter_value = block_size*block_size/10;
    
                for(i=mid; i<r-mid; i++)
                {
                    row_right_outer = imgr.ptr<uchar>(i);
                    row_left_outer  = imgl.ptr<uchar>(i);
                    row_output= output_image.ptr<uchar>(i);
                    for(j=mid; j<c-mid ; j++)
                    {
                        count=0;
                        for(ii=i-mid ; ii<i+mid+1 ; ii++)
                        {
                            // One mistake done previously is that we did not change the pointer value when it was in the inner loop
                            row_right_inner = imgr.ptr<uchar>(ii);
                            for(jj=j-mid ; jj<j+mid+1 ; jj++)
                            {
                                if( row_right_inner[jj] < row_right_outer[j])
                                {
                                    csr[count]=0;
                                }
                                else 
                                {
                                    csr[count]=1;
                                }
    
                                count++;
                            }
                        }
    
                        if((c-mid)>(j+disparity_levels))
                        {
                            min = j+disparity_levels;
                        }
                        else
                        {
                            min = c-mid;
                        }
    
                        size = min-j;
                        double error[size];
                        for( k=j+1; k<min; k++ )
                        {
                            count=0;
                            cmp  =0;
                            for( ii=i-mid ; ii<i+mid+1 ; ii++)
                            {
                                row_left_inner  = imgl.ptr<uchar>(ii);
                                for( kk=k-mid ; kk<k+mid+1 ; kk++)
                                {
                                    if( row_left_inner[kk] < row_left_outer[k])
                                    {
                                        csl[count]=0;
                                    }
                                    else 
                                    {
                                        csl[count]=1;
                                    }
    
                                    if(csl[count]!=csr[count])
                                    {
                                        cmp++;
                                    }
    
                                    count++;
                                }
                            }
    
                            error[k-j] = cmp;
                        }
                        
                        row_output[j] =(uchar) (((double) indexofSmallestElement(error,disparity_levels,filter_value)/(double)disparity_levels*255));                 
                    }
                    
                }
                return output_image;
            }
    
            int indexofSmallestElement(double array[], int size, double filter_value)
            {
                int mini = array[0], idx = 0;
    
                for(int i = 1; i < size; i++)
                {
                    if(array[i] < mini)
                    {
                        idx = i;
                        mini = array[i];
                    }        
                    
                }
    
                if(mini>filter_value)
                {
                    idx = 0;
                }
    
                return idx;        
            }

            std::vector<Mat> censusTransformMbm(std::vector<Mat> imgl, std::vector<Mat> imgr, int disparity_levels )
            {
                // The differnt kernel to be used for multi block matching do not kee
                int kernel[4][2] = {{1,61},{61,1},{11,11},{3,3}}; 

                std::vector<std::vector<Mat>> kernal_output;
                std::vector<Mat> out;
                for(int i=0; i<4; i++)
                {
                    kernal_output.push_back(censusConvolutionVector(imgl[0],imgr[0],kernel[i][0],kernel[i][1],disparity_levels));
                }

                Mat temp;
                for(int i=0; i<kernal_output[0].size(); i++)
                {
                    temp = min(kernal_output[0][i],kernal_output[1][i]);
                    for(int j=2; j<4; j++)
                    {
                        temp = temp * kernal_output[j][i];
                    }
                    out.push_back(temp);
                }

                return out;

            }

            std::vector<Mat> censusConvolutionVector(Mat imgl, Mat imgr, int kernel_hight, int kernel_width, int disparity_levels)
            {
                int row_padding = kernel_hight/2;
                int col_padding = kernel_width/2;
                int r = imgl.rows;
                int c = imgl.cols; 
                int i, j, ii, jj, bits=0;
                Scalar value;
                Mat img_borderl;
                Mat img_borderr;
                Mat output_imagel(imgl.rows,imgl.cols,CV_8U);
                Mat output_imager(imgr.rows,imgr.cols,CV_8U);
                copyMakeBorder(imgl,img_borderl,row_padding,row_padding,col_padding,col_padding,BORDER_CONSTANT,value);
                copyMakeBorder(imgr,img_borderr,row_padding,row_padding,col_padding,col_padding,BORDER_CONSTANT,value);

                uchar* row_right;
                uchar* row_left;
                uchar* row_left_border;
                uchar* row_right_border;
                uchar* row_output_right;
                uchar* row_output_left;

                std::vector<Mat> outputs_left;
                std::vector<Mat> outputs_right;
                
                for(i=0; i<kernel_hight; i++)
                {
                    for(j=0; j<kernel_width; j++)
                    {
                        for(ii=i; ii<r+i ; ii++)
                        {
                            row_right = imgr.ptr<uchar>(ii);
                            row_left  = imgl.ptr<uchar>(ii);
                            row_right_border = img_borderr.ptr<uchar>(ii);
                            row_left_border  = img_borderl.ptr<uchar>(ii);
                            row_output_right = output_imagel.ptr<uchar>(ii);
                            row_output_left  = output_imager.ptr<uchar>(ii);

                            for(jj=j; jj<c+j; jj++)
                            {
                                row_output_right[jj] = (row_output_right[jj] << 1) | (row_right_border[jj] >= row_right[jj]);
                                row_output_left[jj]  = (row_output_left[jj]  << 1) | (row_left_border[jj]  >= row_left[jj]);
                            }
                        }

                        bits++;
                        if(bits%8==0 & bits!=0)
                        {
                            outputs_left.push_back(output_imagel);
                            outputs_right.push_back(output_imager);
                            output_imagel.setTo(0);
                            output_imager.setTo(0);
                        }
                    }
                }
                std::vector<Mat> final_output;
        

                for(i=0; i<disparity_levels ; i++)
                {
                   final_output.push_back(findDifference(outputs_left[i],outputs_right[i],i,outputs_left.size()));
                }

                return final_output;
            }

            Mat findDifference(std::vector<Mat> outputs_left, std::vector<Mat> outputs_right, int disp_value, int size )
            {
                Mat output(outputs_left[0].rows,outputs_left[0].cols,CV_64F);
                uchar* row_img_left; 
                uchar* row_img_right;
                uint64* row_output;

                std::bitset<8> b;

                for(int k=0; k<size ; k++)
                {
                    for(int i=0; i<outputs_right[k].rows ; i++)
                    {
                        row_img_left  = outputs_left[k].ptr<uchar>(i);
                        row_img_right = outputs_right[k].ptr<uchar>(i);
                        row_output    = output.ptr<uint64>(i);
                        for(int j=0; j<(outputs_right[k].rows-disp_value); j++)
                        {
                             b = row_img_left[j+disp_value]^row_img_right[j];
                             row_output[j] += b.count();
                        }
                    }
                }

                return output;
            }

    };

}

#endif // __CENCUS_TRANSFORM__