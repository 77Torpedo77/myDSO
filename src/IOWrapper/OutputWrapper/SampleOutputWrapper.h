/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once
#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"



#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"
#include "util/settings.h"

#include "util/pclSetting.h"

//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;


namespace IOWrap
{

class SampleOutputWrapper : public Output3DWrapper
{
public:
        inline SampleOutputWrapper(pclSetting* &pclSetting_arg)
        {
            _pclSetting = pclSetting_arg;
            std::string thread_id_str = boost::to_string(boost::this_thread::get_id());
            std::cout<<thread_id_str+"//////////////////sample///////////////////"<<std::endl;
            _pclSetting->numPCL = 0;
            _pclSetting->isSavePCL = true;
            _pclSetting->isPCLfileClose = false;
           // pclSetting->strTmpFileName = boost::to_string(boost::this_thread::get_id())+"_tmp.pcd.sample";
            std::cout<<_pclSetting->strTmpFileName+"//////////////////sample_tmp_name///////////////////"<<std::endl;
            pclFile.open(_pclSetting->strTmpFileName);

            printf("OUT: Created SampleOutputWrapper\n");
        }

        pclSetting* _pclSetting;

        virtual ~SampleOutputWrapper()
        {
            if (pclFile.is_open())
            {
                pclFile.close();
            }

            printf("OUT: Destroyed SampleOutputWrapper\n");
        }

        virtual void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) override
        {
            /*
            printf("OUT: got graph with %d edges\n", (int)connectivity.size());

            int maxWrite = 5;

            for(const std::pair<uint64_t,Eigen::Vector2i> &p : connectivity)
            {
                int idHost = p.first>>32;
                int idTarget = p.first & ((uint64_t)0xFFFFFFFF);
                printf("OUT: Example Edge %d -> %d has %d active and %d marg residuals\n", idHost, idTarget, p.second[0], p.second[1]);
                maxWrite--;
                if(maxWrite==0) break;
            }
            */
        }



        virtual void
        publishKeyframes(std::vector<FrameHessian *> &frames, bool final, CalibHessian *HCalib, int mode) override
        {
//            std::string thread_id_str = boost::to_string(boost::this_thread::get_id());
//            std::cout<<thread_id_str+"//////////////////sample_public///////////////////"<<std::endl;
            float fx, fy, cx, cy;
            float fxi, fyi, cxi, cyi;
            //float colorIntensity = 1.0f;
            fx = HCalib->fxl();
            fy = HCalib->fyl();
            cx = HCalib->cxl();
            cy = HCalib->cyl();
            fxi = 1 / fx;
            fyi = 1 / fy;
            cxi = -cx / fx;
            cyi = -cy / fy;

            if (final)
            {
                for (FrameHessian* f : frames)
                {
                    if (f->shell->poseValid)
                    {
                        auto const& m = f->shell->camToWorld.matrix3x4();

                        // use only marginalized points.
                        auto const& points = f->pointHessiansMarginalized;

                        for (auto const* p : points)
                        {
                            float depth = 1.0f / p->idepth;
                            auto const x = (p->u * fxi + cxi) * depth;
                            auto const y = (p->v * fyi + cyi) * depth;
                            auto const z = depth * (1 + 2 * fxi);

                            Eigen::Vector4d camPoint(x, y, z, 1.f);
                            Eigen::Vector3d worldPoint = m * camPoint;

                            if (_pclSetting->isSavePCL && pclFile.is_open())
                            {
                                _pclSetting->isWritePCL = true;

                                pclFile << worldPoint[0] << " " << worldPoint[1] << " " << worldPoint[2] << "\n";

//                                printf("[%d] Point Cloud Coordinate> X: %.2f, Y: %.2f, Z: %.2f\n",
//                                       _pclSetting->numPCL,
//                                         worldPoint[0],
//                                         worldPoint[1],
//                                         worldPoint[2]);

                                _pclSetting->numPCL++;
                                _pclSetting->isWritePCL = false;
                            }
                            else
                            {
                                if (!_pclSetting->isPCLfileClose)
                                {
                                    if (pclFile.is_open())
                                    {
                                        pclFile.flush();
                                        pclFile.close();
                                        _pclSetting->isPCLfileClose = true;
                                    }
                                }
                            }


                         }
                    }
                }
            }
            if (mode==1)
            {
                int point_num=0;
                pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointXYZ point;
                for (FrameHessian* f : frames)
                {
                    if (f->shell->poseValid)
                    {
                        auto const& m = f->shell->camToWorld.matrix3x4();

                        // use only marginalized points.
                        auto const& points = f->pointHessiansMarginalized;

                        for (auto const* p : points)
                        {
                            point_num++;
                            float depth = 1.0f / p->idepth;
                            auto const x = (p->u * fxi + cxi) * depth;
                            auto const y = (p->v * fyi + cyi) * depth;
                            auto const z = depth * (1 + 2 * fxi);

                            Eigen::Vector4d camPoint(x, y, z, 1.f);
                            Eigen::Vector3d worldPoint = m * camPoint;

                            point.x=worldPoint[0];
                            point.y=worldPoint[1];
                            point.z=worldPoint[2];
                            new_cloud->push_back(point);
                            std::cout << worldPoint[0] << " " << worldPoint[1] << " " << worldPoint[2] << std::endl;
                        }
                    }
                }
                std::unique_lock<std::mutex> cloud_lck(cloud_mtx);
                //cloud_vector.insert(cloud_vector.begin()+_pclSetting->view_num_index,new_cloud);
                //cloud_vector.push_back(new_cloud);
                cloud_vector[_pclSetting->view_num_index]=new_cloud;
                cloud_lck.unlock();
                std::cout<<"add cloud:"<<_pclSetting->view_num_index<<",have points:"<<point_num<<std::endl;
            }

        }

        virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override
        {
            /*
            printf("OUT: Current Frame %d (time %f, internal ID %d). CameraToWorld:\n",
                   frame->incoming_id,
                   frame->timestamp,
                   frame->id);
            std::cout << frame->camToWorld.matrix3x4() << "\n";
            */
        }


        virtual void pushLiveFrame(FrameHessian* image) override
        {
            // can be used to get the raw image / intensity pyramid.
        }

        virtual void pushDepthImage(MinimalImageB3* image) override
        {
            // can be used to get the raw image with depth overlay.
        }
        virtual bool needPushDepthImage() override
        {
            return false;
        }

        virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF ) override
        {
            /*
            printf("OUT: Predicted depth for KF %d (id %d, time %f, internal frame-ID %d). CameraToWorld:\n",
                   KF->frameID,
                   KF->shell->incoming_id,
                   KF->shell->timestamp,
                   KF->shell->id);
            std::cout << KF->shell->camToWorld.matrix3x4() << "\n";

            int maxWrite = 5;
            for(int y=0;y<image->h;y++)
            {
                for(int x=0;x<image->w;x++)
                {
                    if(image->at(x,y) <= 0) continue;

                    printf("OUT: Example Idepth at pixel (%d,%d): %f.\n", x,y,image->at(x,y));
                    maxWrite--;
                    if(maxWrite==0) break;
                }
                if(maxWrite==0) break;
            }
            */
        }

        std::ofstream pclFile;


};



}



}
