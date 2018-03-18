/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include "Converter.h"

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize(resolution, resolution, resolution);
    globalMap = boost::make_shared<PointCloud>();

    // viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));
}

void PointCloudMapping::shutdown()
{
    std::cout << "Save point cloud file successfully!" << std::endl;
    {
        unique_lock <mutex> lck(shutDownMutex);
        OcTree tree(0.1);  // create empty tree with resolution 0.1
        for (size_t i = 0; i < keyframes.size(); i++)
        {
            std::cout << "keyframe " << i << "..." << std::endl;
            generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i], &tree);
            tree.updateInnerOccupancy();
        }
        tree.writeBinary("simple_tree.bt");
        std::cout << "wrote example file simple_tree.bt" << std::endl << std::endl;
        std::cout << "now you can use octovis to visualize: octovis simple_tree.bt" << std::endl;
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    // viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
{
    cout << "receive a keyframe, id = " << kf->mnId << endl;
    unique_lock <mutex> lck(keyframeMutex);
    keyframes.push_back(kf);
    colorImgs.push_back(color.clone());
    depthImgs.push_back(depth.clone());

    keyFrameUpdated.notify_one();
}


void PointCloudMapping::generatePointCloud(KeyFrame *kf, cv::Mat &color, cv::Mat &depth, OcTree *tree)
{
    // point cloud is null ptr
    int maxd = 0;
    int size = 0;
    for (int m = 0; m < depth.rows; m += 3)
    {
        for (int n = 0; n < depth.cols; n += 3)
        {
            float d = depth.ptr<float>(m)[n];
            if (d > maxd) maxd = d;
            //if (d < 0.01 || d>1)
            //    continue;
            PointT p;
            p.z = d;
            p.x = (n - kf->cx) * p.z / kf->fx;
            p.y = (m - kf->cy) * p.z / kf->fy;

            p.b = color.ptr<uchar>(m)[n * 3];
            p.g = color.ptr<uchar>(m)[n * 3 + 1];
            p.r = color.ptr<uchar>(m)[n * 3 + 2];

            size++;

            point3d endpoint(p.x, p.y, p.z);
            tree->updateNode(endpoint, true); // integrate 'occupied' measurement
        }
    }
    cout << "generate point cloud for kf " << kf->mnId << ", size=" << size << " with maxd = " << maxd << endl;
}


