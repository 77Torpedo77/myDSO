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



#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"


#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/pclSetting.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"



#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#include <condition_variable>
#include <pcl/io/pcd_io.h>

std::string vignette = "";
std::string gammaCalib = "";
std::string source = "";
std::string calib = "";
double rescale = 1;
bool reverse = false;
bool disableROS = false;
int start=0;
int end=100000;
bool prefetch = false;
float playbackSpeed=0;	// 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
bool preload=false;
bool useSampleOutput=false;


int mode=0;

bool firstRosSpin=false;



using namespace dso;

void test(int &lstart, ImageFolderReader* &reader, int &linc, int &lend,int view_num_index) {


    std::string thread_id_str = boost::to_string(boost::this_thread::get_id());
    std::cout<<thread_id_str+"//////////////////test///////////////////"<<std::endl;

    pclSetting* pclSetting = new class pclSetting(view_num_index);
    std::cout<<pclSetting->strTmpFileName+"//////////////////pango_tmp_name///////////////////"<<std::endl;

    FullSystem* fullSystem = new FullSystem(pclSetting);
    fullSystem->setGammaFunction(reader->getPhotometricGamma());
    fullSystem->linearizeOperation = (playbackSpeed==0);



    IOWrap::PangolinDSOViewer* viewer = 0;
    if(!disableAllDisplay)
    {
        viewer = new IOWrap::PangolinDSOViewer(pclSetting,wG[0],hG[0], false);
        fullSystem->outputWrapper.push_back(viewer);
    }
    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper(pclSetting));

    std::thread runthread([&]() {
        std::vector<int> idsToPlay;
        std::vector<double> timesToPlayAt;
        for (int i = lstart; i >= 0 && i < reader->getNumImages() && linc * i < linc * lend; i += linc) {
            idsToPlay.push_back(i);
            if (timesToPlayAt.size() == 0) {
                timesToPlayAt.push_back((double) 0);
            } else {
                double tsThis = reader->getTimestamp(idsToPlay[idsToPlay.size() - 1]);
                double tsPrev = reader->getTimestamp(idsToPlay[idsToPlay.size() - 2]);
                timesToPlayAt.push_back(timesToPlayAt.back() + fabs(tsThis - tsPrev) / playbackSpeed);
            }
        }


        std::vector<ImageAndExposure *> preloadedImages;
        if (preload) {
            printf("LOADING ALL IMAGES!\n");
            for (int ii = 0; ii < (int) idsToPlay.size(); ii++) {
                int i = idsToPlay[ii];
                preloadedImages.push_back(reader->getImage(i));
            }
        }

        struct timeval tv_start;
        gettimeofday(&tv_start, NULL);
        clock_t started = clock();
        double sInitializerOffset = 0;


        for (int ii = 0; ii < (int) idsToPlay.size(); ii++) {
            if (!fullSystem->initialized)    // if not initialized: reset start time.
            {
                gettimeofday(&tv_start, NULL);
                started = clock();
                sInitializerOffset = timesToPlayAt[ii];
            }

            int i = idsToPlay[ii];


            ImageAndExposure *img;
            if (preload)
                img = preloadedImages[ii];
            else
                img = reader->getImage(i);


            bool skipFrame = false;
            if (playbackSpeed != 0) {
                struct timeval tv_now;
                gettimeofday(&tv_now, NULL);
                double sSinceStart = sInitializerOffset + ((tv_now.tv_sec - tv_start.tv_sec) +
                                                           (tv_now.tv_usec - tv_start.tv_usec) / (1000.0f * 1000.0f));

                if (sSinceStart < timesToPlayAt[ii])
                    usleep((int) ((timesToPlayAt[ii] - sSinceStart) * 1000 * 1000));
                else if (sSinceStart > timesToPlayAt[ii] + 0.5 + 0.1 * (ii % 2)) {
                    printf("SKIPFRAME %d (play at %f, now it is %f)!\n", ii, timesToPlayAt[ii], sSinceStart);
                    skipFrame = true;
                }
            }


            if (!skipFrame) fullSystem->addActiveFrame(img, i);


            delete img;

            if (fullSystem->initFailed || setting_fullResetRequested) {
                
                if (ii < 250 || setting_fullResetRequested) {
                    printf("RESETTING!!\n");

                    std::vector<IOWrap::Output3DWrapper *> wraps = fullSystem->outputWrapper;
                    delete fullSystem;

                    for (IOWrap::Output3DWrapper *ow: wraps) ow->reset();

                    fullSystem = new FullSystem(pclSetting);
                    fullSystem->setGammaFunction(reader->getPhotometricGamma());
                    fullSystem->linearizeOperation = (playbackSpeed == 0);


                    fullSystem->outputWrapper = wraps;

                    setting_fullResetRequested = false;
                }
            }

            if (fullSystem->isLost) {
                printf("LOST!!\n");
                break;
            }

        }
        fullSystem->blockUntilMappingIsFinished();
        clock_t ended = clock();
        struct timeval tv_end;
        gettimeofday(&tv_end, NULL);


        fullSystem->printResult("result.txt");


        int numFramesProcessed = abs(idsToPlay[0] - idsToPlay.back());
        double numSecondsProcessed = fabs(reader->getTimestamp(idsToPlay[0]) - reader->getTimestamp(idsToPlay.back()));
        double MilliSecondsTakenSingle = 1000.0f * (ended - started) / (float) (CLOCKS_PER_SEC);
        double MilliSecondsTakenMT = sInitializerOffset + ((tv_end.tv_sec - tv_start.tv_sec) * 1000.0f +
                                                           (tv_end.tv_usec - tv_start.tv_usec) / 1000.0f);
        printf("\n======================"
               "\n%d Frames (%.1f fps)"
               "\n%.2fms per frame (single core); "
               "\n%.2fms per frame (multi core); "
               "\n%.3fx (single core); "
               "\n%.3fx (multi core); "
               "\n======================\n\n",
               numFramesProcessed, numFramesProcessed / numSecondsProcessed,
               MilliSecondsTakenSingle / numFramesProcessed,
               MilliSecondsTakenMT / (float) numFramesProcessed,
               1000 / (MilliSecondsTakenSingle / numSecondsProcessed),
               1000 / (MilliSecondsTakenMT / numSecondsProcessed));
        //fullSystem->printFrameLifetimes();
        if (setting_logStuff) {
            std::ofstream tmlog;
            tmlog.open("logs/time.txt", std::ios::trunc | std::ios::out);
            tmlog << 1000.0f * (ended - started) / (float) (CLOCKS_PER_SEC * reader->getNumImages()) << " "
                  << ((tv_end.tv_sec - tv_start.tv_sec) * 1000.0f + (tv_end.tv_usec - tv_start.tv_usec) / 1000.0f) /
                     (float) reader->getNumImages() << "\n";
            tmlog.flush();
            tmlog.close();
        }
        // Added by Yo Han.
        if (!pclSetting->isPCLfileClose) {
            ((IOWrap::SampleOutputWrapper *) fullSystem->outputWrapper[1])->pclFile.flush();
            ((IOWrap::SampleOutputWrapper *) fullSystem->outputWrapper[1])->pclFile.close();
            pclSetting->isPCLfileClose = true;
            printf("pcl tmp file is auto closed.\n");
        }
    });
    if(viewer != 0)
        viewer->run();

    runthread.join();

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
        delete ow;
    }

}

void my_exit_handler(int s)
{
	printf("Caught signal %d\n",s);
	exit(1);
}

void exitThread()
{
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	firstRosSpin=true;
	while(true) pause();
}

void settingsDefault(int preset)
{
	printf("\n=============== PRESET Settings: ===============\n");
	if(preset == 0 || preset == 1)
	{
		printf("DEFAULT settings:\n"
				"- %s real-time enforcing\n"
				"- 2000 active points\n"
				"- 5-7 active frames\n"
				"- 1-6 LM iteration each KF\n"
				"- original image resolution\n", preset==0 ? "no " : "1x");

		playbackSpeed = (preset==0 ? 0 : 1);
		preload = preset==1;
		setting_desiredImmatureDensity = 1500;
		setting_desiredPointDensity = 2000;
		setting_minFrames = 5;
		setting_maxFrames = 7;
		setting_maxOptIterations=6;
		setting_minOptIterations=1;

		setting_logStuff = false;
	}

	if(preset == 2 || preset == 3)
	{
		printf("FAST settings:\n"
				"- %s real-time enforcing\n"
				"- 800 active points\n"
				"- 4-6 active frames\n"
				"- 1-4 LM iteration each KF\n"
				"- 424 x 320 image resolution\n", preset==0 ? "no " : "5x");

		playbackSpeed = (preset==2 ? 0 : 5);
		preload = preset==3;
		setting_desiredImmatureDensity = 600;
		setting_desiredPointDensity = 800;
		setting_minFrames = 4;
		setting_maxFrames = 6;
		setting_maxOptIterations=4;
		setting_minOptIterations=1;

		benchmarkSetting_width = 424;
		benchmarkSetting_height = 320;

		setting_logStuff = false;
	}

	printf("==============================================\n");
}

void parseArgument(char* arg)
{
	int option;
	float foption;
	char buf[1000];


    if(1==sscanf(arg,"sampleoutput=%d",&option))
    {
        if(option==1)
        {
            useSampleOutput = true;
            printf("USING SAMPLE OUTPUT WRAPPER!\n");
        }
        return;
    }

    if(1==sscanf(arg,"quiet=%d",&option))
    {
        if(option==1)
        {
            setting_debugout_runquiet = true;
            printf("QUIET MODE, I'll shut up!\n");
        }
        return;
    }

	if(1==sscanf(arg,"preset=%d",&option))
	{
		settingsDefault(option);
		return;
	}


	if(1==sscanf(arg,"rec=%d",&option))
	{
		if(option==0)
		{
			disableReconfigure = true;
			printf("DISABLE RECONFIGURE!\n");
		}
		return;
	}



	if(1==sscanf(arg,"noros=%d",&option))
	{
		if(option==1)
		{
			disableROS = true;
			disableReconfigure = true;
			printf("DISABLE ROS (AND RECONFIGURE)!\n");
		}
		return;
	}

	if(1==sscanf(arg,"nolog=%d",&option))
	{
		if(option==1)
		{
			setting_logStuff = false;
			printf("DISABLE LOGGING!\n");
		}
		return;
	}
	if(1==sscanf(arg,"reverse=%d",&option))
	{
		if(option==1)
		{
			reverse = true;
			printf("REVERSE!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nogui=%d",&option))
	{
		if(option==1)
		{
			disableAllDisplay = true;
			printf("NO GUI!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nomt=%d",&option))
	{
		if(option==1)
		{
			multiThreading = false;
			printf("NO MultiThreading!\n");
		}
		return;
	}
	if(1==sscanf(arg,"prefetch=%d",&option))
	{
		if(option==1)
		{
			prefetch = true;
			printf("PREFETCH!\n");
		}
		return;
	}
	if(1==sscanf(arg,"start=%d",&option))
	{
		start = option;
		printf("START AT %d!\n",start);
		return;
	}
	if(1==sscanf(arg,"end=%d",&option))
	{
		end = option;
		printf("END AT %d!\n",start);
		return;
	}

	if(1==sscanf(arg,"files=%s",buf))
	{
		source = buf;
		printf("loading data from %s!\n", source.c_str());
		return;
	}

	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}

	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignette = buf;
		printf("loading vignette from %s!\n", vignette.c_str());
		return;
	}

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaCalib = buf;
		printf("loading gammaCalib from %s!\n", gammaCalib.c_str());
		return;
	}

	if(1==sscanf(arg,"rescale=%f",&foption))
	{
		rescale = foption;
		printf("RESCALE %f!\n", rescale);
		return;
	}

	if(1==sscanf(arg,"speed=%f",&foption))
	{
		playbackSpeed = foption;
		printf("PLAYBACK SPEED %f!\n", playbackSpeed);
		return;
	}

	if(1==sscanf(arg,"save=%d",&option))
	{
		if(option==1)
		{
			debugSaveImages = true;
			if(42==system("rm -rf images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
			if(42==system("mkdir images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
			if(42==system("rm -rf images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
			if(42==system("mkdir images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
			printf("SAVE IMAGES!\n");
		}
		return;
	}

	if(1==sscanf(arg,"mode=%d",&option))
	{

		mode = option;
		if(option==0)
		{
			printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
		}
		if(option==1)
		{
			printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
			setting_photometricCalibration = 0;
			setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
		}
		if(option==2)
		{
			printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
			setting_photometricCalibration = 0;
			setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_minGradHistAdd=3;
		}
		return;
	}

	printf("could not parse argument \"%s\"!!!!\n", arg);
}

int main( int argc, char** argv )
{
    /*
    ///nice_code
    int main() {
        std::vector<std::thread> threads;
        for (int i = 0; i < 3; ++i) {
            threads.push_back(std::thread([i](){
                init_func(i);
                run_func(i);
            }));
        }
        for (auto& t : threads) {
            t.join();
        }
        return 0;
    }
    ///nice_code
    */


    if (0){
        std::ofstream finalFile("sumFile.pcd");
        finalFile << std::string("# .PCD v.6 - Point Cloud Data file format\n");
        finalFile << std::string("FIELDS x y z\n");
        finalFile << std::string("SIZE 4 4 4\n");
        finalFile << std::string("TYPE F F F\n");
        finalFile << std::string("COUNT 1 1 1\n");
        finalFile << std::string("WIDTH ") << 350941+352370 << std::string("\n");
        finalFile << std::string("HEIGHT 1\n");
        finalFile << std::string("#VIEWPOINT 0 0 0 1 0 0 0\n");
        finalFile << std::string("POINTS ") << 350941+352370 << std::string("\n");
        finalFile << std::string("DATA ascii\n");

        std::ifstream savedFile1("7f19bde81700_tmp.pcd");

        while (!savedFile1.eof())
        {
            finalFile.put(savedFile1.get());
        }

        std::ifstream savedFile2("7f19be682700_tmp.pcd");

        while (!savedFile2.eof())
        {
            finalFile.put(savedFile2.get());
        }

        finalFile.close();
        savedFile1.close();
        savedFile2.close();

        printf("PCL File for 'pcl_data.pcd' is saved.\n");

        return 0;
    }



    std::string thread_id_str = boost::to_string(boost::this_thread::get_id());
    std::cout<<thread_id_str+"//////////////////////main///////////////"<<std::endl;
    //setlocale(LC_ALL, "");
    for(int i=1; i<argc;i++)
        parseArgument(argv[i]);

    // hook crtl+C.
    boost::thread exThread = boost::thread(exitThread);


    ImageFolderReader* reader = new ImageFolderReader(source,calib, gammaCalib, vignette);
    reader->setGlobalCalibration();

    ImageFolderReader* reader2 = new ImageFolderReader(source,calib, gammaCalib, vignette);
    reader2->setGlobalCalibration();

    ImageFolderReader* reader3 = new ImageFolderReader(source,calib, gammaCalib, vignette);
    reader2->setGlobalCalibration();



    if(setting_photometricCalibration > 0 && reader->getPhotometricGamma() == 0)
    {
        printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
        exit(1);
    }




    int lstart=start;
    int lend = end;
    int linc = 1;
    if(reverse)
    {
        printf("REVERSE!!!!");
        lstart=end-1;
        if(lstart >= reader->getNumImages())
            lstart = reader->getNumImages()-1;
        lend = start;
        linc = -1;
    }



//    FullSystem* fullSystem = new FullSystem();
//    fullSystem->setGammaFunction(reader->getPhotometricGamma());
//    fullSystem->linearizeOperation = (playbackSpeed==0);
//
//
//
//    FullSystem* fullSystem2 = new FullSystem();
//    fullSystem2->setGammaFunction(reader->getPhotometricGamma());
//    fullSystem2->linearizeOperation = (playbackSpeed==0);



//    IOWrap::PangolinDSOViewer* viewer = 0;
//    if(!disableAllDisplay)
//    {
//        viewer = new IOWrap::PangolinDSOViewer(wG[0],hG[0], true);
//        fullSystem->outputWrapper.push_back(viewer);
//    }
//    if(useSampleOutput)
//        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());
//
//    /////////////
//
//    IOWrap::PangolinDSOViewer* viewer2 = 0;
//    if(!disableAllDisplay)
//    {
//        viewer2 = new IOWrap::PangolinDSOViewer(wG[0],hG[0], true);
//        fullSystem2->outputWrapper.push_back(viewer2);
//    }
//    if(useSampleOutput)
//        fullSystem2->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());


    //std::thread runthread2(test(lstart,reader,linc,lend,fullSystem));
    int lstart2 = lstart; int linc2 = linc; int lend2 = lend;
    int lstart3 = lstart; int linc3 = linc; int lend3 = lend;
    // to make MacOS happy: run this in dedicated thread -- and use this one to run the GUI.
    std::thread runthread2([&]() {
        test(lstart, reader, linc, lend,0);
    });
    //runthread2.join();
    //int lstart2 = lstart; int linc2 = linc; int lend2 = lend;
    std::thread runthread3([&]() {
        test(lstart2, reader2, linc2, lend2,1);
    });
    std::thread runthread4([&]() {
        test(lstart3, reader3, linc3, lend3,2);
    });


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setMaxCorrespondenceDistance(0.05); // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setTransformationEpsilon(
            1e-8); // 前一个变换矩阵和当前变换矩阵的差异小于阈值时，就认为已经收敛了，是一条收敛条件；// Set the transformation epsilon (criterion 2)
    icp.setEuclideanFitnessEpsilon(
            1); // 还有一条收敛条件是均方误差和小于阈值，停止迭代。// Set the euclidean distance difference epsilon (criterion 3)
    icp.setMaximumIterations(50); // Set the maximum number of iterations (criterion 1)

    pcl::PointCloud<pcl::PointXYZ> final_cloud ; // 存储结果
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;



    while (point_match_flag==0){
        if(std::all_of(init_flag.begin(), init_flag.end(), [](int i) { return i == 1; })){
            final_cloud = *cloud_vector[0];
            for (int i = 1; i < view_num; i++) {
                icp.setInputSource(cloud_vector[i]); // 设置输入点云
                icp.setInputTarget(cloud_vector[0]); // 设置目标点云（输入点云进行仿射变换，得到目标点云）
                icp.align(temp_cloud); // 进行配准，结果存储在Final中
                if (icp.hasConverged()){
                    std::cout << "score: " << icp.getFitnessScore() << std::endl;
                    std::cout << "result transformation:\n" << icp.getFinalTransformation() << std::endl;
                    //pcl::transformPointCloud(temp_cloud, temp_cloud, icp.getFinalTransformation());
                    final_cloud += temp_cloud;
                    temp_cloud.clear();
                }
            }
            point_match_flag=1;
            //init_cv.notify_all();
            //break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    pcl::io::savePCDFileASCII("final.pcd", final_cloud);
    std::cout<<"end main while"<<std::endl;


    //std::thread::id mainThreadId = std::this_thread::get_id();
//    if(viewer != 0){
//        //viewer->run();
//        //viewer2->run();
//    }


    runthread2.join();
    runthread3.join();
    runthread4.join();
//    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
//    {
//        ow->join();
//        delete ow;
//    }
//    for(IOWrap::Output3DWrapper* ow2 : fullSystem2->outputWrapper)
//    {
//        ow2->join();
//        delete ow2;
//    }



//    printf("DELETE FULLSYSTEM!\n");
//    delete fullSystem;

    printf("DELETE READER!\n");
    delete reader;

//    printf("DELETE FULLSYSTEM!\n");
//    delete fullSystem2;

    printf("DELETE READER!\n");
    delete reader2;

    printf("DELETE READER!\n");
    delete reader3;


    printf("EXIT NOW!\n");
    return 0;
}