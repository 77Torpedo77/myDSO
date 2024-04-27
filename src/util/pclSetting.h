//
// Created by ubuntu64 on 2024/4/9.
//
#pragma once
#include <string>
#include <boost/exception/to_string.hpp>
#include <iostream>

#ifndef DSO_MUTITHREADSETTING_H
#define DSO_MUTITHREADSETTING_H

#endif //DSO_MUTITHREADSETTING_H
namespace dso{
    class pclSetting{
    public:
        pclSetting(int _view_num_index) {
            thread_id_str = boost::to_string(boost::this_thread::get_id());
            std::cout<<thread_id_str+"//////////////////pclsetting_init///////////////////"<<std::endl;

            strTmpFileName = thread_id_str+"_tmp.pcd";
            strSaveFileName = thread_id_str+".pcd";
            view_num_index = _view_num_index;
        }
        int numPCL = 0;
        bool isSavePCL = true;
        bool isWritePCL = false;
        bool isPCLfileClose = false;
// thread_local std::string thread_id_str = boost::to_string(boost::this_thread::get_id());
        std::string thread_id_str;
        std::string strTmpFileName;
        std::string strSaveFileName;
        int view_num_index;
    };
}

