/*
 * (c) Copyright 2015-2016 6 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <unistd.h>
#include <sched.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <iostream>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <vector>
#include <stdio.h>

#ifndef _GNU_SOURCE
	#define _GNU_SOURCE
#endif

std::vector<std::string> tokenize(std::string input, std::string delimiters)
{
    std::vector<std::string> values;

    char* range = strtok((char*)input.c_str(), delimiters.c_str());

    while (range != nullptr)
    {
        values.push_back(range);

        range = strtok (nullptr, ",");
    }

    return values;
}

std::vector<int> parseThreadAffinity(std::string affinity)
{
    std::vector<std::string> coreStrings = tokenize(affinity, ",");
    std::vector<int> coreIds;

    for (auto coreString : coreStrings)
    {
        coreIds.push_back(atoi(coreString.c_str()));
    }

    return coreIds;
}

void setThreadAffinity(std::string name, std::string affinity)
{
    if (affinity.empty() || affinity == "-1")
    {
        ROS_INFO("Not setting affinity for %s", name.c_str());
        
        return;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    std::string cores;

    std::vector<int> coreIds = parseThreadAffinity(affinity);

    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
 
    for (auto coreId : coreIds)
    {
        if (coreId < 0 || coreId >= num_cores)
        {
            ROS_ERROR("Invalid core id: %d (Max cores: %d)", coreId, num_cores);

            return;
        }

        CPU_SET(coreId, &cpuset);
    }

    ROS_INFO_STREAM("Setting affinity: " << affinity << " for " << name << ", thread " <<  syscall(SYS_gettid));

    pthread_t current_thread = pthread_self();

    int res = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);

    if (res != 0)
    {
        ROS_ERROR("Could not set %s thread affinity to %s", name.c_str(), affinity.c_str());
    }
}

void niceThread(std::string name, int priority)
{
    pid_t tid = syscall(SYS_gettid);

    ROS_INFO_STREAM("Setting thread priority: " << priority << " for " << name << " on thread " << tid);

    if (::setpriority(PRIO_PROCESS, tid, priority))
    {
        ROS_ERROR("Could not set %s thread nice value to %d", name.c_str(), priority);
    }
}
