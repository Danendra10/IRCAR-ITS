#ifndef __LOGGER_HH_
#define __LOGGER_HH_

#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <string>
#include <cstring>

/**
 * @brief This package will be the header for logger
 * our logger could print with several color
 */

#define RED "\033[0;31m"
#define GREEN "\033[0;32m"
#define YELLOW "\033[0;33m"
#define BLUE "\033[0;34m"
#define MAGENTA "\033[0;35m"
#define CYAN "\033[0;36m"
#define WHITE "\033[0;37m"
#define RESET "\033[0m"

#define LOGDIR "/home/isabellej/update_fira/Log/log.txt"

void Logger(const char *color, const char *msg, ...);
void SaveLog(const char *msg, ...);

#endif