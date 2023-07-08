#include "logger/logger.h"
#include <chrono>
#include <thread>

void Sleep(uint64_t msec)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(msec));
}

void Logger(const char *color, const char *msg, ...)
{
    // SaveLog(msg);
    time_t now = time(0);
    tm *ltm = localtime(&now);
    char buffer[100];
    sprintf(buffer, "[%d-%d-%d %d:%d:%d] => ", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    msg = strcat(buffer, msg);
    va_list args;
    va_start(args, msg);
    printf("%s", color);
    vprintf(msg, args);
    printf("%s\n", RST);
    va_end(args);
}

void SaveLog(const char *msg, ...)
{
    char buffer[100];
    time_t now = time(0);
    tm *ltm = localtime(&now);
    sprintf(buffer, "[%d-%d-%d %d:%d:%d] => ", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    msg = strcat(buffer, msg);
    va_list args;
    va_start(args, msg);
    FILE *fp = fopen(LOGDIR, "a");
    vfprintf(fp, msg, args);
    fclose(fp);
    va_end(args);
}