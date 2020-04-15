#include <stdint.h>
#include <stdbool.h>

static bool is_leap_year(uint16_t year) {
    return (!(year%4) && ((year%100) || !(year%400)));
}

static uint16_t calc_yday(uint16_t year, uint8_t mon, uint8_t day) {
    static const uint16_t cumulative_days[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365};
    if (mon > 12) {
        return (uint16_t)-1;
    }
    return cumulative_days[mon-1] + ((is_leap_year(year) && mon > 2) ? 1 : 0) + day-1;

}

uint32_t date_to_utc_stamp(uint16_t year, uint8_t mon, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    uint16_t yday = calc_yday(year, mon, day);
    if (yday == (uint16_t)-1 || mon > 12 || day > 31 || hour > 24 || min > 60 || sec > 60) {
        return (uint32_t)-1;
    }
    return sec + min*60 + hour*3600 + yday*86400 + (year-1970)*31536000 + ((year-1969)/4)*86400 - ((year-1901)/100)*86400 + ((year-1601)/400)*86400;
}

// test command: gcc <this file> -O3 -march=native -DSTANDALONE_TEST && TZ=UTC time ./a.out
#ifdef STANDALONE_TEST
#include <time.h>
#include <stdio.h>

time_t my_mktime(struct tm* tm) {
    return date_to_utc_stamp(tm->tm_year + 1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
}

int main() {
    uint32_t last_year = 0;
    for (uint32_t i=0; i<(uint32_t)-1; i++) {
        struct tm* std_tm;

        time_t t = (time_t)i;
        std_tm = gmtime(&t);

        if (std_tm->tm_year != last_year) {
            printf("year=%u\n", std_tm->tm_year+1900);
            last_year = std_tm->tm_year;
        }

        uint32_t my_utc = my_mktime(std_tm);
        uint32_t sys_utc = mktime(std_tm);

        if (my_utc != sys_utc || sys_utc != i) {
            printf("fail %u %u %u\n", i, my_utc, sys_utc);
        }
    }

    return 0;
}
#endif
