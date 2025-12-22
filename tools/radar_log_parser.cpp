#include <iostream>
#include <fstream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include "radar.h"

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: radar_log_parser <logfile>\n";
        return 1;
    }
    std::ifstream in(argv[1]);
    if (!in)
    {
        std::cerr << "Failed to open " << argv[1] << "\n";
        return 2;
    }

    RadarAlertManager mgr;
    std::string line;
    unsigned long lineno = 0;
    while (std::getline(in, line))
    {
        lineno++;
        if (line.empty())
            continue;
        std::istringstream ss(line);
        double lat = 0.0, lng = 0.0, speed = 0.0;
        int sats = -1;
        char comma;
        if (!(ss >> lat))
            continue;
        if (ss.peek() == ',')
            ss >> comma;
        if (!(ss >> lng))
            continue;
        if (ss.peek() == ',')
            ss >> comma;
        if (!(ss >> speed))
            continue;
        if (ss.peek() == ',')
            ss >> comma;
        ss >> sats;

        RadarAlertResult res = mgr.update(lat, lng, speed);
        std::cout << lineno << ": " << std::fixed << std::setprecision(6) << lat << ", " << lng;
        std::cout << std::setprecision(1) << ", " << speed << " km/h, sats=" << sats;
        std::cout << " => inZone=" << (res.inZone ? "YES" : "NO");
        std::cout << " severity=" << res.severity;
        if (res.justEntered)
            std::cout << " [ENTER]";
        if (res.justExited)
            std::cout << " [EXIT]";
        if (res.inZone)
            std::cout << " interval=" << res.intervalMs << "ms dur=" << res.durationMs << "ms freq=" << res.freqHz << "Hz";
        std::cout << std::endl;
    }

    return 0;
}
