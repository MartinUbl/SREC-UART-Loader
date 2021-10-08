#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#endif

#ifndef _WIN32
using HANDLE = int;
constexpr int INVALID_HANDLE_VALUE = -1;
#endif

#include <iostream>
#include <fstream>
#include <string>

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage:  uart_flasher.exe <filename> <port identifier>" << std::endl;
        return 3;
    }

    std::string filename = argv[1];

    HANDLE hComm;

#ifdef _WIN32
    hComm = CreateFileA(argv[2], GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
#else
    hComm = open(argv[2], O_RDWR);
#endif

    if (hComm <= INVALID_HANDLE_VALUE)
    {
        std::cerr << "Error in opening serial port" << std::endl;
        return 1;
    }

#ifdef _WIN32
    DCB serialParams;

    GetCommState(hComm, &serialParams);
    serialParams.BaudRate = CBR_115200;
    serialParams.ByteSize = 8;
    serialParams.StopBits = ONESTOPBIT;
    serialParams.Parity = NOPARITY;
    SetCommState(hComm, &serialParams);
#else
    struct termios tty;

    if (tcgetattr(hComm, &tty) != 0)
    {
        std::cerr << "Error in configuring serial port (1)" << std::endl;
        return 2;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(hComm, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error in configuring serial port (2)" << std::endl;
        return 2;
    }
#endif

    std::cout << "Opening serial port successful" << std::endl;

    unsigned long l;
    std::string ln;

    {
        std::ifstream ifile(argv[1]);
        if (!ifile.is_open())
        {
            std::cerr << "Error in opening input file" << std::endl;
#ifdef _WIN32
            CloseHandle(hComm);
#else
            close(hComm);
#endif
            return 1;
        }

        std::string str((std::istreambuf_iterator<char>(ifile)),
                         std::istreambuf_iterator<char>());
        ifile.seekg(0, std::ios::beg);

        int linecnt = 0;
        int linecounter = 0;
        int lineinc = 0;
        for (int j = 0; j < str.size(); j++)
        {
            if (str[j] == '\n')
                linecnt++;
        }

        std::cout << "Line count: " << linecnt << std::endl;

        std::cout << "Uploading..." << std::endl;

        while (std::getline(ifile, ln))
        {
            linecounter++;
#ifdef _WIN32
            WriteFile(hComm, ln.c_str(), static_cast<DWORD>(ln.length()), &l, NULL);
#else
            write(hComm, ln.c_str(), static_cast<int>(ln.length()));
#endif

            if (lineinc < ((100 * linecounter) / linecnt))
            {
                lineinc = ((100 * linecounter) / linecnt);
                std::cout << "Progress: " << lineinc << " %" << std::endl;
            }

            //Sleep(10);
        }
    }

    std::cout << "Launching..." << std::endl;

    ln = "G";
#ifdef _WIN32
    WriteFile(hComm, ln.c_str(), static_cast<DWORD>(ln.length()), &l, NULL);
#else
    write(hComm, ln.c_str(), static_cast<int>(ln.length()));
#endif

    std::cout << "Done. Switching to listener mode." << std::endl << std::endl;

    char c;

#ifdef _WIN32
    while (ReadFile(hComm, &c, 1, &l, NULL))
#else
    while (read(hComm, &c, 1) == 1)
#endif
        std::cout << c;

#ifdef _WIN32
    CloseHandle(hComm);
#else
    close(hComm);
#endif

    return 0;
}
