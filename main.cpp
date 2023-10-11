#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#endif

#include <iostream>
#include <fstream>
#include <string>

#ifndef _WIN32
using HANDLE = int;
constexpr int INVALID_HANDLE_VALUE = -1;
#endif

inline bool Is_Valid_Handle(HANDLE handle) {
#ifdef _WIN32
    return handle != INVALID_HANDLE_VALUE;
#else
    return handle > INVALID_HANDLE_VALUE;
#endif
}

inline HANDLE Open_Port(const std::string& portSpec) {
#ifdef _WIN32
    return CreateFileA(portSpec.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
#elif __APPLE__
    return open(portSpec.c_str(), O_RDWR | O_NONBLOCK);
#else
    return open(portSpec.c_str(), O_RDWR);
#endif
}

inline void Close_Port(HANDLE handle) {
#ifdef _WIN32
    CloseHandle(handle);
#else
    close(handle);
#endif
}

inline bool Set_Port_Parameters(HANDLE handle) {
#ifdef _WIN32
    DCB serialParams;

    GetCommState(handle, &serialParams);
    serialParams.BaudRate = CBR_115200;
    serialParams.ByteSize = 8;
    serialParams.StopBits = ONESTOPBIT;
    serialParams.Parity = NOPARITY;
    SetCommState(handle, &serialParams);
#else
    struct termios tty;

    if (tcgetattr(handle, &tty) != 0)
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

    if (tcsetattr(handle, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error in configuring serial port (2)" << std::endl;
        return false;
    }
#endif

    return true;
}

inline bool Write_Port(HANDLE handle, const std::string& input) {
#ifdef _WIN32
    unsigned long l;
    return WriteFile(handle, input.c_str(), static_cast<DWORD>(input.length()), &l, NULL);
#else
    return write(handle, input.c_str(), static_cast<int>(input.length())) > 0;
#endif
}

inline bool Read_Port_Char(HANDLE handle, char& target) {
#ifdef _WIN32
    unsigned long l = 1;
    return ReadFile(handle, &target, 1, &l, NULL);
#else
    return (read(hComm, &c, 1) == 1);
#endif
}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage:  uart_flasher.exe <filename> <port identifier>" << std::endl;
        return 3;
    }

    std::string filename = argv[1];

    HANDLE hComm = Open_Port(argv[2]);

    if (!Is_Valid_Handle(hComm)) {
        std::cerr << "Error in opening serial port" << std::endl;
        return 1;
    }

    if (!Set_Port_Parameters(hComm)) {
        std::cerr << "Error in configuring serial port" << std::endl;
        return 1;
    }

    std::cout << "Opening serial port successful" << std::endl;

    std::string ln;

    // ifstream scope
    {
        std::ifstream ifile(argv[1]);
        if (!ifile.is_open())
        {
            std::cerr << "Error in opening input file" << std::endl;
            Close_Port(hComm);
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
            Write_Port(hComm, ln);

            if (lineinc < ((100 * linecounter) / linecnt))
            {
                lineinc = ((100 * linecounter) / linecnt);
                std::cout << "Progress: " << lineinc << " %" << std::endl;
            }
        }
    }

    std::cout << "Launching..." << std::endl;

    Write_Port(hComm, "G");

    std::cout << "Done. Switching to listener mode." << std::endl << std::endl;

    char c;
    while (Read_Port_Char(hComm, c))
        std::cout << c;

    Close_Port(hComm);

    return 0;
}
