/**
 * This source file is a part of SREC-Loader software used for educational purposes during
 * the teaching of KIV/OS - Operating Systems course.
 * 
 * Copyright (c) 2021 Martin Ubl, Department of Computer Science and Engineering,
 * University of West Bohemia
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
#include <algorithm>

#ifndef _WIN32
// On systems other than Windows, define HANDLE as int (as it is, in fact, a file descriptor)
using HANDLE = int;
#endif

/*
 * Validates the handle (port opening was successfull)
 */
inline bool Is_Valid_Handle(HANDLE handle) {
#ifdef _WIN32
    // Windows - invalid file handle is represented by a single reserved value
    return handle != INVALID_HANDLE_VALUE;
#else
    // *nixes - invalid file handle is negative descriptor value
    return handle >= 0;
#endif
}

/*
 * Opens the given port for R/W operations
 */
inline HANDLE Open_Port(const std::string& portSpec) {
#ifdef _WIN32
    return CreateFileA(portSpec.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
#elif __APPLE__
    return open(portSpec.c_str(), O_RDWR | O_NONBLOCK);
#else
    return open(portSpec.c_str(), O_RDWR);
#endif
}

/*
 * Closes the given port
 */
inline void Close_Port(HANDLE handle) {
#ifdef _WIN32
    CloseHandle(handle);
#else
    close(handle);
#endif
}

/*
 * Sets the port parameters
 * NOTE: this is highly specific for the purposes of KIV/OS - Operating Systems course, other
 * implementations of SREC UART bootloader on the device side may require different settings
 * 
 * Used parameters (both directions):
 *      - baud rate: 115200
 *      - character size: 8 bits
 *      - stop bits: 1 bit
 *      - parity: none
 *      - echo: no
 */
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

/*
 * Writes a string to a given port handle
 */
inline bool Write_Port(HANDLE handle, const std::string& input) {
#ifdef _WIN32
    unsigned long l;
    return WriteFile(handle, input.c_str(), static_cast<DWORD>(input.length()), &l, NULL);
#else
    return write(handle, input.c_str(), static_cast<int>(input.length())) > 0;
#endif
}

/*
 * Reads a single character from given port handle
 */
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

    const std::string filename = argv[1];
    const HANDLE hComm = Open_Port(argv[2]);

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
        // open the file
        std::ifstream ifile(filename);
        if (!ifile.is_open())
        {
            std::cerr << "Error in opening input file" << std::endl;
            Close_Port(hComm);
            return 1;
        }

        // read the whole file to a string
        std::string str((std::istreambuf_iterator<char>(ifile)), std::istreambuf_iterator<char>());
        // rewind to the start
        ifile.seekg(0, std::ios::beg);

        // count all lines in file, so we can display progress
        auto linecnt = static_cast<size_t>(std::count_if(str.begin(), str.end(), [](char c) { return c == '\n'; }));

        std::cout << "Line count: " << linecnt << std::endl
                  << "Uploading..." << std::endl;

        // upload the contents
        size_t linecounter = 0;
        int lineinc = 0;
        // read a single line
        while (std::getline(ifile, ln))
        {
            // write the line to the port
            linecounter++;
            Write_Port(hComm, ln);

            // display progress every 100 lines
            if (lineinc < ((100 * linecounter) / linecnt))
            {
                lineinc = ((100 * linecounter) / linecnt);
                std::cout << "Progress: " << lineinc << " %" << std::endl;
            }
        }
    }

    std::cout << "Launching..." << std::endl;

    // write the "G" symbol, that instructs the bootloader to jump to uploaded code (uploading finished)
    Write_Port(hComm, "G");

    // switch to listener mode
    std::cout << "Done. Switching to listener mode." << std::endl << std::endl;

    // this just reads all characters from the port and displays them in console; this is a debug feature,
    // that simplifies debugging in early stages of development
    char c;
    while (Read_Port_Char(hComm, c))
        std::cout << c;

    // once we are done, close the port
    Close_Port(hComm);

    return 0;
}
