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
#ifndef NOMINMAX
#define NOMINMAX
#endif
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
#include <filesystem>
#include <vector>
#include <cctype>
#include <cstdint>
#include <stdexcept>
#include <sstream>
#include <utility>
#include <iterator>

enum class NRun_Command {
    None,
    Local,      // run local listener only
    PuTTY,      // run PuTTY on the target serial port
};

enum class Binary_Command : uint8_t {
    Set_Cursor = 0x01,
    Write = 0x02,
    Write_Zero = 0x03,
    Set_Entry = 0x04,
    Go = 0x05,
    Set_Baud = 0x06,
};

constexpr char Binary_ACK = 'K';
constexpr int Default_Baud_Rate = 115200;
constexpr uint32_t Raw_Image_Base_Address = 0x00008000;
constexpr uint32_t Mini_UART_Clock_Hz = 250000000;

struct Program_Block {
    uint32_t address;
    std::vector<uint8_t> data;
};

struct SREC_Image {
    std::vector<Program_Block> blocks;
    uint32_t entry = 0x00008000;
    bool hasEntry = false;
    size_t byteCount = 0;
};

class Progress_Bar {
public:
    Progress_Bar(std::string label, size_t total, std::string unit)
        : m_label(std::move(label)), m_total(total), m_unit(std::move(unit)) {
    }

    ~Progress_Bar() {
        if (m_rendered) {
            std::cout << std::endl;
        }
    }

    void Update(size_t current) {
        if (current > m_total) {
            current = m_total;
        }

        const size_t percent = (m_total == 0) ? 100 : (current * 100) / m_total;
        const size_t filled = (percent * m_width) / 100;

        std::ostringstream line;
        line << '\r' << m_label << " [";
        line << std::string(filled, '#');
        line << std::string(m_width - filled, '.');
        line << "] " << percent << "% (" << current << "/" << m_total << " " << m_unit << ")";

        const std::string output = line.str();
        std::cout << output;
        if (output.size() < m_previousLength) {
            std::cout << std::string(m_previousLength - output.size(), ' ');
        }
        std::cout << std::flush;

        m_previousLength = output.size();
        m_rendered = true;
    }

    void Finish() {
        Update(m_total);
        std::cout << std::endl;
        m_rendered = false;
    }

private:
    std::string m_label;
    size_t m_total;
    std::string m_unit;
    size_t m_previousLength = 0;
    bool m_rendered = false;
    static constexpr size_t m_width = 36;
};

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

uint16_t Baud_To_Mini_UART_Divisor(int baudRate) {
    if (baudRate <= 0) {
        throw std::runtime_error("Invalid baud rate");
    }

    const uint32_t divisor = ((Mini_UART_Clock_Hz + static_cast<uint32_t>(baudRate) * 4) /
                              (static_cast<uint32_t>(baudRate) * 8)) - 1;

    if (divisor > 0xFFFF) {
        throw std::runtime_error("Baud rate divisor is out of range");
    }

    return static_cast<uint16_t>(divisor);
}

#ifndef _WIN32
speed_t Baud_To_Termios_Speed(int baudRate) {
    switch (baudRate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
        default:
            throw std::runtime_error("Unsupported baud rate on this platform");
    }
}
#endif

/*
 * Sets the port parameters
 * NOTE: this is highly specific for the purposes of KIV/OS - Operating Systems course, other
 * implementations of SREC UART bootloader on the device side may require different settings
 * 
 * Used parameters (both directions):
 *      - baud rate: configurable, 115200 by default
 *      - character size: 8 bits
 *      - stop bits: 1 bit
 *      - parity: none
 *      - echo: no
 */
inline bool Set_Port_Parameters(HANDLE handle, int baudRate = Default_Baud_Rate) {
#ifdef _WIN32
    DCB serialParams;

    if (!GetCommState(handle, &serialParams)) {
        std::cerr << "Could not read serial port settings" << std::endl;
        return false;
    }

    serialParams.BaudRate = static_cast<DWORD>(baudRate);
    serialParams.ByteSize = 8;
    serialParams.StopBits = ONESTOPBIT;
    serialParams.Parity = NOPARITY;

    if (!SetCommState(handle, &serialParams)) {
        std::cerr << "Could not apply serial port settings" << std::endl;
        return false;
    }
#else
    struct termios tty;

    if (tcgetattr(handle, &tty) != 0)
    {
        std::cerr << "Could not read serial port settings" << std::endl;
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

    const speed_t speed = Baud_To_Termios_Speed(baudRate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(handle, TCSANOW, &tty) != 0)
    {
        std::cerr << "Could not apply serial port settings" << std::endl;
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
    const char* ptr = input.data();
    size_t remaining = input.size();

    while (remaining > 0) {
        DWORD written = 0;

        if (!WriteFile(handle, ptr, static_cast<DWORD>(remaining), &written, NULL)) {
            return false;
        }

        if (written == 0) {
            return false;
		}

        ptr += written;
        remaining -= written;
    }

    return true;
#else
    const char* ptr = input.data();
    size_t remaining = input.size();

    while (remaining > 0) {
        ssize_t written = write(handle, ptr, remaining);

        if (written < 0) {
            if (errno == EINTR) {
                continue;
			}

            return false;
        }

        if (written == 0) {
            return false;
		}

        ptr += written;
        remaining -= static_cast<size_t>(written);
    }

    return true;
#endif
}

/*
 * Reads a single character from given port handle
 */
inline bool Read_Port_Char(HANDLE handle, char& target) {
#ifdef _WIN32
    DWORD bytesRead = 0;
    return ReadFile(handle, &target, 1, &bytesRead, NULL) && bytesRead == 1;
#else
    return (read(handle, &target, 1) == 1);
#endif
}

/*
 * Finds PuTTY executable using standard paths
 */
const std::string Find_PuTTY() {
    // following paths are just a basic set of paths where PuTTY can be found; TODO: add more paths and more sophisticated search
#ifdef _WIN32
    static const std::vector<std::string> findPaths{
        "putty", "C:\\Program Files\\PuTTY\\putty.exe", "C:\\Program Files (x86)\\PuTTY\\putty.exe"
    };
#else
    static const std::vector<std::string> findPaths{
        "putty", "/usr/bin/putty", "/usr/local/bin/putty"
    };
#endif

    for (const auto& path : findPaths) {
        std::filesystem::path p(path);
        if (std::filesystem::exists(p))
            return p.string();
    }

    return "";
}

void Print_Usage(const char* executable) {
    std::cerr << "Usage:  " << executable << " [-b] [--baud <rate>] <filename> <port identifier> [run_command]" << std::endl;
}

int Parse_Baud_Rate(const std::string& value) {
    size_t parsed = 0;
    int baudRate = 0;

    try {
        baudRate = std::stoi(value, &parsed);
    }
    catch (const std::exception&) {
        throw std::runtime_error("Invalid baud rate");
    }

    if (parsed != value.size() || baudRate <= 0) {
        throw std::runtime_error("Invalid baud rate");
    }

    return baudRate;
}

NRun_Command Parse_Run_Command(std::string runCommandStr) {
    std::transform(runCommandStr.begin(), runCommandStr.end(), runCommandStr.begin(), ::tolower);

    if (runCommandStr == "local") {
        return NRun_Command::Local;
    }

    if (runCommandStr == "putty") {
        return NRun_Command::PuTTY;
    }

    return NRun_Command::None;
}

int Hex_Value(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }

    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (c >= 'A' && c <= 'F') {
        return 10 + c - 'A';
    }

    return -1;
}

uint8_t Parse_Hex_Byte(const std::string& line, size_t offset) {
    if (offset + 1 >= line.size()) {
        throw std::runtime_error("Unexpected end of SREC line");
    }

    const int hi = Hex_Value(line[offset]);
    const int lo = Hex_Value(line[offset + 1]);

    if (hi < 0 || lo < 0) {
        throw std::runtime_error("Invalid hexadecimal digit in SREC line");
    }

    return static_cast<uint8_t>((hi << 4) | lo);
}

void Append_Block(SREC_Image& image, uint32_t address, std::vector<uint8_t> data) {
    if (data.empty()) {
        return;
    }

    image.byteCount += data.size();

    if (!image.blocks.empty()) {
        Program_Block& previous = image.blocks.back();
        if (previous.address + previous.data.size() == address) {
            previous.data.insert(previous.data.end(), data.begin(), data.end());
            return;
        }
    }

    image.blocks.push_back(Program_Block{ address, std::move(data) });
}

std::string Lowercase_Extension(const std::string& filename) {
    std::string ext = std::filesystem::path(filename).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return ext;
}

std::vector<uint8_t> Read_Binary_File(const std::string& filename) {
    std::ifstream ifile(filename, std::ios::binary);
    if (!ifile.is_open()) {
        throw std::runtime_error("Error in opening input file");
    }

    return std::vector<uint8_t>(std::istreambuf_iterator<char>(ifile), std::istreambuf_iterator<char>());
}

uint16_t Read_LE16(const std::vector<uint8_t>& data, size_t offset) {
    if (offset + 2 > data.size()) {
        throw std::runtime_error("Unexpected end of file");
    }

    return static_cast<uint16_t>(data[offset] | (data[offset + 1] << 8));
}

uint32_t Read_LE32(const std::vector<uint8_t>& data, size_t offset) {
    if (offset + 4 > data.size()) {
        throw std::runtime_error("Unexpected end of file");
    }

    return static_cast<uint32_t>(data[offset]) |
           (static_cast<uint32_t>(data[offset + 1]) << 8) |
           (static_cast<uint32_t>(data[offset + 2]) << 16) |
           (static_cast<uint32_t>(data[offset + 3]) << 24);
}

SREC_Image Parse_SREC_File(const std::string& filename) {
    std::ifstream ifile(filename);
    if (!ifile.is_open()) {
        throw std::runtime_error("Error in opening input file");
    }

    SREC_Image image;
    std::string line;
    size_t lineNumber = 0;

    while (std::getline(ifile, line)) {
        lineNumber++;

        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        if (line.empty()) {
            continue;
        }

        if (line.size() < 4 || line[0] != 'S') {
            std::ostringstream msg;
            msg << "Invalid SREC record at line " << lineNumber;
            throw std::runtime_error(msg.str());
        }

        const char type = line[1];
        const uint8_t count = Parse_Hex_Byte(line, 2);
        const size_t expectedSize = 4 + static_cast<size_t>(count) * 2;

        if (line.size() != expectedSize) {
            std::ostringstream msg;
            msg << "Invalid SREC record length at line " << lineNumber;
            throw std::runtime_error(msg.str());
        }

        int addressBytes = 0;
        bool isDataRecord = false;
        bool isEntryRecord = false;

        switch (type) {
            case '0':
            case '5':
            case '6':
                addressBytes = (type == '6') ? 3 : 2;
                break;
            case '1':
                addressBytes = 2;
                isDataRecord = true;
                break;
            case '2':
                addressBytes = 3;
                isDataRecord = true;
                break;
            case '3':
                addressBytes = 4;
                isDataRecord = true;
                break;
            case '7':
                addressBytes = 4;
                isEntryRecord = true;
                break;
            case '8':
                addressBytes = 3;
                isEntryRecord = true;
                break;
            case '9':
                addressBytes = 2;
                isEntryRecord = true;
                break;
            default:
            {
                std::ostringstream msg;
                msg << "Unsupported SREC record type S" << type << " at line " << lineNumber;
                throw std::runtime_error(msg.str());
            }
        }

        if (count < static_cast<uint8_t>(addressBytes + 1)) {
            std::ostringstream msg;
            msg << "Invalid SREC byte count at line " << lineNumber;
            throw std::runtime_error(msg.str());
        }

        uint32_t address = 0;
        uint32_t checksumSum = count;
        size_t offset = 4;

        for (int i = 0; i < addressBytes; i++) {
            const uint8_t value = Parse_Hex_Byte(line, offset);
            checksumSum += value;
            address = (address << 8) | value;
            offset += 2;
        }

        const size_t dataLength = count - addressBytes - 1;
        std::vector<uint8_t> data;
        data.reserve(dataLength);

        for (size_t i = 0; i < dataLength; i++) {
            const uint8_t value = Parse_Hex_Byte(line, offset);
            checksumSum += value;
            data.push_back(value);
            offset += 2;
        }

        checksumSum += Parse_Hex_Byte(line, offset);
        if ((checksumSum & 0xFF) != 0xFF) {
            std::ostringstream msg;
            msg << "Invalid SREC checksum at line " << lineNumber;
            throw std::runtime_error(msg.str());
        }

        if (isDataRecord) {
            Append_Block(image, address, std::move(data));
        }
        else if (isEntryRecord) {
            image.entry = address;
            image.hasEntry = true;
        }
    }

    return image;
}

SREC_Image Parse_Raw_Image_File(const std::string& filename) {
    SREC_Image image;
    std::vector<uint8_t> data = Read_Binary_File(filename);

    image.entry = Raw_Image_Base_Address;
    image.hasEntry = true;
    Append_Block(image, Raw_Image_Base_Address, std::move(data));

    return image;
}

SREC_Image Parse_ELF_File(const std::string& filename) {
    constexpr uint32_t PT_LOAD = 1;
    constexpr size_t ELF32_EHDR_SIZE = 52;
    constexpr size_t ELF32_PHDR_SIZE = 32;

    const std::vector<uint8_t> file = Read_Binary_File(filename);
    if (file.size() < ELF32_EHDR_SIZE ||
        file[0] != 0x7F || file[1] != 'E' || file[2] != 'L' || file[3] != 'F') {
        throw std::runtime_error("Invalid ELF file");
    }

    if (file[4] != 1 || file[5] != 1) {
        throw std::runtime_error("Only 32-bit little-endian ELF files are supported");
    }

    const uint32_t entry = Read_LE32(file, 24);
    const uint32_t phoff = Read_LE32(file, 28);
    const uint16_t phentsize = Read_LE16(file, 42);
    const uint16_t phnum = Read_LE16(file, 44);

    if (phentsize < ELF32_PHDR_SIZE) {
        throw std::runtime_error("Invalid ELF program header size");
    }

    SREC_Image image;
    image.entry = entry;
    image.hasEntry = true;

    for (uint16_t i = 0; i < phnum; i++) {
        const size_t offset = static_cast<size_t>(phoff) + static_cast<size_t>(i) * phentsize;
        if (offset + ELF32_PHDR_SIZE > file.size()) {
            throw std::runtime_error("ELF program header is out of file bounds");
        }

        const uint32_t type = Read_LE32(file, offset);
        if (type != PT_LOAD) {
            continue;
        }

        const uint32_t fileOffset = Read_LE32(file, offset + 4);
        const uint32_t vaddr = Read_LE32(file, offset + 8);
        const uint32_t paddr = Read_LE32(file, offset + 12);
        const uint32_t filesz = Read_LE32(file, offset + 16);
        const uint32_t memsz = Read_LE32(file, offset + 20);

        if (filesz > memsz || static_cast<size_t>(fileOffset) + filesz > file.size()) {
            throw std::runtime_error("ELF load segment is out of file bounds");
        }

        std::vector<uint8_t> segment;
        segment.reserve(memsz);
        segment.insert(segment.end(), file.begin() + fileOffset, file.begin() + fileOffset + filesz);
        segment.resize(memsz, 0);

        Append_Block(image, paddr != 0 ? paddr : vaddr, std::move(segment));
    }

    if (image.blocks.empty()) {
        throw std::runtime_error("ELF file has no loadable segments");
    }

    return image;
}

SREC_Image Load_Binary_Image(const std::string& filename) {
    const std::string ext = Lowercase_Extension(filename);

    if (ext == ".srec" || ext == ".s19" || ext == ".s28" || ext == ".s37") {
        return Parse_SREC_File(filename);
    }

    if (ext == ".elf") {
        return Parse_ELF_File(filename);
    }

    if (ext == ".img" || ext == ".bin") {
        return Parse_Raw_Image_File(filename);
    }

    const std::vector<uint8_t> header = Read_Binary_File(filename);
    if (header.size() >= 4 && header[0] == 0x7F && header[1] == 'E' && header[2] == 'L' && header[3] == 'F') {
        return Parse_ELF_File(filename);
    }

    if (header.size() >= 2 && header[0] == 'S' && std::isdigit(header[1])) {
        return Parse_SREC_File(filename);
    }

    return Parse_Raw_Image_File(filename);
}

void Append_U16BE(std::string& packet, uint16_t value) {
    packet.push_back(static_cast<char>((value >> 8) & 0xFF));
    packet.push_back(static_cast<char>(value & 0xFF));
}

void Append_U32BE(std::string& packet, uint32_t value) {
    packet.push_back(static_cast<char>((value >> 24) & 0xFF));
    packet.push_back(static_cast<char>((value >> 16) & 0xFF));
    packet.push_back(static_cast<char>((value >> 8) & 0xFF));
    packet.push_back(static_cast<char>(value & 0xFF));
}

bool Wait_For_Binary_ACK(HANDLE handle) {
    char c;

    while (Read_Port_Char(handle, c)) {
        if (c == Binary_ACK) {
            return true;
        }

        if (c == 'F') {
            std::string failCode;
            for (int i = 0; i < 3; i++) {
                if (!Read_Port_Char(handle, c)) {
                    break;
                }
                failCode.push_back(c);
            }

            std::cerr << "Bootloader failure";
            if (!failCode.empty()) {
                std::cerr << " F" << failCode;
            }
            std::cerr << std::endl;
            return false;
        }
    }

    return false;
}

bool Send_Command_No_Payload(HANDLE handle, Binary_Command command) {
    std::string packet;
    packet.push_back(static_cast<char>(command));
    return Write_Port(handle, packet);
}

bool Send_Address_Command(HANDLE handle, Binary_Command command, uint32_t address) {
    std::string packet;
    packet.push_back(static_cast<char>(command));
    Append_U32BE(packet, address);
    return Write_Port(handle, packet) && Wait_For_Binary_ACK(handle);
}

bool Send_Write_Command(HANDLE handle, const uint8_t* data, size_t length) {
    std::string packet;
    packet.reserve(3 + length);
    packet.push_back(static_cast<char>(Binary_Command::Write));
    Append_U16BE(packet, static_cast<uint16_t>(length));
    packet.append(reinterpret_cast<const char*>(data), length);
    return Write_Port(handle, packet) && Wait_For_Binary_ACK(handle);
}

bool Send_Zero_Command(HANDLE handle, size_t length) {
    std::string packet;
    packet.push_back(static_cast<char>(Binary_Command::Write_Zero));
    Append_U16BE(packet, static_cast<uint16_t>(length));
    return Write_Port(handle, packet) && Wait_For_Binary_ACK(handle);
}

bool Send_Baud_Command(HANDLE handle, int baudRate) {
    std::string packet;
    packet.push_back(static_cast<char>(Binary_Command::Set_Baud));
    Append_U16BE(packet, Baud_To_Mini_UART_Divisor(baudRate));
    return Write_Port(handle, packet) && Wait_For_Binary_ACK(handle);
}

bool Upload_Text_SREC(HANDLE handle, const std::string& filename) {
    std::string ln;

    std::ifstream ifile(filename);
    if (!ifile.is_open()) {
        std::cerr << "Could not open input file: " << filename << std::endl;
        return false;
    }

    std::string str((std::istreambuf_iterator<char>(ifile)), std::istreambuf_iterator<char>());
    ifile.seekg(0, std::ios::beg);

    auto linecnt = static_cast<size_t>(std::count_if(str.begin(), str.end(), [](char c) { return c == '\n'; }));
    if (!str.empty() && str.back() != '\n') {
        linecnt++;
    }

    std::cout << "Input: SREC text, " << linecnt << " lines" << std::endl
              << "Protocol: legacy text SREC" << std::endl;

    size_t linecounter = 0;
    Progress_Bar progressBar("Upload", linecnt, "lines");
    progressBar.Update(0);
    while (std::getline(ifile, ln)) {
        linecounter++;

        if (!Write_Port(handle, ln)) {
            return false;
        }

        progressBar.Update(linecounter);
    }

    progressBar.Finish();
    return true;
}

bool Upload_Binary_Image(HANDLE handle, const std::string& filename, int baudRate) {
    constexpr size_t Zero_Run_Threshold = 32;
    constexpr size_t Max_Binary_Chunk = 8192;
    constexpr size_t Max_Command_Length = 0xFFFF;

    const SREC_Image image = Load_Binary_Image(filename);

    std::cout << "Input: " << image.blocks.size() << " block(s), " << image.byteCount << " payload bytes" << std::endl
              << "Protocol: SREC-200 binary" << std::endl;

    if (!Write_Port(handle, "UBIN1") || !Wait_For_Binary_ACK(handle)) {
        return false;
    }

    if (baudRate != Default_Baud_Rate) {
        std::cout << "Baud rate: switching to " << baudRate << std::endl;
        if (!Send_Baud_Command(handle, baudRate) || !Set_Port_Parameters(handle, baudRate)) {
            return false;
        }
    }

    size_t uploaded = 0;
    Progress_Bar progressBar("Upload", image.byteCount, "bytes");
    progressBar.Update(0);

    for (const Program_Block& block : image.blocks) {
        if (!Send_Address_Command(handle, Binary_Command::Set_Cursor, block.address)) {
            return false;
        }

        size_t offset = 0;
        while (offset < block.data.size()) {
            size_t zeroRun = 0;
            while (offset + zeroRun < block.data.size() && block.data[offset + zeroRun] == 0) {
                zeroRun++;
            }

            if (zeroRun > Zero_Run_Threshold) {
                while (zeroRun > 0) {
                    const size_t commandLength = std::min(zeroRun, Max_Command_Length);
                    if (!Send_Zero_Command(handle, commandLength)) {
                        return false;
                    }

                    offset += commandLength;
                    uploaded += commandLength;
                    zeroRun -= commandLength;
                    progressBar.Update(uploaded);
                }
            }
            else {
                const size_t chunkStart = offset;
                size_t chunkLength = 0;

                while (offset + chunkLength < block.data.size() && chunkLength < Max_Binary_Chunk) {
                    size_t nextZeroRun = 0;
                    while (offset + chunkLength + nextZeroRun < block.data.size() &&
                           block.data[offset + chunkLength + nextZeroRun] == 0) {
                        nextZeroRun++;
                    }

                    if (nextZeroRun > Zero_Run_Threshold) {
                        break;
                    }

                    if (nextZeroRun > 0) {
                        const size_t remainingChunk = Max_Binary_Chunk - chunkLength;
                        const size_t consumed = std::min(nextZeroRun, remainingChunk);
                        chunkLength += consumed;
                        if (consumed < nextZeroRun) {
                            break;
                        }
                    }
                    else {
                        chunkLength++;
                    }
                }

                if (chunkLength == 0) {
                    chunkLength = std::min(zeroRun, Max_Binary_Chunk);
                }

                if (!Send_Write_Command(handle, block.data.data() + chunkStart, chunkLength)) {
                    return false;
                }

                offset += chunkLength;
                uploaded += chunkLength;
                progressBar.Update(uploaded);
            }
        }
    }

    progressBar.Finish();
    return Send_Address_Command(handle, Binary_Command::Set_Entry, image.entry);
}

int main(int argc, char** argv)
{
    bool binaryMode = false;
    int binaryBaudRate = Default_Baud_Rate;
    std::vector<std::string> positionalArgs;

    try {
        for (int i = 1; i < argc; i++) {
            const std::string arg = argv[i];
            if (arg == "-b" || arg == "--binary") {
                binaryMode = true;
            }
            else if (arg == "--baud") {
                if (i + 1 >= argc) {
                    Print_Usage(argv[0]);
                    return 3;
                }
                binaryBaudRate = Parse_Baud_Rate(argv[++i]);
            }
            else if (arg.rfind("--baud=", 0) == 0) {
                binaryBaudRate = Parse_Baud_Rate(arg.substr(7));
            }
            else {
                positionalArgs.push_back(arg);
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 3;
    }

    if (positionalArgs.size() < 2 || positionalArgs.size() > 3) {
        Print_Usage(argv[0]);
        return 3;
    }

    if (!binaryMode && binaryBaudRate != Default_Baud_Rate) {
        std::cerr << "--baud is available only with binary mode (-b)" << std::endl;
        return 3;
    }

    NRun_Command runCommand = NRun_Command::None;
    if (positionalArgs.size() == 3) {
        runCommand = Parse_Run_Command(positionalArgs[2]);
    }

    const std::string filename = positionalArgs[0];
	const std::string port = positionalArgs[1];
    const HANDLE hComm = Open_Port(port);

    if (!Is_Valid_Handle(hComm)) {
        std::cerr << "Could not open serial port: " << port << std::endl;
        return 1;
    }

    if (!Set_Port_Parameters(hComm)) {
        std::cerr << "Could not configure serial port: " << port << std::endl;
        return 1;
    }

    std::cout << "Serial port: " << port << " at " << Default_Baud_Rate << " baud, 8N1" << std::endl;

    try {
        if (binaryMode) {
            if (!Upload_Binary_Image(hComm, filename, binaryBaudRate)) {
                std::cerr << "Binary upload failed" << std::endl;
                Close_Port(hComm);
                return 1;
            }
        }
        else {
            if (!Upload_Text_SREC(hComm, filename)) {
                std::cerr << "Text upload failed" << std::endl;
                Close_Port(hComm);
                return 1;
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        Close_Port(hComm);
        return 1;
    }

    std::cout << "Starting uploaded image..." << std::endl;

    // write the "G" symbol, that instructs the bootloader to jump to uploaded code (uploading finished)
    Write_Port(hComm, "G");

    std::cout << "Upload complete." << std::endl;

    switch (runCommand) {
        case NRun_Command::Local:
        {
            std::cout << "Serial monitor attached. Press Ctrl+C to stop." << std::endl;

            // this just reads all characters from the port and displays them in console; this is a debug feature,
            // that simplifies debugging in early stages of development
            char c;
            while (Read_Port_Char(hComm, c))
                std::cout << c;

            // once we are done, close the port
            Close_Port(hComm);

            break;
        }
        case NRun_Command::PuTTY:
        {
            std::cout << "Opening PuTTY on " << port << "..." << std::endl;

            // release the port, so PuTTY can open it
            Close_Port(hComm);

            // find PuTTY executable
            const std::string puttyPath = Find_PuTTY();
            if (puttyPath.empty()) {
                std::cerr << "Could not find PuTTY executable" << std::endl;
                return 1;
            }

            system(("\"" + puttyPath + "\" -serial " + port + " -sercfg 115200,8,n,1,N").c_str());
            break;
        }
        default:
            break;
    }

    return 0;
}
