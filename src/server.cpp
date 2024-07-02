#include "socketcan.h"

#include <iomanip>
#include <iostream>
#include <signal.h>

void CtrlC_Handler(int p_sig)
{
    if (p_sig == SIGINT)
    {
        SocketCan::StopServer();
    }
}

int main(int argc, char *argv[])
{
    if (argc == 1)
    {
        std::cout << "usage: " << argv[0] << " <canX>" << std::endl;
        return EXIT_FAILURE;
    }

    signal(SIGINT, CtrlC_Handler);

    std::vector<can_filter> filters;
    can_filter filter;
    filter.can_id = 0;
    filter.can_mask = 0;
    filters.push_back(filter);

    SocketCan::StartServer(argv[1], filters);

    struct can_frame raw_frame;
    while (SocketCan::DequeueIncoming(raw_frame))
    {
        std::cout << std::hex << std::setw(8) << std::setfill('0') << raw_frame.can_id;
        std::cout << " [" << (int)raw_frame.can_dlc << "] ";

        for (int i = 0; i < raw_frame.can_dlc; i++)
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)raw_frame.data[i] << ((i < raw_frame.can_dlc - 1) ? "-" : "");

        std::cout << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}
