#include "socketcan.h"

#include <errno.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

SocketCan &SocketCan::Get()
{
    static SocketCan instance;
    return instance;
}

void SocketCan::StartServer(const char *p_if_name, const std::vector<can_filter> &p_filter_list, int p_can_frame_interval)
{

    if (m_sock != -1) return;

    m_can_frame_interval = p_can_frame_interval;

    struct ifreq ifr;
    struct sockaddr_can sock_addr;

    m_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (m_sock == -1)
    {
        throw std::string(strerror(errno));
    }

    std::vector<can_filter> filter_list;
    for (auto ft : p_filter_list)
    {
        struct can_filter filter;

        filter.can_id = CAN_EFF_FLAG | ft.can_id;
        filter.can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | ft.can_mask;

        filter_list.push_back(filter);
    }

    if (setsockopt(m_sock, SOL_CAN_RAW, CAN_RAW_FILTER, filter_list.data(), sizeof(can_filter) * filter_list.size()) == -1)
    {
        throw std::string(strerror(errno));
    }

    strcpy(ifr.ifr_ifrn.ifrn_name, p_if_name);
    if (ioctl(m_sock, SIOCGIFINDEX, &ifr) == -1)
    {
        throw std::string(strerror(errno));
    }

    sock_addr.can_family = AF_CAN;
    sock_addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(m_sock, reinterpret_cast<struct sockaddr *>(&sock_addr), sizeof(sock_addr)) == -1)
    {
        throw std::string(strerror(errno));
    }

    m_thread_read_incoming = std::thread{&SocketCan::ReadIncoming};
    m_thread_write_outgoing = std::thread{&SocketCan::WriteOutgoing};
}

void SocketCan::StopServer()
{
    if (m_sock != -1)
    {
        {
            std::unique_lock<std::mutex> lock1(m_incoming_mtx, std::defer_lock_t());
            std::unique_lock<std::mutex> lock2(m_outgoing_mtx, std::defer_lock_t());
            lock(lock1, lock2);

            m_socket_closed = true;
            close(m_sock);
            m_sock = -1;

            m_incoming_ready_cv.notify_all();
            m_outgoing_ready_cv.notify_all();
        }

        pthread_cancel(m_thread_read_incoming.native_handle());
        pthread_cancel(m_thread_write_outgoing.native_handle());

        m_thread_read_incoming.join();
        m_thread_write_outgoing.join();
    }
}

void SocketCan::EnqueueOutgoing(struct can_frame &p_frame)
{
    std::lock_guard<std::mutex> lock(m_outgoing_mtx);

    m_outgoing_queue.push(p_frame);
    m_outgoing_ready_cv.notify_one();
}

void SocketCan::EnqueueOutgoing(std::vector<struct can_frame> &p_frame_list)
{
    std::lock_guard<std::mutex> lock(m_outgoing_mtx);

    for (auto &frame : p_frame_list)
        m_outgoing_queue.push(frame);

    m_outgoing_ready_cv.notify_one();
}

bool SocketCan::DequeueIncoming(struct can_frame &p_frame)
{
    std::unique_lock<std::mutex> lock(m_incoming_mtx);

    m_incoming_ready_cv.wait(lock, [&] { return !m_incoming_queue.empty() || m_socket_closed; });

    if (m_socket_closed) return false;

    p_frame = m_incoming_queue.front();
    m_incoming_queue.pop();
    return true;
}

void SocketCan::ReadIncoming()
{
    struct can_frame frame;

    while (!m_socket_closed)
    {
        if (read(m_sock, &frame, sizeof(struct can_frame)) == -1)
        {
            if (m_socket_closed) return; // never happend during tests
            throw std::string(strerror(errno));
        }

        std::lock_guard<std::mutex> lock(m_incoming_mtx);
        m_incoming_queue.push(frame);
        m_incoming_ready_cv.notify_one();
    }
}

void SocketCan::WriteOutgoing()
{
    while (!m_socket_closed)
    {
        std::unique_lock<std::mutex> lock(m_outgoing_mtx);

        m_outgoing_ready_cv.wait(lock, [&] { return !m_outgoing_queue.empty() || m_socket_closed; });
        if (m_socket_closed) return;

        while (!m_outgoing_queue.empty())
        {
            struct can_frame frame = m_outgoing_queue.front();

            if ((write(m_sock, &frame, sizeof(struct can_frame))) == -1)
            {
                throw std::string(strerror(errno));
            }

            m_outgoing_queue.pop();

            std::this_thread::sleep_for(std::chrono::microseconds(m_can_frame_interval));
        }
    }
}

int SocketCan::m_sock = -1;
int SocketCan::m_can_frame_interval = 1;
bool SocketCan::m_socket_closed = false;

std::queue<struct can_frame> SocketCan::m_incoming_queue;
std::queue<struct can_frame> SocketCan::m_outgoing_queue;

std::mutex SocketCan::m_incoming_mtx;
std::mutex SocketCan::m_outgoing_mtx;

std::condition_variable SocketCan::m_incoming_ready_cv;
std::condition_variable SocketCan::m_outgoing_ready_cv;

std::thread SocketCan::m_thread_read_incoming;
std::thread SocketCan::m_thread_write_outgoing;