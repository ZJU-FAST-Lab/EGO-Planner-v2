/**
 * @file reliable_bridge.hpp
 * @author Haojia Li (wisderek@stumail.neu.edu.cn)
 * @brief Reliable bridge for ros data transfer in unstable network.
 * It will estiblish the connections as peer to peer mode.
 * It will reconnect each other autonomously.
 * It can guarantee the data transfer to the device in the strict order.
 * It has a queue for sending data asynchronously. 
 * 
 * Note: This program relies on ZMQ amd ZMQPP.
 * sudo apt install libzmqpp-dev
 * 
 * Core Idea: It would create the sending and receving thread for each devices and processing asynchronously.
 * The index number would correspond to the resource for one device.
 *       
 * @version 0.1
 * @date 2021-08-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __RELIABLE_BRIDGE__
#define __RELIABLE_BRIDGE__
#include "zmq.hpp"
#include "stdio.h"
#include "stdlib.h"
#include "time.h"
#include "zmqpp/zmqpp.hpp"
#include "deque"
#include "map"
#include "ros/ros.h"
#include "thread"
#include "atomic"
#include "mutex"
#include "condition_variable"
#include <boost/shared_array.hpp>

using namespace std;
namespace ser = ros::serialization;

/**
 * @brief overload operator<< for add SerializedMessage to zmqpp::message
 */
zmqpp::message &operator<<(zmqpp::message &in, ros::SerializedMessage const &msg)
{
    in.add_raw(reinterpret_cast<void const *>(msg.buf.get()), msg.num_bytes);
    return in;
}

class ReliableBridge
{

#define OUTPUT_MSG 2

#if (OUTPUT_MSG == 1)
#define print_info(...) ROS_INFO(__VA_ARGS__)
#define print_warning(...)
#elif (OUTPUT_MSG == 2)
#define print_info(...) ROS_INFO(__VA_ARGS__)
#define print_warning(...) ROS_ERROR(__VA_ARGS__)
#else
#define print_info(...)
#define print_warning(...)
#endif

private:
    /**
     * @brief sender thread, send data in queue asynchronously.
     * 
     * @param index ：thread index number
     */
    void transport_msg_thread(int index)
    {
        // int send_ID = ID;
        int ind = index;
        while (thread_flag)
        {
            {
                unique_lock<mutex> locker(*sender_mutex.at(ind));
                cond[ind]->wait(locker);
                auto &buffer = send_data[ind];
                locker.unlock(); //release in time

                while (!buffer->empty() && thread_flag) {

                    auto &data = buffer->front();
                    zmqpp::message send_array;

                    send_array << data.first << data.second.num_bytes << data.second; //  add data
                    if (senders[ind]->send(send_array, false))                               //block here, wait for sending
                    {
                        unique_lock<mutex> locker2(*sender_mutex.at(ind));
                        buffer->pop_front(); //delete the first data
                        locker2.unlock();
                    }
                }
            }
        }
    }
    /**
     * @brief receiver thread
     * Receive data and call the callback function asynchronously.
     * 
     * @param index ：thread index number
     */
    void recv_thread(int index)
    {
        int ind = index;
        int ID = id_list[ind];
      //ROS_ERROR("CREATE REC THREAD %ld", std::hash<std::thread::id>{}(std::this_thread::get_id()));
        while (thread_flag)
        {
            zmqpp::message recv_array;

            if (receivers[ind]->receive(recv_array, false))
            {
                string topic_name;
                size_t data_len;

                // unpack meta data
                recv_array >> topic_name >> data_len;

                // unpack ros messages
                ros::SerializedMessage msg_ser;
                msg_ser.buf.reset(new uint8_t[data_len]);
                memcpy(msg_ser.buf.get(), static_cast<const uint8_t *>(recv_array.raw_data(recv_array.read_cursor())), data_len);
                recv_array.next(); //move read_cursor for next part.
                msg_ser.num_bytes = data_len;
                msg_ser.message_start = msg_ser.buf.get() + 4;

                auto &topic_cb = callback_list[ind];
                const auto &iter = topic_cb->find(topic_name); //find topic
                if (iter != topic_cb->end())
                {
                    iter->second(ID, msg_ser); //go into callback;
                }
            }
            usleep(1);
        }

    }


public:
  bool thread_flag;

  void StopThread() {
    thread_flag = false;
    for (size_t i = 0; i < ip_list.size(); i++) {
      senders[i]->close();
    }


   for (std::thread &th : recv_threads) {
      if(th.joinable()) {
        //std::cout << "join rec thread: " << th.get_id() << std::endl;
        pthread_cancel(th.native_handle());
        std::cout << "rec thread stopped" << std::endl;
      } else {
        std::cout << "can not join rec thread: " << th.get_id() << std::endl;
      }
    }

    for (std::thread &th : send_threads) {
      if(th.joinable()) {
        //std::cout << "join send thread: " << th.get_id() << std::endl;
        pthread_cancel(th.native_handle());
        //th.join();
        std::cout << "send thread stopped" << std::endl;
      } else {
        std::cout << "can not join send thread: " << th.get_id() << std::endl;
      }
    }

  }
    zmqpp::context_t context;

    //for communicating with each device,each device is allocted
    //a sender, a receiver, a send_data queue, for data transfer
    // a sender_mutex,a condition_variable for notificating the sender
    // a send_thread,a recv_thread
    // a callback map for finding the call back function

    vector<unique_ptr<zmqpp::socket>> senders;   //index senders
    vector<unique_ptr<zmqpp::socket>> receivers; //index receivers

    //data queue
    vector<unique_ptr<std::deque<pair<string, ros::SerializedMessage>>>> send_data;

    //mutiple threads
    vector<unique_ptr<mutex>> sender_mutex;
    vector<unique_ptr<condition_variable>> cond;
    vector<std::thread> send_threads;
    vector<std::thread> recv_threads;

    //callback table   index->topic_name-> callback
    vector<unique_ptr<map<string, function<void(int, ros::SerializedMessage &)>>>> callback_list;

    map<int, int> id_remap; // remap id to index
    vector<int> id_list;    //save all the ID
    vector<string> ip_list; //save all IP
    const int self_id;      //self id
    const uint queue_size;

    /**
     * @brief Does the V has suplicate?
     * 
     * @tparam T <type>
     * @param v vector
     * @return true :Contain Duplicate
     * @return false :No Duplicate
     */
    template <typename T>
    static bool containsDuplicate(const vector<T> &v)
    {
        unordered_set<T> s(v.size() * 2);
        for (auto x : v)
            if (!s.insert(x).second)
                return true;
        return false;
    }

    /**
     * @brief Construct a new Reliable Bridge object
     * 
     * @param self_ID self ID (0-99)
     * @param IP_list other device IP
     * @param ID_list other device ID numbers (0-99)
     * @param Queue_size max send message queue for one device
     */
    ReliableBridge(const int self_ID, vector<string> &IP_list, vector<int> &ID_list, uint Queue_size = 10000) : 
    self_id(self_ID), ip_list(IP_list), id_list(ID_list), queue_size(Queue_size), thread_flag(true)
    {
        
        if(ip_list.size() != id_list.size())
        {
            print_warning("[Bridge Warning]: IP doesn't match ID!");
            assert(ip_list.size() == id_list.size());       
        }    
        int maxValue = *max_element(id_list.begin(),id_list.end());
        int minValue = *min_element(id_list.begin(),id_list.end());
        if(id_list.size()  > 100)
        {
            print_warning("[Bridge Warning]: Bridge only supports up to 100 devices!");
            assert(id_list.size()  <= 100);
            return;
        }
        if(maxValue  > 100 || minValue < 0 || self_ID > 100 || self_ID < 0)
        {
            print_warning("[Bridge Warning]: ID is invalid!");
            assert(maxValue  <= 100 && minValue > 0 && self_ID <= 100 && self_ID > 0);
            return;
        }
        if(containsDuplicate(id_list) == true)
        {
            print_warning("[Bridge Warning]: ID list has duplicate!");
            assert(containsDuplicate(id_list) == false);
            return;
        }
        const auto &self_pos = std::find(id_list.begin(), id_list.end(), self_id);

        if (self_pos != id_list.end()) //remove self ID
        {
            int ind = self_pos - id_list.begin();
            id_list.erase(self_pos);
            ip_list.erase(ip_list.begin() + ind);
        }

        for (size_t i = 0; i < ip_list.size(); i++)
        {
            //build index map, remap the ID(may noncontinuous) to index (continuous).
            id_remap.emplace(std::make_pair(id_list[i], i));

            //The bind port number is 30000+self_id*100+other_device_id.
            string url = "tcp://*:" + to_string(30000 +self_id*100 + id_list[i]);
            print_info("[Bridge]: Bind: %s", url.c_str());
            unique_ptr<zmqpp::socket> sender(new zmqpp::socket(context, zmqpp::socket_type::push));
            sender->bind(url); //for others connection
            senders.emplace_back(std::move(sender));

            //build receivers map
            //The port number is 30000+other_device_id*100+self_ID. It will connect to other device.
            url = "tcp://" + ip_list[i] + ":" + to_string(30000 + id_list[i]*100 + self_id);
            print_info("[Bridge]: Connect: %s", url.c_str());
            unique_ptr<zmqpp::socket> receiver(new zmqpp::socket(context, zmqpp::socket_type::pull));
            receiver->connect(url); //connect to others
            receivers.emplace_back(std::move(receiver));

            callback_list.emplace_back(new map<string, function<void(int, ros::SerializedMessage &)>>());

            send_data.emplace_back(new std::deque<pair<string, ros::SerializedMessage>>());
            sender_mutex.emplace_back(new mutex());      //locker
            cond.emplace_back(new condition_variable()); //Notificate the sender to send data immediately.

            // Create sending and recving thread for every device
            send_threads.emplace_back(std::thread(&ReliableBridge::transport_msg_thread, this, i));
            recv_threads.emplace_back(std::thread(&ReliableBridge::recv_thread, this, i));
        }

//        for (std::thread &th : send_threads)
//        {
//            // If thread Object is Joinable then Join that thread.
//            if (th.joinable())
//                th.detach();
//        }
//        for (std::thread &th : recv_threads)
//        {
//            // If thread Object is Joinable then Join that thread.
//            if (th.joinable())
//                th.detach();
//        }
    }
    // ~ReliableBridge();
    /**
     * @brief register a callback for receiving data from a specific device.
     * For processing data, you should register a callback at the begining.
     * Cation: The callback will be used in other threads, so ensure the callback can be reentrant!!
     * Please use local variables.
     * When you want to write into global variables, it is better to use locker to protect them.
     * @param ID :receive data from specific device (MUST IN ID LIST)
     * @param topic_name :specific topic 
     * @param callback :callback function, like `void callback_sample(int ID, ros::SerializedMessage& m)`
     * @example 
     * void callback_sample(int ID, ros::SerializedMessage& m)
     *      {
     *          geometry_msgs::Point msg;
     *          ros::serialization::deserializeMessage(m,msg);
     *          //ID is which device send this message. 
     *          //msg is the ros message.
     *      }
     */
    void register_callback(int ID, string topic_name, function<void(int, ros::SerializedMessage &)> callback)
    {   
        int ind;
        try
        {
            ind = id_remap.at(ID);
        }
        catch(const std::exception& e)
        {
            print_warning("ID is not in the list!");
            return;
        }
        
        callback_list[ind]->emplace(topic_name, callback);
    }

    /**
     * @brief register a callback for receiving data from all devices.
     * For processing data, you should register a callback at the begining.
     * Cation: The callback will be used in other threads, so ensure the callback can be reentrant!!
     * Please use local variables.
     * When you want to write into global variables, it is better to use locker to protect them.
     * @param topic_name :specific topic 
     * @param callback :callback function, like `void callback_sample(int ID, ros::SerializedMessage& m)`
     * @example 
     * void callback_sample(int ID, ros::SerializedMessage& m)
     *      {
     *          geometry_msgs::Point msg;
     *          ros::serialization::deserializeMessage(m,msg);
     *          //ID is which device send this message. 
     *          //msg is the ros message.
     *      }
     */
    void register_callback_for_all(string topic_name, function<void(int, ros::SerializedMessage &)> callback)
    {
        for (size_t i = 0; i < id_list.size(); i++)
        {
            register_callback(id_list[i], topic_name, callback);
        }
    }

    /**
     * @brief send message to a specific device asynchronously
     * This function will push the data in queue and the send thread will send the data immediately.
     * @tparam T <message type>
     * @param ID :Which device do you want to send (MUST IN ID LIST). 
     * @param topic_name :topic name
     * @param msg :ros message you want to send
     * @return int :0 for no error, -1 for queue is full 
     * @example 
     * std_msgs::String text;
     * send_msg_to_one(1,"hello",text);
     */
    template <typename T>
    int send_msg_to_one(int ID, string topic_name, T &msg)
    {
        int ind;
        try
        {
            ind = id_remap.at(ID);
        }
        catch(const std::exception& e)
        {
            print_warning("ID is not in the list!");
            return -2;
        }
        auto &buffer = send_data[ind];
        if (buffer->size() > queue_size)
        {
            print_warning("[Bridge Warning]: ID:%d Send buffer is full", ID);
            return -1; //buffer is full
        }
        {
            unique_lock<mutex> locker(*sender_mutex.at(ind));
            buffer->emplace_back(make_pair(topic_name, ser::serializeMessage<T>(msg)));
            locker.unlock();
            cond[ind]->notify_all();
        }
        return 0;
    }
    /**
     * @brief send message to a all device asynchronously
     * This function will push the data in queue and the send thread will send the data immediately.
     * @tparam T <message type>
     * @param topic_name :topic name
     * @param msg :ros message you want to send
     * @return int :0 for no error, <0 for queue is full 
     * @example 
     * std_msgs::String text;
     * send_msg_to_all(1,"hello",text);
     */
    template <typename T>
    int send_msg_to_all(string topic_name, T &msg)
    {
        int err_code = 0;
        for (size_t i = 0; i < id_list.size(); i++)
        {
            err_code += send_msg_to_one<T>(id_list[i], topic_name, msg);
        }
        return err_code;
    }
};

#endif
