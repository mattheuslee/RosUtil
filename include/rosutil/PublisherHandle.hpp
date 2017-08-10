#pragma once

#ifndef PUBLISHERHANDLE_HPP_
#define PUBLISHERHANDLE_HPP_

#include "ros/ros.h"

#include <string>

namespace rosutil {

template <class T>
class PublisherHandle {

public:
    typedef T message_t;
    typedef bool (*Comparator)(message_t const &, message_t const &);
    typedef bool (*Predicate)(message_t const &);

    PublisherHandle(ros::NodeHandle & nh, std::string const & topicName, unsigned const & bufferSize)
            : PublisherHandle(nh, topicName, bufferSize, alwaysTrueComparator<message_t>, alwaysTruePredicate<message_t>) {
    }

    PublisherHandle(ros::NodeHandle & nh, std::string const & topicName, unsigned const & bufferSize,
                    Comparator const & comparator)
            : PublisherHandle(nh, topicName, bufferSize, comparator, alwaysTruePredicate<message_t>) {
    }

    PublisherHandle(ros::NodeHandle & nh, std::string const & topicName, unsigned const & bufferSize,
                    Predicate const & predicate)
            : PublisherHandle(nh, topicName, bufferSize, alwaysTrueComparator<message_t>, predicate) {
    }

    PublisherHandle(ros::NodeHandle & nh, std::string const & topicName, unsigned const & bufferSize,
                    Comparator const & comparator, Predicate const & predicate)
            : nh_(nh), topicName_(topicName), bufferSize_(bufferSize), comparator_(comparator), predicate_(predicate) {
        publisher_ = nh_.advertise<message_t>(topicName_.c_str(), bufferSize_);
    }

    PublisherHandle(PublisherHandle&&) = delete;

    PublisherHandle(PublisherHandle const &) = delete;

    PublisherHandle& operator=(PublisherHandle const &) = delete;

    bool publish(message_t const & message) {
        if (predicate_(message)) {
            if (comparator_(lastMessage_, message)) {
                publisher_.publish(message);
                lastMessage_ = message;
                return true;
            }
        }
        return false;
    }

    message_t last_message() {
        return lastMessage_;
    }

private:
    Comparator comparator_;
    Predicate predicate_;

    ros::NodeHandle& nh_;
    std::string topicName_;
    unsigned bufferSize_;
    ros::Publisher publisher_;
    message_t lastMessage_;

};

} // rosutil

#endif // PUBLISHERHANDLE_HPP_
