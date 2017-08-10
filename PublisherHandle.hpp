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
            : PublisherHandle(nh, topicName, bufferSize, alwaysTrueComparator_, alwaysTruePredicate_) {
    }

    PublisherHandle(ros::NodeHandle & nh, std::string const & topicName, unsigned const & bufferSize,
                    Comparator const & comparator)
            : PublisherHandle(nh, topicName, bufferSize, comparator, alwaysTruePredicate_) {
    }

    PublisherHandle(ros::NodeHandle & nh, std::string const & topicName, unsigned const & bufferSize,
                    Predicate const & predicate)
            : PublisherHandle(nh, topicName, bufferSize, alwaysTrueComparator_, predicate) {
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
            if (comparator_(last, message)) {
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
    std::string topic_;
    unsigned bufferSize_;
    ros::Publisher publisher_;
    message_t lastMessage_;

    template <class T>
    bool alwaysTruePredicate_(T const &) {
        return true;
    }

    template <class T>
    bool alwaysTrueComparator_(T const &, T const &) {
        return true;
    }

};

} // rosutil

#endif // PUBLISHERHANDLE_HPP_
