#pragma once

#ifndef SUBSCRIBERHANDLE_HPP_
#define SUBSCRIBERHANDLE_HPP_

#include "ros/ros.h"

#include <queue>
#include <string>

namespace rosutil {

template <class T>
class SubscriberHandle {

public:
    typedef T message_t;
    typedef bool (*Comparator)(message_t const &, message_t const &);
    typedef bool (*Predicate)(message_t const &);

    SubscriberHandle(ros::NodeHandle& nh, std::string const & topicName, unsigned const & bufferSize)
            : SubscriberHandle(nh, topicName, bufferSize, alwaysTrueComparator<message_t>, alwaysTruePredicate<message_t>, 1 ) {
    }

    SubscriberHandle(ros::NodeHandle& nh, std::string const & topicName, unsigned const & bufferSize,
                     Comparator const & comparator)
            : SubscriberHandle(nh, topicName, bufferSize, comparator, alwaysTruePredicate<message_t>, 1) {
    }

    SubscriberHandle(ros::NodeHandle& nh, std::string const & topicName, unsigned const & bufferSize,
                     Predicate const & predicate)
            : SubscriberHandle(nh, topicName, bufferSize, alwaysTrueComparator<message_t>, predicate, 1) {
    }

    SubscriberHandle(ros::NodeHandle& nh, std::string const & topicName, unsigned const & bufferSize,
                     unsigned const & queueLength)
            : SubscriberHandle(nh, topicName, bufferSize, alwaysTrueComparator<message_t>, alwaysTruePredicate<message_t>, queueLength) {
    }

    SubscriberHandle(ros::NodeHandle& nh, std::string const & topicName, unsigned const & bufferSize,
                     Comparator const & comparator, Predicate const & predicate)
            : SubscriberHandle(nh, topicName, bufferSize, comparator, predicate, 1) {
    }

    SubscriberHandle(ros::NodeHandle& nh, std::string const & topicName, unsigned const & bufferSize,
                     Predicate const & predicate, unsigned const & queueLength)
            : SubscriberHandle(nh, topicName, bufferSize, alwaysTrueComparator<message_t>, predicate, queueLength) {
    }

    SubscriberHandle(ros::NodeHandle& nh, std::string const & topicName, unsigned const & bufferSize,
                     Comparator const & comparator, unsigned const & queueLength)
            : SubscriberHandle(nh, topicName, bufferSize, comparator, alwaysTruePredicate<message_t>, queueLength) {
    }

    SubscriberHandle(ros::NodeHandle& nh, std::string const & topicName, unsigned const & bufferSize,
                     Comparator const & comparator, Predicate const & predicate, unsigned const & queueLength)
            : nh_(nh), topicName_(topicName), bufferSize_(bufferSize), comparator_(comparator), predicate_(predicate), queueLength_(queueLength) {
        subscriber_ = nh_.subscribe(topicName.c_str(), bufferSize_, &SubscriberHandle::callback_, this);
    }

    SubscriberHandle(SubscriberHandle&&) = delete;

    SubscriberHandle(SubscriberHandle const &) = delete;

    SubscriberHandle& operator=(SubscriberHandle const &) = delete;

    ~SubscriberHandle() = default;

    message_t last_message() {
        isUpdated_ = false;
        return lastMessage_;
    }

    std::queue<message_t> message_queue() {
        isUpdated_ = false;
        return messageQueue_;
    }

    bool is_updated() {
        return isUpdated_;
    }

private:
    Comparator comparator_;
    Predicate predicate_;

    ros::NodeHandle& nh_;
    std::string topicName_;
    unsigned bufferSize_;
    ros::Subscriber subscriber_;
    message_t lastMessage_;
    bool isUpdated_;
    unsigned queueLength_;
    std::queue<message_t> messageQueue_;

    void callback_(typename message_t::ConstPtr const & msgPtr) {
        const message_t& message = *msgPtr;
        if (predicate_(message)) {
            if (comparator_(lastMessage_, message)) {
                isUpdated_ = true;
                lastMessage_ = message;
                messageQueue_.push(message);
                if (messageQueue_.size() > queueLength_) {
                    messageQueue_.pop();
                }
            }
        }
    }

};

} // rosutil

#endif // SUBSCRIBERHANDLE_HPP_
