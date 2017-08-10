#pragma once

#ifndef ROSUTIL_HPP_
#define ROSUTIL_HPP_

#include "ros/ros.h"

namespace rosutil {

template <class T>
bool alwaysTrueComparator(T const &, T const &) {
    return true;
}

template <class T>
bool alwaysTruePredicate(T const &) {
    return true;
}

} // rosutil

#include "rosutil/PublisherHandle.hpp"
#include "rosutil/SubscriberHandle.hpp"

#endif // ROSUTIL_HPP_
