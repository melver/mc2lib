/*
 * Copyright (c) 2014, Marco Elver
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of the software nor the names of its contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MC2LIB_MEMCONSISTENCY_EVENTSETS_HPP_
#define MC2LIB_MEMCONSISTENCY_EVENTSETS_HPP_

#include <mc2lib/elementsetthy.hpp>

#include <string>
#include <sstream>

namespace mc2lib {
namespace memconsistency {

#ifndef MC2LIB_MEMCONSISTENCY_EVENT_CUSTOM_TYPES
typedef unsigned long long Addr;
typedef unsigned long long Pid;
typedef unsigned long long Poi;
#endif

class Iiid {
  public:
    struct Hash {
        size_t operator()(const Iiid& k) const
        {
            return std::hash<unsigned long long>()(k.pid ^ k.poi);
        }
    };

    Iiid() : pid(0), poi(0)
    {}

    Iiid(Pid pid_, Poi poi_) :
        pid(pid_), poi(poi_)
    {}

    operator std::string() const
    {
        std::ostringstream oss;
        oss << "Iiid(" << pid << ", " << poi << ")";
        return oss.str();
    }

    bool operator==(const Iiid& rhs) const
    {
        return pid == rhs.pid && poi == rhs.poi;
    }

    bool operator!=(const Iiid& rhs) const
    {
        return pid != rhs.pid || poi != rhs.poi;
    }

    bool operator<(const Iiid& rhs) const
    {
        return pid < rhs.pid || (pid == rhs.pid && poi < rhs.poi);
    }

    Iiid& operator++()
    {
        ++poi;
        return *this;
    }

    Iiid next() const
    {
        return Iiid(pid, poi + 1);
    }

    Iiid prev() const
    {
        assert(poi > 0);
        return Iiid(pid, poi - 1);
    }

  public:
    Pid pid;
    Poi poi;
};

class Event {
  public:
    struct Hash {
        size_t operator()(const Event& k) const
        {
            return Iiid::Hash()(k.iiid);
        }
    };

    enum Type {
        None    = 0x0,

        Read    = 0x1,

        Write   = 0x2,

        Acquire = 0x4,

        Release = 0x8,

        MemoryOperation = Read | Write | Acquire | Release
    };

    typedef int TypeMask;

    Event()
        : type(None), addr(0)
    {}

    Event(TypeMask type_, Addr addr_, Iiid iiid_)
        : type(type_), addr(addr_), iiid(iiid_)
    {}

    operator std::string() const
    {
        std::ostringstream oss;
        oss << "Event(";

        if (!type) {
            oss << "None";
        } else if (all_type(MemoryOperation)) {
            oss << "MemoryOperation";
        } else {
            bool found_type = false;

            if (all_type(Read)) {
                oss << "Read";
                found_type = true;
            }

            if (all_type(Write)) {
                oss << (found_type ? "|" : "") << "Write";
                found_type = true;
            }

            if (all_type(Acquire)) {
                oss << (found_type ? "|" : "") << "Acquire";
                found_type = true;
            }

            if (all_type(Release)) {
                oss << (found_type ? "|" : "") << "Release";
            }
        }

        oss << ", " << addr << ", " << static_cast<std::string>(iiid) << ")";
        return oss.str();
    }

    bool operator==(const Event& rhs) const
    {
        return type == rhs.type && addr == rhs.addr && iiid == rhs.iiid;
    }

    bool operator!=(const Event& rhs) const
    {
        return type != rhs.type || addr != rhs.addr || iiid != rhs.iiid;
    }

    // This function in no way says anything about event ordering. Used for
    // ordered map.
    bool operator<(const Event& rhs) const
    {
        return iiid < rhs.iiid;
    }

    bool all_type(TypeMask type_mask) const
    {
        assert(type_mask != None);
        return (type & type_mask) == type_mask;
    }

    bool any_type(TypeMask type_mask) const
    {
        assert(type_mask != None);
        return (type & type_mask) != 0;
    }

  public:
    TypeMask type;
    Addr addr;
    Iiid iiid;
};

typedef elementsetthy::ElementSet<Event> EventSet;
typedef elementsetthy::ElementRel<Event> EventRel;
typedef elementsetthy::ElementRelSeq<Event> EventRelSeq;

} /* namespace memconsistency */
} /* namespace mc2lib */

#endif /* MEMCONSISTENCY_EVENTSETS_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
