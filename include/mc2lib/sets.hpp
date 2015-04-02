/*
 * Copyright (c) 2014-2015, Marco Elver
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

#ifndef MC2LIB_SETS_HPP_
#define MC2LIB_SETS_HPP_

#include "config.hpp"

#include <cassert>
#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mc2lib {

/**
 * @namespace mc2lib::sets
 * @brief Sets and maps exposed in a restricted set of set theory.
 */
namespace sets {

/**
 * Checks that a bit mask has all given bits set.
 *
 * @param mask Bit mask to check.
 * @param all Bit mask to check against.
 * @return True if all bits in all are also set in mask, false otherwise.
 */
inline bool all_bitmask(unsigned mask, unsigned all)
{
    assert(all != 0);
    return (mask & all) == all;
}

/**
 * Checks that a bit mask has any of given bits set.
 *
 * @param mask Bit mask to check.
 * @param any Bit mask to check against.
 * @return True if at least any one of the bits in all is set in mask.
 */
inline bool any_bitmask(unsigned mask, unsigned any)
{
    assert(any != 0);
    return (mask & any) != 0;
}

template <class Ts>
class Set {
  public:
    typedef typename Ts::Element Element;
    typedef typename Ts::SetContainer Container;

    Set()
    {}

    explicit Set(const Container& s)
        : set_(s)
    {}

    const Container& get() const
    { return set_; }

    bool operator==(const Set& rhs) const
    {
        return set_ == rhs.set_;
    }

    const Element& insert(const Element& e, bool assert_unique = false)
    {
        auto result = set_.insert(e);
        assert(!assert_unique || result.second);
        return *result.first;
    }

    void erase(const Element& e, bool assert_exists = false)
    {
#ifndef NDEBUG
        auto result =
#endif
        set_.erase(e);
        assert(!assert_exists || result != 0);
    }

    template <class FilterFunc>
    Set filter(FilterFunc filterFunc) const
    {
        Set es;

        for (const auto& e : set_) {
            if (filterFunc(e)) {
                es.insert(e);
            }
        }

        return es;
    }

    void clear()
    { set_.clear(); }

    bool contains(const Element& e) const
    {
        return set_.find(e) != set_.end();
    }

    /*
     * Set union.
     */
    Set& operator+=(const Element& rhs)
    {
        insert(rhs);
        return *this;
    }

    Set operator+(const Element& rhs) const
    {
        Set es = *this;
        return es += rhs;
    }

    Set& operator+=(const Set& rhs)
    {
        if (this != &rhs) {
            set_.insert(rhs.set_.begin(), rhs.set_.end());
        }

        return *this;
    }

    Set operator+(const Set& rhs) const
    {
        Set es = *this;
        return es += rhs;
    }

    /*
     * Set difference.
     */
    Set& operator-=(const Element& rhs)
    {
        erase(rhs);
        return *this;
    }

    Set operator-(const Element& rhs) const
    {
        Set es = *this;
        return es -= rhs;
    }

    Set& operator-=(const Set& rhs)
    {
        if (this == &rhs) {
            clear();
        } else {
            // Cannot use set_.erase with iterator, as rhs may contain elements
            // that do not exist in this set. In such a case, the erase
            // implementation of current GCC segfaults.
            for (const auto& e : rhs.get()) {
                erase(e);
            }
        }

        return *this;
    }

    Set operator-(const Set& rhs) const
    {
        Set es = *this;
        return es -= rhs;
    }

    /*
     * Set intersection
     */
    Set operator&(const Set& rhs) const
    {
        Set es;

        for (const auto& e : rhs.get()) {
            if (contains(e)) {
                es.insert(e);
            }
        }

        return es;
    }

    Set& operator&=(const Set& rhs)
    {
        if (this != &rhs) {
            *this = rhs & *this;
        }

        return *this;
    }

    std::size_t size() const
    { return set_.size(); }

    bool empty() const
    { return set_.empty(); }

    bool subseteq(const Set& es) const
    {
        if (size() > es.size()) return false;

        for (const auto& e : set_) {
            if (!es.contains(e)) {
                return false;
            }
        }

        return true;
    }

    bool subset(const Set& es) const
    {
        return size() < es.size() && subseteq(es);
    }

  protected:
     Container set_;
};

template <class Ts>
class Relation {
  public:
    typedef typename Ts::Element Element;

    typedef typename Ts::template MapContainer<Set<Ts>> Container;

    typedef std::pair<Element, Element> Tuple;

    typedef std::vector<Element> Path;

    /*
     * Lazy operators as properties.
     */
    enum Property {
        None              = 0x0,

        TransitiveClosure = 0x1,
        ReflexiveClosure  = 0x2,
        ReflexiveTransitiveClosure = TransitiveClosure | ReflexiveClosure
    };
    typedef unsigned Properties;

    Relation()
        : props_(None)
    {}

    explicit Relation(const Container& r)
        : props_(None), rel_(r)
    {}

    /*
     * Avoid accessing rel_ directly. Uses of raw should be well justified.
     */
    const Container& raw() const
    { return rel_; }

    Properties props() const
    { return props_; }

    Relation& set_props(Properties props)
    {
        props_ = props;
        return *this;
    }

    Relation& add_props(Properties props)
    {
        props_ |= props;
        return *this;
    }

    Relation& unset_props(Properties props)
    {
        props_ &= ~props;
        return *this;
    }

    bool all_props(Properties all) const
    {
        return all_bitmask(props_, all);
    }

    bool any_props(Properties any) const
    {
        return any_bitmask(props_, any);
    }

    Properties clear_props()
    {
        const auto props = props_;
        props_ = None;
        return props;
    }

    void insert(const Element& e1, const Element& e2, bool assert_unique = false)
    {
        rel_[e1].insert(e2, assert_unique);
    }

    void insert(const Element& e1, const Set<Ts>& e2s)
    {
        if (e2s.empty()) return;
        rel_[e1] += e2s;
    }

    void erase(const Element& e1, const Element& e2, bool assert_exists = false)
    {
        // May not work as expect if ReflexiveClosure is set.
        if (contains__(e1)) {
            rel_[e1].erase(e2, assert_exists);

            if (rel_[e1].empty()) {
                rel_.erase(e1);
            }
        }
    }

    void erase(const Element& e1, const Set<Ts>& e2s)
    {
        if (contains__(e1)) {
            rel_[e1] -= e2s;

            if (rel_[e1].empty()) {
                rel_.erase(e1);
            }
        }
    }

    std::size_t size() const
    {
        std::size_t total = 0;

        if (props()) {
            const auto dom = domain();
            for (const auto& e : dom.get()) {
                total += reachable(e).size();
            }
        } else {
            for (const auto& tuples : rel_) {
                total += tuples.second.size();
            }
        }

        return total;
    }

    template <class Func>
    Func for_each(Func func) const
    {
        const auto dom = domain();

        for (const auto& e1 : dom.get()) {
            const auto reach = reachable(e1);
            for (const auto& e2 : reach.get()) {
                func(e1, e2);
            }
        }

        return std::move(func);
    }

    /*
     * Provide eval() for evaluated view of the relation (with properties
     * evaluated).
     */
    Relation eval() const
    {
        if (!props()) {
            return *this;
        }

        Relation result;

        for_each([&result](const Element& e1, const Element& e2) {
            result.insert(e1, e2);
        });

        return result;
    }

    template <class FilterFunc>
    Relation filter(FilterFunc filterFunc) const
    {
        Relation result;

        for_each([&filterFunc, &result](const Element& e1, const Element& e2) {
            if (filterFunc(e1, e2)) {
                result.insert(e1, e2);
            }
        });

        return result;
    }

    Relation inverse() const
    {
        Relation result;

        for_each([&result](const Element& e1, const Element& e2) {
            result.insert(e2, e1);
        });

        return result;
    }

    /*
     * Relation union.
     */
    Relation& operator+=(const Relation& rhs)
    {
        if (rhs.props()) {
            const auto rhs_domain = rhs.domain();
            for (const auto& e : rhs_domain.get()) {
                rel_[e] += rhs.reachable(e);
            }
        } else {
            for (const auto& tuples : rhs.raw()) {
                rel_[tuples.first] += tuples.second;
            }
        }

        return *this;
    }

    Relation operator+(const Relation& rhs) const
    {
        Relation er = *this;
        return er += rhs;
    }

    Relation& operator+=(const Tuple& rhs)
    {
        insert(rhs.first, rhs.second);
        return *this;
    }

    Relation operator+(const Tuple& rhs) const
    {
        Relation er = *this;
        return er += rhs;
    }

    /*
     * Relation difference.
     */
    Relation& operator-=(const Relation& rhs)
    {
        if (rhs.props()) {
            const auto rhs_domain = rhs.domain();
            for (const auto& e : rhs_domain.get()) {
                erase(e, rhs.reachable(e));
            }
        } else {
            for (const auto& tuples : rhs.raw()) {
                erase(tuples.first, tuples.second);
            }
        }

        return *this;
    }

    Relation operator-(const Relation& rhs) const
    {
        Relation er = *this;
        return er -= rhs;
    }

    /*
     * Relation intersection
     */
    Relation operator&(const Relation& rhs) const
    {
        Relation es;

        const auto this_domain = domain();
        for (const auto& e : this_domain.get()) {
            const auto this_reachable = reachable(e);
            const auto rhs_reachable = rhs.reachable(e);
            Set<Ts> intersect = this_reachable & rhs_reachable;

            if (!intersect.empty()) {
                es.rel_[e] = intersect;
            }
        }

        return es;
    }

    Relation& operator&=(const Relation& rhs)
    {
        *this = rhs & *this;
        return *this;
    }

    void clear()
    { rel_.clear(); }

    bool empty() const
    {
        // Upon erasure, we ensure that an element is not related to an empty
        // set, i.e. in that case it is deleted.
        return rel_.empty();
    }

    bool operator==(const Relation& rhs) const
    {
        return (props() ? eval() : *this).rel_ ==
               (rhs.props() ? rhs.eval() : rhs).rel_;
    }

    /*
     * Return true if two elements are ordered.
     */
    bool R(const Element& e1, const Element& e2, Path *path = nullptr) const
    {
        if (e1 == e2 && all_props(ReflexiveClosure)) {
            if (in_on(e1)) {
                if (path != nullptr) {
                    path->push_back(e1);
                    path->push_back(e1);
                }

                return true;
            }
        }

        Set<Ts> visited;

        if (path != nullptr) {
            FlagSet visiting;

            bool result = R_search(e1, &e2, &visited, &visiting);
            if (result) {
                get_path(path, &e1, &e2, &visiting);
            }

            return result;
        }

        return R_search(e1, &e2, &visited);
    }

    /*
     * Returns all rechable elements from a start element without the start
     * itself, but includes start if start can reach itself (e.g. through
     * cycle).
     */
    Set<Ts> reachable(const Element& e) const
    {
        Set<Ts> visited;

        if (all_props(ReflexiveClosure) && in_on(e)) {
            visited.insert(e);
        }

        if (!all_props(TransitiveClosure)) {
            const auto tuples = rel_.find(e);
            if (tuples != rel_.end()) {
                visited += tuples->second;
            }

            return visited;
        }

        if (!R_search(e, &e, &visited, nullptr,
                      None, SearchMode::RelatedVisitAll)) {
            if (!all_props(ReflexiveClosure)) {
                visited -= e;
            }
        }

        return visited;
    }

    bool irreflexive(Path *cyclic = nullptr) const
    {
        return irreflexive(None, cyclic);
    }

    bool acyclic(Path *cyclic = nullptr) const
    {
        return irreflexive(TransitiveClosure, cyclic);
    }

    /*
     * x→y ∧ y→z ⇒ x→z
     */
    bool transitive() const
    {
        if (all_props(TransitiveClosure)) {
            return true;
        }

        for (const auto& tuples1 : rel_) {
            for (const auto& e1 : tuples1.second.get()) {
                const auto tuples2 = rel_.find(e1);
                if (tuples2 != rel_.end()) {
                    for (const auto& e2 : tuples2->second.get()) {
                        if (!R(tuples1.first, e2)) {
                            return false;
                        }
                    }
                }
            }
        }

        return true;
    }

    /*
     * ∀(x,y) ∈ on×on, x→y ∨ y→x
     */
    bool total_on(const Set<Ts>& on) const
    {
        for (const auto& e1 : on.get()) {
            for (const auto& e2 : on.get()) {
                if (!R(e1, e2) && !R(e2, e1)) {
                    return false;
                }
            }
        }

        return true;
    }

    /*
     * ∀(x,y) ∈ on×on, x→y ∨ y→x ∨ x=y
     */
    bool connex_on(const Set<Ts>& on) const
    {
        for (const auto& e1 : on.get()) {
            for (const auto& e2 : on.get()) {
                if (e1 != e2 && !R(e1, e2) && !R(e2, e1)) {
                    return false;
                }
            }
        }

        return true;
    }

    bool weak_partial_order(const Set<Ts>& on) const
    {
        // (domain ∪ range) in on
        for (const auto& tuples : rel_) {
            if (!on.contains(tuples.first) && !tuples.second.subseteq(on)) {
                return false;
            }
        }

        return transitive() && !irreflexive();
    }

    bool weak_total_order(const Set<Ts>& on) const
    {
        return weak_partial_order(on) && total_on(on);
    }

    bool strict_partial_order(const Set<Ts>& on) const
    {
        // (domain ∪ range) in on
        for (const auto& tuples : rel_) {
            if (!on.contains(tuples.first) && !tuples.second.subseteq(on)) {
                return false;
            }
        }

        return transitive() && irreflexive();
    }

    bool strict_total_order(const Set<Ts>& on) const
    {
        return strict_partial_order(on) && connex_on(on);
    }

    bool in_on(const Element& e) const
    {
        if (contains__(e)) {
            return true;
        } else {
            for (const auto& tuples : rel_) {
                if (tuples.second.contains(e)) {
                    return true;
                }
            }
        }

        return false;
    }

    bool in_domain(const Element& e) const
    {
        if (all_props(ReflexiveClosure)) {
            return in_on(e);
        }

        return contains__(e);
    }

    bool in_range(const Element& e) const
    {
        if (all_props(ReflexiveClosure)) {
            return in_on(e);
        }

        for (const auto& tuples : rel_) {
            if (reachable(tuples.first).contains(e)) {
                return true;
            }
        }

        return false;
    }

    Set<Ts> on() const
    {
        Set<Ts> es;
        for (const auto& tuples : rel_) {
            es.insert(tuples.first);
            es += tuples.second;
        }
        return es;
    }

    Set<Ts> domain() const
    {
        if (all_props(ReflexiveClosure)) {
            // By the fact that the reflexive closure is
            // R ∪ {(x,x). x ∈ S}, where S is the set the relation is on, and
            // thus S contains both the range and domain, constructing the
            // above will yield domain = range = S under the reflexive closure.
            return on();
        }

        Set<Ts> es;
        for (const auto& tuples : rel_) {
            es.insert(tuples.first);
        }

        return es;
    }

    Set<Ts> range() const
    {
        if (all_props(ReflexiveClosure)) {
            // See above.
            return on();
        }

        Set<Ts> es;
        for (const auto& tuples : rel_) {
            es += reachable(tuples.first);
        }
        return es;
    }

  protected:
    typedef typename Ts::template MapContainer<bool> FlagSet;

    enum class SearchMode {
        Related,
        RelatedVisitAll,
        FindCycle
    };

    bool contains__(const Element& e) const
    {
        return rel_.find(e) != rel_.end();
    }

    void get_path(Path *out, const Element* start, const Element *end,
                  FlagSet *visiting, SearchMode mode = SearchMode::Related) const
    {
        assert(out != nullptr && start != nullptr && visiting != nullptr);

        out->push_back(*start);
        (*visiting)[*start] = false;

        while (start != nullptr) {
            const Set<Ts>& next = rel_.find(*start)->second;
            start = nullptr;

            for (const auto& e : next.get()) {
                const auto se = visiting->find(e);
                if (se != visiting->end() && se->second) {
                    out->push_back(e);
                    se->second = false;
                    start = &e;
                    break;
                }
            }
        }

        const Set<Ts>& next = rel_.find(out->back())->second;
        if (mode == SearchMode::FindCycle) {
            assert(end == nullptr);

            for (const auto& e : *out) {
                if (next.contains(e)) {
                    // Final edge
                    out->push_back(e);
                    break;
                }
            }
        } else {
            assert(end != nullptr);

            // This function should only be called if search established there
            // is a path between start->end.
            assert(next.contains(*end));

            out->push_back(*end);
        }
    }

    bool irreflexive(Properties local_props, Path *cyclic) const
    {
        local_props |= props_;

        if (all_bitmask(local_props, ReflexiveClosure) && !empty()) {
            if (cyclic != nullptr) {
                // Pick arbitrary.
                cyclic->push_back(rel_.begin()->first);
                cyclic->push_back(rel_.begin()->first);
            }

            return false;
        }

        Set<Ts> visited;
        FlagSet visiting;

        for (const auto& tuples : rel_) {
            if (R_search(tuples.first, nullptr, &visited, &visiting,
                         local_props, SearchMode::FindCycle)) {

                if (cyclic != nullptr) {
                    get_path(cyclic, &tuples.first, nullptr, &visiting,
                             SearchMode::FindCycle);
                }

                return false;
            }
        }

        return true;
    }

    /**
     * Directed graph search.
     */
    bool R_search(const Element& e1, const Element *e2,
                  Set<Ts>* visited,
                  FlagSet* visiting = nullptr,
                  Properties local_props = None,
                  SearchMode mode = SearchMode::Related) const
    {
        // We always require visited to be set.
        assert(visited != nullptr);

        // Merge call-specific props with global props.
        local_props |= props_;

        const bool is_tran_cl = all_bitmask(local_props, TransitiveClosure);

        R_impl search(this, visited, visiting, is_tran_cl, mode);

        if (e2 == nullptr) {
            // For the purpose of cycle detection, we are looking for e1->*e1.
            // In addition, if the transitive property is set, we require that
            // visiting is provided, as we cannot make assumptions on if visited
            // is reset before every call -- and for performance reasons, this
            // usage is discouraged.

            assert(mode == SearchMode::FindCycle);

            e2 = &e1;

            if (is_tran_cl) {
                assert(visiting != nullptr);
                return search.dfs_rec_find_cycle(e1);
            }
        } else {
            assert(mode != SearchMode::FindCycle);
        }

        return search.dfs_rec(e1, *e2);
    }

    class R_impl {
      public:
        R_impl(const Relation *src, Set<Ts>* visited,
               FlagSet* visiting, bool is_tran_cl, SearchMode mode)
            : src_(src)
            , visited_(visited)
            , visiting_(visiting)
            , is_tran_cl_(is_tran_cl)
            , mode_(mode)
        {}

        bool dfs_rec(const Element& e1, const Element& e2) const
        {
            const auto tuples = src_->raw().find(e1);

            if (tuples == src_->raw().end()) {
                return false;
            }

            if (visiting_ != nullptr) {
                (*visiting_)[e1] = true;
            }

            bool result = false;
            visited_->insert(e1);

            for (const auto& e : tuples->second.get()) {
                if (e == e2) {
                    if (mode_ == SearchMode::RelatedVisitAll) {
                        result = true;
                    } else {
                        return true;
                    }
                }

                if (is_tran_cl_) {
                    if (!visited_->contains(e)) {
                        if (dfs_rec(e, e2)) {
                            if (mode_ == SearchMode::RelatedVisitAll) {
                                result = true;
                            } else {
                                return true;
                            }
                        }

                        // There might not be an edge e -> e2, but we must update
                        // the visited set regardless -- this is only relevant, as
                        // the caller might expect the complete set of visited
                        // nodes if mode == RelatedVisitAll.
                        visited_->insert(e);
                    } else {
                        //assert(mode_ != SearchMode::FindCycle);
                    }
                }
            }

            if (visiting_ != nullptr) {
                (*visiting_)[e1] = false;
            }

            return result;
        }

        bool dfs_rec_find_cycle(const Element& start) const
        {
            //assert(is_tran_cl_);
            //assert(mode_ == SearchMode::FindCycle);
            //assert(visiting_ != nullptr);

            const auto tuples = src_->raw().find(start);

            if (tuples == src_->raw().end()) {
                return false;
            }

            (*visiting_)[start] = true;
            visited_->insert(start);

            for (const auto& e : tuples->second.get()) {
                if (!visited_->contains(e)) {
                    if (dfs_rec_find_cycle(e)) {
                        return true;
                    }
                } else {
                    const auto se = visiting_->find(e);
                    if (se != visiting_->end() && se->second) {
                        // Found a backedge --> cycle!
                        return true;
                    }
                }
            }

            (*visiting_)[start] = false;
            return false;
        }

      private:
        const Relation* src_;
        Set<Ts>* visited_;
        FlagSet* visiting_;
        bool is_tran_cl_;
        SearchMode mode_;
    };

  protected:
    Properties props_;
    Container rel_;
};

template <class Ts>
inline Relation<Ts> operator*(const Set<Ts>& lhs,
                                const Set<Ts>& rhs)
{
    Relation<Ts> er;
    for (const auto& e1 : lhs.get()) {
        for (const auto& e2 : rhs.get()) {
            er.insert(e1, e2);
        }
    }
    return er;
}

template <class Ts>
class RelationOp {
  public:
    RelationOp()
    {}

    explicit RelationOp(const std::vector<Relation<Ts>>& rels) :
        rels_(rels) {}

    virtual ~RelationOp()
    {}

    virtual RelationOp& eval_inplace() = 0;

    virtual Relation<Ts> eval() const = 0;

    void clear()
    { rels_.clear(); }

    /**
     * Optimized for move.
     */
    Relation<Ts> eval_clear()
    {
        if (rels_.empty()) {
            // Already cleared
            return Relation<Ts>();
        }

        eval_inplace();
        assert(rels_.size() == 1);

        auto result = std::move(rels_.back());
        rels_.clear();
        return result;
    }

  protected:
    void add(const Relation<Ts>& er)
    {
        rels_.push_back(er);
    }

    void add(const std::vector<Relation<Ts>>& rels)
    {
        rels_.reserve(rels_.size() + rels.size());
        rels_.insert(rels_.end(), rels.begin(), rels.end());
    }

  protected:
    std::vector<Relation<Ts>> rels_;
};

template <class Ts>
class RelationSeq : public RelationOp<Ts> {
  public:
    typedef typename Ts::Element Element;

    RelationSeq()
    {}

    explicit RelationSeq(const std::vector<Relation<Ts>>& v)
        : RelationOp<Ts>(v)
    {}

    RelationSeq& operator+=(const RelationSeq& rhs)
    {
        add(rhs.rels_);
        return *this;
    }

    RelationSeq operator+(const RelationSeq& rhs) const
    {
        RelationSeq ers = *this;
        return ers += rhs;
    }

    RelationSeq& operator+=(const Relation<Ts>& rhs)
    {
        this->add(rhs);
        return *this;
    }

    RelationSeq operator+(const Relation<Ts>& rhs) const
    {
        RelationSeq ers = *this;
        return ers += rhs;
    }

    RelationOp<Ts>& eval_inplace() override
    {
        while (this->rels_.size() > 1) {
            std::size_t from_idx = this->rels_.size() - 2;
            const auto& first = this->rels_[from_idx];
            const auto& last = this->rels_.back();

            Relation<Ts> er;

            first.for_each([&er, &last](const Element& e1, const Element& e2) {
                if (last.in_domain(e2)) {
                    er.insert(e1, last.reachable(e2));
                }
            });

            this->rels_.erase(this->rels_.end() - 2, this->rels_.end());
            this->rels_.push_back(std::move(er));
        }

        return *this;
    }

    Relation<Ts> eval() const override
    {
        Relation<Ts> er;

        if (this->rels_.empty()) {
            return Relation<Ts>();
        } else if (this->rels_.size() == 1) {
            return this->rels_.back();
        }

        const auto potential_domain = this->rels_.front().domain();
        const auto potential_range = this->rels_.back().range();

        for (const auto& e1 : potential_domain.get()) {
            for (const auto& e2 : potential_range.get()) {
                if (R(e1, e2)) {
                    er.insert(e1, e2);
                }
            }
        }

        return er;
    }

    bool R(const Element& e1, const Element& e2,
           typename Relation<Ts>::Path *path = nullptr, std::size_t seq = 0) const
    {
        if (this->rels_.empty()) {
            return false;
        }

        assert(seq < this->rels_.size());

        if (seq + 1 < this->rels_.size()) {
            const auto& rel = this->rels_[seq];
            std::size_t path_size = 0;

            const Set<Ts> reach = rel.reachable(e1);
            for (const auto& e : reach.get()) {
                if (path != nullptr) {
                    path_size = path->size();
                    rel.R(e1, e, path); // true
                    path->pop_back(); // remove e
                }

                if (R(e, e2, path, seq + 1)) {
                    return true;
                }

                if (path != nullptr) {
                    // e not connected to e2, remove all up to e1 (inclusive).
                    assert(path_size < path->size());
                    path->erase(path->begin() + path_size, path->end());
                }
            }

            return false;
        }

        return this->rels_[seq].R(e1, e2, path);
    }

    bool irreflexive(typename Relation<Ts>::Path *cyclic = nullptr) const
    {
        if (this->rels_.empty()) {
            return true;
        }

        const auto domain = this->rels_.front().domain();
        for (const auto& e : domain.get()) {
            if (R(e, e, cyclic)) {
                return false;
            }
        }

        return true;
    }
};

template <class E>
struct Types {
    typedef E Element;

    typedef std::unordered_set<Element, typename Element::Hash> SetContainer;

#if defined(__GNUC__) && (__GNUC__ == 4 && (__GNUC_MINOR__ == 6))
    template <class T>
    class MapContainer : public std::unordered_map<Element, T, typename Element::Hash>
    {};
#else
    // Only works for GCC > 4.6
    template <class T>
    using MapContainer = std::unordered_map<Element, T, typename Element::Hash>;
#endif
};

} /* namespace sets */
} /* namespace mc2lib */

#endif /* MC2LIB_SETS_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
