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

#ifndef MC2LIB_ELEMENTSETTHY_HPP_
#define MC2LIB_ELEMENTSETTHY_HPP_

#include <cassert>
#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mc2lib {

/**
 * @namespace mc2lib::elementsetthy
 * @brief Sets and maps exposed in a restricted set of set theory.
 */
namespace elementsetthy {

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
class ElementSet {
  public:
    typedef typename Ts::Element Element;
    typedef typename Ts::Set Set;

    ElementSet()
    {}

    ElementSet(const Set& s)
        : set_(s)
    {}

    const Set& get() const
    { return set_; }

    bool operator==(const ElementSet& rhs) const
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
        auto result = set_.erase(e);
        assert(!assert_exists || result != 0);
    }

    /*
     * Set union.
     */
    ElementSet& operator+=(const Element& rhs)
    {
        insert(rhs);
        return *this;
    }

    ElementSet operator+(const Element& rhs) const
    {
        ElementSet es = *this;
        return es += rhs;
    }

    ElementSet& operator+=(const ElementSet& rhs)
    {
        if (this == &rhs) return *this;
        set_.insert(rhs.set_.begin(), rhs.set_.end());
        return *this;
    }

    ElementSet operator+(const ElementSet& rhs) const
    {
        ElementSet es = *this;
        return es += rhs;
    }

    /*
     * Set difference.
     */
    ElementSet& operator-=(const Element& rhs)
    {
        erase(rhs);
        return *this;
    }

    ElementSet operator-(const Element& rhs) const
    {
        ElementSet es = *this;
        return es -= rhs;
    }

    ElementSet& operator-=(const ElementSet& rhs)
    {
        if (this == &rhs) {
            clear();
        } else {
            for (const auto& e : rhs.get()) {
                erase(e);
            }
        }

        return *this;
    }

    ElementSet operator-(const ElementSet& rhs) const
    {
        ElementSet es = *this;
        return es -= rhs;
    }

    /*
     * Set intersection
     */
    ElementSet operator&(const ElementSet& rhs) const
    {
        ElementSet es;
        for (const auto& e : rhs.get()) {
            if (contains(e)) {
                es += e;
            }
        }
        return es;
    }

    ElementSet& operator&=(const ElementSet& rhs)
    {
        *this = rhs & *this;
        return *this;
    }

    void clear()
    { set_.clear(); }

    bool contains(const Element& e) const
    {
        return set_.find(e) != set_.end();
    }

    std::size_t size() const
    { return set_.size(); }

    bool empty() const
    { return set_.empty(); }

    bool subseteq(const ElementSet& es) const
    {
        for (const auto& e : set_) {
            if (!es.contains(e)) {
                return false;
            }
        }
        return true;
    }

    bool subset(const ElementSet& es) const
    {
        return subseteq(es) && size() < es.size();
    }

    template <class FilterFunc>
    ElementSet filter(FilterFunc filterFunc) const
    {
        ElementSet es;
        for (const auto& e : set_) {
            if (filterFunc(e)) {
                es += e;
            }
        }
        return es;
    }

  protected:
     Set set_;
};

template <class Ts>
class ElementRel {
  public:
    typedef typename Ts::Element Element;

    typedef typename Ts::template Map<ElementSet<Ts>> Relation;

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

    ElementRel()
        : props_(None)
    {}

    ElementRel(const Relation& r)
        : props_(None), rel_(r)
    {}

    /*
     * Avoid accessing rel_ directly. Uses of raw should be well justified.
     */
    const Relation& raw() const
    { return rel_; }

    Properties props() const
    { return props_; }

    ElementRel& set_props(Properties props)
    {
        props_ = props;
        return *this;
    }

    ElementRel& add_props(Properties props)
    {
        props_ |= props;
        return *this;
    }

    ElementRel& unset_props(Properties props)
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

    void insert(const Element& e1, const ElementSet<Ts>& e2s)
    {
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

    void erase(const Element& e1, const ElementSet<Ts>& e2s)
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
    void iterate(Func func) const
    {
        const auto dom = domain();
        for (const auto& e1 : dom.get()) {
            const auto reach = reachable(e1);
            for (const auto& e2 : reach.get()) {
                func(e1, e2);
            }
        }
    }

    /*
     * Provide eval() for evaluated view of the relation (with properties
     * evaluated).
     */
    ElementRel eval() const
    {
        if (!props()) {
            return *this;
        }

        ElementRel result;

        iterate([&result](const Element& e1, const Element& e2) {
            result.insert(e1, e2);
        });

        return result;
    }

    template <class FilterFunc>
    ElementRel filter(FilterFunc filterFunc) const
    {
        ElementRel result;

        iterate([&filterFunc, &result](const Element& e1, const Element& e2) {
            if (filterFunc(e1, e2)) {
                result.insert(e1, e2);
            }
        });

        return result;
    }

    ElementRel inverse() const
    {
        ElementRel result;

        iterate([&result](const Element& e1, const Element& e2) {
            result.insert(e2, e1);
        });

        return result;
    }

    /*
     * Relation union.
     */
    ElementRel& operator+=(const ElementRel& rhs)
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

    ElementRel operator+(const ElementRel& rhs) const
    {
        ElementRel er = *this;
        return er += rhs;
    }

    ElementRel& operator+=(const Tuple& rhs)
    {
        insert(rhs.first, rhs.second);
        return *this;
    }

    ElementRel operator+(const Tuple& rhs) const
    {
        ElementRel er = *this;
        return er += rhs;
    }

    /*
     * Relation difference.
     */
    ElementRel& operator-=(const ElementRel& rhs)
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

    ElementRel operator-(const ElementRel& rhs) const
    {
        ElementRel er = *this;
        return er -= rhs;
    }

    /*
     * Relation intersection
     */
    ElementRel operator&(const ElementRel& rhs) const
    {
        ElementRel es;

        const auto this_domain = domain();
        for (const auto& e : this_domain.get()) {
            const auto this_reachable = reachable(e);
            const auto rhs_reachable = rhs.reachable(e);
            ElementSet<Ts> intersect = this_reachable & rhs_reachable;
            if (!intersect.empty()) {
                es.rel_[e] = intersect;
            }
        }

        return es;
    }

    ElementRel& operator&=(const ElementRel& rhs)
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

    bool operator==(const ElementRel& rhs) const
    {
        const auto& l = props() ? eval() : *this;
        const auto& r = rhs.props() ? rhs.eval() : rhs;
        return l.rel_ == r.rel_;
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

        ElementSet<Ts> visited;

        if (path != nullptr) {
            FlagSet visiting;

            bool result = R_impl(e1, &e2, &visited, &visiting);
            if (result) {
                get_path(path, &e1, &e2, &visiting);
            }

            return result;
        }

        return R_impl(e1, &e2, &visited);
    }

    /*
     * Returns all rechable elements from a start element without the start
     * itself, but includes start if start can reach itself (e.g. through
     * cycle).
     */
    ElementSet<Ts> reachable(const Element& e) const
    {
        ElementSet<Ts> visited;

        if (all_props(ReflexiveClosure) && in_on(e)) {
            visited += e;
        }

        if (!all_props(TransitiveClosure)) {
            const auto tuples = rel_.find(e);
            if (tuples != rel_.end()) {
                visited += tuples->second;
            }

            return visited;
        }

        if (!R_impl(e, &e, &visited, nullptr,
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
    bool total_on(const ElementSet<Ts>& on) const
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
    bool connex_on(const ElementSet<Ts>& on) const
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

    bool weak_partial_order(const ElementSet<Ts>& on) const
    {
        // (domain ∪ range) in on
        for (const auto& tuples : rel_) {
            if (!on.contains(tuples.first) && !tuples.second.subseteq(on)) {
                return false;
            }
        }

        return transitive() && !irreflexive();
    }

    bool weak_total_order(const ElementSet<Ts>& on) const
    {
        return weak_partial_order(on) && total_on(on);
    }

    bool strict_partial_order(const ElementSet<Ts>& on) const
    {
        // (domain ∪ range) in on
        for (const auto& tuples : rel_) {
            if (!on.contains(tuples.first) && !tuples.second.subseteq(on)) {
                return false;
            }
        }

        return transitive() && irreflexive();
    }

    bool strict_total_order(const ElementSet<Ts>& on) const
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

    ElementSet<Ts> on() const
    {
        ElementSet<Ts> es;
        for (const auto& tuples : rel_) {
            es += tuples.first;
            es += tuples.second;
        }
        return es;
    }

    ElementSet<Ts> domain() const
    {
        if (all_props(ReflexiveClosure)) {
            // By the fact that the reflexive closure is
            // R ∪ {(x,x). x ∈ S}, where S is the set the relation is on, and
            // thus S contains both the range and domain, constructing the
            // above will yield domain = range = S under the reflexive closure.
            return on();
        }

        ElementSet<Ts> es;
        for (const auto& tuples : rel_) {
            es += tuples.first;
        }

        return es;
    }

    ElementSet<Ts> range() const
    {
        if (all_props(ReflexiveClosure)) {
            // See above.
            return on();
        }

        ElementSet<Ts> es;
        for (const auto& tuples : rel_) {
            es += reachable(tuples.first);
        }
        return es;
    }

  protected:
    typedef typename Ts::template Map<bool> FlagSet;

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
            const ElementSet<Ts>& next = rel_.find(*start)->second;
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

        const ElementSet<Ts>& next = rel_.find(out->back())->second;
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

        ElementSet<Ts> visited;
        FlagSet visiting;

        for (const auto& tuples : rel_) {
            if (R_impl(tuples.first, nullptr, &visited, &visiting,
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

    /*** SEARCH IMPLEMENTATION OF DIRECTED GRAPHS **
     *
     * Returns true if the two elements are ordered. If visit_all is set, all
     * reachable nodes are visited; these are stored in visited (including
     * the start node).
     *
     * If e2 == nullptr, returns true if a definite cycle has been detected;
     * returning true here does not necessarily imply that e1 can reach itself.
     *
     * If e2 == nullptr and !visit_all, stops as soon as a cycle has been
     * detected.
     */

    /*
     * Common setup.
     */
    bool R_impl(const Element& e1, const Element *e2,
                ElementSet<Ts>* visited,
                FlagSet* visiting = nullptr,
                Properties local_props = None,
                SearchMode mode = SearchMode::Related) const
    {
        // We always require visited to be set.
        assert(visited != nullptr);

        // Merge call-specific props with global props.
        local_props |= props_;

        if (e2 == nullptr) {
            // For the purpose of cycle detection, we are looking for e1->*e1.
            // In addition, if the transitive property is set, we require that
            // visiting is provided, as we cannot make assumptions on if visited
            // is reset before every call -- and for performance reasons, this
            // usage is discouraged.

            assert(mode == SearchMode::FindCycle);

            if (all_bitmask(local_props, TransitiveClosure)) {
                assert(visiting != nullptr);
            }

            e2 = &e1;
        } else {
            assert(mode != SearchMode::FindCycle);
        }

        return R_dfs_rec(e1, *e2, visited, visiting, local_props, mode);
    }

    /*
     * Recursive DFS
     */
    bool R_dfs_rec(const Element& e1, const Element& e2,
                   ElementSet<Ts>* visited,
                   FlagSet* visiting,
                   Properties local_props,
                   SearchMode mode) const
    {
        const auto tuples = rel_.find(e1);

        if (tuples == rel_.end()) {
            return false;
        }

        if (visiting != nullptr) {
            (*visiting)[e1] = true;
        }

        bool result = false;
        (*visited) += e1;

        for (const auto& e : tuples->second.get()) {
            if (e == e2) {
                if (mode == SearchMode::RelatedVisitAll) {
                    result = true;
                } else {
                    return true;
                }
            }

            if (all_bitmask(local_props, TransitiveClosure)) {
                if (!visited->contains(e)) {
                    if (R_dfs_rec(e, e2, visited, visiting,
                                  local_props, mode)) {
                        if (mode == SearchMode::RelatedVisitAll) {
                            result = true;
                        } else {
                            return true;
                        }
                    }

                    // There might not be an edge e -> e2, but we must update
                    // the visited set regardless -- this is only relevant, as
                    // the caller should only expect the complete set of
                    // visited nodes is visit_all == true.
                    (*visited) += e;
                } else if(mode == SearchMode::FindCycle) {
                    const auto se = visiting->find(e);
                    if (se != visiting->end() && se->second) {
                        // Found a backedge --> cycle!
                        return true;
                    }
                }
            }
        }

        if (visiting != nullptr) {
            (*visiting)[e1] = false;
        }

        return result;
    }

  protected:
    Properties props_;
    Relation rel_;
};

template <class Ts>
inline ElementRel<Ts> operator*(const ElementSet<Ts>& lhs,
                                const ElementSet<Ts>& rhs)
{
    ElementRel<Ts> er;
    for (const auto& e1 : lhs.get()) {
        for (const auto& e2 : rhs.get()) {
            er.insert(e1, e2);
        }
    }
    return er;
}

template <class Ts>
class ElementRelOp {
  public:
    ElementRelOp()
    {}

    ElementRelOp(const std::vector<ElementRel<Ts>>& rels) :
        rels_(rels) {}

    virtual ~ElementRelOp()
    {}

    virtual ElementRel<Ts> eval_inplace() = 0;

    virtual ElementRel<Ts> eval() const = 0;

  protected:
    void add(const ElementRel<Ts>& er)
    {
        rels_.push_back(er);
    }

    void add(const std::vector<ElementRel<Ts>>& rels)
    {
        rels_.reserve(rels_.size() + rels.size());
        rels_.insert(rels_.end(), rels.begin(), rels.end());
    }

  protected:
    std::vector<ElementRel<Ts>> rels_;
};

template <class Ts>
class ElementRelSeq : public ElementRelOp<Ts> {
  public:
    typedef typename Ts::Element Element;

    ElementRelSeq()
    {}

    ElementRelSeq(const std::vector<ElementRel<Ts>>& v)
        : ElementRelOp<Ts>(v)
    {}

    ElementRelSeq& operator+=(const ElementRelSeq& rhs)
    {
        add(rhs.rels_);
        return *this;
    }

    ElementRelSeq operator+(const ElementRelSeq& rhs) const
    {
        ElementRelSeq ers = *this;
        return ers += rhs;
    }

    ElementRelSeq& operator+=(const ElementRel<Ts>& rhs)
    {
        this->add(rhs);
        return *this;
    }

    ElementRelSeq operator+(const ElementRel<Ts>& rhs) const
    {
        ElementRelSeq ers = *this;
        return ers += rhs;
    }

    ElementRel<Ts> eval_inplace()
    {
        if (this->rels_.empty()) {
            return ElementRel<Ts>();
        }

        while (this->rels_.size() > 1) {
            std::size_t from_idx = this->rels_.size() - 2;
            const auto first = this->rels_[from_idx];
            const auto last = this->rels_.back();

            ElementRel<Ts> er;

            first.iterate([&er, &last](const Element& e1, const Element& e2) {
                if (last.in_domain(e2)) {
                    er.insert(e1, last.reachable(e2));
                }
            });

            this->rels_.erase(this->rels_.end() - 2, this->rels_.end());
            this->rels_.push_back(std::move(er));
        }

        return this->rels_.back();
    }

    ElementRel<Ts> eval() const
    {
        ElementRel<Ts> er;

        if (this->rels_.empty()) {
            return ElementRel<Ts>();
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
           typename ElementRel<Ts>::Path *path = nullptr, std::size_t seq = 0) const
    {
        if (this->rels_.empty()) {
            return false;
        }

        assert(seq < this->rels_.size());

        if (seq + 1 < this->rels_.size()) {
            const auto& rel = this->rels_[seq];
            std::size_t path_size = 0;

            const ElementSet<Ts> reach = rel.reachable(e1);
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

    bool irreflexive(typename ElementRel<Ts>::Path *cyclic = nullptr) const
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

    typedef std::unordered_set<Element, typename Element::Hash> Set;

#if defined(__GNUC__) && (__GNUC__ == 4 && (__GNUC_MINOR__ == 6))
    template <class T>
    class Map : public std::unordered_map<Element, T, typename Element::Hash>
    {};
#else
    // Only works for GCC > 4.6
    template <class T>
    using Map = std::unordered_map<Element, T, typename Element::Hash>;
#endif
};

} /* namespace elementsetthy */
} /* namespace mc2lib */

#endif /* MC2LIB_ELEMENTSETTHY_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
