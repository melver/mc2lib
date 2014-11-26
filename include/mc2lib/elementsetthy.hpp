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

#ifndef MC2LIB_ELEMENTSETTHY_HPP_
#define MC2LIB_ELEMENTSETTHY_HPP_

#include <cassert>
#include <unordered_set>
#include <vector>

#ifndef MC2LIB_ElementRel_UNORDERED
#   define MC2LIB_ElementRel_UNORDERED 1
#endif

#ifndef MC2LIB_ElementRel_R_impl
#   define MC2LIB_ElementRel_R_impl R_default
#endif

#if MC2LIB_ElementRel_UNORDERED
#   include <unordered_map>
#else
#   include <map>
#endif

namespace mc2lib {
namespace elementsetthy {

inline bool all_bitmask(unsigned mask, unsigned all)
{
    assert(all != 0);
    return (mask & all) == all;
}

inline bool any_bitmask(unsigned mask, unsigned any)
{
    assert(any != 0);
    return (mask & any) != 0;
}

template <class Element>
class ElementSet {
  public:
    typedef std::unordered_set<Element, typename Element::Hash> Set;

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

    /*
     * Set union.
     */
    ElementSet& operator+=(const Element& rhs)
    {
        set_.insert(rhs);
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
        set_.erase(rhs);
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
                set_.erase(e);
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

    size_t size() const
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

template <class Element>
class ElementRel {
  public:
#if MC2LIB_ElementRel_UNORDERED
    typedef std::unordered_map<Element, ElementSet<Element>,
                               typename Element::Hash> Relation;
#else
    typedef std::map<Element, ElementSet<Element>> Relation;
#endif

    typedef std::pair<Element, Element> Tuple;

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

    virtual ~ElementRel()
    {}

    /*
     * Avoid accessing rel_ directly. Uses of raw should be well justified.
     */
    const Relation& raw() const
    { return rel_; }

    /*
     * Provide eval() for actual view of the relation (with properties
     * evaluated).
     */
    std::vector<Tuple> eval() const
    {
        std::vector<Tuple> vec;
        const auto dom = domain();
        for (const auto& e1 : dom.get()) {
            const auto reach = reachable(e1);
            for (const auto& e2 : reach.get()) {
                vec.emplace_back(e1, e2);
            }
        }
        return vec;
    }

    Properties props() const
    { return props_; }

    void set_props(Properties props)
    {
        props_ = props;
    }

    void add_props(Properties props)
    {
        props_ |= props;
    }

    void unset_props(Properties props)
    {
        props_ &= ~props;
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
        rel_[rhs.first] += rhs.second;
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
        const auto diff_tuples = [&](const Element& e1,
                                     const ElementSet<Element>& e2s) {
            if (contains(e1)) {
                rel_[e1] -= e2s;

                if (rel_[e1].empty()) {
                    rel_.erase(e1);
                }
            }
        };

        if (rhs.props()) {
            const auto rhs_domain = rhs.domain();
            for (const auto& e : rhs_domain.get()) {
                diff_tuples(e, rhs.reachable(e));
            }
        } else {
            for (const auto& tuples : rhs.raw()) {
                diff_tuples(tuples.first, tuples.second);
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
            ElementSet<Element> intersect = this_reachable & rhs_reachable;
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

    bool contains(const Element& e) const
    {
        return rel_.find(e) != rel_.end();
    }

    /*
     * Return true if two elements are ordered.
     */
    bool R(const Element& e1, const Element& e2) const
    {
        if (e1 == e2 && all_props(ReflexiveClosure)) {
            if (in_on(e1)) {
                return true;
            }
        }

        ElementSet<Element> visited;
        return MC2LIB_ElementRel_R_impl(e1, &e2, visited);
    }

    /*
     * Returns all rechable elements from a start element without the start
     * itself, but includes start if start can reach itself (e.g. through
     * cycle).
     */
    ElementSet<Element> reachable(const Element& e) const
    {
        ElementSet<Element> visited;

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

        if (!MC2LIB_ElementRel_R_impl(e, &e, visited, None, true)) {
            if (!all_props(ReflexiveClosure)) {
                visited -= e;
            }
        }

        return visited;
    }

    /*
     * If 'which' is set, the Element which is part of the cycle or from which
     * the cycle can be reached will be copied.
     */
    bool irreflexive(Element *which = nullptr) const
    {
        return irreflexive(None, which);
    }

    bool acyclic(Element *which = nullptr) const
    {
        return irreflexive(TransitiveClosure, which);
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
     * (domain ∪ range) ⊆ on
     */
    bool partial_order(const ElementSet<Element>& on) const
    {
        for (const auto& tuples : rel_) {
            if (!on.contains(tuples.first) && !tuples.second.subseteq(on)) {
                return false;
            }
        }
        return transitive() && irreflexive();
    }

    /*
     * ∀(x,y) ∈ on×on, x→y ∨ y→x
     */
    bool total_on(const ElementSet<Element>& on) const
    {
        ElementSet<Element> rem = on;

        for (const auto& tuples : rel_) {
            if (on.contains(tuples.first) && tuples.second.subseteq(on)) {
                rem -= tuples.first;
                rem -= tuples.second;

                if (rem.empty()) {
                    break;
                }
            }
        }

        return rem.empty();
    }

    bool strict_total_order(const ElementSet<Element>& on) const
    {
        return partial_order(on) && total_on(on);
    }

    void insert(const Element& e1, const Element& e2)
    {
        rel_[e1] += e2;
    }

    size_t size() const
    {
        size_t total = 0;

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

    template <class FilterFunc>
    ElementRel filter(FilterFunc filterFunc) const
    {
        ElementRel er;
        for (const auto& tuples : rel_) {
            for (const auto& e : tuples.second.get()) {
                const auto tuple = std::make_pair(tuples.first, e);
                if (filterFunc(tuple)) {
                    er += tuple;
                }
            }
        }
        return er;
    }

    bool in_on(const Element& e) const
    {
        if (contains(e)) {
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

    ElementSet<Element> on() const
    {
        ElementSet<Element> es;
        for (const auto& tuples : rel_) {
            es += tuples.first;
            es += tuples.second;
        }
        return es;
    }

    ElementSet<Element> domain() const
    {
        if (all_props(ReflexiveClosure)) {
            // By the fact that the reflexive closure is
            // R ∪ {(x,x). x ∈ S}, where S is the set the relation is on, and
            // thus S contains both the range and domain, constructing the
            // above will yield domain = range = S under the reflexive closure.
            return on();
        }

        ElementSet<Element> es;
        for (const auto& tuples : rel_) {
            es += tuples.first;
        }

        return es;
    }

    ElementSet<Element> range() const
    {
        if (all_props(ReflexiveClosure)) {
            // See above.
            return on();
        }

        ElementSet<Element> es;
        for (const auto& tuples : rel_) {
            es += reachable(tuples.first);
        }
        return es;
    }

  protected:

    bool irreflexive(Properties local_props, Element *which) const
    {
        local_props |= props_;

        if (all_bitmask(local_props, ReflexiveClosure)) {
            return false;
        }

        for (const auto& tuples : rel_) {
            ElementSet<Element> visited;
            if (MC2LIB_ElementRel_R_impl(tuples.first, nullptr,
                                         visited, local_props)) {
                if (which != nullptr) {
                    *which = tuples.first;
                }
                return false;
            }
        }

        return true;
    }

    /*** SEARCH IMPLEMENTATIONS OF DIRECTED GRAPHS **
     *
     * Returns true if the two elements are ordered. If visit_all is set, all
     * reachable nodes are visisted; these are stored in visisted (including
     * the start node).
     *
     * If e2 == nullptr, returns true if a definite cycle has been detected;
     * returning true here does not necessarily imply that e1 can reach itself.
     *
     * If e2 == nullptr and !visit_all, stops as soon as a cycle has been
     * detected.
     */

    /*
     * Default implementation wrapper.
     */
    virtual bool R_default(const Element& e1, const Element *e2,
                           ElementSet<Element>& visited,
                           Properties local_props = None,
                           bool visit_all = false) const
    {
        local_props |= props_;
        return R_dfs_rec(e1, e2, visited, local_props, visit_all);
    }

    /*
     * Recursive DFS
     */
    bool R_dfs_rec(const Element& e1, const Element *e2,
                   ElementSet<Element>& visited,
                   Properties local_props,
                   bool visit_all) const
    {
        const auto tuples = rel_.find(e1);

        if (tuples == rel_.end()) {
            return false;
        }

        // Currently a cycle does imply that e1 can reach itself, and this is
        // applied in the most outer call only.
        if (e2 == nullptr) {
            e2 = &e1;
        }

        bool result = false;
        visited += e1;

        for (const auto& e : tuples->second.get()) {
            if (e2 != nullptr && e == *e2) {
                if (visit_all) {
                    result = true;
                } else {
                    return true;
                }
            }

            if (all_bitmask(local_props, TransitiveClosure)) {
                if (!visited.contains(e)) {
                    if (R_dfs_rec(e, e2, visited, local_props, visit_all)) {
                        if (visit_all) {
                            result = true;
                        } else {
                            return true;
                        }
                    }
                    visited += e;
                }
            }
        }

        return result;
    }

  protected:
    Properties props_;
    Relation rel_;
};

/*
 * Cross product of ElementSets
 */
template <class Element>
inline ElementRel<Element> operator*(const ElementSet<Element>& lhs,
                                     const ElementSet<Element>& rhs)
{
    ElementRel<Element> er;
    for (const auto& e1 : lhs.get()) {
        for (const auto& e2 : rhs.get()) {
            er.insert(e1, e2);
        }
    }
    return er;
}

template <class Element>
class ElementRelOp {
  public:
    ElementRelOp()
    {}

    ElementRelOp(const std::vector<ElementRel<Element>>& rels) :
        rels_(rels) {}

    virtual ~ElementRelOp()
    {}

    size_t size() const
    {
        size_t total = 0;
        for (const auto& rel : rels_) {
            total += rel.size();
        }
        return total;
    }

    virtual ElementRel<Element> eval() const = 0;

  protected:
    void add(const ElementRel<Element>& er)
    {
        rels_.push_back(er);
    }

    void add(const std::vector<ElementRel<Element>>& rels)
    {
        rels_.reserve(rels_.size() + rels.size());
        rels_.insert(rels_.end(), rels.begin(), rels.end());
    }

  protected:
    std::vector<ElementRel<Element>> rels_;
};

template <class Element>
class ElementRelSeq : public ElementRelOp<Element> {
  public:
    ElementRelSeq()
    {}

    ElementRelSeq(const std::vector<ElementRel<Element>>& v)
        : ElementRelOp<Element>(v)
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

    ElementRelSeq& operator+=(const ElementRel<Element>& rhs)
    {
        this->add(rhs);
        return *this;
    }

    ElementRelSeq operator+(const ElementRel<Element>& rhs) const
    {
        ElementRelSeq ers = *this;
        return ers += rhs;
    }

    ElementRel<Element> eval() const
    {
        ElementRel<Element> er;
        if (!this->size()) return er;
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

    bool R(const Element& e1, const Element& e2, size_t seq = 0) const
    {
        if (!this->size()) return false;
        assert(seq < this->rels_.size());

        if (seq + 1 < this->rels_.size()) {
            const auto& rel = this->rels_[seq];

            const ElementSet<Element> reach = rel.reachable(e1);
            for (const auto& e : reach.get()) {
                if (R(e, e2, seq+1)) {
                    return true;
                }
            }

            return false;
        }

        return this->rels_[seq].R(e1, e2);
    }

    bool irreflexive() const
    {
        if (!this->size()) return true;

        const auto domain = this->rels_.front().domain();
        for (const auto& e : domain.get()) {
            if (R(e, e)) {
                return false;
            }
        }
        return true;
    }
};

} /* namespace elementsetthy */
} /* namespace mc2lib */

#endif /* ELEMENTSETTHY_HPP_ */

/* vim: set ts=4 sts=4 sw=4 et : */
