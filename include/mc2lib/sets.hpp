/*
 * Copyright (c) 2014-2016, Marco Elver
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

#include <cassert>
#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <utility>
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
template <class T>
inline bool AllBitmask(T mask, T all) {
  assert(all != 0);
  return (mask & all) == all;
}

/**
 * Checks that a bit mask has any of given bits set.
 *
 * @param mask Bit mask to check.
 * @param any Bit mask to check against.
 * @return True if at least any one of the bits in any is set in mask.
 */
template <class T>
inline bool AnyBitmask(T mask, T any) {
  assert(any != 0);
  return (mask & any) != 0;
}

/**
 * @brief Abstracts over container library's set implementation.
 *
 * Provides additional functions and operators not provided in the standard
 * library.
 */
template <class Ts>
class Set {
 public:
  typedef typename Ts::Element Element;
  typedef typename Ts::SetContainer Container;

  Set() {}

  explicit Set(Container s) : set_(std::move(s)) {}

  /**
   * Provide access to underlying container.
   *
   * @return Reference to underlying container.
   */
  const Container& get() const { return set_; }

  bool operator==(const Set& rhs) const { return set_ == rhs.set_; }

  bool operator!=(const Set& rhs) const { return set_ != rhs.set_; }

  /**
   * Insert element.
   *
   * @param e Element to be inserted.
   * @param assert_unique Assert that element does not exist in container.
   * @return Reference to inserted Element.
   */
  const Element& Insert(const Element& e, bool assert_unique = false) {
    auto result = set_.insert(e);
    assert(!assert_unique || result.second);
    return *result.first;
  }

  /**
   * Insert element.
   *
   * @param e Element to be inserted.
   * @param assert_unique Assert that element does not exist in container.
   * @return Reference to inserted Element.
   */
  const Element& Insert(Element&& e, bool assert_unique = false) {
    auto result = set_.emplace(std::move(e));
    assert(!assert_unique || result.second);
    return *result.first;
  }

  /**
   * Erase element.
   *
   * @param e Element to be erased.
   * @param assert_exists Assert that element exists.
   */
  bool Erase(const Element& e, bool assert_exists = false) {
    auto result = set_.erase(e);
    assert(!assert_exists || result != 0);
    return result != 0;
  }

  template <class FilterFunc>
  Set Filter(FilterFunc filterFunc) const {
    Set res;

    for (const auto& e : set_) {
      if (filterFunc(e)) {
        res.Insert(e);
      }
    }

    return res;
  }

  void Clear() { set_.clear(); }

  bool Contains(const Element& e) const { return set_.find(e) != set_.end(); }

  /**
   * Set union.
   */
  Set& operator|=(const Set& rhs) {
    if (this != &rhs) {
      set_.insert(rhs.set_.begin(), rhs.set_.end());
    }

    return *this;
  }

  Set& operator|=(Set&& rhs) {
    if (empty()) {
      set_ = std::move(rhs.set_);
    } else {
      set_.insert(rhs.set_.begin(), rhs.set_.end());
    }

    return *this;
  }

  /**
   * Set difference.
   */
  Set& operator-=(const Set& rhs) {
    if (this == &rhs) {
      Clear();
    } else {
      // Cannot use set_.erase with iterator, as rhs may contain elements
      // that do not exist in this set. In such a case, the erase
      // implementation of current GCC segfaults.
      for (const auto& e : rhs.get()) {
        Erase(e);
      }
    }

    return *this;
  }

  /**
   * Set intersection
   */
  Set& operator&=(const Set& rhs) {
    if (this != &rhs) {
      for (auto it = set_.begin(); it != set_.end();) {
        if (!rhs.Contains(*it)) {
          it = set_.erase(it);
          continue;
        }

        it++;
      }
    }

    return *this;
  }

  std::size_t size() const { return set_.size(); }

  bool empty() const { return set_.empty(); }

  bool SubsetEq(const Set& s) const {
    if (size() > s.size()) return false;

    for (const auto& e : set_) {
      if (!s.Contains(e)) {
        return false;
      }
    }

    return true;
  }

  bool Subset(const Set& s) const { return size() < s.size() && SubsetEq(s); }

 protected:
  Container set_;
};

template <class Ts>
inline Set<Ts> operator|(const Set<Ts>& lhs, const Set<Ts>& rhs) {
  Set<Ts> res = lhs;
  return res |= rhs;
}

template <class Ts>
inline Set<Ts> operator|(Set<Ts>&& lhs, const Set<Ts>& rhs) {
  lhs |= rhs;
  return std::move(lhs);
}

template <class Ts>
inline Set<Ts> operator|(const Set<Ts>& lhs, Set<Ts>&& rhs) {
  rhs |= lhs;
  return std::move(rhs);
}

template <class Ts>
inline Set<Ts> operator|(Set<Ts>&& lhs, Set<Ts>&& rhs) {
  lhs |= rhs;
  return std::move(lhs);
}

template <class Ts>
inline Set<Ts> operator-(const Set<Ts>& lhs, const Set<Ts>& rhs) {
  Set<Ts> res = lhs;
  return res -= rhs;
}

template <class Ts>
inline Set<Ts> operator-(Set<Ts>&& lhs, const Set<Ts>& rhs) {
  lhs -= rhs;
  return std::move(lhs);
}

template <class Ts>
inline Set<Ts> operator-(const Set<Ts>& lhs, Set<Ts>&& rhs) {
  Set<Ts> res = lhs;
  return res -= rhs;
}

template <class Ts>
inline Set<Ts> operator-(Set<Ts>&& lhs, Set<Ts>&& rhs) {
  lhs -= rhs;
  return std::move(lhs);
}

template <class Ts>
inline Set<Ts> operator&(const Set<Ts>& lhs, const Set<Ts>& rhs) {
  Set<Ts> res;

  for (const auto& e : rhs.get()) {
    if (lhs.Contains(e)) {
      res.Insert(e);
    }
  }

  return res;
}

template <class Ts>
inline Set<Ts> operator&(Set<Ts>&& lhs, const Set<Ts>& rhs) {
  lhs &= rhs;
  return std::move(lhs);
}

template <class Ts>
inline Set<Ts> operator&(const Set<Ts>& lhs, Set<Ts>&& rhs) {
  rhs &= lhs;
  return std::move(rhs);
}

template <class Ts>
inline Set<Ts> operator&(Set<Ts>&& lhs, Set<Ts>&& rhs) {
  lhs &= rhs;
  return std::move(lhs);
}

template <class Ts>
class Relation {
 public:
  typedef typename Ts::Element Element;

  typedef typename Ts::template MapContainer<Set<Ts>> Container;

  typedef std::pair<Element, Element> Tuple;

  typedef std::vector<Element> Path;

  /**
   * Lazy operators as properties.
   *
   * All in-place operators (e.g. |=) evaluate the current relation in place
   * and clear properties.
   */
  typedef unsigned Properties;

  // Properties {{{

  static constexpr Properties kNone = 0x0;
  static constexpr Properties kTransitiveClosure = 0x1;
  static constexpr Properties kReflexiveClosure = 0x2;
  static constexpr Properties kReflexiveTransitiveClosure =
      kTransitiveClosure | kReflexiveClosure;

  // }}}

  Relation() : props_(kNone) {}

  explicit Relation(Container r) : props_(kNone), rel_(std::move(r)) {}

  /**
   * Avoid accessing underlying container directly if possible! Uses of get()
   * should be justified.
   */
  const Container& get() const { return rel_; }

  Properties props() const { return props_; }

  Relation& set_props(Properties props) {
    props_ = props;
    return *this;
  }

  Relation& add_props(Properties props) {
    props_ |= props;
    return *this;
  }

  Relation& unset_props(Properties props) {
    props_ &= ~props;
    return *this;
  }

  bool all_props(Properties all) const { return AllBitmask(props_, all); }

  bool any_props(Properties any) const { return AnyBitmask(props_, any); }

  Properties clear_props() {
    const auto props = props_;
    props_ = kNone;
    return props;
  }

  void Insert(const Element& e1, const Element& e2,
              bool assert_unique = false) {
    rel_[e1].Insert(e2, assert_unique);
  }

  void Insert(const Element& e1, Element&& e2, bool assert_unique = false) {
    rel_[e1].Insert(std::move(e2), assert_unique);
  }

  void Insert(const Element& e1, const Set<Ts>& e2s) {
    if (e2s.empty()) return;
    rel_[e1] |= e2s;
  }

  void Insert(const Element& e1, Set<Ts>&& e2s) {
    if (e2s.empty()) return;
    rel_[e1] |= std::move(e2s);
  }

  bool Erase(const Element& e1, const Element& e2, bool assert_exists = false) {
    // May not work as expect if kReflexiveClosure is set.
    if (Contains__(e1)) {
      bool result = rel_[e1].Erase(e2, assert_exists);

      if (rel_[e1].empty()) {
        rel_.erase(e1);
      }

      return result;
    }

    return false;
  }

  void Erase(const Element& e1, const Set<Ts>& e2s) {
    if (Contains__(e1)) {
      rel_[e1] -= e2s;

      if (rel_[e1].empty()) {
        rel_.erase(e1);
      }
    }
  }

  /**
   * Total size of the relation (i.e. number of tuples or edges); uses
   * properties.
   *
   * @return Total number of tuples (or edges).
   */
  std::size_t size() const {
    std::size_t total = 0;

    if (props()) {
      const auto dom = Domain();
      for (const auto& e : dom.get()) {
        total += Reachable(e).size();
      }
    } else {
      for (const auto& tuples : rel_) {
        total += tuples.second.size();
      }
    }

    return total;
  }

  /**
   * Iterates over each tuple in the relation (uses properties).
   *
   * @param func A function taking two parameters of type Element.
   */
  template <class Func>
  Func for_each(Func func) const {
    const auto dom = Domain();

    for (const auto& e1 : dom.get()) {
      const auto reach = Reachable(e1);
      for (const auto& e2 : reach.get()) {
        func(e1, e2);
      }
    }

    return std::move(func);
  }

  /**
   * Evaluated view of the relation, with properties evaluated.
   *
   * @return Evaluated Relation.
   */
  Relation Eval() const {
    if (props() == kNone) {
      return *this;
    }

    Relation result;

    for_each([&result](const Element& e1, const Element& e2) {
      result.Insert(e1, e2);
    });

    return result;
  }

  /**
   * Evaluate all properties in-place.
   *
   * @return Reference to this object.
   */
  Relation& EvalInplace() {
    if (props() == kNone) {
      return *this;
    }

    Relation result;

    for_each([&result](const Element& e1, const Element& e2) {
      result.Insert(e1, e2);
    });

    clear_props();
    rel_ = std::move(result.rel_);
    return *this;
  }

  template <class FilterFunc>
  Relation Filter(FilterFunc filterFunc) const {
    Relation result;

    for_each([&filterFunc, &result](const Element& e1, const Element& e2) {
      if (filterFunc(e1, e2)) {
        result.Insert(e1, e2);
      }
    });

    return result;
  }

  /**
   * @return R^-1 = {(y,x) | (x,y) ∈ R}
   */
  Relation Inverse() const {
    Relation result;

    for_each([&result](const Element& e1, const Element& e2) {
      result.Insert(e2, e1);
    });

    return result;
  }

  /**
   * Relation union.
   */
  Relation& operator|=(const Relation& rhs) {
    EvalInplace();

    if (rhs.props()) {
      const auto rhs_domain = rhs.Domain();
      for (const auto& e : rhs_domain.get()) {
        rel_[e] |= rhs.Reachable(e);
      }
    } else {
      for (const auto& tuples : rhs.get()) {
        rel_[tuples.first] |= tuples.second;
      }
    }

    return *this;
  }

  /**
   * Relation difference.
   */
  Relation& operator-=(const Relation& rhs) {
    EvalInplace();

    if (rhs.props()) {
      const auto rhs_domain = rhs.Domain();
      for (const auto& e : rhs_domain.get()) {
        Erase(e, rhs.Reachable(e));
      }
    } else {
      for (const auto& tuples : rhs.get()) {
        Erase(tuples.first, tuples.second);
      }
    }

    return *this;
  }

  /**
   * Relation intersection
   */
  Relation& operator&=(const Relation& rhs) {
    EvalInplace();

    for (auto it = rel_.begin(); it != rel_.end();) {
      it->second &= rhs.Reachable(it->first);

      if (it->second.empty()) {
        it = rel_.erase(it);
        continue;
      }

      it++;
    }

    return *this;
  }

  void Clear() { rel_.clear(); }

  bool empty() const {
    // Upon erasure, we ensure that an element is not related to an empty
    // set, i.e. in that case it is deleted.
    return rel_.empty();
  }

  bool operator==(const Relation& rhs) const {
    return (props() ? Eval() : *this).rel_ ==
           (rhs.props() ? rhs.Eval() : rhs).rel_;
  }

  bool operator!=(const Relation& rhs) const { return !((*this) == rhs); }

  /**
   * Check if (e1, e2) is in the relation. This effectively does a search if
   * there is an edge from e1 to e2.
   *
   * @param e1 first element.
   * @param e2 second element
   * @param path Optional; return path from e1 to e2.
   * @return true if (e1, e2) is in the relation.
   */
  bool R(const Element& e1, const Element& e2, Path* path = nullptr) const {
    if (e1 == e2 && all_props(kReflexiveClosure)) {
      if (InOn(e1)) {
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
        GetPath(path, &e1, &e2, &visiting);
      }

      return result;
    }

    return R_search(e1, &e2, &visited);
  }

  /**
   * Returns all rechable elements from a start element without the start
   * itself, but includes start if start can reach itself (e.g. through
   * cycle).
   *
   * @param e Start element.
   * @return Set of reachable elements.
   */
  Set<Ts> Reachable(const Element& e) const {
    Set<Ts> visited;

    if (all_props(kReflexiveClosure) && InOn(e)) {
      visited.Insert(e);
    }

    if (!all_props(kTransitiveClosure)) {
      const auto tuples = rel_.find(e);
      if (tuples != rel_.end()) {
        visited |= tuples->second;
      }

      return visited;
    }

    if (!R_search(e, &e, &visited, nullptr, kNone,
                  SearchMode::kRelatedVisitAll)) {
      if (!all_props(kReflexiveClosure)) {
        visited.Erase(e);
      }
    }

    return visited;
  }

  bool Irreflexive(Path* cyclic = nullptr) const {
    return Irreflexive(kNone, cyclic);
  }

  bool Acyclic(Path* cyclic = nullptr) const {
    return Irreflexive(kTransitiveClosure, cyclic);
  }

  /**
   * x→y ∧ y→z ⇒ x→z
   */
  bool Transitive() const {
    if (all_props(kTransitiveClosure)) {
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

  /**
   * ∀(x,y) ∈ on×on, x→y ∨ y→x
   */
  bool TotalOn(const Set<Ts>& on) const {
    for (const auto& e1 : on.get()) {
      for (const auto& e2 : on.get()) {
        if (!R(e1, e2) && !R(e2, e1)) {
          return false;
        }
      }
    }

    return true;
  }

  /**
   * ∀(x,y) ∈ on×on, x→y ∨ y→x ∨ x=y
   */
  bool ConnexOn(const Set<Ts>& on) const {
    for (const auto& e1 : on.get()) {
      for (const auto& e2 : on.get()) {
        if (e1 != e2 && !R(e1, e2) && !R(e2, e1)) {
          return false;
        }
      }
    }

    return true;
  }

  bool WeakPartialOrder(const Set<Ts>& on) const {
    // (domain ∪ range) in on
    for (const auto& tuples : rel_) {
      if (!on.Contains(tuples.first) && !tuples.second.SubsetEq(on)) {
        return false;
      }
    }

    return Transitive() && !Irreflexive();
  }

  bool WeakTotalOrder(const Set<Ts>& on) const {
    return WeakPartialOrder(on) && TotalOn(on);
  }

  bool StrictPartialOrder(const Set<Ts>& on) const {
    // (domain ∪ range) in on
    for (const auto& tuples : rel_) {
      if (!on.Contains(tuples.first) && !tuples.second.SubsetEq(on)) {
        return false;
      }
    }

    return Transitive() && Irreflexive();
  }

  bool StrictTotalOrder(const Set<Ts>& on) const {
    return StrictPartialOrder(on) && ConnexOn(on);
  }

  bool InOn(const Element& e) const {
    if (Contains__(e)) {
      return true;
    } else {
      for (const auto& tuples : rel_) {
        if (tuples.second.Contains(e)) {
          return true;
        }
      }
    }

    return false;
  }

  bool InDomain(const Element& e) const {
    if (all_props(kReflexiveClosure)) {
      return InOn(e);
    }

    return Contains__(e);
  }

  bool InRange(const Element& e) const {
    if (all_props(kReflexiveClosure)) {
      return InOn(e);
    }

    for (const auto& tuples : rel_) {
      if (Reachable(tuples.first).Contains(e)) {
        return true;
      }
    }

    return false;
  }

  Set<Ts> On() const {
    Set<Ts> res;
    for (const auto& tuples : rel_) {
      res.Insert(tuples.first);
      res |= tuples.second;
    }
    return res;
  }

  Set<Ts> Domain() const {
    if (all_props(kReflexiveClosure)) {
      // By the fact that the reflexive closure is
      // R ∪ {(x,x). x ∈ S}, where S is the set the relation is on, and
      // thus S contains both the range and domain, constructing the
      // above will yield domain = range = S under the reflexive closure.
      return On();
    }

    Set<Ts> res;
    for (const auto& tuples : rel_) {
      res.Insert(tuples.first);
    }

    return res;
  }

  Set<Ts> Range() const {
    if (all_props(kReflexiveClosure)) {
      // See above.
      return On();
    }

    Set<Ts> res;
    for (const auto& tuples : rel_) {
      res |= Reachable(tuples.first);
    }

    return res;
  }

  bool SubsetEq(const Relation& rhs) const {
    const Relation diff = (*this - rhs);
    return diff.empty();
  }

  bool Subset(const Relation& rhs) const {
    return size() < rhs.size() && SubsetEq(rhs);
  }

 protected:
  typedef typename Ts::template MapContainer<bool> FlagSet;

  /**
   * Search mode.
   */
  enum class SearchMode {
    /**
     * Check if two elements are related.
     */
    kRelated,

    /**
     * Check if two elements are related, but visit all elements.
     */
    kRelatedVisitAll,

    /**
     * Only find a cycle from a given element (node).
     */
    kFindCycle
  };

  /**
   * @return true if e in rel_.
   */
  bool Contains__(const Element& e) const { return rel_.find(e) != rel_.end(); }

  /**
   * Get path from start to end.
   */
  void GetPath(Path* out, const Element* start, const Element* end,
               FlagSet* visiting,
               SearchMode mode = SearchMode::kRelated) const {
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
    if (mode == SearchMode::kFindCycle) {
      assert(end == nullptr);

      for (const auto& e : *out) {
        if (next.Contains(e)) {
          // Final edge
          out->push_back(e);
          break;
        }
      }
    } else {
      assert(end != nullptr);

      // This function should only be called if search established there
      // is a path between start->end.
      assert(next.Contains(*end));

      out->push_back(*end);
    }
  }

  /**
   * Check that relation is irreflexive.
   *
   * @param local_props Call specific properties.
   * @param cyclic Optional parameter, in which the cycle is returned, if
   *               result is false.
   * @return true if irreflexive, false otherwise.
   */
  bool Irreflexive(Properties local_props, Path* cyclic) const {
    local_props |= props_;

    if (AllBitmask(local_props, kReflexiveClosure) && !empty()) {
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
      if (R_search(tuples.first, nullptr, &visited, &visiting, local_props,
                   SearchMode::kFindCycle)) {
        if (cyclic != nullptr) {
          GetPath(cyclic, &tuples.first, nullptr, &visiting,
                  SearchMode::kFindCycle);
        }

        return false;
      }
    }

    return true;
  }

  /**
   * Check if two elements are related, i.e. there exists an edge or path from
   * e1 to e2. This is implemented as a directed graph search.
   *
   * @param e1 First element.
   * @param e2 Second element.
   * @param visited Visited element (node) set.
   * @param visiting Currently visiting elements (nodes).
   * @param local_props Call specific properties.
   * @param mode Search mode.
   *
   * @return true if there exists an edge or path, false otherwise.
   */
  bool R_search(const Element& e1, const Element* e2, Set<Ts>* visited,
                FlagSet* visiting = nullptr, Properties local_props = kNone,
                SearchMode mode = SearchMode::kRelated) const {
    // We always require visited to be set.
    assert(visited != nullptr);

    // Merge call-specific props with global props.
    local_props |= props_;

    const bool is_tran_cl = AllBitmask(local_props, kTransitiveClosure);

    R_impl search(this, visited, visiting, is_tran_cl, mode);

    if (e2 == nullptr) {
      // For the purpose of cycle detection, we are looking for e1->*e1.
      // In addition, if the transitive property is set, we require that
      // visiting is provided, as we cannot make assumptions on if visited
      // is reset before every call -- and for performance reasons, this
      // usage is discouraged.

      assert(mode == SearchMode::kFindCycle);

      e2 = &e1;

      if (is_tran_cl) {
        assert(visiting != nullptr);
        return search.DfsRecFindCycle(e1);
      }
    } else {
      assert(mode != SearchMode::kFindCycle);
    }

    return search.DfsRec(e1, *e2);
  }

  /**
   * Helper class to check if two elements are related.
   */
  class R_impl {
   public:
    R_impl(const Relation* src, Set<Ts>* visited, FlagSet* visiting,
           bool is_tran_cl, SearchMode mode)
        : src_(src),
          visited_(visited),
          visiting_(visiting),
          is_tran_cl_(is_tran_cl),
          mode_(mode) {}

    /**
     * Recursive DFS implementation, searching if there exists a path from e1
     * to e2.
     */
    bool DfsRec(const Element& e1, const Element& e2) const {
      const auto tuples = src_->get().find(e1);

      if (tuples == src_->get().end()) {
        return false;
      }

      if (visiting_ != nullptr) {
        (*visiting_)[e1] = true;
      }

      bool result = false;
      visited_->Insert(e1);

      for (const auto& e : tuples->second.get()) {
        if (e == e2) {
          if (mode_ == SearchMode::kRelatedVisitAll) {
            result = true;
          } else {
            return true;
          }
        }

        if (is_tran_cl_) {
          if (!visited_->Contains(e)) {
            if (DfsRec(e, e2)) {
              if (mode_ == SearchMode::kRelatedVisitAll) {
                result = true;
              } else {
                return true;
              }
            }

            // There might not be an edge e -> e2, but we must update
            // the visited set regardless -- this is only relevant, as
            // the caller might expect the complete set of visited
            // nodes if mode == RelatedVisitAll.
            visited_->Insert(e);
          } else {
            // assert(mode_ != SearchMode::kFindCycle);
          }
        }
      }

      if (visiting_ != nullptr) {
        (*visiting_)[e1] = false;
      }

      return result;
    }

    /**
     * DFS optimized to just find a cycle; elides some branches that are not
     * needed compared to DfsRec.
     */
    bool DfsRecFindCycle(const Element& start) const {
      // assert(is_tran_cl_);
      // assert(mode_ == SearchMode::kFindCycle);
      // assert(visiting_ != nullptr);

      const auto tuples = src_->get().find(start);

      if (tuples == src_->get().end()) {
        return false;
      }

      (*visiting_)[start] = true;
      visited_->Insert(start);

      for (const auto& e : tuples->second.get()) {
        if (!visited_->Contains(e)) {
          if (DfsRecFindCycle(e)) {
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
inline Relation<Ts> operator*(const Set<Ts>& lhs, const Set<Ts>& rhs) {
  Relation<Ts> res;

  for (const auto& e1 : lhs.get()) {
    for (const auto& e2 : rhs.get()) {
      res.Insert(e1, e2);
    }
  }

  return res;
}

template <class Ts>
inline Relation<Ts> operator|(const Relation<Ts>& lhs,
                              const Relation<Ts>& rhs) {
  Relation<Ts> res = lhs;
  return res |= rhs;
}

template <class Ts>
inline Relation<Ts> operator|(Relation<Ts>&& lhs, const Relation<Ts>& rhs) {
  lhs |= rhs;
  return std::move(lhs);
}

template <class Ts>
inline Relation<Ts> operator|(const Relation<Ts>& lhs, Relation<Ts>&& rhs) {
  rhs |= lhs;
  return std::move(rhs);
}

template <class Ts>
inline Relation<Ts> operator|(Relation<Ts>&& lhs, Relation<Ts>&& rhs) {
  lhs |= rhs;
  return std::move(lhs);
}

template <class Ts>
inline Relation<Ts> operator-(const Relation<Ts>& lhs,
                              const Relation<Ts>& rhs) {
  Relation<Ts> res = lhs;
  return res -= rhs;
}

template <class Ts>
inline Relation<Ts> operator-(Relation<Ts>&& lhs, const Relation<Ts>& rhs) {
  lhs -= rhs;
  return std::move(lhs);
}

template <class Ts>
inline Relation<Ts> operator-(const Relation<Ts>& lhs, Relation<Ts>&& rhs) {
  Relation<Ts> res = lhs;
  return res -= rhs;
}

template <class Ts>
inline Relation<Ts> operator-(Relation<Ts>&& lhs, Relation<Ts>&& rhs) {
  lhs -= rhs;
  return std::move(lhs);
}

template <class Ts>
inline Relation<Ts> operator&(const Relation<Ts>& lhs,
                              const Relation<Ts>& rhs) {
  Relation<Ts> res;

  const auto lhs_domain = lhs.Domain();
  for (const auto& e : lhs_domain.get()) {
    Set<Ts> intersect = lhs.Reachable(e) & rhs.Reachable(e);
    // insert checks if empty or not
    res.Insert(e, std::move(intersect));
  }

  return res;
}

template <class Ts>
inline Relation<Ts> operator&(Relation<Ts>&& lhs, const Relation<Ts>& rhs) {
  lhs &= rhs;
  return std::move(lhs);
}

template <class Ts>
inline Relation<Ts> operator&(const Relation<Ts>& lhs, Relation<Ts>&& rhs) {
  rhs &= lhs;
  return std::move(rhs);
}

template <class Ts>
inline Relation<Ts> operator&(Relation<Ts>&& lhs, Relation<Ts>&& rhs) {
  lhs &= rhs;
  return std::move(lhs);
}

/**
 * Relation operator base class.
 * No derived class shall define a destructor!
 */
template <class Ts>
class RelationOp {
 public:
  RelationOp() {}

  explicit RelationOp(std::vector<Relation<Ts>> rels)
      : rels_(std::move(rels)) {}

  /*//virtual ~RelationOp()
   *
   * No destructor, so compiler can implicitly create move constructor and
   * assignment operators.
  */

  /**
   * Evaluate in-place, where postcondition is rels_.size() <= 1. This avoids
   * some of the copying overhead of Eval(), and can therefore be more
   * efficient.
   *
   * @return Reference to this object.
   */
  virtual RelationOp& EvalInplace() = 0;

  /**
   * Evaluate operator, computing the result of the opertor.
   *
   * @return Relation representing view of operator.
   */
  virtual Relation<Ts> Eval() const = 0;

  void Clear() { rels_.clear(); }

  /**
   * Evaluate operator in-place, and clearing it, returning the evaluated
   * Relation. Optimized for move, and should be used where the operator is
   * used as a temporary.
   *
   * @return Relation representing view of operator before call.
   */
  Relation<Ts> EvalClear() {
    if (rels_.empty()) {
      // Already cleared
      return Relation<Ts>();
    }

    EvalInplace();
    assert(rels_.size() == 1);

    Relation<Ts> result = std::move(rels_.back());
    rels_.clear();
    return result;  // NRVO
  }

 protected:
  void Add(const Relation<Ts>& er) { rels_.push_back(er); }

  void Add(Relation<Ts>&& er) { rels_.push_back(std::move(er)); }

  void Add(const std::vector<Relation<Ts>>& rels) {
    rels_.reserve(rels_.size() + rels.size());
    rels_.insert(rels_.end(), rels.begin(), rels.end());
  }

 protected:
  std::vector<Relation<Ts>> rels_;
};

/**
 * Operator ";".
 */
template <class Ts>
class RelationSeq : public RelationOp<Ts> {
 public:
  typedef typename Ts::Element Element;

  RelationSeq() {}

  explicit RelationSeq(std::vector<Relation<Ts>> v)
      : RelationOp<Ts>(std::move(v)) {}

  RelationSeq& operator+=(const Relation<Ts>& rhs) {
    this->Add(rhs);
    return *this;
  }

  RelationSeq& operator+=(Relation<Ts>&& rhs) {
    this->Add(std::move(rhs));
    return *this;
  }

  RelationSeq& operator+=(const RelationSeq& rhs) {
    this->Add(rhs.rels_);
    return *this;
  }

  RelationOp<Ts>& EvalInplace() override {
    while (this->rels_.size() > 1) {
      std::size_t from_idx = this->rels_.size() - 2;
      const auto& first = this->rels_[from_idx];
      const auto& last = this->rels_.back();

      Relation<Ts> er;

      first.for_each([&er, &last](const Element& e1, const Element& e2) {
        if (last.InDomain(e2)) {
          er.Insert(e1, last.Reachable(e2));
        }
      });

      this->rels_.erase(this->rels_.end() - 2, this->rels_.end());
      this->rels_.push_back(std::move(er));
    }

    return *this;
  }

  Relation<Ts> Eval() const override {
    Relation<Ts> er;

    if (this->rels_.empty()) {
      return Relation<Ts>();
    } else if (this->rels_.size() == 1) {
      return this->rels_.back();
    }

    const auto potential_domain = this->rels_.front().Domain();
    const auto potential_range = this->rels_.back().Range();

    for (const auto& e1 : potential_domain.get()) {
      for (const auto& e2 : potential_range.get()) {
        if (R(e1, e2)) {
          er.Insert(e1, e2);
        }
      }
    }

    return er;
  }

  /**
   * Check if (e1, e2) is in the relation. This effectively does a search if
   * there is an edge from e1 to e2.
   *
   * @param e1 First element.
   * @param e2 Second element.
   * @param path Optional; return path from e1 to e2.
   * @return true if related, false otherwise.
   */
  bool R(const Element& e1, const Element& e2,
         typename Relation<Ts>::Path* path = nullptr,
         std::size_t seq = 0) const {
    if (this->rels_.empty()) {
      return false;
    }

    assert(seq < this->rels_.size());

    if (seq + 1 < this->rels_.size()) {
      const auto& rel = this->rels_[seq];
      std::size_t path_size = 0;

      const Set<Ts> reach = rel.Reachable(e1);
      for (const auto& e : reach.get()) {
        if (path != nullptr) {
          path_size = path->size();
          rel.R(e1, e, path);  // true
          path->pop_back();    // remove e
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

  /**
   * Check if irreflexive.
   *
   * @param cyclic Optional parameter, in which the cycle is returned, if
   *               result is false.
   * @return true if irreflexive, false otherwise.
   */
  bool Irreflexive(typename Relation<Ts>::Path* cyclic = nullptr) const {
    if (this->rels_.empty()) {
      return true;
    }

    const auto domain = this->rels_.front().Domain();
    for (const auto& e : domain.get()) {
      if (R(e, e, cyclic)) {
        return false;
      }
    }

    return true;
  }
};

template <class Ts>
inline RelationSeq<Ts> operator+(const RelationSeq<Ts>& lhs,
                                 const Relation<Ts>& rhs) {
  RelationSeq<Ts> res = lhs;
  return res += rhs;
}

template <class Ts>
inline RelationSeq<Ts> operator+(RelationSeq<Ts>&& lhs,
                                 const Relation<Ts>& rhs) {
  lhs += rhs;
  return std::move(lhs);
}

template <class Ts>
inline RelationSeq<Ts> operator+(const RelationSeq<Ts>& lhs,
                                 Relation<Ts>&& rhs) {
  RelationSeq<Ts> res = lhs;
  return res += std::move(rhs);
}

template <class Ts>
inline RelationSeq<Ts> operator+(RelationSeq<Ts>&& lhs, Relation<Ts>&& rhs) {
  lhs += std::move(rhs);
  return std::move(lhs);
}

template <class Ts>
inline RelationSeq<Ts> operator+(const RelationSeq<Ts>& lhs,
                                 const RelationSeq<Ts>& rhs) {
  RelationSeq<Ts> res = lhs;
  return res += rhs;
}

template <class Ts>
inline RelationSeq<Ts> operator+(RelationSeq<Ts>&& lhs,
                                 const RelationSeq<Ts>& rhs) {
  lhs += rhs;
  return std::move(lhs);
}

template <class Ts>
inline RelationSeq<Ts> operator+(const RelationSeq<Ts>& lhs,
                                 RelationSeq<Ts>&& rhs) {
  RelationSeq<Ts> res = lhs;
  return res += rhs;
}

template <class Ts>
inline RelationSeq<Ts> operator+(RelationSeq<Ts>&& lhs, RelationSeq<Ts>&& rhs) {
  lhs += rhs;
  return std::move(lhs);
}

/**
 * @brief Helper class to instantiate types used by Set, Relation, etc.
 *
 * Set, Relation, etc. take a template parameter that provides Element, and
 * SetContainer or MapContainer; this class can be used to instantiate a class
 * to be passed as the template parameter to Set and Relation.
 */
template <class E, class Hash = typename E::Hash>
struct Types {
  typedef E Element;

  typedef std::unordered_set<Element, Hash> SetContainer;

  template <class T>
  using MapContainer = std::unordered_map<Element, T, Hash>;
};

}  // namespace sets
}  // namespace mc2lib

#endif /* MC2LIB_SETS_HPP_ */

/* vim: set ts=2 sts=2 sw=2 et : */
