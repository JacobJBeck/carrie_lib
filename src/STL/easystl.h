// Copyright 2016 Carrie Rebhuhn
#ifndef STL_EASYSTL_H_
#define STL_EASYSTL_H_
#include <algorithm>
#include <vector>

namespace easystl {
//! Clears using the swap idiom
/*template <class Container>
void clear(Container *q) {
    Container empty;
    std::swap(*q, empty);
}*/

template <class ptr>
void clear(std::vector<ptr*> &v) {
    while (v.size()) {
        delete v.back();
        v.pop_back();
    }
}

template <class ptr>
void clear(std::list<ptr*> &v) {
    while (v.size()) {
        delete v.back();
        v.pop_back();
    }
}

//! Remove-erase-if idiom
template <class Container, class UnaryPredicate>
void remove_erase_if(Container* stl, UnaryPredicate pred) {
    stl->erase(std::remove_if(stl->begin(), stl->end(), pred), stl->end());
}

template <class Container, class T>
void remove_element(Container* stl, T el) {
    stl->erase(std::find(stl->begin(), stl->end(), el));
}
}  // namespace easystl
#endif  // STL_EASYSTL_H_
