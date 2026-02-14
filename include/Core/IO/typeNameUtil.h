#pragma once 

constexpr const char* find_type_start(const char* s) {
    return (*s == '=') ? (s + 2) : find_type_start(s + 1);
}

template <typename T>
constexpr const char* get_typename() {
    return find_type_start(__PRETTY_FUNCTION__);
}