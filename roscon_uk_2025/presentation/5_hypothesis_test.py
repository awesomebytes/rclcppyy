#!/usr/bin/env python3
from hypothesis import given, strategies as st
import cppyy

# A function that accepts strings but will throw if the string contains "DDS"
cppyy.cppdef("""
#include <iostream>
void accept_string(std::string s) {
    if (s.find("DDS") != std::string::npos) {
        throw std::runtime_error("Oh no! It's DDS!");
    }
    return;
}
""")

# Example C++ function that accepts strings
accept_string_cpp = cppyy.gbl.accept_string
@given(st.text(alphabet="ABCDES", min_size=3, max_size=9))
def test_cpp_strings(s):
    print(f"(c++) called with {s}")
    accept_string_cpp(s)
    assert isinstance(s, str)

# Example pure python
@given(st.integers())
def test_integers(n):
    print(f"called with {n}")
    assert isinstance(n, int)

test_integers()

test_cpp_strings()

# # Define a C++ functon that takes an integer and prints it
# cppyy.cppdef("""
# #include <iostream>
# void accept_integer(int n) {
#     // We do nothing
#     return;
# }
# """)

# # Example C++ function
# accept_integer_cpp = cppyy.gbl.accept_integer
# @given(st.integers())
# def test_cpp_integers(n):
#     print(f"(c++) called with {n}")
#     accept_integer_cpp(n)
#     assert isinstance(n, int)

# test_cpp_integers()

