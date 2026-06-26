
include (CheckCXXSourceCompiles)
set(FILESYSTEM_CODE "
    #include <filesystem>
    int main() {
        std::filesystem::path p(\"/\");
        return std::filesystem::exists(p) ? 0 : 1;
    }
")

check_cxx_source_compiles("${FILESYSTEM_CODE}" HAVE_STD_FILESYSTEM)

if (NOT HAVE_STD_FILESYSTEM)
    # Try linking with stdc++fs (GCC < 9, Clang < 9)
    set(CMAKE_REQUIRED_LIBRARIES stdc++fs)
    check_cxx_source_compiles("${FILESYSTEM_CODE}" HAVE_STD_FILESYSTEM_STDCXXFS)
    if (HAVE_STD_FILESYSTEM_STDCXXFS)
        set(FILESYSTEM_LIBRARY stdc++fs)
        set(HAVE_STD_FILESYSTEM TRUE)
    else()
        # Try linking with c++fs (older Clang/libc++)
        set(CMAKE_REQUIRED_LIBRARIES c++fs)
        check_cxx_source_compiles("${FILESYSTEM_CODE}" HAVE_STD_FILESYSTEM_CXXFS)
        if (HAVE_STD_FILESYSTEM_CXXFS)
            set(FILESYSTEM_LIBRARY c++fs)
            set(HAVE_STD_FILESYSTEM TRUE)
        endif()
    endif()
    unset(CMAKE_REQUIRED_LIBRARIES)
endif()
