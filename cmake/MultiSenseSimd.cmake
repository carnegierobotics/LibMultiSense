include(CheckCXXSourceCompiles)
include(CMakePushCheckState)

# Compile-only checks also work when cross compiling. In particular, do not
# decide support from the build machine's CPU or CMAKE_HOST_SYSTEM_PROCESSOR.
function(multisense_check_avx2 result options)
    set(${result} OFF PARENT_SCOPE)
    set(${options} "" PARENT_SCOPE)
    if (MSVC)
        set(avx2_options /arch:AVX2)
    elseif (CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$")
        set(avx2_options -mavx2)
    else()
        return()
    endif()

    cmake_push_check_state(RESET)
    check_cxx_source_compiles([[
        #if !(defined(__i386__) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_X64)) || defined(_M_ARM64EC)
        #error AVX2 requires an x86 target
        #endif
        #if defined(_MSC_VER)
        #include <intrin.h>
        int main() {
            int registers[4];
            __cpuidex(registers, 1, 0);
            return registers[0] + static_cast<int>(_xgetbv(0));
        }
        #else
        int main() {
            __builtin_cpu_init();
            return __builtin_cpu_supports("avx2") ? 0 : 1;
        }
        #endif
    ]] MULTISENSE_COMPILER_HAS_AVX2_DISPATCH)

    if (MULTISENSE_COMPILER_HAS_AVX2_DISPATCH)
        string(JOIN " " CMAKE_REQUIRED_FLAGS ${avx2_options})
        check_cxx_source_compiles([[
            #include <immintrin.h>
            int main(int argc, char**) {
                const __m256i value = _mm256_set1_epi32(argc);
                return _mm256_extract_epi32(_mm256_add_epi32(value, value), 0);
            }
        ]] MULTISENSE_COMPILER_HAS_AVX2)
        if (MULTISENSE_COMPILER_HAS_AVX2)
            set(${result} ON PARENT_SCOPE)
            set(${options} "${avx2_options}" PARENT_SCOPE)
        endif()
    endif()
    cmake_pop_check_state()
endfunction()
