
include (CheckCXXSourceCompiles)
check_cxx_source_compiles (
    "int main() { constexpr int x = 0; }"
    HAVE_CONSTEXPR
)
if (HAVE_CONSTEXPR)
    add_definitions(-DCRL_HAVE_CONSTEXPR)
endif ()
