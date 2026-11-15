# Arrow 18 reads these URLs from the configure process environment. Populate it
# from persistent CMake cache entries so a later plain make needs no shell exports.
foreach(variable IN ITEMS ARROW_XSIMD_URL ARROW_MIMALLOC_URL ARROW_BOOST_URL)
    if(DEFINED ${variable} AND NOT "${${variable}}" STREQUAL "")
        set(ENV{${variable}} "${${variable}}")
    endif()
endforeach()
