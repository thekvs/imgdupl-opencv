# - Find libyajl
# Find the native YAJL headers and libraries.
#
#  YAJL_INCLUDE_DIRS - where to find libyajl headers
#  YAJL_LIBRARIES    - List of libraries when using libyajl.
#  YAJL_FOUND        - True if libyajl found.

# Look for the header file.
find_path(YAJL_INCLUDE_DIR NAMES yajl/yajl_gen.h yajl/yajl_tree.h)
mark_as_advanced(YAJL_INCLUDE_DIR)

# Look for the library.
find_library(YAJL_LIBRARY NAMES yajl)
mark_as_advanced(YAJL_LIBRARY)

# handle the QUIETLY and REQUIRED arguments and set LIBYAJL_FOUND to TRUE if 
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(yajl DEFAULT_MSG YAJL_LIBRARY YAJL_INCLUDE_DIR)

if (YAJL_FOUND)
  set(YAJL_LIBRARIES ${YAJL_LIBRARY})
  set(YAJL_INCLUDE_DIRS ${YAJL_INCLUDE_DIR})
endif ()
