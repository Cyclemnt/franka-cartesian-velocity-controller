# find_path: 
# - The general signature of this command is: 'find_path (<VAR> name1 [path1 path2 ...])'. 
# - It is used to find a directory containing the named file. 
# - In this case, it is used to find the path to the Gurobi header file 'gurobi_c.h'. 
#   - It sets the variable 'GUROBI_INCLUDE_DIRS' to the directory containing the header file mentioned above.
#   - The 'HINTS' option allows specifying additional paths to search for the header file, such as the '${GUROBI_DIR}' and '$ENV{GUROBI_HOME}'.

find_path(
  GUROBI_INCLUDE_DIRS
  NAMES gurobi_c.h
  HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
  PATH_SUFFIXES include)

# find_library: 
# - The general signature of this command is: 'find_library (<VAR> name1 [path1 path2 ...])'. 
# - It is used to find a library. 
# - In this case, it is used to find the Gurobi library file. 
#   - It sets the variable 'GUROBI_LIBRARY' to the path of the library file.
#   - The 'NAMES' option allows specifying the possible names of the library, such as 'gurobi', 'gurobi95', 'gurobi100' and 'gurobi110'.
#   - The 'HINTS' option allows specifying additional paths to search for the library file, such as the '${GUROBI_DIR}' and '$ENV{GUROBI_HOME}'.
#   - The 'PATH_SUFFIXES' option allows specifying additional subdirectories in which to search for the library file, such as 'lib'.

find_library(
  GUROBI_LIBRARY
  NAMES gurobi gurobi110 gurobi120 gurobi130
  HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
  PATH_SUFFIXES lib)

# The content of the following conditional block is executed only if the variable 'CXX' is true. 
# - The first instruction, which is 'if(MSVC)', checks if the used compiler is 'MSVC' (Microsoft Visual C++) and sets some variables accordingly.

if(NOT DEFINED CXX)
  set(CXX TRUE)
endif()

if(CXX)
  if(MSVC)
    # determine Visual Studio year
    if(MSVC_TOOLSET_VERSION EQUAL 142)
      set(MSVC_YEAR "2019")
    elseif(MSVC_TOOLSET_VERSION EQUAL 141)
      set(MSVC_YEAR "2017")
    elseif(MSVC_TOOLSET_VERSION EQUAL 140)
      set(MSVC_YEAR "2015")
    endif()

    if(MT)
      set(M_FLAG "mt")
    else()
      set(M_FLAG "md")
    endif()

    find_library(
      GUROBI_CXX_LIBRARY
      NAMES gurobi_c++${M_FLAG}${MSVC_YEAR}
      HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
      PATH_SUFFIXES lib)
    find_library(
      GUROBI_CXX_DEBUG_LIBRARY
      NAMES gurobi_c++${M_FLAG}d${MSVC_YEAR}
      HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
      PATH_SUFFIXES lib)
  else()
    find_library(
      GUROBI_CXX_LIBRARY
      NAMES gurobi_c++
      HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
      PATH_SUFFIXES lib)
    set(GUROBI_CXX_DEBUG_LIBRARY ${GUROBI_CXX_LIBRARY})
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI DEFAULT_MSG GUROBI_LIBRARY GUROBI_INCLUDE_DIRS)
