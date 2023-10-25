# tinympcConfig.cmake

include(CMakeFindDependencyMacro)

find_dependency(Catch2)
find_dependency(Eigen3)
find_dependency(OsqpEigen)

include("${CMAKE_CURRENT_LIST_DIR}/tinympc_targets.cmake")
