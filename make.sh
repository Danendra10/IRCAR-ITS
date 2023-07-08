rm -rf /build /devel

catkin_make --only-pkg-with-deps msg_collection
catkin_make --only-pkg-with-deps utils
catkin_make --only-pkg-with-deps motion

catkin_make -DCATKIN_WHITELIST_PACKAGES=
