<<<<<<< HEAD
rm -rf /build /devel

=======
>>>>>>> master
catkin_make --only-pkg-with-deps msg_collection
catkin_make --only-pkg-with-deps utils
catkin_make --only-pkg-with-deps motion

<<<<<<< HEAD
=======

>>>>>>> master
catkin_make -DCATKIN_WHITELIST_PACKAGES=
