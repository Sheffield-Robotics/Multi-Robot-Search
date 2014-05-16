Multi-Robot-Search
==================

Software useful for coordinated search with large teams of robots 


==================
Installation for Mac:

1) install xcode and macports
2) run:
xcode-select --install 
sudo port install cgal +qt4 +universal libgeotiff opencv boost

3) go into build directory and run
cmake ../modules
make

4) go into the bin directoy and run
./gui_exe -f ../maps/freiburg.tiff

