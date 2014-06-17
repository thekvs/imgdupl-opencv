## Description

This is an experiment to use [OpenCV](http://opencv.org/) to detect duplicate images.
It's a purely research project not intended to be used in a production enviroment with
big data sets.

## Building

This is a standard CMake project and build procedure is quite straightforward. Assuming that the
source files are located in the ```/path/to/imgdupl-opencv/``` directory one needs to perform
following steps:

```
$ mkdir /path/to/imgdupl-opencv-buildroot/
$ cd /path/to/imgdupl-opencv-buildroot/
$ cmake /path/to/imgdupl-opencv/ -DCMAKE_BUILD_TYPE=Release
$ make
```

## Usage

### Build database

First you need to create a database with extracted keypoints and features of an image set.
To do this you use build-imgdupl-db utility. For a while invocation is very simple and basically
looks like:

```
$ ./build-imgdupl-db directory database
```

Where directory contains images and database is a file in filesystem. Utility recursively walks
through directory tree and fills database with corresponding data. We use SQLite engine for data
management.

Example:

```
$ ./build-imgdupl-db /mnt/disk/Photos/Party/ /tmp/images.db
```

### Run an imgdupl process

Example:

```
$ ./imgdupl --db-file /tmp/images.db --config imgdupl.cfg
```

### Usage from a command line

```
$ curl --data-binary "@file.jpg" -H "Content-Type: application/octet-stream" -X POST http://127.0.0.1:9090/check
```

Here file.jpg is a name of a file on the disk, i.e. /full/path/to/image_001.jpg.
