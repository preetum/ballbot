#!/bin/bash

echo -n "Should I install the prerequisites for you? [y/N] "
read character
case $character in
    [Yy] )
	sudo apt-get install swig python-dev libcv-dev
        ;;
    * )
esac

swig -c++ -python -shadow BlobResult.i
swig -c++ -python -shadow Blob.i

if [ $HOSTTYPE == "x86_64" ]; then
	echo "Compiling for 64-bit."
	gcc -c -fPIC BlobResult.cpp BlobResult_wrap.cxx BlobExtraction.cpp Blob.cpp Blob_wrap.cxx -I/usr/include/python2.5 `pkg-config --cflags opencv`
else
	echo "You are not 64-bit."
	gcc -c BlobResult.cpp BlobResult_wrap.cxx BlobExtraction.cpp Blob.cpp Blob_wrap.cxx -I/usr/include/python2.5 `pkg-config --cflags opencv`
fi

mkdir blobs

ld -shared Blob.o Blob_wrap.o -o blobs/_Blob.so `pkg-config --libs opencv`
ld -shared BlobResult.o BlobResult_wrap.o BlobExtraction.o Blob.o -o blobs/_BlobResult.so `pkg-config --libs opencv`

touch blobs/__init__.py
mv Blob.py blobs
mv BlobResult.py blobs

echo "Done."
